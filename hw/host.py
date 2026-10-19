"""Wall-sanding host: the 5 N admittance loop on real hardware or dry.

Same phase machine (move -> acquire -> sweep -> retract), same Admittance and
Coverage classes, same kinematics/IK as the MuJoCo sim (sim/run_demo.py,
sim/robot.py). The only difference: force comes from servo-load estimation
instead of MuJoCo contacts.

Dry run (no hardware):  python host.py --dry --passes 3 [--fast]
Real robot:             python host.py --port COM3 --enable [--no-base]
Telemetry/calibration:  python host.py --port COM3 --hold --enable
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sim'))
sys.path.insert(0, str(ROOT / 'hw'))

import control
import wall_geometry as wg
import feetech
from feetech import ServoSpec, DryBus, SerialBus, BusError
from kinematics import Kinematics
from force_est import ForceEstimator

STANDOFF = 0.025
BASELINE_Y = wg.WALL_Y - 0.003
AXIS = np.array([0.0, -1.0, wg.PRETILT])
AXIS = AXIS / np.linalg.norm(AXIS)
LOGS = Path(__file__).resolve().parent / 'logs'

SERVOS = [
    ServoSpec('shoulder_pan', 1),
    ServoSpec('shoulder_lift', 2),
    ServoSpec('elbow_flex', 3),
    ServoSpec('wrist_flex', 4),
    ServoSpec('wrist_roll', 5),
]
CONFIG = Path(__file__).resolve().parent / 'robot.json'


def load_servos(path=None):
    """Per-robot calibration: ids, signs, zero offsets, load signs, torque."""
    import json as _json
    p = Path(path) if path else CONFIG
    if not p.exists():
        return list(SERVOS)
    cfg = _json.loads(p.read_text())
    specs = []
    for entry in cfg.get('servos', []):
        specs.append(ServoSpec(entry['name'], int(entry['id']),
                               sign=float(entry.get('sign', 1)),
                               offset_rad=float(entry.get('offset_rad', 0.0)),
                               load_sign=float(entry.get('load_sign', 1)),
                               max_torque_nm=float(entry.get('max_torque_nm', 2.94))))
    return specs or list(SERVOS)


def write_config(specs, tau_bias=None, path=None):
    import json as _json
    p = Path(path) if path else CONFIG
    cfg = {'servos': [{'name': s.name, 'id': s.servo_id, 'sign': s.sign,
                       'offset_rad': s.offset_rad, 'load_sign': s.load_sign,
                       'max_torque_nm': s.max_torque_nm} for s in specs]}
    if tau_bias is not None:
        cfg['tau_bias'] = [float(v) for v in tau_bias]
    p.write_text(_json.dumps(cfg, indent=2))
    return p


def load_bias(path=None):
    import json as _json
    p = Path(path) if path else CONFIG
    if not p.exists():
        return None
    return _json.loads(p.read_text()).get('tau_bias')


def quintic(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (10 + u * (-15 + 6 * u))


def build_passes(n):
    plan = []
    for k in range(min(n, len(wg.PASS_Z))):
        z = wg.PASS_Z[k]
        if k % 2 == 0:
            xs, xe = wg.SWEEP_X
        else:
            xe, xs = wg.SWEEP_X
        plan.append({'z': z, 'xs': xs, 'xe': xe})
    return plan


class DryBase:
    def __init__(self, x0=0.0, rate=0.6):
        self.cmd = x0
        self.x = x0
        self.rate = rate

    def move_to(self, x):
        self.cmd = float(x)

    def read(self):
        return float(self.x)

    def tick(self, dt):
        self.x += float(np.clip(self.cmd - self.x, -self.rate * dt, self.rate * dt))


class LockedBase:
    def __init__(self, x0=0.0):
        self.x = float(x0)

    def move_to(self, x):
        pass

    def read(self):
        return self.x

    def tick(self, dt):
        pass


class DryWall:
    """Pad-footprint spring contact + load injection into DryBus.

    Depth is averaged over 13 sample points across the 45 mm pad face (softens
    the seam step like the compliant pad does), force adds a damping term on
    the depth rate (viscoelastic pad), and joint loads are injected into the
    DryBus one tick before they are read (sensor delay).
    """

    def __init__(self, kin, specs, bus, stiffness=9000.0, damping=40.0,
                 friction=0.02, seed=1):
        self.kin = kin
        self.specs = specs
        self.bus = bus
        self.k = stiffness
        self.c = damping
        self.friction = friction
        self.rng = np.random.default_rng(seed)
        self.force = 0.0
        self.prev_depth = None
        offs = [(0.0, 0.0)]
        for a in np.linspace(0, 2 * np.pi, 5)[:-1]:
            offs.append((0.0225 * np.cos(a), 0.0225 * np.sin(a)))
        for a in np.linspace(0, 2 * np.pi, 9)[:-1]:
            offs.append((0.045 * np.cos(a), 0.045 * np.sin(a)))
        self.offs = np.array(offs)

    def depth(self, p, R):
        pts = p[None, :] + self.offs[:, :1] * R[None, :, 0] + self.offs[:, 1:] * R[None, :, 1]
        ys = np.array([wg.wall_surface_y(px, pz) for px, pz in pts[:, [0, 2]]])
        return float(np.clip(ys - pts[:, 1], 0.0, None).mean())

    def step(self, q, bx, dt):
        p, R, _, _, _, _, _ = self.kin.fk(q, bx)
        d = self.depth(p, R)
        rate = 0.0 if self.prev_depth is None else (d - self.prev_depth) / dt
        self.prev_depth = d
        F = max(0.0, self.k * d + self.c * min(max(rate, -0.1), 0.1))
        n = -R[:, 2]
        n = n / (np.linalg.norm(n) + 1e-12)
        Jp, _, _, _, _, _, _ = self.kin.jac(q, bx)
        tau_g = self.kin.gravity_torque(q, bx)[:5]
        tau_ext = Jp[:, :5].T @ (F * n)
        stic = self.friction * np.tanh((tau_g + tau_ext) / 0.05)
        tau_true = tau_g + tau_ext + stic + self.rng.normal(0.0, 0.008, 5)
        for s, t in zip(self.specs, tau_true):
            self.bus.pretend_load[s.servo_id] = s.from_torque(t)
        self.force = F
        return F


class HostRunner:
    def __init__(self, args):
        self.args = args
        self.kin = Kinematics()
        self.specs = load_servos(args.config)
        self.dry = args.port is None
        self.hz = args.hz
        self.max_force = args.max_force
        self.plan = build_passes(args.passes)
        self.adm = control.Admittance()
        self.cov = control.Coverage()
        self.fest = ForceEstimator(self.kin, tau_bias=load_bias(args.config))
        self.log = {k: [] for k in
                    ['t', 'phase', 'ix', 'cmd', 'pad', 'f', 'f_dry', 'p', 'pe', 'ae', 'base']}
        self.bad_ik = 0
        self.overruns = 0
        self.force_events = 0
        self.estop = Path(args.estop) if args.estop else None
        p0 = self.plan[0]
        self.start = np.array([p0['xs'],
                               wg.wall_surface_y(p0['xs'], p0['z']) + STANDOFF,
                               p0['z']])
        if self.dry:
            self.bus = DryBus(self.specs)
            self.base = DryBase(p0['xs'])
            self.wall = DryWall(self.kin, self.specs, self.bus)
            q0, _, pe, ae = self.kin.ik(self.start, axis=AXIS, bx=p0['xs'])
            if pe > 0.002:
                raise RuntimeError(f'dry init IK failed: {pe*1000:.1f} mm')
            for s, qi in zip(self.specs, q0):
                self.bus.pos[s.servo_id] = float(s.to_ticks(qi))
                self.bus.goal[s.servo_id] = float(s.to_ticks(qi))
            self.kin.prev_sol = q0.copy()
        else:
            self.bus = SerialBus(args.port)
            self.base = LockedBase(args.base0) if args.no_base else DryBase(args.base0)
            self.wall = None
            st = self._read_states_checked()
            self.bus.enable_torque(self.specs, True)
            print('torque enabled on servos:', [s.servo_id for s in self.specs])
            q_act = self.measured_q(st)
            self.kin.prev_sol = q_act.copy()
        self.phase = 'move'
        self.phase_t0 = 0.0
        self.pass_idx = 0
        self.t_ctrl = 0.0
        self.pose0 = self.kin.fk(*self._measured_qx(self._read_states()))[0].copy()

    def _read_states(self):
        return self.bus.read_states(self.specs)

    def _read_states_checked(self):
        st = self._read_states()
        missing = [s.servo_id for s in self.specs if s.servo_id not in st]
        if missing:
            raise BusError(f'servos not responding: {missing}')
        return st

    def measured_q(self, st):
        return np.array([s.to_rad(st[s.servo_id]['pos']) for s in self.specs])

    def measured_loads(self, st):
        return np.array([s.to_torque(st[s.servo_id]['load']) for s in self.specs])

    def _measured_qx(self, st):
        return self.measured_q(st), self.base.read()

    def phase_target(self, t):
        p = self.plan[self.pass_idx]
        x0, y0, z0 = self.pose0
        if self.phase == 'move':
            T = 2.0
            u = quintic(t / T)
            x = x0 + (p['xs'] - x0) * u
            z = z0 + (p['z'] - z0) * u
            ytar = wg.wall_surface_y(p['xs'], p['z']) + STANDOFF
            y = y0 + (ytar - y0) * u
            return x, y, z, t >= T, False
        if self.phase == 'acquire':
            T = 2.6
            x, z = p['xs'], p['z']
            if self.args.mode == 'controlled':
                y = wg.wall_surface_y(x, z) - self.adm.p
            else:
                y = BASELINE_Y
            return x, y, z, t >= T, True
        if self.phase == 'sweep':
            T = abs(p['xe'] - p['xs']) / wg.SWEEP_V
            x = p['xs'] + (p['xe'] - p['xs']) * min(1.0, t / T)
            z = p['z']
            if self.args.mode == 'controlled':
                y = wg.wall_surface_y(x, z) - self.adm.p
            else:
                y = BASELINE_Y
            return x, y, z, t >= T, True
        if self.phase == 'retract':
            T = 0.6
            u = quintic(t / T)
            z = p['z']
            xt = x0
            yt = wg.wall_surface_y(xt, z) + STANDOFF
            x = xt
            y = y0 + (yt - y0) * u
            return x, y, z, t >= T, False
        raise ValueError(self.phase)

    def advance(self, pad):
        if self.phase == 'move':
            self.phase = 'acquire'
            self.adm.reset(p=-STANDOFF)
        elif self.phase == 'acquire':
            self.phase = 'sweep'
        elif self.phase == 'sweep':
            self.phase = 'retract'
        elif self.phase == 'retract':
            self.pass_idx += 1
            if self.pass_idx >= len(self.plan):
                self.phase = 'done'
                return
            self.phase = 'move'
        self.phase_t0 = self.t_ctrl
        self.pose0 = pad.copy()

    def tick(self):
        dt = 1.0 / self.hz
        self.t_ctrl += dt
        self.bus.tick(dt)
        self.base.tick(dt)
        st = self._read_states()
        tmax = max(st[s.servo_id]['temp'] for s in self.specs)
        vmin = min(st[s.servo_id]['voltage'] for s in self.specs)
        if tmax > self.args.max_temp:
            raise RuntimeError(f'servo over-temperature: {tmax} C')
        if 0.0 < vmin < self.args.min_volt:
            raise RuntimeError(f'servo under-voltage: {vmin:.1f} V')
        q_act = self.measured_q(st)
        loads = self.measured_loads(st)
        bx_act = self.base.read()
        f_dry = self.wall.step(q_act, bx_act, dt) if self.dry else 0.0
        f_est = self.fest.update(q_act, bx_act, loads)
        pad = self.kin.fk(q_act, bx_act)[0]
        t = self.t_ctrl - self.phase_t0
        x, y, z, done, in_sweep = self.phase_target(t)
        self.base.move_to(x)
        if self.args.mode == 'controlled' and self.phase in ('acquire', 'sweep'):
            self.adm.update(f_est, dt)
        q_cmd, _, pe, ae = self.kin.track(np.array([x, y, z]), axis=AXIS, bx=bx_act)
        if pe > 0.006 or ae > 12.0:
            self.bad_ik += 1
        moves = [(s.servo_id, s.to_ticks(qi), self.args.servo_speed, self.args.servo_acc)
                 for s, qi in zip(self.specs, q_cmd)]
        self.bus.sync_write_goal(moves)
        if self.phase == 'sweep':
            self.cov.add(float(pad[0]), float(pad[2]), dt, f_est >= wg.F_CONTACT_MIN)
        phase_id = {'move': 0, 'acquire': 1, 'sweep': 2, 'retract': 3}[self.phase]
        for k, v in [('t', self.t_ctrl), ('phase', phase_id), ('ix', self.pass_idx),
                     ('cmd', (x, y, z)), ('pad', tuple(pad)), ('f', f_est),
                     ('f_dry', f_dry), ('p', self.adm.p), ('pe', pe), ('ae', ae),
                     ('base', bx_act)]:
            self.log[k].append(v)
        if f_est > self.max_force:
            self.force_events += 1
            self.adm.reset(p=min(self.adm.p, -0.02))
            print(f'  !! force {f_est:.1f} N > {self.max_force:.0f} N: backing off')
        if done:
            self.advance(pad)

    def run(self):
        dt = 1.0 / self.hz
        if sys.platform == 'win32':
            try:
                import ctypes
                ctypes.WinDLL('winmm').timeBeginPeriod(1)
            except Exception:
                pass
        t_wall0 = time.perf_counter()
        next_t = t_wall0
        guard_steps = int(600 / dt)
        steps = 0
        while self.phase != 'done' and steps < guard_steps:
            if self.estop and self.estop.exists():
                print('  !! ESTOP file present, aborting')
                break
            self.tick()
            steps += 1
            if not self.args.fast:
                next_t += dt
                w = next_t - time.perf_counter()
                if w > 0.002:
                    time.sleep(w - 0.001)
                while time.perf_counter() < next_t:
                    pass
                if time.perf_counter() - next_t > 0.5 * dt:
                    self.overruns += 1
                    next_t = time.perf_counter()
        wall = time.perf_counter() - t_wall0
        return self.summarize(wall, steps)

    def summarize(self, wall, steps):
        d = {k: np.array(v) if k in ('cmd', 'pad') else np.array(v, dtype=float)
             for k, v in self.log.items()}
        sweep = d['phase'] == 2
        f = d['f'][sweep]
        e = f - wg.F_DES
        pad = d['pad'][sweep]
        met = {
            'mode': self.args.mode,
            'dry': bool(self.dry),
            'wall_clock_s': round(wall, 1),
            'ctrl_ticks': steps,
            'ik_failures': self.bad_ik,
            'overruns': self.overruns,
            'force_events': self.force_events,
            'sweep_ticks': int(sweep.sum()),
            'force_mean': round(float(f.mean()), 3) if len(f) else 0.0,
            'force_err_mean_abs': round(float(np.abs(e).mean()), 3) if len(f) else 0.0,
            'force_err_p95_abs': round(float(np.percentile(np.abs(e), 95)), 3) if len(f) else 0.0,
            'force_max': round(float(f.max()), 3) if len(f) else 0.0,
            'contact_duty': round(float((f >= wg.F_CONTACT_MIN).mean()), 4) if len(f) else 0.0,
            'ik_pos_err_mean_mm': round(float(np.nanmean(d['pe']) * 1000), 2),
            'coverage_frac': round(self.cov.fraction(), 4),
        }
        if self.dry and len(f):
            fd = d['f_dry'][sweep]
            met['dry_force_bias'] = round(float((f - fd).mean()), 3)
        return met, d

    def save(self, met, d):
        LOGS.mkdir(exist_ok=True)
        tag = time.strftime('%Y%m%d_%H%M%S') + ('_dry' if self.dry else '')
        with open(LOGS / f'{tag}.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            cols = ['t', 'phase', 'ix', 'f', 'f_dry', 'p', 'pe', 'ae', 'base',
                    'pad_x', 'pad_y', 'pad_z']
            w.writerow(cols)
            n = len(self.log['t'])
            for i in range(n):
                w.writerow([self.log['t'][i], self.log['phase'][i], self.log['ix'][i],
                            self.log['f'][i], self.log['f_dry'][i], self.log['p'][i],
                            self.log['pe'][i], self.log['ae'][i], self.log['base'][i],
                            self.log['pad'][i][0], self.log['pad'][i][1],
                            self.log['pad'][i][2]])
        (LOGS / f'{tag}.json').write_text(json.dumps(met, indent=2))
        return tag

    def calib(self):
        print('bias calibration: arm powered, OFF the wall, keep it still ~2 s')
        for _ in range(10):
            self.bus.tick(0.05)
            self._read_states()
            time.sleep(0.05)
        tau_sum = np.zeros(5)
        n = 0
        t_end = time.perf_counter() + 1.5
        while time.perf_counter() < t_end:
            st = self._read_states()
            q = self.measured_q(st)
            bx = self.base.read()
            loads = self.measured_loads(st)
            tau_g = self.kin.gravity_torque(q, bx)[:5]
            tau_sum += loads - tau_g
            n += 1
            time.sleep(0.05)
        bias = tau_sum / max(1, n)
        self.fest.tau_bias = bias
        p = write_config(self.specs, bias, self.args.config)
        print('tau_bias =', np.round(bias, 4).tolist())
        print('written to', p)

    def hold(self):
        print('hold mode: torque on, telemetry only. Ctrl+C to exit.')
        t0 = time.perf_counter()
        try:
            while True:
                self.bus.tick(0.1)
                st = self._read_states()
                q_act = self.measured_q(st)
                loads = self.measured_loads(st)
                bx_act = self.base.read()
                if self.dry:
                    self.wall.step(q_act, bx_act, 0.1)
                f_est = self.fest.update(q_act, bx_act, loads)
                tmin = min(st[s.servo_id]['temp'] for s in self.specs)
                tmax = max(st[s.servo_id]['temp'] for s in self.specs)
                vmin = min(st[s.servo_id]['voltage'] for s in self.specs)
                print(f'  f_est {f_est:+6.2f} N   q {np.round(q_act, 2).tolist()}   '
                      f'temp {tmin}-{tmax} C   {vmin:.1f} V')
                time.sleep(max(0.0, 0.1 - (time.perf_counter() - t0) % 0.1))
        except KeyboardInterrupt:
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry', action='store_true', help='force dry mode (no hardware)')
    ap.add_argument('--port', default=None, help='serial port, e.g. COM3')
    ap.add_argument('--enable', action='store_true',
                    help='required for real hardware: enables servo torque')
    ap.add_argument('--mode', choices=['controlled', 'baseline'], default='controlled')
    ap.add_argument('--passes', type=int, default=3)
    ap.add_argument('--hz', type=float, default=50.0)
    ap.add_argument('--fast', action='store_true', help='dry: run unpaced')
    ap.add_argument('--no-base', action='store_true', help='rail locked (bench tests)')
    ap.add_argument('--base0', type=float, default=0.0)
    ap.add_argument('--servo-speed', type=int, default=800, help='ticks/s move limit')
    ap.add_argument('--servo-acc', type=int, default=50)
    ap.add_argument('--max-force', type=float, default=20.0,
                    help='backoff guard (sim seam peak is 33 N; use ~15 for first real tests)')
    ap.add_argument('--max-temp', type=float, default=70.0)
    ap.add_argument('--min-volt', type=float, default=9.5)
    ap.add_argument('--estop', default=None, help='path to an ESTOP file (polled)')
    ap.add_argument('--hold', action='store_true', help='telemetry only, no motion')
    ap.add_argument('--calib', action='store_true', help='measure load bias off-wall')
    ap.add_argument('--config', default=None, help='path to robot.json calibration')
    args = ap.parse_args()
    if args.port and not args.enable:
        print('refusing to enable torque on hardware without --enable')
        return 2
    runner = HostRunner(args)
    try:
        if args.calib:
            runner.calib()
            return 0
        if args.hold:
            runner.hold()
            return 0
        met, data = runner.run()
        print(json.dumps(met, indent=2))
        tag = runner.save(met, data)
        print('log:', LOGS / f'{tag}.csv')
        return 0
    finally:
        runner.bus.torque_off_all(runner.specs)
        runner.bus.close()
        print('torque off, bus closed')


if __name__ == '__main__':
    sys.exit(main())
