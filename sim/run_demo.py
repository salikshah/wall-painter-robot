"""Run the wall-sanding demo: controlled (admittance) or baseline (position-only).

Controlled: the pad face tracks the local wall surface with a force loop holding
~5 N; baseline: fixed depth into the nominal flat plane, no force feedback.

Usage: python run_demo.py --mode controlled --passes 3 [--render] [--tag name]
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import mujoco

import control
import robot
import wall_geometry as wg

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / 'sim/reports'
DT = 0.002
CTRL_DECIM = 10
CTRL_DT = DT * CTRL_DECIM
BASELINE_Y = wg.WALL_Y - 0.003
STANDOFF = 0.025
AXIS = np.array([0.0, -1.0, wg.PRETILT])
AXIS = AXIS / np.linalg.norm(AXIS)
RES = {450: (800, 450), 720: (1280, 720), 1080: (1920, 1080), 1440: (2560, 1440)}
FPS = 1.0 / (DT * 10)

# the HUD is client-facing: name the phases in plain language, not the
# internal runner states.
PHASE_LABEL = {'move': 'positioning', 'acquire': 'seeking contact',
               'sweep': 'sanding', 'retract': 'lifting off', 'done': 'done'}


def quintic(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (10 + u * (-15 + 6 * u))


def plan_paint(m, cov):
    plan = []
    for g in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
        if not name.startswith('paint_'):
            continue
        px, pz = float(m.geom_pos[g][0]), float(m.geom_pos[g][2])
        fx = (px - cov.x0) / cov.cell
        fz = (pz - cov.z0) / cov.cell
        ix0 = int(np.clip(np.floor(fx), 0, cov.nx - 1))
        iz0 = int(np.clip(np.floor(fz), 0, cov.nz - 1))
        plan.append((g, iz0, min(iz0 + 1, cov.nz - 1),
                     ix0, min(ix0 + 1, cov.nx - 1),
                     float(np.clip(fx - ix0, 0.0, 1.0)),
                     float(np.clip(fz - iz0, 0.0, 1.0))))
    return plan


def update_paint(m, cov, plan, last, t_sat=0.55, a_max=0.92):
    ct = np.clip(cov.cov_time / t_sat, 0.0, 1.0)
    for g, iz0, iz1, ix0, ix1, wx, wz in plan:
        a = (ct[iz0, ix0] * (1 - wx) * (1 - wz) + ct[iz0, ix1] * wx * (1 - wz) +
             ct[iz1, ix0] * (1 - wx) * wz + ct[iz1, ix1] * wx * wz) * a_max
        if abs(a - last[g]) > 0.004:
            m.geom_rgba[g, 3] = a
            last[g] = a


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


def canonical_events(mode):
    """Cutaway timings read from the recorded canonical run (same seed)."""
    tag = 'ctl5' if mode == 'controlled' else 'base5'
    t_contact, t_seam, t_event = 4.5, 8.6, 12.0
    p = REPORTS / f'{tag}.npz'
    if p.exists():
        d = np.load(p)
        t, ph, f, pad = d['t'], d['phase'], d['force'], d['pad']
        sw = ph == 2
        idx = np.where(sw & (f >= wg.F_CONTACT_MIN))[0]
        if len(idx):
            t_contact = float(t[idx[0]])
        idx = np.where(sw & (pad[:, 0] > wg.SEAM_X))[0]
        if len(idx):
            t_seam = float(t[idx[0]])
        idx = np.where(sw & (t > t_contact + 1.2))[0]
        if len(idx):
            if mode == 'controlled':
                t_event = float(t[idx[int(np.argmax(np.abs(f[idx] - wg.F_DES)))]])
            else:
                drop = np.where((pad[idx, 0] > 0.03) & (f[idx] < 0.5))[0]
                if len(drop):
                    late = drop[t[idx[drop]] > t[idx[-1]] - 8.0]
                    t_event = float(t[idx[late[0] if len(late) else drop[-1]]])
                else:
                    t_event = float(t[idx[int(np.argmax(f[idx]))]])
    return t_contact, t_seam, t_event


def build_shots(mode, t_total):
    """Shot list for the full director pass: one wide take + three macros."""
    t_contact, t_seam, t_event = canonical_events(mode)
    return [
        {'name': 'main', 't0': 0.0, 't1': t_total, 'hud': True,
         'cam': {'az': 240.0, 'el': -9.0, 'd0': 0.88, 'd1': 0.82,
                 'look': (-0.02, -0.25, 0.365)}},
        {'name': 'macro_touch', 't0': max(0.0, t_contact - 1.75),
         't1': t_contact + 1.75, 'hud': False,
         'cam': {'az': 252.0, 'el': -7.0, 'd0': 0.42, 'd1': 0.40,
                 'look': ('pad', -0.005, 0.018, 0.008)}},
        {'name': 'macro_seam', 't0': max(0.0, t_seam - 1.5),
         't1': t_seam + 1.5, 'hud': False,
         'cam': {'az': 246.0, 'el': -12.0, 'd0': 0.46, 'd1': 0.44,
                 'look': ('pad', 0.0, 0.015, 0.005)}},
        {'name': 'macro_event', 't0': max(0.0, t_event - 1.5),
         't1': t_event + 1.5, 'hud': False,
         'cam': {'az': 232.0, 'el': -10.0, 'd0': 0.46, 'd1': 0.44,
                 'look': ('pad', 0.005, 0.015, 0.0)}},
    ]


class Runner:
    def __init__(self, mode, passes=4, render=False, tag=None, video=None, res=450,
                 shots='main'):
        self.mode = mode
        self.arm = robot.Arm()
        self.plan = build_passes(passes)
        self.adm = control.Admittance()
        self.cov = control.Coverage()
        self.log = {k: [] for k in
                    ['t', 'phase', 'cmd', 'pad', 'force', 'ncon', 'pe', 'ae', 'p', 'base']}
        self.bad_ik = 0
        self.render = render
        self.video = video
        self.frames_dir = None
        self.shots = []
        if render:
            import shutil
            out_tag = tag or f'run_{mode}'
            self.frames_dir = REPORTS / f'frames_{out_tag}'
            if self.frames_dir.exists():
                shutil.rmtree(self.frames_dir)
            self.frames_dir.mkdir(parents=True)
            w, h = RES.get(res, (800, 450))
            self.renderer = mujoco.Renderer(self.arm.m, width=w, height=h)
            self.scene_opt = mujoco.MjvOption()
            self.cam = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(self.arm.m, self.cam)
            if shots == 'full':
                total = sum(2.0 + 2.6 + abs(p['xe'] - p['xs']) / wg.SWEEP_V + 0.6
                            for p in self.plan) + 1.0
                self.shots = build_shots(mode, total)
            else:
                self.shots = [{'name': 'main', 't0': 0.0, 't1': 1e9, 'hud': True,
                               'cam': {'az': 240.0, 'el': -9.0, 'd0': 0.88,
                                       'd1': 0.88, 'look': (-0.02, -0.25, 0.365)}}]
            for sh in self.shots:
                (self.frames_dir / sh['name']).mkdir(parents=True, exist_ok=True)
                sh['count'] = 0
            (REPORTS / f'shots_{out_tag}.json').write_text(json.dumps(
                {'mode': mode, 'fps': FPS, 'res': res,
                 'shots': [{k: sh[k] for k in ('name', 't0', 't1', 'hud')}
                           for sh in self.shots]}, indent=2))
            self.paint_plan = plan_paint(self.arm.m, self.cov)
            self.paint_alpha = {g: 0.0 for g, *_ in self.paint_plan}
            self.frame_count = 0
        # start pose: retracted at first pass start
        p0 = self.plan[0]
        y0 = wg.wall_surface_y(p0['xs'], p0['z']) + STANDOFF + wg.PAD_FACE_OFF
        self.start = np.array([p0['xs'], y0, p0['z']])
        self.arm.reset(np.zeros(5), base=p0['xs'], settle=0)
        q0, pe, ae = self.arm.ik(self.start, axis=AXIS)
        self.arm.reset(q0, base=p0['xs'], settle=400)
        self.phase = 'move'
        self.phase_t0 = 0.0
        self.pass_idx = 0
        self.pose0 = self.arm.pad_pose()[0].copy()
        self.steps = 0
        self.ctrl_steps = 0

    def phase_target(self, t):
        p = self.plan[self.pass_idx]
        x0, y0, z0 = self.pose0
        if self.phase == 'move':
            T = 2.0
            u = quintic(t / T)
            x = x0 + (p['xs'] - x0) * u
            z = z0 + (p['z'] - z0) * u
            ytar = wg.wall_surface_y(p['xs'], p['z']) + STANDOFF + wg.PAD_FACE_OFF
            y = y0 + (ytar - y0) * u
            done = t >= T
            return x, y, z, done, False
        if self.phase == 'acquire':
            T = 2.6
            x, z = p['xs'], p['z']
            if self.mode == 'controlled':
                y = wg.wall_surface_y(x, z) - self.adm.p + wg.PAD_FACE_OFF
            else:
                y = BASELINE_Y + wg.PAD_FACE_OFF
            return x, y, z, t >= T, True
        if self.phase == 'sweep':
            T = abs(p['xe'] - p['xs']) / wg.SWEEP_V
            x = p['xs'] + (p['xe'] - p['xs']) * min(1.0, t / T)
            z = p['z']
            if self.mode == 'controlled':
                y = wg.wall_surface_y(x, z) - self.adm.p + wg.PAD_FACE_OFF
            else:
                y = BASELINE_Y + wg.PAD_FACE_OFF
            return x, y, z, t >= T, True
        if self.phase == 'retract':
            T = 0.6
            u = quintic(t / T)
            z = p['z']
            xt = x0
            x = xt
            yt = wg.wall_surface_y(xt, z) + STANDOFF + wg.PAD_FACE_OFF
            y = y0 + (yt - y0) * u
            return x, y, z, t >= T, False
        raise ValueError(self.phase)

    def advance(self):
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
        self.phase_t0 = self.arm.d.time
        self.pose0 = self.arm.pad_pose()[0].copy()

    def tick(self):
        t = self.arm.d.time - self.phase_t0
        x, y, z, done, in_sweep = self.phase_target(t)
        self.arm.set_base(x)
        pe, ae = self.arm.track(np.array([x, y, z]), axis=AXIS)
        if pe > 0.006 or ae > 12.0:
            self.bad_ik += 1
        f, ncon = self.arm.pad_force()
        pad = self.arm.pad_pose()[0]
        if self.phase in ('acquire', 'sweep') and self.mode == 'controlled':
            self.adm.update(f, CTRL_DT, healthy=(pe < 0.004) or (f < 0.8))
        phase_id = {'move': 0, 'acquire': 1, 'sweep': 2, 'retract': 3, 'done': 4}[self.phase]
        for k, v in [('t', self.arm.d.time), ('phase', phase_id),
                     ('cmd', (x, y, z)), ('pad', tuple(pad)), ('force', f),
                     ('ncon', ncon), ('pe', pe), ('ae', ae),
                     ('p', self.adm.p), ('base', self.arm.base())]:
            self.log[k].append(v)
        if self.phase in ('acquire', 'sweep'):
            self.cov.add(float(pad[0]), float(pad[2]), CTRL_DT, f >= wg.F_CONTACT_MIN)
        if done:
            self.advance()

    def run(self):
        wall0 = time.time()
        n_steps = int((sum(2.0 + 2.6 + abs(p['xe'] - p['xs']) / wg.SWEEP_V + 0.6
                          for p in self.plan) + 1.0) / DT)
        frame_every = 10
        while self.phase != 'done' and self.steps < n_steps:
            if self.steps % CTRL_DECIM == 0:
                if self.phase in ('move', 'retract') and self.ctrl_steps % 2 == 1:
                    pass
                else:
                    self.tick()
                self.ctrl_steps += 1
            self.arm.step()
            self.steps += 1
            if self.render and self.steps % frame_every == 0:
                update_paint(self.arm.m, self.cov, self.paint_plan, self.paint_alpha)
                t = self.arm.d.time
                for sh in self.shots:
                    if sh['t0'] <= t < sh['t1']:
                        self.render_frame(sh)
        wall = time.time() - wall0
        return self.summarize(wall)

    def apply_cam(self, sh, t):
        c = sh['cam']
        u = min(1.0, max(0.0, (t - sh['t0']) / max(1e-6, sh['t1'] - sh['t0'])))
        e = quintic(u)
        self.cam.distance = c['d0'] + (c['d1'] - c['d0']) * e
        self.cam.azimuth = c['az']
        self.cam.elevation = c['el']
        look = c['look']
        if look[0] == 'pad':
            pad = self.arm.pad_pose()[0]
            self.cam.lookat[:] = (pad[0] + look[1], pad[1] + look[2], pad[2] + look[3])
        else:
            self.cam.lookat[:] = look

    def render_frame(self, sh):
        from PIL import Image
        f, ncon = self.arm.pad_force()
        self.apply_cam(sh, self.arm.d.time)
        self.renderer.update_scene(self.arm.d, camera=self.cam,
                                   scene_option=self.scene_opt)
        img = Image.fromarray(self.renderer.render())
        if sh['hud']:
            img = make_hud(img, self, f)
        img.save(self.frames_dir / sh['name'] / f"f_{sh['count']:05d}.png")
        sh['count'] += 1
        self.frame_count += 1

    def summarize(self, wall):
        d = {k: np.array(v) if k in ('cmd', 'pad') else np.array(v, dtype=float)
             for k, v in self.log.items()}
        sweep = d['phase'] == 2
        f = d['force'][sweep]
        e = f - wg.F_DES
        pe = d['pe'][sweep]
        ae = d['ae'][sweep]
        pad = d['pad'][sweep]
        yface = np.array([wg.wall_surface_y(px, pz) for px, pz in pad[:, [0, 2]]])
        depth = yface - (pad[:, 1] - wg.PAD_FACE_OFF)
        met = {
            'mode': self.mode,
            'wall_clock_s': round(wall, 1),
            'sim_time_s': round(float(d['t'][-1]) if len(d['t']) else 0.0, 2),
            'ctrl_ticks': self.ctrl_steps,
            'ik_failures': self.bad_ik,
            'sweep_ticks': int(sweep.sum()),
            'force_mean': round(float(f.mean()), 3),
            'force_err_mean_abs': round(float(np.abs(e).mean()), 3),
            'force_err_p95_abs': round(float(np.percentile(np.abs(e), 95)), 3),
            'force_max': round(float(f.max()), 3),
            'contact_duty': round(float((f >= wg.F_CONTACT_MIN).mean()), 4),
            'dropout_frac': round(float((f < wg.F_CONTACT_MIN).mean()), 4),
            'ik_pos_err_mean_mm': round(float(np.nanmean(pe) * 1000), 2),
            'ik_pos_err_max_mm': round(float(np.nanmax(pe) * 1000), 2),
            'axis_err_mean_deg': round(float(np.nanmean(ae)), 2),
            'axis_err_max_deg': round(float(np.nanmax(ae)), 2),
            'press_depth_max_mm': round(float(depth.max()) * 1000, 2),
            'coverage_frac': round(self.cov.fraction(), 4),
            'proxy_wall_contacts': int(self.arm.proxy_wall_contacts()),
        }
        return met, d


def make_hud(img, runner, force):
    from PIL import ImageDraw, ImageFont
    dr = ImageDraw.Draw(img, 'RGBA')
    s = img.height / 720.0

    def P(v):
        return int(round(v * s))

    try:
        f_big = ImageFont.truetype('arial.ttf', P(22))
        f_med = ImageFont.truetype('arial.ttf', P(15))
    except OSError:
        f_big = f_med = ImageFont.load_default()

    def frame(x0, y0, x1, y1, radius, fill):
        try:
            dr.rounded_rectangle([P(x0), P(y0), P(x1), P(y1)], radius=P(radius),
                                 fill=fill)
        except AttributeError:
            dr.rectangle([P(x0), P(y0), P(x1), P(y1)], fill=fill)

    frame(10, 10, 320, 118, 10, (14, 18, 26, 205))
    f_txt = f'{force:4.1f} N' if force > 0.05 else ' --.- N'
    if force < 0.8:
        col = (255, 176, 64, 255)
    elif force <= 8.0:
        col = (140, 230, 160, 255)
    else:
        col = (255, 98, 82, 255)
    dr.text((P(22), P(16)), f_txt, fill=col, font=f_big)
    dr.text((P(152), P(25)), f'/ {wg.F_DES:.0f} N target',
            fill=(170, 178, 188, 255), font=f_med)
    bx0, bx1, by = 22, 306, 56
    frame(bx0, by, bx1, by + 10, 5, (255, 255, 255, 34))
    w = (bx1 - bx0) * min(force, 15.0) / 15.0
    tgt = (bx1 - bx0) * wg.F_DES / 15.0
    if w > 2:
        frame(bx0, by, bx0 + w, by + 10, 5, col)
    dr.line([P(bx0 + tgt), P(by - 3), P(bx0 + tgt), P(by + 13)],
            fill=(255, 255, 255, 210), width=max(1, P(2)))
    dr.text((P(22), P(74)),
            f'pass {runner.pass_idx + 1}/{len(runner.plan)}   '
            f'{PHASE_LABEL.get(runner.phase, runner.phase)}',
            fill=(205, 212, 222, 255), font=f_med)
    dr.text((P(200), P(74)), 'CONTROLLED' if runner.mode == 'controlled' else 'BASELINE',
            fill=(150, 220, 160, 255) if runner.mode == 'controlled' else (240, 170, 120, 255),
            font=f_med)
    dr.text((P(22), P(96)), f'strip sanded {runner.cov.fraction() * 100:4.1f} %',
            fill=(205, 212, 222, 255), font=f_med)
    if force > 9.0:
        frame(336, 10, 486, 40, 8, (185, 44, 32, 225))
        dr.text((P(346), P(17)), 'FORCE HIGH', fill=(255, 255, 255, 255), font=f_med)
    elif 0 < force < 0.8 and runner.phase in ('acquire', 'sweep'):
        frame(336, 10, 486, 40, 8, (215, 152, 32, 225))
        dr.text((P(346), P(17)), 'LOST CONTACT', fill=(30, 30, 30, 255), font=f_med)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['controlled', 'baseline'], default='controlled')
    ap.add_argument('--passes', type=int, default=3)
    ap.add_argument('--render', action='store_true')
    ap.add_argument('--res', type=int, choices=[450, 720, 1080, 1440], default=450)
    ap.add_argument('--tag', default=None)
    ap.add_argument('--save', action='store_true')
    ap.add_argument('--shots', choices=['main', 'full'], default='main')
    args = ap.parse_args()
    runner = Runner(args.mode, args.passes, render=args.render, tag=args.tag,
                    res=args.res, shots=args.shots)
    met, data = runner.run()
    print(json.dumps(met, indent=2))
    if args.save:
        tag = args.tag or f'run_{args.mode}_p{args.passes}'
        np.savez(REPORTS / f'{tag}.npz',
                 t=data['t'], phase=data['phase'], force=data['force'],
                 pad=data['pad'], cmd=data['cmd'], pe=data['pe'], ae=data['ae'],
                 p=data['p'], base=data['base'],
                 cov=data['phase'], cov_time=runner.cov.cov_time,
                 cov_count=runner.cov.count)
        h = hashlib.sha256()
        for k in ['t', 'force', 'pad', 'cmd']:
            h.update(np.round(data[k], 6).tobytes())
        (REPORTS / f'{tag}.sha256').write_text(h.hexdigest())
        print('saved', tag, 'hash', h.hexdigest()[:16])
    if args.render:
        tag = args.tag or f'run_{args.mode}'
        (REPORTS / f'shots_{tag}.json').write_text(json.dumps(
            {'mode': args.mode, 'fps': FPS, 'res': args.res,
             'shots': [{k: sh[k] for k in ('name', 't0', 't1', 'hud', 'count')}
                       for sh in runner.shots]}, indent=2))
        for sh in runner.shots:
            print(f"  {sh['name']}: {sh['count']} frames")
        print('frames:', runner.frame_count, 'in', runner.frames_dir)


if __name__ == '__main__':
    main()
