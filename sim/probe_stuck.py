"""Reproduce the run's stuck state, then interrogate IK seeds live."""
import sys

import numpy as np

import control
import robot
import wall_geometry as wg

DT, DECIM = 0.002, 10
CDT = DT * DECIM
AXIS = np.array([0.0, -1.0, wg.PRETILT])
AXIS = AXIS / np.linalg.norm(AXIS)

ARM = robot.Arm()
adm = control.Admittance()
PZ = float(sys.argv[1]) if len(sys.argv) > 1 else wg.PASS_Z[1]
p0 = {'xs': -0.14, 'xe': 0.14, 'z': PZ}
print(f'probe_stuck z={PZ}')
arm = ARM
y0 = wg.wall_surface_y(p0['xs'], p0['z']) + 0.025
arm.reset(np.zeros(5), base=p0['xs'], settle=0)
q0, pe, ae = arm.ik(np.array([p0['xs'], y0, p0['z']]), axis=AXIS)
arm.reset(q0, base=p0['xs'], settle=400)
adm.reset(p=-0.025)

T_sweep = 0.28 / wg.SWEEP_V
n_ctrl = int((2.0 + 2.6 + T_sweep + 0.6) / CDT)
phase = 'move'
t_phase = 0.0
frozen = None
stats = []
for i in range(n_ctrl):
    if phase == 'move':
        t = t_phase
        if t >= 2.0:
            phase, t_phase = 'acquire', 0.0
            adm.reset(p=-0.025)
            continue
        u = t / 2.0
        x = p0['xs']
        y = y0
        z = p0['z']
    elif phase == 'acquire':
        t = t_phase
        if t >= 2.6:
            phase, t_phase = 'sweep', 0.0
            continue
        x, z = p0['xs'], p0['z']
        y = wg.wall_surface_y(x, z) - adm.p
    elif phase == 'sweep':
        t = t_phase
        if t >= T_sweep:
            phase, t_phase = 'retract', 0.0
            continue
        x = p0['xs'] + (p0['xe'] - p0['xs']) * min(1.0, t / T_sweep)
        z = p0['z']
        y = wg.wall_surface_y(x, z) - adm.p
    else:
        break
    arm.set_base(x)
    pe, ae = arm.track(np.array([x, y, z]), axis=AXIS)
    for _ in range(DECIM):
        arm.step()
    f, ncon = arm.pad_force()
    adm.update(f, CDT, healthy=pe < 0.004)
    if phase == 'sweep':
        stats.append((x, f, pe, ae, adm.p))
    if pe > 0.003 and frozen is None:
        frozen = (i, x, z, y, pe, ae, f)
        print(f'STUCK at tick {i} x={x:+.3f} y={y:+.4f} pe={pe*1000:.2f}mm '
              f'ae={ae}deg f={f:.1f} p={adm.p*1000:.2f}mm base={arm.base():+.3f}')
        break
    t_phase += CDT

if frozen is not None:
    i, x, z, y, pe, ae, f = frozen
    target = np.array([x, y, z])
    print('\nper-seed descend results for the failing target:')
    seeds = []
    if arm.prev_sol is not None:
        seeds.append(('prev_sol', arm.prev_sol.copy()))
    seeds.append(('cur_qpos', arm.d.qpos[arm.qa].copy()))
    if arm.bank_p is not None:
        dist2 = np.linalg.norm(arm.bank_p - target, axis=1)
        for k, b in enumerate(np.argsort(dist2)[:6]):
            seeds.append((f'bank{b}', arm.bank_q[b].copy()))
    for k, s in enumerate(robot.MANUAL_SEEDS):
        seeds.append((f'man{k}', s.copy()))
    for name, s in seeds:
        q, pe1, ae1 = arm._descend(s, target, AXIS, 85, 0.01)
        q, pe2, ae2 = arm._descend(q, target, AXIS, 85, 0.03)
        q, pe3, ae3 = arm._polish(q, target, AXIS, 40)
        cost = pe3 + 0.08 * np.radians(ae3)
        print(f'  {name:9s} pe={pe3*1000:7.2f}mm ae={ae3:6.2f} cost={cost:.5f}')
else:
    st = np.array(stats)
    force = st[:, 1]
    pe_mm = st[:, 2] * 1000
    ae = st[:, 3]
    duty = float((force >= wg.F_CONTACT_MIN).mean())
    print('no stuck tick; sweep summary:')
    print(f'  force mean={force.mean():.2f} max={force.max():.2f} duty={duty:.2f}')
    print(f'  pe mean={pe_mm.mean():.2f}mm max={pe_mm.max():.2f}mm '
          f'frac>3mm={float((pe_mm > 3).mean()):.2f}')
    print(f'  ae mean={ae.mean():.2f} max={ae.max():.2f}')
    print(f'  p mean={st[:, 4].mean()*1000:+.2f}mm min={st[:, 4].min()*1000:+.2f} '
          f'max={st[:, 4].max()*1000:+.2f}')
    xb = st[:, 0]
    print('  per 40mm bucket: x0  duty  f_mean  f_max  p_mean')
    for k in range(7):
        x0 = -0.14 + k * 0.04
        m = (xb >= x0) & (xb < x0 + 0.04)
        if not m.any():
            continue
        print(f'    {x0:+.2f}: duty={float((force[m] >= wg.F_CONTACT_MIN).mean()):.2f} '
              f'f_mean={force[m].mean():5.2f} f_max={force[m].max():6.2f} '
              f'p_mean={st[m, 4].mean()*1000:+6.2f}')
