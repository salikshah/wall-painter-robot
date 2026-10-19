"""Cross-validate hw/kinematics.py against the MuJoCo sim model.

Checks, on random configurations and the real sweep targets:
  1. FK pad position + pad axis vs MuJoCo 'padsite'
  2. gravity torque vs MuJoCo qfrc_bias (zero velocity)
  3. analytic Jacobian vs central finite differences
  4. warm-started IK convergence + tick time along the actual sweep
"""
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sim'))
sys.path.insert(0, str(ROOT / 'hw'))

import mujoco
import robot as sim_robot
import wall_geometry as wg
import kinematics as hwkin

AXIS = np.array([0.0, -1.0, wg.PRETILT])
AXIS = AXIS / np.linalg.norm(AXIS)
rng = np.random.default_rng(0)

arm = sim_robot.Arm()
K = hwkin.Kinematics()

print('== FK vs MuJoCo (150 random configs, full joint range) ==')
wp = wa = 0.0
for _ in range(150):
    q = K.lo + (K.hi - K.lo) * rng.random(5)
    bx = rng.uniform(*hwkin.BASE_RANGE)
    arm.d.qpos[arm.qa] = q
    arm.d.qpos[arm.q_base] = bx
    mujoco.mj_forward(arm.m, arm.d)
    p_sim = arm.d.site_xpos[arm.sid].copy()
    R_sim = arm.d.site_xmat[arm.sid].reshape(3, 3)
    p_hw, R_hw, _, _, _, _, _ = K.fk(q, bx)
    wp = max(wp, np.linalg.norm(p_sim - p_hw) * 1e6)
    wa = max(wa, np.degrees(np.arccos(np.clip(np.dot(R_sim[:, 2], R_hw[:, 2]), -1, 1))))
print(f'  max pad pos err   {wp:8.3f} um')
print(f'  max pad axis err  {wa:8.5f} deg')
assert wp < 10.0 and wa < 1e-3, 'FK mismatch vs MuJoCo'

print('== gravity torque vs MuJoCo qfrc_bias (60 random configs) ==')
wg_max = 0.0
for _ in range(60):
    q = K.lo + (K.hi - K.lo) * rng.random(5)
    bx = rng.uniform(*hwkin.BASE_RANGE)
    arm.d.qpos[arm.qa] = q
    arm.d.qpos[arm.q_base] = bx
    arm.d.qvel[:] = 0.0
    mujoco.mj_forward(arm.m, arm.d)
    qfrc = arm.d.qfrc_bias[arm.va].copy()
    G = K.gravity_torque(q, bx)
    wg_max = max(wg_max, np.abs(G[:5] - qfrc).max())
print(f'  max |tau_grav err| {wg_max:8.4f} N.m')
print('  (residual is sim-only invisible proxy masses; build_model group 3)')
assert wg_max < 0.06, 'gravity mismatch vs MuJoCo'

print('== analytic Jacobian vs central differences (30 random configs) ==')
wj = 0.0
for _ in range(30):
    q = K.lo + 0.02 + (K.hi - K.lo - 0.04) * rng.random(5)
    bx = rng.uniform(hwkin.BASE_RANGE[0] + 0.02, hwkin.BASE_RANGE[1] - 0.02)
    Jp, Ja, _, _, _, _, _ = K.jac(q, bx)
    Jn_p = np.zeros((3, 6))
    Jn_a = np.zeros((3, 6))
    h = 1e-6
    for k in range(6):
        e = np.zeros(6)
        e[k] = h
        pp, Rp, _, _, _, _, _ = K.fk(q + e[:5], bx + e[5])
        pm, Rm, _, _, _, _, _ = K.fk(q - e[:5], bx - e[5])
        Jn_p[:, k] = (pp - pm) / (2 * h)
        Jn_a[:, k] = (Rp[:, 2] - Rm[:, 2]) / (2 * h)
    wj = max(wj, np.abs(Jp - Jn_p).max(), np.abs(Ja - Jn_a).max())
print(f'  max |J err| {wj:.3e}')
assert wj < 1e-5, 'analytic Jacobian mismatch'

print('== warm-started sweep (250 targets, mimics the control loop) ==')
K.prev_sol = None
K.track(np.array([wg.SWEEP_X[0], wg.wall_surface_y(wg.SWEEP_X[0], wg.PASS_Z[0]) - 0.001,
                  wg.PASS_Z[0]]), axis=AXIS, bx=wg.SWEEP_X[0])
wpe = wae = 0.0
times = []
for i in range(250):
    x = wg.SWEEP_X[0] + (wg.SWEEP_X[1] - wg.SWEEP_X[0]) * i / 249.0
    z = wg.PASS_Z[0]
    p = 0.0006 + 0.0004 * np.sin(i * 0.2)
    y = wg.wall_surface_y(x, z) - p
    t = time.perf_counter()
    q, b, pe, ae = K.track(np.array([x, y, z]), axis=AXIS, bx=x)
    times.append(time.perf_counter() - t)
    wpe = max(wpe, pe * 1000)
    wae = max(wae, ae)
times = np.array(times)
print(f'  max pos err {wpe:6.3f} mm   max axis err {wae:6.3f} deg')
print(f'  tick time p50 {np.median(times)*1000:5.2f} ms   p95 {np.percentile(times, 95)*1000:5.2f} ms'
      f'   max {times.max()*1000:5.1f} ms  (budget 20 ms at 50 Hz)')
assert wpe < 0.9, 'warm-started IK inaccurate'
assert np.median(times) < 0.010 and np.percentile(times, 95) < 0.025, 'IK too slow for 50 Hz'

print('== IK on sweep grid (15 x 3, cold start) ==')
wpe = wae = 0.0
for x in np.linspace(wg.SWEEP_X[0], wg.SWEEP_X[1], 15):
    for z in wg.PASS_Z:
        y = wg.wall_surface_y(x, z) - 0.001
        q, b, pe, ae = K.ik(np.array([x, y, z]), axis=AXIS, bx=x)
        wpe = max(wpe, pe * 1000)
        wae = max(wae, ae)
print(f'  max pos err {wpe:6.3f} mm   max axis err {wae:6.3f} deg')
assert wpe < 2.0 and wae < 3.0, 'IK fails on sweep grid'
print('ALL KINEMATICS CHECKS PASS')
