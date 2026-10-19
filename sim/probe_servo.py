"""Stationary press test: acquire and hold on one spot; print force trace."""
import sys

import numpy as np

import control
import robot
import wall_geometry as wg

x = float(sys.argv[1]) if len(sys.argv) > 1 else -0.10
z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.38
tilt = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
hold_s = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0

axis = np.array([0.0, -1.0, tilt])
axis = axis / np.linalg.norm(axis)

arm = robot.Arm()
yface = wg.wall_surface_y(x, z)
arm.reset(np.zeros(5), base=x, settle=0)
q0, pe, ae = arm.ik(np.array([x, yface + 0.035, z]), axis=axis)
arm.reset(q0, base=x, settle=400)
adm = control.Admittance()
adm.reset(p=-0.03)

n = int((hold_s + 2.5) / 0.02)
print(f'x={x:+.2f} z={z:+.2f} pretilt={tilt:+.3f}  (creep first, then hold)')
log = []
for i in range(n):
    y_t = yface - adm.p
    arm.set_base(x)
    pe, ae = arm.track(np.array([x, y_t, z]), axis=axis)
    for _ in range(10):
        arm.step()
    f, ncon = arm.pad_force()
    adm.update(f, 0.02)
    pad = arm.pad_pose()[0]
    site_mat = arm.pad_pose()[1]
    ax_err = np.degrees(np.arccos(np.clip(site_mat[:, 2] @ np.array([0.0, -1.0, 0.0]), -1, 1)))
    log.append((i * 0.02, f, adm.p, (yface - pad[1]) * 1000, ax_err, pe * 1000))

print('    t     F_N     p_mm  depth_mm  realax_deg  pe_mm')
for t, f, p, d, a, pe in log:
    if abs(t / 0.2 - round(t / 0.2)) < 1e-6:
        print(f'{t:6.1f}  {f:6.2f}  {p*1000:+6.2f}  {d:+8.2f}  {a:8.2f}  {pe:6.2f}')
tail = np.array([f for t, f, *_ in log if t > 2.5 + hold_s - 2.0])
print(f'hold-mean={tail.mean():.2f}N  std={tail.std():.2f}N  max={tail.max():.2f}N')
