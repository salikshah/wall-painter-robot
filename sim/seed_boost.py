"""Enrich the seed bank with retracted (off-wall) poses on the current layout."""
import numpy as np
import mujoco

import robot
import wall_geometry as wg

AXIS = np.array([0.0, -1.0, wg.PRETILT])
AXIS = AXIS / np.linalg.norm(AXIS)

arm = robot.Arm()
data = np.load('seed_bank.npz')
bank_q, bank_p = data['q'].copy(), data['p'].copy()
print('existing bank', bank_q.shape)

new_q, new_p = [], []
for z in [0.30, 0.34, 0.37, 0.40, 0.44, 0.48]:
    yf = wg.wall_surface_y(0.0, z)
    for off in [-0.003, -0.001, 0.010, 0.025, 0.060]:
        arm.reset(np.zeros(5), base=0.0, settle=0)
        q, pe, ae = arm.ik(np.array([0.0, yf + off, z]), axis=AXIS)
        if pe < 0.002 and ae < 10.0:
            arm.ikd.qpos[:] = arm.d.qpos
            arm.ikd.qpos[arm.qa] = q
            mujoco.mj_forward(arm.m, arm.ikd)
            new_q.append(q)
            new_p.append(arm.ikd.site_xpos[arm.sid].copy())
            print(f'  z={z:.2f} off={off*1000:+5.1f}mm -> keep pe={pe*1000:.2f} ae={ae:.1f}')
        else:
            print(f'  z={z:.2f} off={off*1000:+5.1f}mm -> drop pe={pe*1000:.2f} ae={ae:.1f}')

if new_q:
    bank_q = np.vstack([bank_q, np.array(new_q)])
    bank_p = np.vstack([bank_p, np.array(new_p)])
np.savez('seed_bank.npz', q=bank_q, p=bank_p)
print('new bank size', len(bank_q))
