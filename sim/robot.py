"""Arm wrapper for the wall-sanding sim.

IK core adapted from moss-jev live/physics.py (Apache-2.0, Laurent Genoud): damped
least squares on the padsite, with an added constraint aligning the site z-axis to
a desired world direction (the wall normal condition).
"""
from pathlib import Path
import numpy as np
import mujoco

import build_model

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / 'sim/reports/model_wallfin.xml'
SEED_BANK = Path(__file__).resolve().parent / 'seed_bank.npz'
JOINTS = build_model.JOINTS
WALL_GEOMS = {'wall_a', 'wall_b', 'blob'}
MANUAL_SEEDS = [
    np.array([-1.574, 1.584, 0.811, 0.747, -0.776]),
    np.array([1.575, -1.74, 1.328, 0.412, -1.27]),
    np.array([-1.568, 1.586, 1.255, 0.304, -0.946]),
    np.array([-1.573, 0.464, 1.579, 1.103, 1.054]),
]


# Joint-centring weight for the IK. It cannot fix the folded pose: the task is
# exactly determined (5 joints, 6 constraints), so the fold is the unique solution
# and a weight strong enough to matter collapses the pad's axis alignment instead
# (measured: 2.6 deg -> 43 deg at 0.08). It is kept anyway, because at this weight
# it measurably helps the solver converge at the railed-back geometry -- ik pe max
# 0.80 mm vs 1.00 mm, and steady force p95 3.0 N vs 5.0 N. Do not raise it.
POSTURE_W = 0.004


def load_seed_bank():
    if SEED_BANK.exists():
        data = np.load(SEED_BANK)
        return data['q'], data['p']
    return None, None



class Arm:
    def __init__(self, model_path=None):
        if model_path is None:
            build_model.build()
            model_path = MODEL_PATH
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.d = mujoco.MjData(self.m)
        self.ikd = mujoco.MjData(self.m)
        self.jids = [self.m.joint(n).id for n in JOINTS]
        lo, hi = self.m.jnt_range[self.jids].T
        self.qmid = 0.5 * (lo + hi)
        self.qa = np.array([self.m.jnt_qposadr[j] for j in self.jids])
        self.va = np.array([self.m.jnt_dofadr[j] for j in self.jids])
        self.q_base = int(self.m.jnt_qposadr[self.m.joint('base_x').id])
        self.v_base = int(self.m.jnt_dofadr[self.m.joint('base_x').id])
        self.sid = self.m.site('padsite').id
        self.pad_gid = self.m.geom('pad').id
        self.wall_gids = {self.m.geom(n).id for n in WALL_GEOMS}
        self.proxy_gids = {g for g in range(self.m.ngeom)
                           if (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, g) or '')
                           .endswith('_proxy')}
        self.prev_sol = None
        self.ctrl_ids = {n: self.m.actuator(n).id for n in JOINTS + ['base_x']}
        self.bank_q, self.bank_p = load_seed_bank()

    def reset(self, q0, settle=200, ctrl=None, base=0.0):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:] = 0
        self.d.qpos[self.qa] = q0
        self.d.qpos[self.q_base] = base
        if ctrl is None:
            ctrl = q0
        for n, val in zip(JOINTS, ctrl):
            self.d.ctrl[self.ctrl_ids[n]] = val
        self.d.ctrl[self.ctrl_ids['base_x']] = base
        mujoco.mj_forward(self.m, self.d)
        for _ in range(settle):
            self.step()
        self.prev_sol = self.d.qpos[self.qa].copy()

    def set_ctrl(self, q, base=None):
        for n, val in zip(JOINTS, q):
            self.d.ctrl[self.ctrl_ids[n]] = val
        if base is not None:
            self.d.ctrl[self.ctrl_ids['base_x']] = base

    def set_base(self, value):
        self.d.ctrl[self.ctrl_ids['base_x']] = value

    def base(self):
        return float(self.d.qpos[self.q_base])

    def step(self):
        mujoco.mj_step(self.m, self.d)
        if not np.isfinite(self.d.qpos).all():
            raise RuntimeError('Non-finite physics state')

    def pad_pose(self):
        return (self.d.site_xpos[self.sid].copy(),
                self.d.site_xmat[self.sid].reshape(3, 3))

    def pad_force(self):
        f = 0.0
        n = 0
        w = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 == self.pad_gid and g2 in self.wall_gids) or \
               (g2 == self.pad_gid and g1 in self.wall_gids):
                mujoco.mj_contactForce(self.m, self.d, i, w)
                f += abs(w[0])
                n += 1
        return f, n

    def proxy_wall_contacts(self):
        n = 0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 in self.proxy_gids and g2 in self.wall_gids) or \
               (g2 in self.proxy_gids and g1 in self.wall_gids):
                n += 1
        return n

    def _score(self, q, target, axis):
        d, m = self.ikd, self.m
        d.qpos[:] = self.d.qpos
        d.qpos[self.qa] = q
        mujoco.mj_forward(m, d)
        pe = float(np.linalg.norm(target - d.site_xpos[self.sid]))
        a = d.site_xmat[self.sid].reshape(3, 3)[:, 2]
        ae = float(np.degrees(np.arccos(np.clip(np.dot(a, axis), -1, 1))))
        return pe, ae

    def _posture_rows(self, q, w):
        """Extra Jacobian rows pulling q toward the middle of its range."""
        return w * (self.qmid - q), w * np.eye(len(q))

    def _descend(self, q0, target, axis, iters, aw, pw=None):
        if pw is None:
            pw = POSTURE_W
        d, m = self.ikd, self.m
        lo, hi = m.jnt_range[self.jids].T
        q = q0.copy()
        best = (q.copy(),) + self._score(q, target, axis)
        for it in range(iters):
            d.qpos[:] = self.d.qpos
            d.qpos[self.qa] = q
            mujoco.mj_forward(m, d)
            pos_err = target - d.site_xpos[self.sid]
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, jp, jr, self.sid)
            a = d.site_xmat[self.sid].reshape(3, 3)[:, 2]
            ja = np.cross(jr[:, self.va].T, a).T
            err = np.r_[pos_err, aw * (axis - a)]
            J = np.vstack([jp[:, self.va], aw * ja])
            if pw > 0.0:
                # homotopy: steer into an extended basin early, then hand over
                # completely so the task can converge.
                w = pw * (1.0 - it / max(1, iters))
                if w > 0.0:
                    perr, pJ = self._posture_rows(q, w)
                    err = np.r_[err, perr]
                    J = np.vstack([J, pJ])
            if np.linalg.norm(pos_err) < 0.0008 and (axis - a) @ (axis - a) < 0.0003:
                break
            delta = J.T @ np.linalg.solve(J @ J.T + np.eye(J.shape[0]) * 1.5e-5, err)
            q = np.clip(q + np.clip(delta, -.12, .12), lo + .005, hi - .005)
        pe, ae = self._score(q, target, axis)
        if pe + 0.08 * np.radians(ae) < best[1] + 0.08 * np.radians(best[2]):
            best = (q.copy(), pe, ae)
        return best

    def _polish(self, q0, target, axis, iters=25, aw=0.03, clamp=0.02):
        d, m = self.ikd, self.m
        lo, hi = m.jnt_range[self.jids].T
        q = q0.copy()
        best = (q.copy(),) + self._score(q, target, axis)
        for _ in range(iters):
            d.qpos[:] = self.d.qpos
            d.qpos[self.qa] = q
            mujoco.mj_forward(m, d)
            pos_err = target - d.site_xpos[self.sid]
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, jp, jr, self.sid)
            a = d.site_xmat[self.sid].reshape(3, 3)[:, 2]
            if np.linalg.norm(pos_err) < 0.001 and (axis - a) @ (axis - a) < 0.01:
                break
            ja = np.cross(jr[:, self.va].T, a).T
            err = np.r_[pos_err, aw * (axis - a)]
            J = np.vstack([jp[:, self.va], aw * ja])
            delta = J.T @ np.linalg.solve(J @ J.T + np.eye(J.shape[0]) * 1.5e-5, err)
            q = np.clip(q + np.clip(delta, -clamp, clamp), lo + .005, hi - .005)
        pe, ae = self._score(q, target, axis)
        if pe + 0.08 * np.radians(ae) < best[1] + 0.08 * np.radians(best[2]):
            best = (q.copy(), pe, ae)
        return best

    def ik(self, target, axis=(0.0, -1.0, 0.0), seed=None, iters=85, tol=0.0002):
        target = np.asarray(target, dtype=float)
        axis = np.asarray(axis, dtype=float)
        seeds = []
        if seed is not None:
            seeds.append(np.asarray(seed, dtype=float).copy())
        else:
            if self.prev_sol is not None:
                seeds.append(self.prev_sol.copy())
            seeds.append(self.d.qpos[self.qa].copy())
        seeds.append(np.zeros(5))
        if self.bank_p is not None:
            dist2 = np.linalg.norm(self.bank_p - target, axis=1)
            for i in np.argsort(dist2)[:6]:
                seeds.append(self.bank_q[i].copy())
        seeds += [s.copy() for s in MANUAL_SEEDS]
        best = None
        for q0 in seeds:
            q, pe, ae = self._descend(q0, target, axis, iters, 0.01)
            q, pe, ae = self._descend(q, target, axis, iters, 0.03)
            if best is None or pe + 0.08 * np.radians(ae) < best[1] + 0.08 * np.radians(best[2]):
                best = (q, pe, ae)
            if pe < 0.003 and ae < 5.0:
                break
        if best[1] > 0.003 or best[2] > 5.0:
            rng = np.random.default_rng(42)
            for k in range(8):
                qj = best[0] + rng.normal(0.0, 0.06 if k < 5 else 0.12, size=len(best[0]))
                q, pe, ae = self._descend(qj, target, axis, iters, 0.01)
                q, pe, ae = self._polish(q, target, axis)
                if pe + 0.08 * np.radians(ae) < best[1] + 0.08 * np.radians(best[2]):
                    best = (q, pe, ae)
                if pe < 0.002 and ae < 4.0:
                    break
        q, pe, ae = self._polish(best[0], target, axis, iters=40)
        if pe + 0.08 * np.radians(ae) < best[1] + 0.08 * np.radians(best[2]):
            best = (q, pe, ae)
        if best[1] > 0.001:
            for q0 in (best[0], np.zeros(5)) + tuple(s.copy() for s in MANUAL_SEEDS):
                q, pe, ae = self._descend(q0, target, axis, 150, 0.005)
                if pe < best[1]:
                    best = (q, pe, ae)
                if best[1] < 0.0004:
                    break
        return best

    def track(self, target, axis=(0.0, -1.0, 0.0), max_err=0.006, max_axis_deg=12.0):
        q, pos_err, axis_err = self.ik(target, axis=axis)
        if pos_err < max_err:
            self.prev_sol = q.copy()
        self.set_ctrl(q)
        return pos_err, axis_err
