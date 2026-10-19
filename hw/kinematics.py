"""Pure-numpy kinematics for the SO-101 arm + sander toolhead (no MuJoCo).

Real-robot counterpart of sim/robot.py: same chain, same 'padsite' conventions,
same damped-least-squares IK policy. Parses assets/so101.urdf directly.

Chain: world -> base_link (rail slide base_x along +x) -> shoulder_pan ->
shoulder_lift -> elbow_flex -> wrist_flex -> wrist_roll -> gripper_link ->
(fixed) gripper_frame_link -> toolhead; pad face at the toolhead origin.

Frames follow sim/build_model.py: base_link at wall_geometry.ARM_BASE with zero
yaw, toolhead offset [-0.0079, 0.000218, TOOL_LEN] in gripper_frame_link.

Jacobians are analytic (all joints rotate about their local +z; base slides
along world +x), cross-checked against finite differences in
check_kinematics.py.
"""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sim'))
import wall_geometry as wg

URDF = ROOT / 'assets/so101.urdf'
SEED_BANK = ROOT / 'sim/seed_bank.npz'

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
BASE_POS = np.array(wg.ARM_BASE)
BASE_RANGE = (-0.14, 0.14)
TOOLHEAD_POS = np.array([-0.0079, 0.000218, 0.104])

MANUAL_SEEDS = [
    np.array([-1.574, 1.584, 0.811, 0.747, -0.776]),
    np.array([1.575, -1.74, 1.328, 0.412, -1.27]),
    np.array([-1.568, 1.586, 1.255, 0.304, -0.946]),
    np.array([-1.573, 0.464, 1.579, 1.103, 1.054]),
]

EXTRA_COMS = [
    ('base_link', np.array([0.0, 0.0, -0.083]), 0.8),
    ('gripper_link', np.array([0.0202, 0.0188, -0.0234]), 0.012),
    ('toolhead', np.array([0.0, 0.0, -0.15]), 0.10),
    ('toolhead', np.array([0.0, 0.0, -0.0204]), 0.05),
    ('toolhead', np.array([0.0, 0.0, -0.0624]), 0.12),
]


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_R(axis, q):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def _parse_chain():
    root = ET.parse(URDF).getroot()
    joints = {}
    for j in root.findall('joint'):
        if j.find('parent') is None:
            continue
        joints[j.get('name')] = j
    links = {l.get('name'): l for l in root.findall('link')}
    chain = []
    link = 'base_link'
    seen = set()
    while link != 'gripper_frame_link':
        children = [j for j in joints.values() if j.find('parent').get('link') == link]
        nxt = [j for j in children if j.find('child').get('link') != 'moving_jaw_so101_v1_link']
        if len(nxt) != 1:
            raise RuntimeError(f'ambiguous chain at {link}')
        j = nxt[0]
        o = j.find('origin')
        xyz = np.array([float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()])
        rpy = [float(v) for v in (o.get('rpy') if o is not None else '0 0 0').split()]
        a = j.find('axis')
        axis = np.array([float(v) for v in (a.get('xyz') if a is not None else '0 0 1').split()])
        lim = j.find('limit')
        lo = float(lim.get('lower')) if lim is not None else -math.pi
        hi = float(lim.get('upper')) if lim is not None else math.pi
        chain.append({'name': j.get('name'), 'type': j.get('type'),
                      'child': j.find('child').get('link'),
                      'xyz': xyz, 'rpy': rpy, 'axis': axis, 'lo': lo, 'hi': hi})
        link = j.find('child').get('link')
        if link in seen:
            raise RuntimeError('joint cycle')
        seen.add(link)
    coms = []
    for lname in ['base_link', 'shoulder_link', 'upper_arm_link', 'lower_arm_link',
                  'wrist_link', 'gripper_link']:
        ine = links[lname].find('inertial')
        o = ine.find('origin')
        c = np.array([float(v) for v in o.get('xyz').split()])
        m = float(ine.find('mass').get('value'))
        coms.append((lname, c, m))
    for lname, c, m in EXTRA_COMS:
        coms.append((lname, c, m))
    return chain, coms


CHAIN, COMS = _parse_chain()
REVOLUTE = [j for j in CHAIN if j['type'] != 'fixed']
NQ = 5
NJ = NQ + 1


def _trans(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


CHAIN_T = [(_trans(j['xyz'], j['rpy']), j['axis'] if j['type'] != 'fixed' else None)
           for j in CHAIN]


def load_seed_bank():
    if SEED_BANK.exists():
        data = np.load(SEED_BANK)
        return data['q'], data['p']
    return None, None


class Kinematics:
    def __init__(self):
        self.lo = np.array([j['lo'] for j in REVOLUTE])
        self.hi = np.array([j['hi'] for j in REVOLUTE])
        self.axis_names = [j['name'] for j in REVOLUTE]
        self.prev_sol = None
        self.bank_q, self.bank_p = load_seed_bank()

    def fk(self, q, bx=0.0):
        T = np.eye(4)
        T[:3, 3] = BASE_POS + np.array([float(bx), 0.0, 0.0])
        link_T = {'base_link': T.copy()}
        link_nj = {'base_link': 0}
        axes = []
        qi = 0
        for (Tj, axis), j in zip(CHAIN_T, CHAIN):
            T = T @ Tj
            if axis is not None:
                axes.append((T[:3, 3].copy(), (T[:3, :3] @ axis).copy()))
                R = np.eye(4)
                R[:3, :3] = axis_R(axis, float(q[qi]))
                T = T @ R
                qi += 1
            link_T[j['child']] = T.copy()
            link_nj[j['child']] = qi
        link_nj['toolhead'] = qi
        gf = link_T['gripper_frame_link']
        T_tool = gf.copy()
        T_tool[:3, 3] = gf[:3, 3] + gf[:3, :3] @ TOOLHEAD_POS
        link_T['toolhead'] = T_tool
        coms = np.empty((len(COMS), 3))
        masses = np.empty(len(COMS))
        com_nj = np.empty(len(COMS), dtype=int)
        for i, (lname, c, m) in enumerate(COMS):
            TL = link_T[lname]
            coms[i] = TL[:3, 3] + TL[:3, :3] @ c
            masses[i] = m
            com_nj[i] = link_nj[lname]
        return T_tool[:3, 3], T_tool[:3, :3], coms, masses, link_T, axes, com_nj

    def jac(self, q, bx=0.0):
        q = np.asarray(q, dtype=float)
        p0, R0, c0, masses, _, axes, com_nj = self.fk(q, bx)
        a = R0[:, 2]
        Jp = np.zeros((3, NJ))
        Ja = np.zeros((3, NJ))
        for k, (pj, zj) in enumerate(axes):
            Jp[:, k] = np.cross(zj, p0 - pj)
            Ja[:, k] = np.cross(zj, a)
        Jp[:, NQ] = np.array([1.0, 0.0, 0.0])
        return Jp, Ja, p0, R0, c0, masses, axes

    def gravity_torque(self, q, bx=0.0):
        _, _, coms, masses, _, axes, com_nj = self.fk(q, bx)
        tau = np.zeros(NJ)
        for i, m in enumerate(masses):
            for j in range(com_nj[i]):
                pj, zj = axes[j]
                d = coms[i] - pj
                tau[j] += m * 9.81 * (zj[0] * d[1] - zj[1] * d[0])
        return tau

    def _score(self, q, bx, target, axis):
        p, R, _, _, _, _, _ = self.fk(q, bx)
        pe = float(np.linalg.norm(target - p))
        a = R[:, 2]
        ae = float(np.degrees(np.arccos(np.clip(np.dot(a, axis), -1, 1))))
        return pe, ae

    def _step(self, q, bx, target, axis, aw, clamp, free_base=False):
        Jp, Ja, p0, R0, _, _, _ = self.jac(q, bx)
        err = np.r_[target - p0, aw * (axis - R0[:, 2])]
        nc = NJ if free_base else NQ
        J = np.vstack([Jp[:, :nc], aw * Ja[:, :nc]])
        delta = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 1.5e-5, err)
        qn = np.clip(q + np.clip(delta[:NQ], -clamp, clamp), self.lo + .005, self.hi - .005)
        bn = bx
        if free_base:
            bn = float(np.clip(bx + np.clip(delta[NQ], -clamp, clamp),
                               BASE_RANGE[0] + .005, BASE_RANGE[1] - .005))
        return qn, bn

    def _descend(self, q, bx, target, axis, iters, aw, free_base=False):
        pe, ae = self._score(q, bx, target, axis)
        best = (q.copy(), bx, pe, ae)
        for _ in range(iters):
            if pe < 0.0008 and ae < 0.99:
                break
            q, bx = self._step(q, bx, target, axis, aw, 0.12, free_base)
            pe, ae = self._score(q, bx, target, axis)
            if pe + 0.08 * math.radians(ae) < best[2] + 0.08 * math.radians(best[3]):
                best = (q.copy(), bx, pe, ae)
        return best

    def _polish(self, q, bx, target, axis, iters=25, aw=0.03, clamp=0.02):
        pe, ae = self._score(q, bx, target, axis)
        best = (q.copy(), bx, pe, ae)
        for _ in range(iters):
            if pe < 0.001 and ae < 5.72:
                break
            q, bx = self._step(q, bx, target, axis, aw, clamp)
            pe, ae = self._score(q, bx, target, axis)
            if pe + 0.08 * math.radians(ae) < best[2] + 0.08 * math.radians(best[3]):
                best = (q.copy(), bx, pe, ae)
        return best

    def ik(self, target, axis=(0.0, -1.0, 0.0), seed=None, bx=0.0, iters=85):
        target = np.asarray(target, dtype=float)
        axis = np.asarray(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        seeds = []
        if seed is not None:
            seeds.append(np.asarray(seed, dtype=float).copy())
        elif self.prev_sol is not None:
            seeds.append(self.prev_sol.copy())
        seeds.append(np.zeros(NQ))
        if self.bank_p is not None:
            dist2 = np.linalg.norm(self.bank_p - target, axis=1)
            for i in np.argsort(dist2)[:6]:
                seeds.append(self.bank_q[i].copy())
        seeds += [s.copy() for s in MANUAL_SEEDS]
        best = None
        for q0 in seeds:
            q, b, pe, ae = self._descend(q0, bx, target, axis, iters, 0.01)
            q, b, pe, ae = self._descend(q, b, target, axis, iters, 0.03)
            if best is None or pe + 0.08 * math.radians(ae) < best[2] + 0.08 * math.radians(best[3]):
                best = (q, b, pe, ae)
            if pe < 0.003 and ae < 5.0:
                break
        q, b, pe, ae = self._polish(best[0], best[1], target, axis, iters=40)
        if pe + 0.08 * math.radians(ae) < best[2] + 0.08 * math.radians(best[3]):
            best = (q, b, pe, ae)
        if best[2] > 0.001:
            for q0 in (best[0], np.zeros(NQ)) + tuple(s.copy() for s in MANUAL_SEEDS):
                q, b, pe, ae = self._descend(q0, best[1], target, axis, 150, 0.005)
                if pe < best[2]:
                    best = (q, b, pe, ae)
                if best[2] < 0.0004:
                    break
        return best

    def track(self, target, axis=(0.0, -1.0, 0.0), bx=0.0, max_err=0.006):
        q, b, pe, ae = self.ik(target, axis=axis, bx=bx)
        if pe < max_err:
            self.prev_sol = q.copy()
        return q, b, pe, ae
