"""Pad-force estimation from servo loads + gravity compensation.

tau_meas (per joint, from Present_Load) = tau_gravity + J^T F_ext (+ dynamics,
friction). Subtracting the model gravity term and projecting the residual on
u = J^T n (n = contact normal = -pad axis) recovers the pad force along the
contact normal.

Validated against MuJoCo gravity in check_kinematics.py; on real hardware the
load sign per joint must be confirmed during bring-up (see README).
"""
import numpy as np


class ForceEstimator:
    def __init__(self, kin, tau_bias=None):
        self.kin = kin
        self.tau_bias = np.zeros(5) if tau_bias is None else np.asarray(tau_bias, float)
        self.last = {}

    def update(self, q, bx, tau_meas5):
        q = np.asarray(q, dtype=float)
        tau_meas5 = np.asarray(tau_meas5, dtype=float)
        tau_g = self.kin.gravity_torque(q, bx)[:5]
        tau_res = tau_meas5 - tau_g - self.tau_bias
        Jp, _, p, R, _, _, _ = self.kin.jac(q, bx)
        n = -R[:, 2]
        n = n / (np.linalg.norm(n) + 1e-12)
        u = Jp[:, :5].T @ n
        f = float(tau_res @ u / (float(u @ u) + 1e-12))
        self.last = {'f': f, 'tau_res': tau_res, 'tau_g': tau_g, 'u': u, 'n': n}
        return f

    def calibrate_bias(self, q, bx, tau_meas5):
        """Record residual torques while airborne (no contact) as bias."""
        tau_g = self.kin.gravity_torque(q, bx)[:5]
        self.tau_bias = np.asarray(tau_meas5, float) - tau_g
        return self.tau_bias
