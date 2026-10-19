"""Admittance force control + coverage tracking for the wall-sanding demo."""
import numpy as np

import wall_geometry as wg


class Admittance:
    """Depth admittance on measured pad force.

    Two modes: creep toward the wall while airborne, admittance when touching.
    p > 0 presses the commanded pad face p meters past the local surface.
    """

    # k_a is the depth-per-newton gain. 1.2e-4 was tuned for the old rail
    # position; with the rail set back the arm is less stiff at the wall and the
    # steady error opened up (p95 5.0 N against a 4.5 N gate). 3.0e-4 closes it
    # to 3.0 N and lifts contact duty from 0.949 to 0.978. 6.0e-4 overshoots and
    # the p95 returns to 5.0 N, so this is the sweet spot, not a monotonic one.
    def __init__(self, f_des=wg.F_DES, k_a=3.0e-4, k_i=4.0e-4,
                 tau_f=0.05, rate_in=0.015, rate_out=0.035,
                 p_min=-0.03, p_max=0.010, f_creep=0.8):
        self.f_des = f_des
        self.k_a = k_a
        self.k_i = k_i
        self.tau_f = tau_f
        self.rate_in = rate_in
        self.rate_out = rate_out
        self.p_min = p_min
        self.p_max = p_max
        self.f_creep = f_creep
        self.reset()

    def reset(self, p=None):
        self.f_f = 0.0
        self.it = 0.0
        self.p = self.p_min if p is None else p

    def update(self, f_meas, dt, healthy=True):
        a = dt / (self.tau_f + dt)
        self.f_f += a * (f_meas - self.f_f)
        if not healthy:
            p_t = self.p - 0.002 * dt
        elif self.f_f < self.f_creep:
            p_t = self.p + self.rate_in * dt
        else:
            e = self.f_f - self.f_des
            u = -self.k_a * e
            if abs(e) < 6.0:
                self.it += -self.k_i * e * dt
                self.it = float(np.clip(self.it, -0.002, 0.002))
            p_t = u + self.it
        dp = float(np.clip(p_t - self.p, -self.rate_out * dt, self.rate_in * dt))
        self.p = float(np.clip(self.p + dp, self.p_min, self.p_max))
        return self.p


class Coverage:
    def __init__(self, rect=wg.COVER_RECT, cell=wg.COVER_CELL, radius=wg.COVER_R_EFF):
        self.x0, self.x1, self.z0, self.z1 = rect
        self.cell = cell
        self.radius = radius
        self.nx = int(round((self.x1 - self.x0) / cell)) + 1
        self.nz = int(round((self.z1 - self.z0) / cell)) + 1
        self.count = np.zeros((self.nz, self.nx), dtype=np.int32)
        self.cov_time = np.zeros((self.nz, self.nx))
        self.trace = []

    def add(self, x, z, dt, in_contact):
        self.trace.append((x, z, in_contact))
        if not in_contact:
            return
        ix0 = max(0, int((x - self.radius - self.x0) / self.cell))
        ix1 = min(self.nx - 1, int((x + self.radius - self.x0) / self.cell) + 1)
        iz0 = max(0, int((z - self.radius - self.z0) / self.cell))
        iz1 = min(self.nz - 1, int((z + self.radius - self.z0) / self.cell) + 1)
        for iz in range(iz0, iz1 + 1):
            cz = self.z0 + iz * self.cell
            for ix in range(ix0, ix1 + 1):
                cx = self.x0 + ix * self.cell
                if (cx - x) ** 2 + (cz - z) ** 2 <= self.radius ** 2:
                    self.cov_time[iz, ix] += dt

    def fraction(self, min_time=0.02):
        return float((self.cov_time >= min_time).mean())
