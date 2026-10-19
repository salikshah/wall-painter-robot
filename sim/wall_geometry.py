"""Scene geometry constants and analytic wall surface for the wall-sanding demo.

World frame: origin on the floor under the arm mount. +x runs right along the
wall, -y points from the robot toward the wall, +z is up. The wall surface
faces +y (toward the robot).

The wall is two thick drywall panels facing the robot:
  panel A  x in [-0.30, 0.00]   flat   surface y = WALL_Y
  panel B  x in [ 0.00, 0.31]   recessed by RECESS at the seam, then tilting
           AWAY from the robot by TILT as x grows; the panel body extends
           behind its face plane, away from the robot.
A screw-head blob on panel A (2.5 mm proud) is the second irregularity.
"""
import math

WALL_Y = -0.246
WALL_Z = (0.27, 0.47)
WALL_THICK = 0.05
PANEL_A_X = (-0.30, 0.00)
PANEL_B_X = (0.00, 0.31)
SEAM_X = 0.0
RECESS = 0.004
TILT = math.radians(1.5)
SEAM_Y = WALL_Y - RECESS

BLOB_X = -0.075
BLOB_Z = 0.335
BLOB_HALF = (0.012, 0.00125, 0.012)
BLOB_PROUD = 0.0025

# Rail carriage position. y is the set-back from the wall face (WALL_Y = -0.246):
# 0.15 m puts the arm 0.40 m from the wall. That is the working distance the arm
# wants -- at y = 0 the shoulder sits only ~0.29 m from the pad, about half its
# ~0.58 m of chain, so the IK is forced to fold the shoulder and elbow against
# their limits (shoulder_lift 95%, elbow_flex 90% of range). A joint-centring
# term in the IK cannot fix that: the task is exactly determined, so the fold is
# the unique solution. Moving the rail back lets it work extended (shoulder 67%,
# elbow 75%) at the same 0.00 mm / 0.96 deg accuracy. Note y = 0.08 is a dead
# zone -- unreachable, duty 0.87, peak 57 N.
ARM_BASE = (-0.04, 0.15, 0.308)
PAD_R = 0.05
PAD_HL = 0.0072
PAD_DOME_R = 0.15
PAD_FACE_OFF = 0.0
TOOL_LEN = 0.104
PAD_X_REL = 0.04

PASS_Z = (0.34, 0.37, 0.40)
SWEEP_X = (-0.14, 0.14)
SWEEP_V = 0.035
PRETILT = 0.02
F_DES = 5.0

COVER_RECT = (SWEEP_X[0], SWEEP_X[1], PASS_Z[0], PASS_Z[-1])
COVER_CELL = 0.01
COVER_R_EFF = 0.045
F_CONTACT_MIN = 1.5


def wall_surface_y(x, z):
    if x < SEAM_X:
        return WALL_Y
    return WALL_Y - RECESS - x * math.tan(TILT)


def panel_b_placement():
    """Place panel B so its front-face left corner meets (SEAM_X, SEAM_Y)."""
    th = -TILT
    c, s = math.cos(th), math.sin(th)
    hx, hy = (PANEL_B_X[1] - PANEL_B_X[0]) / 2.0, WALL_THICK / 2.0
    lx, ly = -hx, hy
    wx, wy = lx * c - ly * s, lx * s + ly * c
    return (SEAM_X - wx, SEAM_Y - wy, th)
