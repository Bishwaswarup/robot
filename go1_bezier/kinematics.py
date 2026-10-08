"""Go1 leg kinematics (analytic IK / FK).

Leg order everywhere in this package: FR, FL, RR, RL (same as the MuJoCo
Menagerie go1.xml actuator order).

Frames: body frame = trunk frame (x forward, y left, z up).
Foot positions passed to IK are relative to the trunk origin.
Joint convention matches Menagerie: q = [abduction, thigh, knee], all zero
= leg straight down; positive thigh swings the foot backward; knee < 0.
"""
import numpy as np

LEGS = ("FR", "FL", "RR", "RL")
HIP_OFFSETS = np.array([[0.1881, -0.04675, 0.0],
                        [0.1881, 0.04675, 0.0],
                        [-0.1881, -0.04675, 0.0],
                        [-0.1881, 0.04675, 0.0]])
L_ABD = np.array([-0.08, 0.08, -0.08, 0.08])   # signed thigh offset (y)
L_THIGH = 0.213
L_CALF = 0.213

# Joint limits from go1.xml
Q_MIN = np.array([-0.863, -0.686, -2.818])
Q_MAX = np.array([0.863, 4.501, -0.888])


def leg_ik(p_hip, leg):
    """Foot position relative to the hip (abduction) joint -> [q_abd, q_thigh, q_knee]."""
    x, y, z = p_hip
    l1 = L_ABD[leg]
    d2 = y * y + z * z
    h = np.sqrt(max(d2 - l1 * l1, 1e-9))           # leg length projected in sagittal plane
    q1 = np.arctan2(h * y + l1 * z, l1 * y - h * z)
    r2 = x * x + h * h
    c3 = (r2 - L_THIGH ** 2 - L_CALF ** 2) / (2 * L_THIGH * L_CALF)
    q3 = -np.arccos(np.clip(c3, -1.0, 1.0))
    a = L_THIGH + L_CALF * np.cos(q3)
    b = L_CALF * np.sin(q3)
    q2 = np.arctan2(-x, h) - np.arctan2(b, a)
    return np.clip(np.array([q1, q2, q3]), Q_MIN, Q_MAX)


def leg_fk(q, leg):
    """Inverse of leg_ik: joint angles -> foot position relative to hip joint."""
    q1, q2, q3 = q
    l1 = L_ABD[leg]
    xs = -(L_THIGH * np.sin(q2) + L_CALF * np.sin(q2 + q3))
    zs = -(L_THIGH * np.cos(q2) + L_CALF * np.cos(q2 + q3))   # = -h
    c, s = np.cos(q1), np.sin(q1)
    return np.array([xs, c * l1 - s * zs, s * l1 + c * zs])


def body_ik(feet_body):
    """feet_body: (4,3) foot positions in trunk frame -> (12,) joint targets.
    Vectorised version of leg_ik over the four legs."""
    p = feet_body - HIP_OFFSETS
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    l1 = L_ABD
    h = np.sqrt(np.maximum(y * y + z * z - l1 * l1, 1e-9))
    q1 = np.arctan2(h * y + l1 * z, l1 * y - h * z)
    c3 = (x * x + h * h - L_THIGH ** 2 - L_CALF ** 2) / (2 * L_THIGH * L_CALF)
    q3 = -np.arccos(np.clip(c3, -1.0, 1.0))
    q2 = np.arctan2(-x, h) - np.arctan2(L_CALF * np.sin(q3), L_THIGH + L_CALF * np.cos(q3))
    return np.clip(np.stack([q1, q2, q3], axis=1), Q_MIN, Q_MAX).ravel()
