"""Static body-pose control and the trick sequence (sit, paw waving, height range).

The four feet are planted at their current ground positions; the trunk pose
(x, y, z, roll, pitch) is moved in a frame fixed to the ground, and IK puts the
feet where they must be in the body frame. Individual legs can be lifted to a
hip-relative target and waved.

    seq = PoseSequencer(model, data, TRICKS)
    seq.start()
    while not seq.done: seq.step(); mujoco.mj_step(model, data)
"""
import numpy as np
import mujoco
from .kinematics import HIP_OFFSETS, LEGS, body_ik

LEG_IDX = {n: i for i, n in enumerate(LEGS)}


def rot_rpy(roll, pitch, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch),
                              np.cos(yaw), np.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3 - 2 * s)


class Keyframe:
    """body: (x, y, z, roll, pitch) of the trunk in the ground frame (m, rad).
    lift: {leg: hip-relative foot target in body frame}.  wave: {leg: (amp_y, amp_z, Hz)}."""

    def __init__(self, duration, body, lift=None, wave=None, label=""):
        self.duration = duration
        self.body = np.asarray(body, float)
        self.lift = lift or {}
        self.wave = wave or {}
        self.label = label


class PoseSequencer:
    def __init__(self, model, data, keyframes, prefix=""):
        self.m, self.d = model, data
        self.keyframes = keyframes
        self.act_ids = np.array([model.actuator(f"{prefix}{leg}_{j}").id
                                 for leg in LEGS for j in ("hip", "thigh", "calf")])
        self.trunk = model.body(f"{prefix}trunk").id
        self.foot_sites = [model.site(f"{prefix}{leg}").id for leg in LEGS]
        self.done = True

    # ground frame G: origin on the floor below the trunk at start(), yaw-aligned
    def start(self):
        mujoco.mj_forward(self.m, self.d)
        p = self.d.xpos[self.trunk].copy()
        R = self.d.xmat[self.trunk].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        self.G_origin = np.array([p[0], p[1], 0.0])
        self.G_R = rot_rpy(0, 0, yaw)
        feet_w = np.array([self.d.site_xpos[s] for s in self.foot_sites])
        self.feet_G = (feet_w - self.G_origin) @ self.G_R          # rows: R^T (p - o)
        self.body0 = np.array([0.0, 0.0, p[2], 0.0, 0.0])
        self.prev = Keyframe(0, self.body0)
        self.k, self.t_k, self.t = 0, 0.0, 0.0
        self.done = False
        self.label = self.keyframes[0].label

    def _feet_body(self, body, lift, lift_blend):
        x, y, z, roll, pitch = body
        R = rot_rpy(roll, pitch)
        ground = (self.feet_G - np.array([x, y, z])) @ R           # R^T (p - b)
        feet = ground.copy()
        for leg, target in lift.items():
            i = LEG_IDX[leg]
            up = HIP_OFFSETS[i] + np.asarray(target)
            feet[i] = (1 - lift_blend[leg]) * ground[i] + lift_blend[leg] * up
        return feet

    def step(self):
        if self.done:
            return
        dt = self.m.opt.timestep
        kf = self.keyframes[self.k]
        s = smoothstep(self.t_k / max(0.6 * kf.duration, 1e-6))   # move in first 60%
        body = (1 - s) * self.prev.body + s * kf.body
        # legs lifted in either keyframe: blend between ground and lift target
        lift, blend = {}, {}
        for leg in set(self.prev.lift) | set(kf.lift):
            a, b = leg in self.prev.lift, leg in kf.lift
            lift[leg] = kf.lift[leg] if b else self.prev.lift[leg]
            if a and b:
                lift[leg] = (1 - s) * np.asarray(self.prev.lift[leg]) + s * np.asarray(kf.lift[leg])
            blend[leg] = 1.0 if (a and b) else (s if b else 1 - s)
        feet = self._feet_body(body, lift, blend)
        for leg, (ay, az, hz) in kf.wave.items():
            i = LEG_IDX[leg]
            w = smoothstep((self.t_k - 0.6 * kf.duration) / 0.3)    # wave after the lift
            ph = 2 * np.pi * hz * self.t_k
            feet[i, 1] += w * ay * np.sin(ph)
            feet[i, 2] += w * az * np.sin(2 * ph) * 0.5 + w * az * 0.5
        self.d.ctrl[self.act_ids] = body_ik(feet)
        self.feet_target = feet
        self.t_k += dt
        self.t += dt
        if self.t_k >= kf.duration:
            self.prev = Keyframe(0, body, {l: lift[l] for l in kf.lift})
            self.prev.body = kf.body
            self.k += 1
            self.t_k = 0.0
            if self.k >= len(self.keyframes):
                self.done = True
            else:
                self.label = self.keyframes[self.k].label
