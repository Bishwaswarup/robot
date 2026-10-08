"""Bezier trot gait generator with an explicit gait phase.

Swing: 12-point Bezier curve. Control-point pattern from spot_mini_mini
(OpenQuadruped, MIT License, (c) 2020 Maurice Rahme), which follows the MIT
Cheetah swing trajectory (Hyun et al., IJRR 2014). Differences from the original:
  * one global phase clock phi in [0,1) with fixed per-leg offsets, readable
    at any time (needed by the swing-level, gait-aware controller);
  * swing starts at the actual lift-off position and ends at the touchdown
    target for the *current* command, so velocity changes never make the foot
    target jump;
  * stance feet are integrated at -v_leg (stationary on ground when the body
    tracks the command), with yaw handled as v_leg = v + w x r_foot.
"""
import numpy as np
from math import comb

# Normalised along-step pattern of the 12 swing control points
# (spot_mini_mini uses -L, -1.4L, -1.5L x3, 0 x3, 1.5L x2, 1.4L, L; mapped to [0,1]).
_S = (np.array([-1, -1.4, -1.5, -1.5, -1.5, 0, 0, 0, 1.5, 1.5, 1.4, 1.0]) + 1.0) / 2.0
_Z = np.array([0, 0, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0, 0])
_N = len(_S) - 1
_BINOM = np.array([comb(_N, k) for k in range(_N + 1)], dtype=float)


def bernstein(t):
    k = np.arange(_N + 1)
    return _BINOM * t ** k * (1 - t) ** (_N - k)


class BezierTrotGait:
    """Phase-clocked trot. Leg order FR, FL, RR, RL."""

    def __init__(self, nominal_feet, period=0.36, duty=0.5, clearance=0.07,
                 offsets=(0.0, 0.5, 0.5, 0.0), max_step=0.12):
        self.nominal = np.asarray(nominal_feet, dtype=float)   # (4,3) body frame
        self.period = period
        self.duty = duty
        self.clearance = clearance
        self.offsets = np.asarray(offsets, dtype=float)
        self.max_step = max_step          # max half-stride (m), workspace guard
        self.reset()

    def reset(self):
        self.phi = 0.0
        self.disp = np.zeros((4, 3))      # foot displacement from nominal
        self.liftoff = np.zeros((4, 3))
        self.in_swing = np.zeros(4, dtype=bool)

    # ---- phase access -------------------------------------------------
    @property
    def T_stance(self):
        return self.duty * self.period

    @property
    def T_swing(self):
        return (1 - self.duty) * self.period

    def leg_phase(self):
        """Per-leg phase in [0,1): [0,duty) stance, [duty,1) swing."""
        return (self.phi + self.offsets) % 1.0

    def contact_schedule(self):
        return self.leg_phase() < self.duty

    # ---- main update --------------------------------------------------
    def leg_velocities(self, v_cmd, w_cmd):
        """Planar velocity each foot must sweep in stance: v + w x r."""
        r = self.nominal[:, :2]
        vx = v_cmd[0] - w_cmd * r[:, 1]
        vy = v_cmd[1] + w_cmd * r[:, 0]
        return np.stack([vx, vy], axis=1)

    def step(self, dt, v_cmd, w_cmd, stand=False, v_meas=None, w_meas=None, k_raibert=0.0):
        """Advance the clock and return (4,3) foot targets in body frame."""
        v_leg = self.leg_velocities(np.asarray(v_cmd, float), float(w_cmd))
        # Raibert foot-placement feedback: step further in the direction the
        # body is moving faster than commanded (balance, not speed tracking)
        if v_meas is not None and k_raibert > 0:
            dv = self.leg_velocities(np.asarray(v_meas, float), float(w_meas)) - v_leg
        else:
            dv = np.zeros((4, 2))
        if stand:
            # bring feet back to nominal and freeze clock
            self.disp *= 0.9
            self.in_swing[:] = False
            return self.nominal + self.disp

        self.phi = (self.phi + dt / self.period) % 1.0
        ph = self.leg_phase()
        swing = ph >= self.duty
        new_swing = swing & ~self.in_swing
        self.liftoff[new_swing] = self.disp[new_swing]
        self.in_swing = swing

        st = ~swing                                   # stance: sweep backward
        self.disp[st, :2] -= v_leg[st] * dt
        self.disp[st, 2] = 0.0
        if swing.any():                               # swing: Bezier to touchdown target
            s = (ph[swing] - self.duty) / (1 - self.duty)
            B = np.stack([bernstein(si) for si in s])             # (n,12)
            target = np.clip(v_leg[swing] * self.T_stance / 2 + k_raibert * dv[swing],
                             -self.max_step, self.max_step)
            start = self.liftoff[swing, :2]
            frac = B @ _S                                          # (n,)
            self.disp[swing, :2] = start + frac[:, None] * (target - start)
            self.disp[swing, 2] = self.clearance * (B @ _Z)
        lim = 1.6 * self.max_step
        np.clip(self.disp[:, :2], -lim, lim, out=self.disp[:, :2])
        return self.nominal + self.disp
