"""Local stability of the walking gait: Poincare (stride) map + Floquet multipliers.

The gait clock is time-driven, so sampling the state once per gait period T gives
a stroboscopic Poincare map  x_{k+1} = P(x_k).  A periodic gait is a fixed point
x* = P(x*).  Linearise numerically,  dx_{k+1} ~= J dx_k,  J = dP/dx at x*.
The eigenvalues of J are the Floquet multipliers:
    all |lambda| < 1  -> locally (orbitally) stable gait
    max |lambda|      -> how much of a perturbation survives one stride
                         (e.g. 0.5: half; ln(0.01)/ln(0.5) ~ 7 strides to decay 99%)
x, y and yaw are dropped from the state: shifting/rotating the robot on flat ground
gives the same gait, so those directions are neutral (multiplier exactly 1).

Run:  python scripts/floquet.py
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import copy
import numpy as np
import mujoco
from go1_bezier import load_single, Go1BezierController

CYCLIC = (0, 1, 5)          # free-joint dofs: x, y, yaw (neutral on flat ground)


class Sim:
    def __init__(self, cmd, **ctrl_kw):
        self.m, self.d = load_single()
        self.c = Go1BezierController(self.m, self.d, acc_max=None, **ctrl_kw)
        self.c.set_command(*cmd)
        self.T = self.c.gait.period
        self.n_period = int(round(self.T / self.m.opt.timestep))
        self.keep = np.array([i for i in range(self.m.nv) if i not in CYCLIC])

    def run(self, n):
        for _ in range(n):
            self.c.step(); mujoco.mj_step(self.m, self.d)

    def snapshot(self):
        st = np.empty(mujoco.mj_stateSize(self.m, mujoco.mjtState.mjSTATE_PHYSICS))
        mujoco.mj_getState(self.m, self.d, st, mujoco.mjtState.mjSTATE_PHYSICS)
        return st, copy.deepcopy(self.c.gait.__dict__), self.c.v_int.copy(), self.c._tick

    def restore(self, snap):
        st, g, vi, tick = snap
        mujoco.mj_setState(self.m, self.d, st, mujoco.mjtState.mjSTATE_PHYSICS)
        self.c.gait.__dict__.update(copy.deepcopy(g))
        self.c.v_int[:] = vi
        self.c._tick = tick
        mujoco.mj_forward(self.m, self.d)

    def coords(self, qpos_ref):
        """Reduced coordinates: position difference in the tangent space + velocity
        (+ PI integrator state, part of the closed loop)."""
        dq = np.empty(self.m.nv)
        mujoco.mj_differentiatePos(self.m, dq, 1.0, qpos_ref, self.d.qpos)
        extra = self.c.v_int.copy() if self.c.k_i > 0 else np.zeros(0)   # integrator is state only if used
        return np.concatenate([dq[self.keep], self.d.qvel.copy(), extra])

    def perturb(self, i, eps):
        nk = len(self.keep)
        if i < nk:
            dv = np.zeros(self.m.nv); dv[self.keep[i]] = eps
            mujoco.mj_integratePos(self.m, self.d.qpos, dv, 1.0)
        elif i < nk + self.m.nv:
            self.d.qvel[i - nk] += eps
        else:
            self.c.v_int[i - nk - self.m.nv] += eps
        mujoco.mj_forward(self.m, self.d)


def floquet(cmd, eps=1e-5, settle_periods=40, **ctrl_kw):
    s = Sim(cmd, **ctrl_kw)
    s.run(s.n_period * settle_periods)              # converge onto the limit cycle
    x0 = s.snapshot()
    q0 = s.d.qpos.copy()
    base0 = s.coords(q0)
    s.run(s.n_period)
    drift = np.linalg.norm(s.coords(q0) - base0)        # ~0 if we sit on the limit cycle
    s.restore(x0)
    s.run(s.n_period)
    f0 = s.coords(q0)                                # P(x*) (unperturbed)
    n = len(base0)
    J = np.zeros((n, n))
    for i in range(n):
        cols = []
        for sgn in (+1, -1):
            s.restore(x0)
            s.perturb(i, sgn * eps)
            s.run(s.n_period)
            cols.append(s.coords(q0))
        J[:, i] = (cols[0] - cols[1]) / (2 * eps)
    lam = np.linalg.eigvals(J)
    lam = lam[np.argsort(-np.abs(lam))]
    v = s.c.body_velocity()
    return lam, drift, v


if __name__ == "__main__":
    configs = {
        "open loop (no PI, no Raibert)": dict(k_i=0.0, k_raibert=0.0),
        "Raibert only":                  dict(k_i=0.0, k_raibert=0.1),
        "full (PI + Raibert, default)":  dict(),
    }
    for cmd in [(0.0, 0, 0), (0.3, 0, 0), (0.6, 0, 0), (-0.3, 0, 0), (0, 0.25, 0)]:
        print(f"\ncommand {cmd}")
        for name, kw in configs.items():
            lam, drift, v = floquet(cmd, **kw)
            k = np.log(0.01) / np.log(max(abs(lam[0]), 1e-9)) if abs(lam[0]) < 1 else np.inf
            print(f"  {name:32s} max|lambda| {abs(lam[0]):.3f}  next {abs(lam[1]):.3f} {abs(lam[2]):.3f}"
                  f" | {'STABLE' if abs(lam[0]) < 1 else 'UNSTABLE'}"
                  f" (99% decay ~{k:.0f} strides) | orbit drift {drift:.1e} | v {v.round(2)}")
