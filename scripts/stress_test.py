"""Random abrupt command changes: counts falls and tracking error.

Run:  python scripts/stress_test.py
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import mujoco
from go1_bezier import load_single
from go1_bezier import Go1BezierController


def stress(seed=0, T=60.0, hold=(0.5, 2.0), v_lim=((-0.4, 0.8), (-0.3, 0.3)), w_max=1.0, **kw):
    rng = np.random.default_rng(seed)
    m, d = load_single()
    c = Go1BezierController(m, d, **kw)
    for _ in range(250):
        c.step(); mujoco.mj_step(m, d)
    t, t_next, falls, errs, tilt = 0.0, 0.0, 0, [], 0.0
    while t < T:
        if t >= t_next:
            c.set_command(rng.uniform(*v_lim[0]), rng.uniform(*v_lim[1]),
                          rng.uniform(-w_max, w_max))
            t_next = t + rng.uniform(*hold)
        c.step(); mujoco.mj_step(m, d); t += m.opt.timestep
        errs.append(np.abs(c.cmd_target - c.body_velocity()))
        up = d.xmat[1].reshape(3, 3)[2, 2]                 # cos(tilt)
        tilt = max(tilt, np.degrees(np.arccos(np.clip(up, -1, 1))))
        if d.qpos[2] < 0.15 or up < 0.5:
            falls += 1
            mujoco.mj_resetDataKeyframe(m, d, 0); c.gait.reset(); c.v_int[:] = 0
    e = np.array(errs).mean(0)
    return falls, e, tilt


if __name__ == "__main__":
    for seed in range(3):
        f, e, tilt = stress(seed)
        print(f"seed {seed}: falls {f} | mean |err| vx {e[0]:.3f} vy {e[1]:.3f} wz {e[2]:.3f} | max tilt {tilt:.1f} deg")
