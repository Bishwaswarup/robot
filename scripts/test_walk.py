"""Headless check: does the Go1 track velocity commands without falling?

Run:  python scripts/test_walk.py
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import mujoco
from go1_bezier import load_single
from go1_bezier import Go1BezierController


def run(cmds, duration=4.0, verbose=True, **ctrl_kw):
    m, d = load_single()
    c = Go1BezierController(m, d, **ctrl_kw)
    for _ in range(int(0.5 / m.opt.timestep)):          # settle standing
        c.step(); mujoco.mj_step(m, d)
    results = []
    for (vx, vy, wz) in cmds:
        c.set_command(vx, vy, wz)
        vs, zs, rp = [], [], []
        n = int(duration / m.opt.timestep)
        for k in range(n):
            c.step(); mujoco.mj_step(m, d)
            if k > n // 3:
                vs.append(c.body_velocity()); zs.append(d.qpos[2])
                w, x, y, z = d.qpos[3:7]
                rp.append([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                           np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))])
        vs, rp = np.array(vs), np.array(rp)
        r = dict(cmd=(vx, vy, wz), v=vs.mean(0), z=np.mean(zs),
                 rp_max=np.abs(rp).max(0), fell=min(zs) < 0.15)
        results.append(r)
        if verbose:
            print(f"cmd {vx:+.2f} {vy:+.2f} {wz:+.2f} | meas {r['v'][0]:+.2f} {r['v'][1]:+.2f} "
                  f"{r['v'][2]:+.2f} | z {r['z']:.3f} | max roll/pitch "
                  f"{np.degrees(r['rp_max'][0]):4.1f}/{np.degrees(r['rp_max'][1]):4.1f} deg"
                  f"{'  FELL' if r['fell'] else ''}")
    return results


CMDS = [(0, 0, 0), (0.3, 0, 0), (0.6, 0, 0), (-0.3, 0, 0),
        (0, 0.25, 0), (0, 0, 0.8), (0.4, 0, 0.5)]

if __name__ == "__main__":
    run(CMDS)
