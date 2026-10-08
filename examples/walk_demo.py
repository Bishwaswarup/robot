"""Watch the Bezier-gait Go1(s) walk in the MuJoCo viewer.

macOS:   mjpython examples/walk_demo.py            (the viewer needs mjpython on Mac)
         mjpython examples/walk_demo.py --n 9      (a 3x3 team, all following the same script)
Linux/Windows: python examples/walk_demo.py

The robot runs a scripted sequence of velocity commands (printed in the
terminal). Close the window to stop.
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer
from go1_bezier import load_single, load_team
from go1_bezier import Go1BezierController

SCRIPT = [  # (duration s, vx, vy, wz, label)
    (2.0, 0.0, 0.0, 0.0, "stand / march in place"),
    (4.0, 0.4, 0.0, 0.0, "forward 0.4 m/s"),
    (3.0, 0.7, 0.0, 0.0, "forward 0.7 m/s"),
    (3.0, 0.3, 0.0, 0.8, "arc left"),
    (3.0, 0.0, 0.25, 0.0, "sidestep left"),
    (3.0, 0.0, -0.25, 0.0, "sidestep right"),
    (3.0, 0.0, 0.0, -1.0, "turn in place (right)"),
    (3.0, -0.3, 0.0, 0.0, "backward"),
    (3.0, 0.5, 0.2, -0.4, "diagonal + turn"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1, help="number of robots")
    ap.add_argument("--spacing", type=float, default=1.2)
    args = ap.parse_args()

    if args.n == 1:
        m, d = load_single()
        ctrls = [Go1BezierController(m, d)]
    else:
        side = int(np.ceil(np.sqrt(args.n)))
        pos = [(args.spacing * (i % side), args.spacing * (i // side), 0.0) for i in range(args.n)]
        m, d = load_team(pos)
        ctrls = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(args.n)]

    trunk0 = m.body("trunk" if args.n == 1 else "go1_0/trunk").id
    seg, seg_end = -1, 0.0
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = trunk0
        viewer.cam.distance = 2.0 + 0.4 * np.sqrt(args.n)
        while viewer.is_running():
            t_wall = time.time()
            if d.time >= seg_end:                           # next scripted command
                seg = (seg + 1) % len(SCRIPT)
                dur, vx, vy, wz, label = SCRIPT[seg]
                seg_end = d.time + dur
                for c in ctrls:
                    c.set_command(vx, vy, wz)
                print(f"t={d.time:5.1f}s  {label:24s} cmd=({vx:+.2f}, {vy:+.2f}, {wz:+.2f})")
            for _ in range(10):                             # 10 physics steps per frame
                for c in ctrls:
                    c.step()
                mujoco.mj_step(m, d)
            viewer.sync()
            v = ctrls[0].body_velocity()
            if int(d.time * 50) % 25 == 0:
                print(f"          measured=({v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f})  phase={ctrls[0].phase:.2f}",
                      end="\r")
            time.sleep(max(0.0, 10 * m.opt.timestep - (time.time() - t_wall)))


if __name__ == "__main__":
    main()
