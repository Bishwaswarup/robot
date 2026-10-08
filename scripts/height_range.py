"""How low / high can the Go1 stand, and at which heights can it still walk?

Part 1 (static): commands stand heights from 0.08 to 0.40 m with the pose
controller and reports the trunk height actually reached.
Part 2 (walking): runs the Bezier trot at several stand heights and reports
velocity tracking, tilt and falls.

Run:  python scripts/height_range.py
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
import mujoco
from go1_bezier import load_single, PoseSequencer, Keyframe, Go1BezierController


def static_heights(heights=np.arange(0.08, 0.41, 0.02)):
    print("STATIC  commanded -> reached trunk height")
    for h in heights:
        m, d = load_single()
        for _ in range(300):
            mujoco.mj_step(m, d)
        seq = PoseSequencer(m, d, [Keyframe(2.0, (0, 0, h, 0, 0)), Keyframe(1.0, (0, 0, h, 0, 0))])
        seq.start()
        while not seq.done:
            seq.step(); mujoco.mj_step(m, d)
        z = d.xpos[m.body("trunk").id][2]
        note = "  (trunk on ground)" if z < 0.10 else ("  (legs at full extension)" if h - z > 0.02 else "")
        print(f"  {h:.2f} -> {z:.3f} m{note}")


def walking(heights=(0.12, 0.14, 0.16, 0.20, 0.25, 0.27, 0.30, 0.34, 0.36, 0.37),
            cmds=((0.4, 0, 0), (0.7, 0, 0), (0, 0.25, 0), (0, 0, 1.0), (-0.3, 0, 0))):
    print("\nWALKING  stand height | per command: measured (vx,vy,wz), max tilt deg; X = fell")
    for hh in heights:
        line = f"  hip-foot {hh:.2f} m |"
        for cmd in cmds:
            m, d = load_single()
            c = Go1BezierController(m, d, height=hh)
            for _ in range(500):
                c.step(); mujoco.mj_step(m, d)
            c.set_command(*cmd)
            vs, tilt, fell = [], 0.0, False
            for k in range(2000):
                c.step(); mujoco.mj_step(m, d)
                up = d.xmat[1][8]
                tilt = max(tilt, np.degrees(np.arccos(np.clip(up, -1, 1))))
                if d.qpos[2] < 0.08 or up < 0.5:
                    fell = True
                    break
                if k > 1000:
                    vs.append(c.body_velocity())
            if fell:
                line += "        X        |"
            else:
                v = np.mean(vs, 0)
                main = int(np.argmax(np.abs(cmd)))
                line += f" {v[main]:+.2f}/{cmd[main]:+.2f} {tilt:4.1f}° |"
        print(line)


if __name__ == "__main__":
    static_heights()
    walking()
