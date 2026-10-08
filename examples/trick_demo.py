"""Trick routine in the MuJoCo viewer: per-leg check + height range.

  1. stand -> lowest stand -> highest stand -> lie down -> stand up
  2. sit on hind legs, wave front RIGHT, then front LEFT
  3. sit on front legs, wave hind LEFT, then hind RIGHT
  4. walk at normal, crouched (0.18 m) and tall (0.32 m) height

macOS:  mjpython examples/trick_demo.py      Linux/Windows: python examples/trick_demo.py
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import time
import mujoco
import mujoco.viewer
from go1_bezier import load_single, PoseSequencer, Go1BezierController
from go1_bezier.tricks import build_tricks

WALK = [  # (duration, height, vx, vy, wz, label)
    (3.0, 0.27, 0.4, 0.0, 0.0, "walk, normal height 0.27 m"),
    (4.0, 0.18, 0.3, 0.0, 0.5, "crouch-walk 0.18 m"),
    (4.0, 0.32, 0.4, 0.0, -0.5, "tall walk 0.32 m"),
    (3.0, 0.27, 0.0, 0.0, 0.0, "back to normal, stop"),
]


def main():
    m, d = load_single()
    seq = PoseSequencer(m, d, build_tricks())
    walker = None
    w_idx, w_end, last = -1, 0.0, None
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = m.body("trunk").id
        viewer.cam.distance = 1.4
        for _ in range(500):                       # settle
            mujoco.mj_step(m, d)
        seq.start()
        while viewer.is_running():
            t0 = time.time()
            for _ in range(10):
                if not seq.done:
                    seq.step()
                    label = seq.label
                else:
                    if walker is None:
                        walker = Go1BezierController(m, d)
                    if d.time >= w_end:
                        w_idx += 1
                        if w_idx >= len(WALK):     # start over
                            seq.start(); walker, w_idx, w_end = None, -1, 0.0
                            break
                        dur, h, vx, vy, wz, wlabel = WALK[w_idx]
                        w_end = d.time + dur
                        walker.set_height(h)
                        walker.set_command(vx, vy, wz)
                    walker.step()
                    label = WALK[w_idx][5]
                mujoco.mj_step(m, d)
                if label != last:
                    print(f"t={d.time:6.1f}s  {label}")
                    last = label
            viewer.sync()
            time.sleep(max(0.0, 10 * m.opt.timestep - (time.time() - t0)))


if __name__ == "__main__":
    main()
