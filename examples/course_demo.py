"""Watch the obstacle course: one robot or a formation.

macOS:  mjpython examples/course_demo.py --team 6 --config predictive:phase+gaslc_equal:q
        mjpython examples/course_demo.py --team 1 --config reactive:limiter
        --team 1 | 4 | 6 | 9      --speed 0.3 (reference speed, m/s)
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer
import course_eval as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", type=int, default=6)
    ap.add_argument("--config", default="predictive:phase+gaslc_equal:q")
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    state = {}

    def on_step(m, d, k, pos):
        if "viewer" not in state:
            v = mujoco.viewer.launch_passive(m, d)
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 9.0, -40, 90
            state.update(viewer=v, wall=time.time(), sim=d.time)
        v = state["viewer"]
        if not v.is_running():
            return False
        if k % 10 == 0:
            v.cam.lookat[:] = [*np.mean(pos, 0), 0.0]
            v.sync()
            lag = (d.time - state["sim"]) - (time.time() - state["wall"])
            if lag > 0:
                time.sleep(lag)
        return True

    r = C.run(dict(team=a.team, config=a.config, seed=a.seed, speed=a.speed, urgent_ttc=0.8, record=0), on_step)
    print({k: v for k, v in r.items() if k not in ("paths", "mover_path")})
    if "viewer" in state:
        state["viewer"].close()


if __name__ == "__main__":
    main()
