"""Render an MP4 of the scripted walk (no viewer window needed).

python examples/record_video.py                 -> go1_walk.mp4
python examples/record_video.py --n 9 --out team.mp4
(headless Linux: prefix with MUJOCO_GL=egl or MUJOCO_GL=osmesa)
Requires: pip install imageio imageio-ffmpeg
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import argparse
import numpy as np
import mujoco
import imageio
from go1_bezier import load_single, load_team
from go1_bezier import Go1BezierController
from walk_demo import SCRIPT


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--spacing", type=float, default=1.2)
    ap.add_argument("--out", default="go1_walk.mp4")
    ap.add_argument("--seconds", type=float, default=sum(s[0] for s in SCRIPT))
    ap.add_argument("--fps", type=int, default=30)
    args = ap.parse_args()

    if args.n == 1:
        m, d = load_single()
        ctrls = [Go1BezierController(m, d)]
        track = m.body("trunk").id
    else:
        side = int(np.ceil(np.sqrt(args.n)))
        pos = [(args.spacing * (i % side), args.spacing * (i // side), 0.0) for i in range(args.n)]
        m, d = load_team(pos)
        ctrls = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(args.n)]
        track = m.body(f"go1_{args.n // 2}/trunk").id
    renderer = mujoco.Renderer(m, 352, 640)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = track
    cam.distance = 1.8 + 0.6 * np.sqrt(args.n)
    cam.elevation = -25
    cam.azimuth = 135
    steps_per_frame = int(round(1 / (args.fps * m.opt.timestep)))
    seg, seg_end, frames = -1, 0.0, []
    while d.time < args.seconds:
        if d.time >= seg_end:
            seg = (seg + 1) % len(SCRIPT)
            dur, vx, vy, wz, label = SCRIPT[seg]
            seg_end = d.time + dur
            for c in ctrls:
                c.set_command(vx, vy, wz)
        for _ in range(steps_per_frame):
            for c in ctrls:
                c.step()
            mujoco.mj_step(m, d)
        renderer.update_scene(d, cam)
        frames.append(renderer.render())
    imageio.mimsave(args.out, frames, fps=args.fps, quality=7)
    print("wrote", args.out, len(frames), "frames")


if __name__ == "__main__":
    main()
