"""Render README media (GIFs and stills) offscreen.

Run:  python scripts/make_media.py                 (all; macOS works as is,
                                                   headless Linux: MUJOCO_GL=osmesa or egl)
      python scripts/make_media.py --only trot,swap16
Out:  docs/media/go1_trot.gif, go1_closeup.png, swarm_swap16.gif, swarm_swap16_paths.png,
      swarm_gate.gif, swarm_gate_paths.png, course_team6.gif, course_team6_paths.png
Needs: pip install imageio pillow
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import numpy as np
import mujoco
from PIL import Image

OUT = os.path.join(HERE, "..", "docs", "media")
FULL = "predictive:phase+gaslc_equal:q"


def save_gif(frames, path, fps):
    ims = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=64) for f in frames]
    ims[0].save(path, save_all=True, append_images=ims[1:], duration=int(1000 / fps), loop=0, optimize=True)
    print("wrote", path, f"({os.path.getsize(path) / 1e6:.1f} MB, {len(frames)} frames)")


PALETTE = np.array([[0.89, 0.10, 0.11], [0.22, 0.49, 0.72], [0.30, 0.69, 0.29], [0.60, 0.31, 0.64],
                    [1.00, 0.50, 0.00], [0.65, 0.34, 0.16], [0.97, 0.51, 0.75], [0.10, 0.62, 0.62]])


def team_style(m, n):
    """Plain light floor (small GIFs, good contrast) and one colour per robot trunk."""
    f = m.geom("floor").id
    m.geom_matid[f] = -1
    m.geom_rgba[f] = [0.93, 0.93, 0.91, 1]
    cols = []
    for i in range(n):
        t = m.body(f"go1_{i}/trunk").id
        c = PALETTE[i % len(PALETTE)]
        for g in np.where((m.geom_bodyid == t) & (m.geom_group == 2))[0]:
            m.geom_matid[g] = -1
            m.geom_rgba[g] = [*c, 1]
        cols.append(c)
    return cols


class Recorder:
    """Called after each physics step; renders every `every`-th step from a camera.
    With trails=True each robot leaves a fading dotted path of its trunk position."""

    def __init__(self, w, h, every, cam_fn, max_frames, still_at=None, trails=True, trail_len=120, dot=0.05):
        self.w, self.h, self.every, self.cam_fn, self.max_frames = w, h, every, cam_fn, max_frames
        self.frames, self.r, self.still_at, self.still = [], None, still_at, None
        self.trails, self.trail_len, self.dot = trails, trail_len, dot

    def _setup(self, m):
        m.vis.global_.offwidth = max(m.vis.global_.offwidth, self.w)
        m.vis.global_.offheight = max(m.vis.global_.offheight, self.h)
        n = 0
        while True:
            try:
                m.body(f"go1_{n}/trunk")
                n += 1
            except KeyError:
                break
        self.ids = [m.body(f"go1_{i}/trunk").id for i in range(n)]
        self.cols = team_style(m, n)
        self.hist = [[] for _ in range(n)]
        self.r = mujoco.Renderer(m, self.h, self.w, max_geom=20000)
        self.cam = mujoco.MjvCamera()

    def _draw_trails(self, d):
        sc = self.r.scene
        for i, b in enumerate(self.ids):
            self.hist[i].append(d.xpos[b][:2].copy())
            pts = self.hist[i][-self.trail_len:]
            for j, q in enumerate(pts):
                if sc.ngeom >= sc.maxgeom:
                    return
                a = 0.25 + 0.6 * (j + 1) / len(pts)
                mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER,
                                    np.array([self.dot, 0.005, 0]), np.array([q[0], q[1], 0.006]),
                                    np.eye(3).ravel(), np.array([*self.cols[i], a], np.float32))
                sc.ngeom += 1

    def __call__(self, m, d, k, *_):
        if self.r is None:
            self._setup(m)
        if k % self.every:
            return True
        self.cam_fn(self.cam, m, d)
        self.r.update_scene(d, self.cam)
        if self.trails:
            self._draw_trails(d)
        img = self.r.render().copy()
        self.frames.append(img)
        if self.still_at is not None and len(self.frames) == self.still_at:
            self.still = img
        return len(self.frames) < self.max_frames


def fixed_cam(lookat, distance, elevation, azimuth):
    def f(cam, m, d):
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = lookat
        cam.distance, cam.elevation, cam.azimuth = distance, elevation, azimuth
    return f


def trot():
    from go1_bezier import load_single, Go1BezierController
    m, d = load_single()
    c = Go1BezierController(m, d)
    r = mujoco.Renderer(m, 300, 400)
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 960
    hi = mujoco.Renderer(m, 720, 960)                 # high-resolution still
    cam = mujoco.MjvCamera()
    trunk = m.body("trunk").id
    frames, still = [], None
    for _ in range(250):
        c.step(); mujoco.mj_step(m, d)
    script = [(2.0, (0.4, 0, 0)), (2.0, (0.3, 0, 0.6)), (1.5, (0, 0.25, 0)), (1.5, (0.5, 0, 0))]
    every = int(round((1 / 15) / m.opt.timestep))
    k = 0
    for dur, cmd in script:
        c.set_command(*cmd)
        for _ in range(int(dur / m.opt.timestep)):
            c.step(); mujoco.mj_step(m, d); k += 1
            if k % every == 0:
                cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                cam.lookat[:] = d.xpos[trunk] + np.array([0, 0, -0.05])
                cam.distance, cam.elevation, cam.azimuth = 1.6, -18, 135
                r.update_scene(d, cam)
                frames.append(r.render().copy())
                if len(frames) == 18:
                    hi.update_scene(d, cam)
                    still = hi.render().copy()
    save_gif(frames, os.path.join(OUT, "go1_trot.gif"), 15)
    Image.fromarray(still).save(os.path.join(OUT, "go1_closeup.png"))
    print("wrote", os.path.join(OUT, "go1_closeup.png"))


def swarm(scenario, name, lookat, distance, elevation, seconds, speedup, fps=10, size=(640, 400)):
    """GIF of one run of the full system (seed 0) + a still of the last frame (full paths)."""
    import swarm_eval as SE
    dt = 0.002
    every = int(round(speedup / fps / dt))          # physics steps per frame
    rec = Recorder(*size, every, fixed_cam(lookat, distance, elevation, 90), int(seconds / (every * dt)),
                   trail_len=10 ** 6)
    job = dict(scenario=scenario, config=FULL, seed=0, urgent_ttc=0.8, lookahead=0.36, record=0, **SE.ROBUST)
    SE.run(job, on_step=rec)
    save_gif(rec.frames, os.path.join(OUT, name + ".gif"), fps)
    Image.fromarray(rec.frames[-1]).save(os.path.join(OUT, name + "_paths.png"))
    print("wrote", os.path.join(OUT, name + "_paths.png"))


def course():
    """Follow-camera GIF of the team of 6 on the obstacle course + an overview still with all paths."""
    import course_eval as C
    from go1_bezier.course import obstacle_course
    dt = 0.002
    fps, speedup = 10, 8
    every = int(round(speedup / fps / dt))

    def follow(cam, m, d):
        c = np.mean([d.xpos[b] for b in rec.ids], axis=0)
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [c[0] + 1.0, 0.0, 0.0]
        cam.distance, cam.elevation, cam.azimuth = 10.0, -60, 90
    rec = Recorder(560, 340, every, follow, 10 ** 6, trail_len=12)
    state = {}

    def step(m, d, k, pos):
        state["m"], state["d"] = m, d
        return rec(m, d, k)
    C.run(dict(team=6, config=FULL, seed=0, speed=0.45, urgent_ttc=0.8, record=0, turn_gate=1, escape=1),
          on_step=step)
    save_gif(rec.frames, os.path.join(OUT, "course_team6.gif"), fps)

    # overview: whole course, full paths
    q = np.array([o[0] for o in obstacle_course(6)["obstacles"]])
    lo, hi = q.min(0) - 1.0, q.max(0) + 1.0
    m, d = state["m"], state["d"]
    w, h = 1200, 400
    m.vis.global_.offwidth, m.vis.global_.offheight = max(m.vis.global_.offwidth, w), max(m.vis.global_.offheight, h)
    rec.r.close()                                    # one GL context at a time (osmesa)
    r = mujoco.Renderer(m, h, w, max_geom=20000)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0]
    cam.distance = max(0.42 * (hi[0] - lo[0]), 1.25 * (hi[1] - lo[1]))
    cam.elevation, cam.azimuth = -89, 90
    r.update_scene(d, cam)
    rec.r, rec.trail_len, rec.dot = r, 10 ** 6, 0.09
    for hh in rec.hist:                              # redraw complete paths without appending
        hh.pop()
    rec._draw_trails(d)
    img = r.render()
    rows = np.where((np.abs(img.astype(int) - img[0, 0].astype(int)).sum(2) > 30).any(1))[0]
    img = img[max(rows[0] - 15, 0):rows[-1] + 15]          # trim empty floor above/below
    Image.fromarray(img).save(os.path.join(OUT, "course_team6_paths.png"))
    print("wrote", os.path.join(OUT, "course_team6_paths.png"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="trot,swap16,gate,course")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    todo = a.only.split(",")
    if "trot" in todo:
        trot()
    if "swap16" in todo:
        swarm("swap16", "swarm_swap16", [0, 0, 0], 17.5, -80, 40.0, speedup=3)
    if "gate" in todo:
        swarm("formation6_gate", "swarm_gate", [6.5, 0, 0], 14.5, -89, 60.0, speedup=4)
    if "course" in todo:
        course()


if __name__ == "__main__":
    main()
