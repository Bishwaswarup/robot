"""Supplementary video (Online Resource 1): reactive baseline vs full system, side by side.

Segments (seed 0, final --robust configuration):
  1  16-robot antipodal swap            2  9-robot formation through pillars
  3  6-robot formation through a gate   4  6-robot team on the 53 m obstacle course

Run:  python scripts/make_video.py                    (all segments, ~15-30 min; headless Linux: MUJOCO_GL=osmesa)
      python scripts/make_video.py --only gate        (one segment, for a quick look)
Out:  docs/media/online_resource_1.mp4   (H.264, 1280x464, 25 fps)
      docs/media/video_parts/<segment>.mp4
Needs: pip install imageio imageio-ffmpeg pillow
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import tempfile
from multiprocessing import Pool

import numpy as np
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont
from matplotlib import font_manager
import mujoco

from make_media import Recorder, fixed_cam

ROOT = os.path.join(HERE, "..")
OUT = os.path.join(ROOT, "docs", "media")
BASE, FULL = "reactive:limiter", "predictive:phase+gaslc_equal:q"
FPS, PW, PH, BAR = 25, 640, 400, 64            # panel size; header bar height
DT = 0.002
TITLE = "Gait-Phase-Aware Command Timing for Decentralized Quadruped Swarms"

SEGMENTS = {
    "swap16": dict(kind="swarm", scenario="swap16", speedup=3, limit=60,
                   cam=dict(lookat=[0, 0, 0], distance=17.5, elevation=-80),
                   title="16-robot antipodal swap",
                   text="Each robot walks to the opposite side of a 6 m circle (3x real time)."),
    "pillars": dict(kind="swarm", scenario="formation9_pillars", speedup=4, limit=60,
                    cam=dict(lookat=[6.0, 0, 0], distance=15.0, elevation=-89),
                    title="9-robot formation through a pillar field",
                    text="A 3x3 formation travels 14 m through five pillars (4x real time)."),
    "gate": dict(kind="swarm", scenario="formation6_gate", speedup=4, limit=70,
                 cam=dict(lookat=[6.5, 0, 0], distance=14.5, elevation=-89),
                 title="6-robot formation through a 1.9 m gate",
                 text="The team narrows, queues through the opening and re-forms (4x real time)."),
    "course": dict(kind="course", speedup=8, limit=160,
                   title="6-robot team on the 53 m obstacle course",
                   text="Pole slalom, gate, chicane, two moving obstacles, pillar field (8x real time)."),
}


def font(size, bold=False):
    path = font_manager.findfont(font_manager.FontProperties(family="DejaVu Sans",
                                                             weight="bold" if bold else "normal"))
    return ImageFont.truetype(path, size)


class StreamRecorder(Recorder):
    """Recorder that streams frames to an mp4 instead of keeping them in memory; stops at `limit` s."""

    def __init__(self, path, every, cam_fn, limit, trail_len):
        super().__init__(PW, PH, every, cam_fn, 10 ** 9, trail_len=trail_len)
        self.w_ = imageio.get_writer(path, fps=FPS, codec="libx264", quality=8, macro_block_size=16)
        self.limit, self.t0, self.n = limit, None, 0

    def __call__(self, m, d, k, *_):
        if self.t0 is None:
            self.t0 = d.time
        super().__call__(m, d, k)
        while self.frames:
            self.w_.append_data(self.frames.pop(0))
            self.n += 1
        return d.time - self.t0 < self.limit

    def close(self):
        self.w_.close()


def render_panel(args):
    """Render one configuration of one segment to a temporary mp4. Returns (path, frames)."""
    name, cfg, path = args
    seg = SEGMENTS[name]
    every = int(round(seg["speedup"] / FPS / DT))
    if seg["kind"] == "swarm":
        import swarm_eval as SE
        c = seg["cam"]
        rec = StreamRecorder(path, every, fixed_cam(c["lookat"], c["distance"], c["elevation"], 90),
                             seg["limit"], trail_len=60)
        SE.run(dict(scenario=seg["scenario"], config=cfg, seed=0, urgent_ttc=0.8, lookahead=0.36, record=0,
                    **SE.ROBUST), on_step=rec)
    else:
        import course_eval as C

        def follow(cam, m, d):
            cen = np.mean([d.xpos[b] for b in rec.ids], axis=0)
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = [cen[0] + 1.0, 0.0, 0.0]
            cam.distance, cam.elevation, cam.azimuth = 10.0, -60, 90
        rec = StreamRecorder(path, every, follow, seg["limit"], trail_len=20)
        C.run(dict(team=6, config=cfg, seed=0, speed=0.45, urgent_ttc=0.8, record=0, turn_gate=1, escape=1),
              on_step=lambda m, d, k, pos: rec(m, d, k))
    rec.close()
    print(f"  rendered {name} / {cfg}: {rec.n} frames", flush=True)
    return path, rec.n


def card(lines, seconds, sizes):
    """Title card: centred lines of text on white."""
    im = Image.new("RGB", (2 * PW, PH + BAR), "white")
    dr = ImageDraw.Draw(im)
    fonts = [font(s, bold=(i == 0)) for i, s in enumerate(sizes)]
    hs = [dr.textbbox((0, 0), ln, font=f)[3] for ln, f in zip(lines, fonts)]
    y = (im.height - sum(hs) - 18 * (len(lines) - 1)) // 2
    for ln, f, h in zip(lines, fonts, hs):
        w = dr.textbbox((0, 0), ln, font=f)[2]
        dr.text(((im.width - w) // 2, y), ln, fill=(30, 30, 30), font=f)
        y += h + 18
    return [np.asarray(im)] * int(seconds * FPS)


def compose(name, left, right, writers):
    """Side by side with a header bar, streamed into every writer in `writers`; the shorter run holds its
    last frame ('finished')."""
    seg = SEGMENTS[name]
    nl, nr = imageio.get_reader(left).count_frames(), imageio.get_reader(right).count_frames()
    il, ir = iter(imageio.get_reader(left)), iter(imageio.get_reader(right))
    f_hdr, f_small = font(22, True), font(16)
    for f in card([seg["title"], seg["text"], "left: reactive baseline     right: full system"], 3.0, [34, 22, 20]):
        for w in writers:
            w.append_data(f)
    a = b = None
    frame = None
    for i in range(max(nl, nr)):
        a = next(il, a) if i < nl else a
        b = next(ir, b) if i < nr else b
        im = Image.new("RGB", (2 * PW, PH + BAR), "white")
        im.paste(Image.fromarray(a), (0, BAR))
        im.paste(Image.fromarray(b), (PW, BAR))
        dr = ImageDraw.Draw(im)
        dr.line([(PW, BAR), (PW, PH + BAR)], fill=(120, 120, 120), width=2)
        t = i * seg["speedup"] / FPS
        for x0, lab, n in ((0, "Reactive baseline", nl), (PW, "Full system", nr)):
            dr.text((x0 + 16, 10), lab, fill=(30, 30, 30), font=f_hdr)
            sub = (f"finished at {n * seg['speedup'] / FPS:.0f} s" if i >= n
                   else f"t = {t:4.1f} s   ({seg['speedup']}x)")
            dr.text((x0 + 16, 38), sub, fill=(90, 90, 90), font=f_small)
        frame = np.asarray(im)
        for w in writers:
            w.append_data(frame)
    for _ in range(FPS):                                            # hold the end for 1 s
        for w in writers:
            w.append_data(frame)


def writer(path):
    return imageio.get_writer(path, fps=FPS, codec="libx264", quality=8, macro_block_size=16)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=",".join(SEGMENTS))
    ap.add_argument("--workers", type=int, default=min(4, os.cpu_count() or 1))
    a = ap.parse_args()
    names = a.only.split(",")
    parts = os.path.join(OUT, "video_parts")
    os.makedirs(parts, exist_ok=True)
    tmp = tempfile.mkdtemp()
    jobs = [(n, cfg, os.path.join(tmp, f"{n}_{'base' if cfg == BASE else 'full'}.mp4")) for n in names
            for cfg in (BASE, FULL)]
    with Pool(min(a.workers, len(jobs))) as pool:
        pool.map(render_panel, jobs)
    full = set(names) == set(SEGMENTS)
    main_w = writer(os.path.join(OUT, "online_resource_1.mp4")) if full else None
    intro = card([TITLE, "Online Resource 1", "Simulated Unitree Go1 robots in MuJoCo; reactive baseline vs full system",
                  "Coloured dots trace each robot's recent path"], 4.0, [28, 24, 20, 18])
    for f in (intro if main_w else []):
        main_w.append_data(f)
    for n in names:
        pw = writer(os.path.join(parts, f"{n}.mp4"))
        compose(n, os.path.join(tmp, f"{n}_base.mp4"), os.path.join(tmp, f"{n}_full.mp4"),
                [pw] + ([main_w] if main_w else []))
        pw.close()
        print("wrote", os.path.join(parts, f"{n}.mp4"), flush=True)
    if main_w:
        for f in card(["Code, data and scripts:", "github.com/Bishwaswarup/robot"], 3.0, [26, 24]):
            main_w.append_data(f)
        main_w.close()
        out = os.path.join(OUT, "online_resource_1.mp4")
        print("wrote", out, f"({os.path.getsize(out) / 1e6:.1f} MB)")

if __name__ == "__main__":
    main()
