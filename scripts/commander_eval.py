"""Closed-loop test of phase-aware command timing (the paper's main claim).

STEP test: from a steady trot (v0 = 0 or 0.4 m/s) the "swarm layer" requests a
large velocity change at a RANDOM moment (random gait phase, fresh state noise).
Four strategies handle it:
    raw            no slew limiter, release immediately
    limiter        uniform slew limiter, release immediately   (current default)
    phase          no limiter, released at the best phase from the capability map
    phase+limiter  limiter, released at the best phase
Measured from the moment of the REQUEST (so waiting counts against a strategy):
    t50 / t90  time until the smoothed velocity has covered 50% / 90% of the change
    peak tilt, falls, settle time (tilt < 5 deg, stride-averaged error < 0.1 m/s)
Same condition + same timing + same noise for all four strategies -> paired comparison.

STREAM test: 60 s of random commands re-requested at 10 Hz (like a swarm controller),
each strategy, several seeds: falls, tilt, tracking error w.r.t. the REQUESTED velocity.

Run:   python scripts/commander_eval.py --quick      (~2 min)
       python scripts/commander_eval.py              (full)
Out:   results/commander_eval.csv, results/commander_stream.csv,
       results/commander_summary.txt, results/commander_tradeoff.png
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import csv
import zlib
import itertools
import time
from multiprocessing import Pool

import numpy as np
import mujoco
import pilot_phase as PP                                   # baseline snapshots + restore
from go1_bezier import CapabilityMap, PushEnvelope, Go1BezierController, load_single
from go1_bezier.gasl import parse_strategy, make_commander as _make_commander
from go1_bezier.commander import PhaseAwareCommander, STRATEGIES

ROOT = os.path.join(HERE, "..")
MAP = os.path.join(ROOT, "data", "capability_map_bezier.npz")
ENV = os.path.join(ROOT, "data", "push_envelope_bezier.npz")
WINDOW = 2.5
_G = {}


def _init(menagerie_env):
    PP._init_worker(menagerie_env)
    _G["cmap"] = CapabilityMap.load(MAP)
    _G["env"] = PushEnvelope.load(ENV) if os.path.exists(ENV) else None
    _G["gasl"] = {}


def parse_gasl(strategy):
    kw = parse_strategy(strategy)
    return (None, None) if kw is None else ("gasl", kw)


def make_commander(c, strategy):
    return _make_commander(c, strategy, _G["cmap"], _G["env"], _G["gasl"])


# ------------------------------------------------------------------------------ step test
def step_trial(t):
    lim = t["strategy"] in ("limiter", "phase+limiter", "phase_soft") or parse_gasl(t["strategy"])[0] is not None
    B = PP._baseline(t["v0"], lim)
    m, d, c = B["m"], B["d"], B["c"]
    mujoco.mj_setState(m, d, B["phys"], mujoco.mjtState.mjSTATE_PHYSICS)
    PP._set_controller_state(c, B["ctrl"])
    d.xfrc_applied[:] = 0
    rng = np.random.default_rng(t["seed"])                 # same for all strategies
    d.qvel[0:2] += rng.normal(0, 0.05, 2)
    d.qvel[3:6] += rng.normal(0, 0.10, 3)
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep
    for _ in range(int(rng.uniform(0, c.gait.period) / dt)):  # random request moment
        c.step(); mujoco.mj_step(m, d)
    phi_req = c.gait.phi

    com = make_commander(c, t["strategy"])
    start = c.cmd_target.copy()
    if t["dir"].startswith("yaw"):
        target = start + np.array([0, 0, (2.0 if t["dir"] == "yaw+" else -2.0) * t["size"]])
    else:
        a = np.radians(float(t["dir"]))
        target = start + t["size"] * np.array([np.cos(a), np.sin(a), 0.0])
    target[0] = np.clip(target[0], -c.v_max[0], c.v_max[0])
    target[1] = np.clip(target[1], -c.v_max[1], c.v_max[1])
    target[2] = np.clip(target[2], -c.w_max, c.w_max)
    delta = target - start
    yaw_event = t["dir"].startswith("yaw")
    com.request(*target)

    n, per = int(WINDOW / dt), int(round(c.gait.period / dt))
    half = per // 2
    vbuf, sbuf = np.zeros((per, 3)), np.zeros((half, 3))
    v_meas0 = c.body_velocity()
    t50 = t90 = np.nan
    tilt = np.zeros(n); bad = np.ones(n, bool)
    fell, release = False, np.nan
    for k in range(n):
        com.step(); mujoco.mj_step(m, d)
        if np.isnan(release) and not com.waiting:
            release = (k + 1) * dt
        up = d.xmat[B["trunk"]][8]
        tilt[k] = np.degrees(np.arccos(np.clip(up, -1, 1)))
        v = c.body_velocity()
        vbuf[k % per] = v
        sbuf[k % half] = v
        vs = sbuf.mean(0) if k >= half else v_meas0          # half-period smoothing for t50/t90
        if yaw_event:
            prog = (vs[2] - start[2]) / delta[2] if abs(delta[2]) > 1e-6 else 1.0
        else:
            dv = delta[:2]
            prog = float(np.dot(vs[:2] - start[:2], dv) / max(np.dot(dv, dv), 1e-9))
        if np.isnan(t50) and prog >= 0.5:
            t50 = (k + 1) * dt
        if np.isnan(t90) and prog >= 0.9:
            t90 = (k + 1) * dt
        va = vbuf.mean(0)
        bad[k] = (k < per) or tilt[k] > PP.TILT_OK or np.linalg.norm(va[:2] - target[:2]) > PP.V_OK \
            or abs(va[2] - target[2]) > PP.W_OK
        if d.xpos[B["trunk"]][2] < 0.12 or up < 0.5:
            fell = True
            break
    idx = np.nonzero(bad[:k + 1])[0]
    settle = np.nan if fell else ((idx[-1] + 1) * dt if len(idx) and idx[-1] < n - 1 else WINDOW)
    return dict(t, phi_request=round(phi_req, 4), wait_s=round(release, 4) if np.isfinite(release) else np.nan,
                t50=round(t50, 4) if np.isfinite(t50) else np.nan,
                t90=round(t90, 4) if np.isfinite(t90) else np.nan,
                peak_tilt=round(float(tilt[:k + 1].max()), 3), fell=int(fell),
                settle=round(settle, 4) if np.isfinite(settle) else np.nan)


# ---------------------------------------------------------------------------- stream test
def stream_trial(t):
    rng = np.random.default_rng(t["seed"])
    m, d = load_single()
    c = Go1BezierController(m, d)
    com = make_commander(c, t["strategy"])
    for _ in range(250):
        com.step(); mujoco.mj_step(m, d)
    dt, T = m.opt.timestep, t["seconds"]
    target, t_next, tick = np.zeros(3), 0.0, int(0.1 / dt)
    tilts, errs, falls, k = [], [], 0, 0
    while k * dt < T:
        if k * dt >= t_next:                               # swarm decides a new velocity
            target = np.array([rng.uniform(-0.4, 0.8), rng.uniform(-0.3, 0.3), rng.uniform(-1.0, 1.0)])
            t_next = k * dt + rng.uniform(0.5, 2.0)
        if k % tick == 0:                                  # 10 Hz re-request
            com.request(*target)
        com.step(); mujoco.mj_step(m, d)
        up = d.xmat[1][8]
        tilts.append(np.degrees(np.arccos(np.clip(up, -1, 1))))
        errs.append(np.linalg.norm(c.body_velocity()[:2] - target[:2]))
        if d.qpos[2] < 0.15 or up < 0.5:
            falls += 1
            mujoco.mj_resetDataKeyframe(m, d, 0)
            c.gait.reset(); c.v_int[:] = 0; c.cmd[:] = 0; c.cmd_target[:] = 0
            com.pending = None
        k += 1
    waits = [w for (_, _, w, key) in com.log if key is not None]
    return dict(t, falls=falls, n_fallback=getattr(com, "n_fallback", 0), mean_tilt=round(float(np.mean(tilts)), 3),
                p99_tilt=round(float(np.percentile(tilts, 99)), 3), max_tilt=round(float(np.max(tilts)), 2),
                mean_err=round(float(np.mean(errs)), 4), n_big=len(waits),
                mean_wait=round(float(np.mean(waits)), 4) if waits else 0.0)


# ------------------------------------------------------------------------------ analysis
def boot_ci(x, n=2000, seed=0):
    x = np.asarray(x, float); x = x[np.isfinite(x)]
    if len(x) < 2:
        return np.nan, np.nan
    rng = np.random.default_rng(seed)
    b = rng.choice(x, (n, len(x))).mean(1)
    return np.percentile(b, 2.5), np.percentile(b, 97.5)


def analyse(rows, srows, out):
    present = list(dict.fromkeys([r["strategy"] for r in rows] + [r["strategy"] for r in srows]))
    strategies = present
    lines = []
    P = lambda s="": (print(s), lines.append(s))
    f = lambda r, k: float(r[k]) if r[k] not in ("", "nan", None) else np.nan
    P("=" * 86)
    P("STEP TEST  (large command changes requested at random moments; times from the REQUEST)")
    P("=" * 86)
    for v0 in sorted({float(r["v0"]) for r in rows}):
        R = [r for r in rows if float(r["v0"]) == v0]
        strat = [s for s in strategies if any(r["strategy"] == s for r in R)]
        P(f"\nv0 = {v0} m/s   ({len(R) // len(strat)} paired trials per strategy)")
        P(f"  {'strategy':15s} {'falls':>6s} {'tilt mean':>9s} {'p90':>6s} {'t50':>7s} {'t90':>7s} "
          f"{'settle':>7s} {'wait':>7s}")
        for s in strat:
            Rs = [r for r in R if r["strategy"] == s]
            g = lambda k: np.array([f(r, k) for r in Rs])
            P(f"  {s:15s} {100 * np.mean(g('fell')):5.1f}% {np.nanmean(g('peak_tilt')):8.2f}° "
              f"{np.nanpercentile(g('peak_tilt'), 90):5.1f}° {np.nanmean(g('t50')):6.3f}s {np.nanmean(g('t90')):6.3f}s "
              f"{np.nanmean(g('settle')):6.2f}s {np.nanmean(g('wait_s')):6.3f}s")
        # paired differences: phase vs limiter, phase+limiter vs limiter
        key = lambda r: (r["dir"], r["size"], r["timing"])
        by = {s: {key(r): r for r in R if r["strategy"] == s} for s in strat}
        pairs = [(s, "limiter") for s in strat if s not in ("limiter", "raw")] + [("phase", "raw")]
        for a, b in pairs:
            if a not in by or b not in by:
                continue
            ks = [k for k in by[a] if k in by[b]]
            P(f"  paired {a} - {b}:")
            for metric, unit in (("peak_tilt", "deg"), ("t50", "s"), ("t90", "s"), ("settle", "s")):
                diff = np.array([f(by[a][k], metric) - f(by[b][k], metric) for k in ks])
                lo, hi = boot_ci(diff)
                better = np.nanmean(diff < 0)
                P(f"     {metric:9s} mean diff {np.nanmean(diff):+7.3f} {unit}  95% CI [{lo:+.3f}, {hi:+.3f}]  "
                  f"({100 * better:.0f}% of trials lower)")
    if srows:
        P("\n" + "=" * 86)
        P("STREAM TEST  (60 s random commands re-requested at 10 Hz; error vs REQUESTED velocity)")
        P("=" * 86)
        P(f"  {'strategy':15s} {'falls/min':>9s} {'mean tilt':>9s} {'p99 tilt':>9s} {'max tilt':>9s} "
          f"{'track err':>10s} {'mean wait':>9s} {'fallbacks':>9s}")
        for s in strategies:
            S = [r for r in srows if r["strategy"] == s]
            if not S:
                continue
            g = lambda k: np.array([float(r[k]) for r in S])
            mins = np.sum(g("seconds")) / 60
            P(f"  {s:15s} {np.sum(g('falls')) / mins:9.2f} {np.mean(g('mean_tilt')):8.2f}° "
              f"{np.mean(g('p99_tilt')):8.2f}° {np.max(g('max_tilt')):8.1f}° {np.mean(g('mean_err')):8.3f} m/s "
              f"{np.mean(g('mean_wait')):8.3f}s "
              f"{np.mean([float(r.get('n_fallback', 0) or 0) for r in S]):8.1f}")
    with open(os.path.join(out, "commander_summary.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    if rows:
        plot(rows, out)


def plot(rows, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    f = lambda r, k: float(r[k]) if r[k] not in ("", "nan", None) else np.nan
    v0s = sorted({float(r["v0"]) for r in rows})
    style = {"raw": ("o", "#8a8f98"), "limiter": ("^", "#8a8f98"), "phase": ("D", "#1f5fa8"),
             "phase+limiter": ("v", "#1f5fa8"), "phase_safe": ("s", "#0b3d7a"), "phase_soft": ("P", "#0b3d7a"),
             "gasl_fast": ("*", "#c2410c"), "gasl_equal": ("X", "#c2410c")}
    fig, axes = plt.subplots(1, len(v0s), figsize=(5.8 * len(v0s), 4.4), squeeze=False)
    for j, v0 in enumerate(v0s):
        ax = axes[0, j]
        for s in dict.fromkeys(r["strategy"] for r in rows):
            Rs = [r for r in rows if float(r["v0"]) == v0 and r["strategy"] == s]
            if not Rs:
                continue
            x = np.array([f(r, "t50") for r in Rs]); y = np.array([f(r, "peak_tilt") for r in Rs])
            xm, ym = np.nanmean(x), np.nanmean(y)
            xl, xh = boot_ci(x); yl, yh = boot_ci(y)
            mk, col = style.get(s.split("@")[0], ("o", "#c2410c" if "gasl" in s else "#8a8f98"))
            ax.errorbar(xm, ym, xerr=[[xm - xl], [xh - xm]], yerr=[[ym - yl], [yh - ym]],
                        fmt=mk, ms=9, color=col, capsize=3)
            ax.annotate(s, (xm, ym), textcoords="offset points", xytext=(8, 5), fontsize=9)
        fam = []
        for s in dict.fromkeys(r["strategy"] for r in rows):
            if s == "limiter" or s.startswith("limiter*"):
                Rs = [r for r in rows if float(r["v0"]) == v0 and r["strategy"] == s]
                fam.append((np.nanmean([f(r, "t50") for r in Rs]), np.nanmean([f(r, "peak_tilt") for r in Rs])))
        if len(fam) > 1:
            fam.sort()
            ax.plot(*zip(*fam), "-", color="#8a8f98", lw=1, zorder=0, label="uniform limiter family")
            ax.legend(fontsize=8, frameon=False)
        ax.set_xlabel("t50: time from request to 50% of the velocity change (s)")
        ax.set_ylabel("mean peak tilt (deg)")
        ax.set_title(f"v0 = {v0} m/s", fontsize=10)
        ax.grid(alpha=0.25)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Command handling strategies: response time vs. tilt (lower-left is better; 95% CI).\n"
                 "Gait-aware shaping helps only if its points lie BELOW the uniform-limiter curve.",
                 fontsize=10)
    fig.tight_layout()
    fn = os.path.join(out, "commander_tradeoff.png")
    fig.savefig(fn, dpi=130)
    plt.close(fig)
    print("wrote", fn)


def run_pool(fn, jobs, workers, label):
    out, t0 = [], time.time()
    with Pool(workers, initializer=_init, initargs=(os.environ.get("MENAGERIE_PATH", ""),)) as pool:
        for i, r in enumerate(pool.imap_unordered(fn, jobs, chunksize=4)):
            out.append(r)
            if (i + 1) % max(1, len(jobs) // 20) == 0:
                el = time.time() - t0
                print(f"  {label}: {i + 1}/{len(jobs)}  {el / 60:.1f} min, "
                      f"~{el / (i + 1) * (len(jobs) - i - 1) / 60:.1f} min left", flush=True)
    return out


def write_csv(rows, path):
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--timings", type=int, default=16, help="random request moments per condition")
    ap.add_argument("--stream-seeds", type=int, default=6)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    ap.add_argument("--analyse-only", action="store_true")
    ap.add_argument("--strategies", default="limiter*0.75,limiter,limiter*1.35,limiter*1.7,"
                                            "gasl_equal,gaslc_equal,gaslc_x1.35,gaslc_x1.7")
    ap.add_argument("--stream-only", action="store_true", help="skip the step test")
    ap.add_argument("--v0s", default="0.0,0.4")
    ap.add_argument("--sizes", default="0.4,0.6,0.8")
    ap.add_argument("--dirs", default="0,45,90,135,180,225,270,315,yaw+,yaw-")
    a = ap.parse_args()
    strategies = a.strategies.split(",")
    if not os.path.exists(MAP):
        sys.exit(f"{MAP} missing - run scripts/capability_map.py first")
    os.makedirs(a.out, exist_ok=True)
    tag = "_quick" if a.quick else ""
    p_step, p_stream = (os.path.join(a.out, f"commander_eval{tag}.csv"),
                        os.path.join(a.out, f"commander_stream{tag}.csv"))
    if a.analyse_only:
        rows = list(csv.DictReader(open(p_step)))
        srows = list(csv.DictReader(open(p_stream))) if os.path.exists(p_stream) else []
    else:
        dirs = a.dirs.split(",")
        v0s, sizes = [float(x) for x in a.v0s.split(",")], [float(x) for x in a.sizes.split(",")]
        timings, seeds, secs = a.timings, a.stream_seeds, 60.0
        if a.quick:
            dirs, v0s, sizes, timings, seeds, secs = ["45", "90", "270", "yaw+"], [0.4], [0.6], 4, 1, 20.0
        jobs = []
        for v0, dr, sz, tm in itertools.product(v0s, dirs, sizes, range(timings)):
            seed = 1 + zlib.crc32(f"{v0}|{dr}|{sz}|{tm}".encode()) % 1_000_000   # reproducible
            for s in strategies:
                jobs.append(dict(v0=v0, dir=dr, size=sz, timing=tm, seed=int(seed), strategy=s))
        if a.stream_only:
            rows = list(csv.DictReader(open(p_step))) if os.path.exists(p_step) else []
        else:
            print(f"step test: {len(jobs)} trials on {a.workers} workers")
            rows = run_pool(step_trial, jobs, a.workers, "step")
            write_csv(rows, p_step)
        sjobs = [dict(strategy=s, seed=1000 + i, seconds=secs) for i in range(seeds) for s in strategies]
        print(f"stream test: {len(sjobs)} runs of {secs:.0f} s")
        srows = run_pool(stream_trial, sjobs, a.workers, "stream")
        write_csv(srows, p_stream)
        rows = [{k: str(v) for k, v in r.items()} for r in rows]
        srows = [{k: str(v) for k, v in r.items()} for r in srows]
    analyse(rows, srows, a.out)


if __name__ == "__main__":
    main()
