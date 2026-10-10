"""Paper figures (vector PDF + 300 dpi PNG) from the final --robust results.

Run:  python scripts/paper_figures.py                 (reads results/paper/*_robust, results/course, data/)
      python scripts/paper_figures.py --course-paths results/course/course_paths.json
Out:  paper/figures/fig2_phase.pdf, fig3_primary.pdf, fig4_ablation.pdf, fig5_lookahead.pdf, fig6_course.pdf,
      fig7_controls.pdf (results/paper/timing_robust), fig8_map_context.pdf (results/map_validation)
      (fig2_phase is Fig. 1 of the paper; Fig. 2, the stack diagram, is drawn in TikZ in the LaTeX source.)
"""
import os
import sys
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)

BASE, FULL = "reactive:limiter", "predictive:phase+gaslc_equal:q"
BLUE, RED, GREY, DARK, LIGHT = "#2f6db5", "#c0504d", "#9a9a9a", "#333333", "#d9d9d9"
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["DejaVu Serif"], "mathtext.fontset": "dejavuserif",
    "font.size": 8.5, "axes.titlesize": 9, "axes.labelsize": 8.5, "xtick.labelsize": 7.5,
    "ytick.labelsize": 7.5, "legend.fontsize": 7.5, "axes.spines.top": False, "axes.spines.right": False,
    "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6, "pdf.fonttype": 42,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02})
W = 6.6                               # inches, full text width

SCEN = [("swap4", "Swap, 4"), ("formation6_gate", "Gate, 6"), ("swap8", "Swap, 8"),
        ("formation9_pillars", "Pillars, 9"), ("swap16", "Swap, 16"), ("formation16_pillars", "Pillars, 16"),
        ("swap32", "Swap, 32")]


def save(fig, out, name):
    os.makedirs(out, exist_ok=True)
    fig.savefig(os.path.join(out, name + ".pdf"))
    fig.savefig(os.path.join(out, name + ".png"), dpi=300)
    plt.close(fig)
    print("wrote", os.path.join(out, name + ".pdf"))


from paper_tables import signflip_p, holm      # one implementation for tables and figures


def paired(d, sc, cfg, col):
    b = d[(d.scenario == sc) & (d.config == BASE)].set_index("seed")[col]
    c = d[(d.scenario == sc) & (d.config == cfg)].set_index("seed")[col]
    s = b.index.intersection(c.index)
    return b.loc[s], c.loc[s]


# ---------------------------------------------------------------- Fig. 2: phase dependence
def fig_phase(out):
    cm = np.load(os.path.join(ROOT, "data", "capability_map_bezier.npz"), allow_pickle=True)
    env = np.load(os.path.join(ROOT, "data", "push_envelope_bezier.npz"), allow_pickle=True)
    dirs = list(cm["cmd_raw/dirs"])
    ti = [dirs.index(k) for k in ("0", "45", "90", "135", "180", "225", "270", "315")]
    size_i = list(cm["cmd_raw/size"]).index(0.6)
    T = cm["cmd_raw/tilt"][1, size_i][ti]                    # v0 = 0.4 m/s, change of 0.6 m/s
    ph = cm["cmd_raw/phase"]
    G = env["fall/G"][1]                                     # v0 = 0.4: [dir, phase]
    R = G / G.max(axis=1, keepdims=True)
    edirs, eph = env["fall/dirs"], env["fall/phases"]

    fig, ax = plt.subplots(1, 2, figsize=(W, 2.45), gridspec_kw=dict(wspace=0.62))
    cmap = LinearSegmentedColormap.from_list("t", ["#f7f7f7", "#9ecae1", "#2171b5", "#08306b"])
    ext = [0, 1, -22.5, 337.5]
    im = ax[0].imshow(T, aspect="auto", origin="lower", cmap=cmap, extent=ext, vmin=0, vmax=28)
    ax[0].set_yticks(np.arange(0, 360, 45))
    ax[0].set_xlabel("gait phase at which the change is released")
    ax[0].set_ylabel("direction of change (deg, 0 = forward)")
    ax[0].set_title("(a) Peak tilt after a 0.6 m/s command step", loc="left")
    cb = fig.colorbar(im, ax=ax[0], pad=0.02, fraction=0.05)
    cb.set_label("peak tilt (deg)")
    for k, d in enumerate([0, 45, 90, 135, 180, 225, 270, 315]):
        j = int(np.argmin(T[k]))
        ax[0].plot(ph[j] + 1 / 32, d, marker="o", ms=3.2, mfc="white", mec=DARK, mew=0.7)
    cmap2 = LinearSegmentedColormap.from_list("r", ["#a50f15", "#fcbba1", "#f7f7f7"])
    step = 360 / len(edirs)
    im2 = ax[1].imshow(R, aspect="auto", origin="lower", cmap=cmap2, vmin=0.4, vmax=1.0,
                       extent=[0, 1, -step / 2, 360 - step / 2])
    ax[1].set_yticks(np.arange(0, 360, 45))
    ax[1].set_xlabel("gait phase at which the push arrives")
    ax[1].set_ylabel("push direction (deg)")
    ax[1].set_title("(b) Push the robot survives, relative to best phase", loc="left")
    cb2 = fig.colorbar(im2, ax=ax[1], pad=0.02, fraction=0.05)
    cb2.set_label("fraction of best-phase impulse")
    for a in ax:
        a.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
    fig.text(0.01, -0.13, "Walking forward at 0.4 m/s. Open circles in (a): phase with the lowest predicted tilt "
             "for each direction.\nThe pattern repeats every half period with the lateral direction mirrored.",
             fontsize=7, color="#555555")
    save(fig, out, "fig2_phase")


# ---------------------------------------------------------------- Fig. 3: primary results
def fig_primary(main, scal, out):
    d = pd.concat([main, scal], ignore_index=True)
    mets = [("urgent_per_robot_min", -1, "Fewer urgent events"), ("p99_tilt", -1, "Lower p99 tilt"),
            ("min_dist", +1, "Larger closest distance")]
    rows = []
    for sc, lab in SCEN:
        for m, sign, _ in mets:
            b, c = paired(d, sc, FULL, m)
            x = (c - b).values
            rows.append(dict(sc=sc, lab=lab, m=m, imp=sign * 100 * x.mean() / b.mean(), p=signflip_p(x),
                             n=len(x)))
    t = pd.DataFrame(rows)
    t["ph"] = holm(t.p)
    t.to_csv(os.path.join(out, "fig3_primary_data.csv"), index=False)
    fig, ax = plt.subplots(1, 3, figsize=(W, 2.6), sharey=True, gridspec_kw=dict(wspace=0.12))
    y = np.arange(len(SCEN))[::-1]
    lo, hi = -50, 45
    for a, (m, _, title) in zip(ax, mets):
        s = t[t.m == m].set_index("sc").loc[[k for k, _ in SCEN]]
        for yy, (_, r) in zip(y, s.iterrows()):
            v = max(lo, r.imp)
            sig = r.ph < 0.05
            col = (BLUE if r.imp > 0 else RED) if sig else "white"
            a.barh(yy, v, height=0.62, color=col, edgecolor=(col if sig else GREY), linewidth=0.8,
                   alpha=0.9 if r.imp > 0 else 0.65)
            txt = f"{r.imp:+.0f}".replace("-", "\u2212")
            if r.imp < lo:
                a.text(lo + 5, yy, txt, va="center", ha="left", fontsize=7, color="white")
                a.plot([lo + 0.5, lo + 2.5], [yy - 0.35, yy + 0.35], color="white", lw=1.5)
            else:
                a.text(v + (1.5 if v >= 0 else -1.5), yy, txt, va="center", ha="left" if v >= 0 else "right",
                       fontsize=7, color=DARK)
        a.axvline(0, color=DARK, lw=0.6)
        a.set_xlim(lo, hi)
        a.set_title(title)
        a.grid(axis="x", color=LIGHT, lw=0.5)
        a.set_axisbelow(True)
    ax[0].set_yticks(y)
    ax[0].set_yticklabels([f"{lab} robots" for _, lab in SCEN])
    fig.supxlabel("improvement of the full system over the baseline (%), paired by seed; significance after Holm correction",
                  fontsize=7.5, y=-0.04)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=BLUE, label="significant, better"),
                        Patch(color=RED, alpha=0.65, label="significant, worse"),
                        Patch(facecolor="white", edgecolor=GREY, label="not significant")],
               loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.17))
    save(fig, out, "fig3_primary")
    return t


# ---------------------------------------------------------------- Fig. 4: ablation
def fig_ablation(main, out):
    cfgs = [("reactive:limiter:q", "Queueing only"), ("predictive:limiter", "Prediction only"),
            ("predictive:phase+limiter", "+ phase timing"), ("predictive:phase+gaslc_equal", "+ gait-aware limiter"),
            (FULL, "Full system (+ queueing)")]
    scen = [("swap4", "Swap, 4"), ("swap8", "Swap, 8"), ("formation9_pillars", "Pillars, 9"),
            ("formation6_gate", "Gate, 6")]
    shades = ["#cfcfcf", "#a6a6a6", "#9ecae1", "#4292c6", "#08519c"]
    fig, ax = plt.subplots(1, 2, figsize=(W, 2.5), gridspec_kw=dict(wspace=0.22))
    for a, (m, title) in zip(ax, [("urgent_per_robot_min", "Urgent events per robot-minute (lower = better)"),
                                  ("p99_tilt", "99th-percentile tilt (lower = better)")]):
        x = np.arange(len(scen))
        w = 0.16
        for k, ((cfg, lab), col) in enumerate(zip(cfgs, shades)):
            vals = []
            for sc, _ in scen:
                b, c = paired(main, sc, cfg, m)
                vals.append(100 * (c - b).mean() / b.mean())
            vals = np.array(vals)
            a.bar(x + (k - 2) * w, np.clip(vals, -45, 45), w * 0.92, color=col, label=lab, edgecolor="none")
            for xi, v in zip(x + (k - 2) * w, vals):
                if v > 45:
                    a.text(xi, 46, f"+{v:.0f}", ha="center", va="bottom", fontsize=6, rotation=90, color=DARK)
        a.axhline(0, color=DARK, lw=0.6)
        a.set_xticks(x)
        a.set_xticklabels([l for _, l in scen])
        a.set_ylim(-45, 62)
        a.set_ylabel("change vs baseline (%)")
        a.set_title(title)
        a.grid(axis="y", color=LIGHT, lw=0.5)
        a.set_axisbelow(True)
    ax[0].legend(loc="upper center", bbox_to_anchor=(1.1, -0.13), ncol=5, frameon=False, handlelength=1.2)
    save(fig, out, "fig4_ablation")


def _boot(x, n=4000, seed=0):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    bs = np.random.default_rng(seed).choice(x, (n, len(x))).mean(1)
    return x.mean(), np.percentile(bs, 2.5), np.percentile(bs, 97.5)


# ---------------------------------------------------------------- Fig. 7: timing controls
def fig_controls(tim, out):
    """Change vs baseline (%, mean of per-seed paired differences, 95% bootstrap CI) for the three timed
    configurations that differ ONLY in when the held change is released."""
    cfgs = [(FULL.replace("phase+", "worst+"), "Worst phase (map)", "#bdbdbd"),
            (FULL.replace("phase+", "sham+"), "Random wait (sham)", "#7f7f7f"),
            (FULL, "Best phase (map) = full system", BLUE)]
    scen = [("swap4", "Swap, 4"), ("swap8", "Swap, 8"), ("formation9_pillars", "Pillars, 9"),
            ("formation6_gate", "Gate, 6")]
    fig, ax = plt.subplots(1, 2, figsize=(W, 2.5), gridspec_kw=dict(wspace=0.22))
    for a, (m, title) in zip(ax, [("urgent_per_robot_min", "Urgent events per robot-minute"),
                                  ("p99_tilt", "99th-percentile tilt")]):
        x = np.arange(len(scen))
        w = 0.26
        for k, (cfg, lab, col) in enumerate(cfgs):
            for xi, (sc, _) in zip(x, scen):
                b, c = paired(tim, sc, cfg, m)
                mu, lo, hi = _boot(100 * (c.values - b.values) / b.mean())
                top = 60 if m == "urgent_per_robot_min" else 30
                a.bar(xi + (k - 1) * w, min(mu, top), w * 0.9, color=col, edgecolor="none",
                      label=lab if xi == 0 else None)
                if hi <= top:
                    a.errorbar(xi + (k - 1) * w, mu, yerr=[[mu - lo], [hi - mu]], color=DARK, lw=0.7, capsize=1.5)
                else:                                   # off-scale (4-robot swap): print the value instead
                    a.text(xi + (k - 1) * w, top + 1, f"+{mu:.0f}", ha="center", va="bottom", fontsize=6,
                           rotation=90, color=DARK)
        a.axhline(0, color=DARK, lw=0.6)
        a.set_ylim(-52, 78) if m == "urgent_per_robot_min" else a.set_ylim(-30, 34)
        a.set_xticks(x)
        a.set_xticklabels([l for _, l in scen])
        a.set_ylabel("change vs reactive baseline (%)")
        a.set_title(title + " (lower = better)")
        a.grid(axis="y", color=LIGHT, lw=0.5)
        a.set_axisbelow(True)
    ax[0].legend(loc="upper center", bbox_to_anchor=(1.1, -0.13), ncol=3, frameon=False, handlelength=1.2)
    save(fig, out, "fig7_controls")


# ---------------------------------------------------------------- Fig. 8: map inside the swarm
def fig_map_context(ev, op, out):
    fig, ax = plt.subplots(1, 2, figsize=(W, 3.0), gridspec_kw=dict(width_ratios=[1, 1.15], wspace=1.05))
    a = ax[0]
    a.scatter(op.pred_tilt, op.meas_tilt, s=2, color=BLUE, alpha=0.15, lw=0)
    lim = [0, max(op.meas_tilt.max(), op.pred_tilt.max()) + 0.5]
    a.plot(lim, lim, color=DARK, lw=0.7, ls="--")
    a.set_xlim(lim); a.set_ylim(lim)
    a.set_xlabel("predicted peak tilt (deg)")
    a.set_ylabel("measured peak tilt (deg)")
    e = op.pred_tilt - op.meas_tilt
    a.set_title("(a) Absolute prediction", loc="left")
    a.text(0.04, 0.96, f"bias {e.mean():+.1f}$^\\circ$\nMAE {e.abs().mean():.1f}$^\\circ$", transform=a.transAxes,
           va="top", fontsize=7)
    a = ax[1]
    comb = (ev.d_planar >= .15) & (ev.d_yaw >= .15)
    strata = [("all events", ev.index == ev.index),
              ("previous change < 1 period ago", ev.since_release < 0.36),
              ("previous change $\\geq$ 1 period ago", ev.since_release >= 0.36),
              ("single-axis change", ~comb), ("combined planar + yaw", comb),
              ("speed $\\geq$ 0.4 m/s", ev.speed >= 0.4), ("speed 0.2-0.4 m/s", (ev.speed >= .2) & (ev.speed < .4)),
              ("speed < 0.2 m/s", ev.speed < 0.2),
              ("change 0.2-0.3", ev["size"] < 0.3), ("change 0.3-0.5", (ev["size"] >= .3) & (ev["size"] < .5)),
              ("change $\\geq$ 0.5", ev["size"] >= 0.5)]
    y = np.arange(len(strata))[::-1]
    for yi, (lab, mk) in zip(y, strata):
        sub = ev[mk]
        mu, lo, hi = _boot(sub.tilt_mapbest - sub.tilt_random)
        a.errorbar(mu, yi, xerr=[[mu - lo], [hi - mu]], fmt="o", ms=3.2, color=BLUE if yi == y[0] else DARK,
                   lw=0.8, capsize=1.5)
        a.text(0.25, yi, f"n={len(sub)}", va="center", fontsize=6.5, color=GREY)
    for yb in (y[0] - 0.5, y[2] - 0.5, y[4] - 0.5, y[7] - 0.5):    # group separators
        a.axhline(yb, color=LIGHT, lw=0.6)
    a.axvline(0, color=DARK, lw=0.6)
    a.set_yticks(y)
    a.set_yticklabels([l for l, _ in strata])
    a.set_xlim(-1.4, 0.45)
    a.set_xlabel("tilt at map's phase $-$ random wait (deg)")
    a.set_title("(b) Value of the map's phase choice", loc="left")
    a.grid(axis="x", color=LIGHT, lw=0.5)
    a.set_axisbelow(True)
    save(fig, out, "fig8_map_context")


# ---------------------------------------------------------------- Fig. 5: lookahead
def fig_lookahead(look, out):
    Ls = [0.18, 0.36, 0.54, 0.72]
    scen = [("swap8", "Swap, 8 robots", BLUE, "o"), ("formation6_gate", "Gate, 6 robots", "#e6550d", "s")]
    mets = [("urgent_frac", 100, "(a) Urgent share (%)"),
            ("big_per_robot_min", 1, "(b) Command changes / min"),
            ("p99_tilt", 1, "(c) p99 tilt (deg)")]
    fig, ax = plt.subplots(1, 3, figsize=(W, 2.2), gridspec_kw=dict(wspace=0.42))
    for a, (m, k, title) in zip(ax, mets):
        for sc, lab, col, mk in scen:
            g = look[look.scenario == sc]
            ys = [k * g[g.config == f"predictive@{L}:phase+gaslc_equal:q"][m].mean() for L in Ls]
            a.plot(Ls, ys, "-", marker=mk, ms=3.5, color=col, lw=1.1, label=lab)
            a.axhline(k * g[g.config == BASE][m].mean(), color=col, lw=0.8, ls=":")
        a.axvline(0.36, color=LIGHT, lw=3, zorder=0)
        a.set_xticks(Ls)
        a.set_xlabel("lookahead L (s)")
        a.set_title(title, loc="left", fontsize=8.5)
        a.grid(axis="y", color=LIGHT, lw=0.5)
        a.set_axisbelow(True)
    h, l = ax[0].get_legend_handles_labels()
    fig.legend(h, l, frameon=False, loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.22))
    fig.text(0.01, -0.27, "Solid: full system at each lookahead; dotted: reactive baseline of the same colour. "
             "Grey band: L = 0.36 s (one gait period), used in all other experiments.", fontsize=7, color="#555555")
    save(fig, out, "fig5_lookahead")


# ---------------------------------------------------------------- Fig. 6: course paths
def fig_course(paths_json, out):
    from go1_bezier.course import obstacle_course
    runs = json.load(open(paths_json))
    course = obstacle_course(6, speed=0.45)
    fig, ax = plt.subplots(len(runs), 1, figsize=(W, 1.75 * len(runs)), sharex=True,
                           gridspec_kw=dict(hspace=0.35))
    cols = ["#08519c", "#3182bd", "#6baed6", "#e6550d", "#fd8d3c", "#fdae6b"]
    names = {BASE: "Baseline (reactive)", FULL: "Full system"}
    for a, r in zip(np.atleast_1d(ax), runs):
        for (x, y), rad in course["obstacles"]:
            a.add_patch(plt.Circle((x, y), rad, color="#8c6d4f", lw=0))
        for mv in course["movers"]:
            a.plot([mv.a[0], mv.b[0]], [mv.a[1], mv.b[1]], ":", color=RED, lw=0.8)
        W_ = course["plan"].W
        a.plot(W_[:, 0], W_[:, 1], "--", color=GREY, lw=0.6)
        for i, p in enumerate(r["paths"]):
            p = np.array(p)
            if len(p):
                a.plot(p[:, 0], p[:, 1], "-", lw=0.8, color=cols[i % 6])
        s = r["stats"]
        a.set_title(f"{names.get(r['cfg'], r['cfg'])}: {s['time']:.0f} s, p99 tilt {s['p99_tilt']:.1f}°, "
                    f"max {s['max_tilt']:.0f}°, robot contacts {s['rr_contacts']}", loc="left", fontsize=8)
        for name, x0, x1 in course["sections"]:
            a.axvline(x0, color=LIGHT, lw=0.5)
            a.text((x0 + x1) / 2, 4.55, name, ha="center", fontsize=6.5, color="#555555")
        a.set_xlim(-5.8, 39.8)
        a.set_ylim(-4.6, 5.0)
        a.set_aspect("equal")
        a.set_ylabel("y (m)")
    np.atleast_1d(ax)[-1].set_xlabel("x (m)")
    save(fig, out, "fig6_course")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suffix", default="_robust")
    ap.add_argument("--course-paths", default=os.path.join(ROOT, "results", "course", "course_paths.json"))
    ap.add_argument("--out", default=os.path.join(ROOT, "paper", "figures"))
    a = ap.parse_args()
    P = os.path.join(ROOT, "results", "paper")
    main_ = pd.read_csv(os.path.join(P, "main" + a.suffix, "swarm_eval.csv"))
    scal = pd.read_csv(os.path.join(P, "scaling" + a.suffix, "swarm_eval.csv"))
    look = pd.read_csv(os.path.join(P, "lookahead" + a.suffix, "swarm_eval.csv"))
    fig_phase(a.out)
    fig_primary(main_, scal, a.out)
    fig_ablation(main_, a.out)
    fig_lookahead(look, a.out)
    tim = os.path.join(P, "timing" + a.suffix, "swarm_eval.csv")
    if os.path.exists(tim):
        fig_controls(pd.read_csv(tim), a.out)
    mv = os.path.join(ROOT, "results", "map_validation")
    if os.path.exists(os.path.join(mv, "events.csv")):
        fig_map_context(pd.read_csv(os.path.join(mv, "events.csv")), pd.read_csv(os.path.join(mv, "options.csv")), a.out)
    if os.path.exists(a.course_paths):
        fig_course(a.course_paths, a.out)
    else:
        print("no course paths file:", a.course_paths, "- skipping Fig. 6")


if __name__ == "__main__":
    main()
