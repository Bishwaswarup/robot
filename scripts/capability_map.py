"""Build and evaluate the locomotion capability map from the pilot data.

  1. symmetry check  (left-right mirror  <->  half-cycle phase shift)
  2. build the map   -> data/capability_map_bezier.npz   (go1_bezier.capability)
  3. validation      held-out repeat and held-out phases; phase-aware vs phase-blind
  4. timing benefit  if the swarm layer may DELAY a command change by up to W gait
                     periods to release it at the best phase, what do we gain over
                     (a) releasing immediately and (b) the uniform slew limiter?
                     Cross-validated: choices from a map built on repeats 0-1,
                     outcomes read from repeat 2.

Run:  python scripts/capability_map.py [--csv results/pilot_phase.csv]
Out:  data/capability_map_bezier.npz, results/capability_summary.txt, results/capmap_*.png
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import argparse
import csv
import warnings
import numpy as np
warnings.filterwarnings("ignore", message="Mean of empty slice")
from go1_bezier.capability import CapabilityMap, PLANAR_DIRS

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
PERIOD = 0.36
LINES = []


def P(s=""):
    print(s)
    LINES.append(s)


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["kindk"] = "push" if r["kind"] == "push" else ("cmd_lim" if r["limiter"] == "True" else "cmd_raw")
        r["v0"], r["mag"], r["phi"] = float(r["v0"]), float(r["mag"]), round(float(r["phi"]) % 1, 4)
        r["fell"], r["rep"] = int(r["fell"]), int(r["rep"])
        r["peak_tilt"] = float(r["peak_tilt"])
        r["rec"] = float(r["recovery_s"]) if r["recovery_s"] not in ("", "nan") else np.nan
    return rows


def symmetry_check(rows):
    import collections
    g = collections.defaultdict(list)
    for r in rows:
        if r["dir"].isdigit():
            g[(r["kindk"], r["v0"], r["mag"], int(r["dir"]), r["phi"])].append(r["peak_tilt"])
    m = {k: np.mean(v) for k, v in g.items()}
    pairs, ctrl = [], []
    for (k, v0, mg, th, ph), val in m.items():
        mk = (k, v0, mg, (-th) % 360, round((ph + 0.5) % 1, 4))
        ck = (k, v0, mg, th, round((ph + 0.5) % 1, 4))
        if mk in m:
            pairs.append((val, m[mk]))
        if ck in m:
            ctrl.append((val, m[ck]))
    a, b = np.array(pairs), np.array(ctrl)
    P("1) TROT MIRROR SYMMETRY (peak tilt, cell means)")
    P(f"   mirror (theta -> -theta, phi -> phi+0.5): r = {np.corrcoef(a.T)[0, 1]:.3f}, "
      f"mean |diff| = {np.abs(a[:, 0] - a[:, 1]).mean():.2f} deg")
    P(f"   control (phi -> phi+0.5 only):            r = {np.corrcoef(b.T)[0, 1]:.3f}, "
      f"mean |diff| = {np.abs(b[:, 0] - b[:, 1]).mean():.2f} deg")


def _scores(rows, cmap, blind=False):
    """R2 for tilt (survivors), Brier for falls. blind=True: map averaged over phase."""
    yt, pt, yf, pf = [], [], [], []
    for r in rows:
        d = r["dir"] if r["dir"].startswith("yaw") else float(r["dir"])
        if blind:
            ph = np.arange(16) / 16
            pr = {m: np.nanmean([cmap.predict(r["kindk"], r["v0"], d, r["mag"], p, m) for p in ph])
                  for m in ("tilt", "fall")}
        else:
            pr = cmap.predict(r["kindk"], r["v0"], d, r["mag"], r["phi"])
        if not r["fell"] and np.isfinite(pr["tilt"]):
            yt.append(r["peak_tilt"]); pt.append(pr["tilt"])
        yf.append(r["fell"]); pf.append(pr["fall"])
    yt, pt = np.array(yt), np.array(pt)
    r2 = 1 - ((yt - pt) ** 2).sum() / ((yt - yt.mean()) ** 2).sum()
    mae = np.abs(yt - pt).mean()
    brier = np.mean((np.array(yf) - np.array(pf)) ** 2)
    return r2, mae, brier


def validation(rows):
    P("\n3) VALIDATION  (tilt: R2 / mean abs error on survivors; falls: Brier score, lower = better)")
    train = [r for r in rows if r["rep"] < 2]
    test = [r for r in rows if r["rep"] == 2]
    cm = CapabilityMap.from_rows(train)
    for kind in ("cmd_lim", "cmd_raw", "push"):
        te = [r for r in test if r["kindk"] == kind]
        a = _scores(te, cm)
        b = _scores(te, cm, blind=True)
        P(f"   held-out repeat, {kind:7s}: phase-aware R2 {a[0]:.3f} MAE {a[1]:.2f} deg Brier {a[2]:.3f} | "
          f"phase-blind R2 {b[0]:.3f} MAE {b[1]:.2f} deg Brier {b[2]:.3f}")
    even = [r for r in rows if round(r["phi"] * 16) % 2 == 0]
    odd = [r for r in rows if round(r["phi"] * 16) % 2 == 1]
    cm8 = CapabilityMap.from_rows(even)
    for kind in ("cmd_lim", "cmd_raw", "push"):
        a = _scores([r for r in odd if r["kindk"] == kind], cm8)
        P(f"   held-out phases (map from 8 phases, predict the 8 in between), {kind:7s}: "
          f"R2 {a[0]:.3f} MAE {a[1]:.2f} deg Brier {a[2]:.3f}")


def timing_benefit(rows):
    P("\n4) PHASE-AWARE COMMAND TIMING  (cross-validated; mean over directions, sizes and the")
    P("   phase at which the command change is requested)")
    P("   settle = wait for the chosen phase + recovery time")
    train = [r for r in rows if r["rep"] < 2 and r["kind"] == "cmd"]
    test = {(r["kindk"], r["v0"], r["dir"], r["mag"], r["phi"]): r
            for r in rows if r["rep"] == 2 and r["kind"] == "cmd"}
    cm = CapabilityMap.from_rows(train)
    phases = np.arange(16) / 16
    strategies = [("no limiter, release now", "cmd_raw", 0.0), ("no limiter, wait <= 1/4 cycle", "cmd_raw", 0.25),
                  ("no limiter, wait <= 1/2 cycle", "cmd_raw", 0.5), ("slew limiter, release now", "cmd_lim", 0.0),
                  ("slew limiter, wait <= 1/2 cycle", "cmd_lim", 0.5)]
    results = {}
    for v0 in (0.0, 0.4):
        for subset, mags in (("all sizes", (0.2, 0.4, 0.6, 0.8)), ("large (0.6-0.8 m/s)", (0.6, 0.8))):
            P(f"\n   v0 = {v0} m/s, {subset}")
            P(f"   {'strategy':34s} {'mean tilt':>9s} {'p90 tilt':>9s} {'settle':>8s}  {'mean wait':>9s}")
            for name, kind, W in strategies:
                tilts, settles, waits = [], [], []
                for d in [str(x) for x in PLANAR_DIRS] + ["yaw+", "yaw-"]:
                    dd = d if d.startswith("yaw") else float(d)
                    for mg in mags:
                        for ph0 in phases:
                            nsteps = int(round(W * 16))
                            cands = [round((ph0 + k / 16) % 1, 4) for k in range(nsteps + 1)]
                            pred = [cm.predict(kind, v0, dd, mg, c, "tilt") for c in cands]
                            k = int(np.nanargmin(pred))
                            r = test[(kind, v0, d, mg, cands[k])]
                            tilts.append(r["peak_tilt"]); waits.append(k / 16 * PERIOD)
                            settles.append(k / 16 * PERIOD + (r["rec"] if np.isfinite(r["rec"]) else 2.0))
                results[(v0, subset, name)] = (np.mean(tilts), np.percentile(tilts, 90), np.mean(settles), np.mean(waits))
                P(f"   {name:34s} {np.mean(tilts):7.2f}° {np.percentile(tilts, 90):7.2f}° "
                  f"{np.mean(settles):6.2f} s {np.mean(waits):7.3f} s")
    return results


def plots(rows, cmap, results, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed - skipping plots)")
        return
    for kind, metric, lab in (("cmd_raw", "tilt", "peak tilt (deg)"), ("cmd_lim", "tilt", "peak tilt (deg)"),
                              ("push", "fall", "fall probability"), ("push", "tilt", "peak tilt (deg), survivors")):
        ax_ = cmap.axes[kind]
        sizes, v0s = ax_["size"], ax_["v0"]
        fig, axes = plt.subplots(len(sizes), len(v0s), figsize=(5.2 * len(v0s), 2.4 * len(sizes)),
                                 squeeze=False, sharex=True)
        G = cmap.grids[kind][metric]
        vmax = 1.0 if metric == "fall" else np.nanpercentile(G, 99)
        for i, sz in enumerate(sizes):
            for j, v0 in enumerate(v0s):
                ax = axes[i, j]
                im = ax.imshow(G[j, i], aspect="auto", origin="lower", cmap="Blues", vmin=0, vmax=vmax,
                               extent=(-0.5 / 16, 1 - 0.5 / 16, -0.5, len(ax_["dirs"]) - 0.5))
                ax.set_yticks(range(len(ax_["dirs"])))
                ax.set_yticklabels([d + ("°" if d.isdigit() else "") for d in ax_["dirs"]], fontsize=7)
                unit = "N" if kind == "push" else "m/s"
                ax.set_title(f"v0 {v0} m/s, size {sz:g} {unit}", fontsize=9)
                if i == len(sizes) - 1:
                    ax.set_xlabel("gait phase")
                ax.axvline(0.5, color="white", lw=0.8, alpha=0.6)
        fig.colorbar(im, ax=axes, fraction=0.02, label=lab)
        name = {"cmd_raw": "command change, no limiter", "cmd_lim": "command change, slew limiter",
                "push": "push 0.1 s"}[kind]
        fig.suptitle(f"Capability map - {name}: {lab}  (symmetrized; phase 0-0.5 FR+RL stance)", fontsize=10)
        fn = os.path.join(out, f"capmap_{kind}_{metric}.png")
        fig.savefig(fn, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print("wrote", fn)

    # trade-off: tilt vs settle time
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    marker = {"no limiter, release now": "o", "no limiter, wait <= 1/4 cycle": "s",
              "no limiter, wait <= 1/2 cycle": "D", "slew limiter, release now": "^",
              "slew limiter, wait <= 1/2 cycle": "v"}
    for j, v0 in enumerate((0.0, 0.4)):
        ax = axes[j]
        for name, mk in marker.items():
            t, _, s, _ = results[(v0, "large (0.6-0.8 m/s)", name)]
            col = "#1f5fa8" if name.startswith("no") else "#8a8f98"
            ax.plot(s, t, mk, ms=9, color=col)
            ax.annotate(name, (s, t), textcoords="offset points", xytext=(8, 4), fontsize=8)
        ax.set_xlabel("settle time: wait + recovery (s)")
        ax.set_ylabel("mean peak tilt (deg)")
        ax.set_title(f"large command changes, v0 = {v0} m/s", fontsize=10)
        ax.grid(alpha=0.25)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Phase-aware timing vs. uniform slew limiting (lower-left is better; cross-validated)",
                 fontsize=10)
    fig.tight_layout()
    fn = os.path.join(out, "capmap_timing_tradeoff.png")
    fig.savefig(fn, dpi=130)
    plt.close(fig)
    print("wrote", fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(ROOT, "results", "pilot_phase.csv"))
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    ap.add_argument("--map", default=os.path.join(ROOT, "data", "capability_map_bezier.npz"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.makedirs(os.path.dirname(a.map), exist_ok=True)
    rows = load(a.csv)
    symmetry_check(rows)
    cmap = CapabilityMap.from_rows(rows)
    cmap.save(a.map)
    P(f"\n2) MAP BUILT from {len(rows)} trials -> {os.path.relpath(a.map, ROOT)}")
    validation(rows)
    res = timing_benefit(rows)
    with open(os.path.join(a.out, "capability_summary.txt"), "w") as f:
        f.write("\n".join(LINES) + "\n")
    plots(rows, cmap, res, a.out)


if __name__ == "__main__":
    main()
