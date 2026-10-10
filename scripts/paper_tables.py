"""Paper tables from the final (--robust) runs: full system vs baseline on the pre-registered
primary metrics, paired by seed, sign-flip permutation p-values with Holm correction across
all primary tests, plus safety totals (falls, failures) for every configuration.

Run:  python scripts/paper_tables.py                       (reads results/paper/*_robust, results/course)
      python scripts/paper_tables.py --suffix _robust_v1   (another set of runs)
Out:  results/paper/paper_tables.md, results/paper/primary_tests.csv
"""
import os
import sys
import argparse
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
BASE, FULL = "reactive:limiter", "predictive:phase+gaslc_equal:q"
PRIMARY = [("urgent_per_robot_min", "urgent events / robot-min", -1),
           ("p99_tilt", "p99 tilt (deg)", -1),
           ("min_dist", "closest robot distance (m)", +1)]
ORDER = ["swap4", "swap8", "swap16", "swap32", "formation9_pillars", "formation6_gate", "formation16_pillars"]


def signflip_p(x, n_perm=20000, seed=0):
    x = np.asarray(x, float)
    if np.allclose(x, 0):
        return 1.0
    if len(x) <= 16:                                   # exact
        signs = np.array(np.meshgrid(*[[-1, 1]] * len(x))).reshape(len(x), -1).T
    else:
        signs = np.random.default_rng(seed).choice([-1, 1], size=(n_perm, len(x)))
    return float((np.abs((signs * x).mean(1)) >= abs(x.mean()) - 1e-12).mean())


def holm(p):
    p = np.asarray(p, float)
    order, m, out, run = np.argsort(p), len(p), np.empty(len(p)), 0.0
    for k, i in enumerate(order):
        run = max(run, (m - k) * p[i])
        out[i] = min(1.0, run)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suffix", default="_robust")
    ap.add_argument("--course", default=os.path.join(ROOT, "results", "course", "course_eval.csv"))
    a = ap.parse_args()
    P = os.path.join(ROOT, "results", "paper")
    frames = []
    for suite in ("main", "scaling"):
        f = os.path.join(P, suite + a.suffix, "swarm_eval.csv")
        if os.path.exists(f):
            frames.append(pd.read_csv(f).assign(suite=suite))
    d = pd.concat(frames, ignore_index=True)
    rows = []
    for sc in [s for s in ORDER if s in set(d.scenario)]:
        b = d[(d.scenario == sc) & (d.config == BASE)].set_index("seed").sort_index()
        c = d[(d.scenario == sc) & (d.config == FULL)].set_index("seed").sort_index()
        seeds = b.index.intersection(c.index)
        b, c = b.loc[seeds], c.loc[seeds]
        for m, label, good in PRIMARY:
            x = (c[m] - b[m]).values
            rows.append(dict(scenario=sc, robots=int(b.N.iloc[0]), seeds=len(seeds), metric=label,
                             base=b[m].mean(), full=c[m].mean(), diff=x.mean(),
                             rel=100 * x.mean() / b[m].mean(), p=signflip_p(x),
                             better=bool(np.sign(x.mean()) == good)))
    t = pd.DataFrame(rows)
    t["p_holm"] = holm(t.p)
    t.to_csv(os.path.join(P, "primary_tests.csv"), index=False)

    L = ["# Paper tables (full system vs baseline)", "",
         f"Full = `{FULL}`, baseline = `{BASE}`; paired by seed; sign-flip permutation p; "
         f"Holm over all {len(t)} primary tests.", "",
         "| scenario | robots | seeds | metric | baseline | full | change | p (Holm) |",
         "|---|---|---|---|---|---|---|---|"]
    for r in t.itertuples():
        star = "**" if r.p_holm < 0.01 else ("*" if r.p_holm < 0.05 else "")
        L.append(f"| {r.scenario} | {r.robots} | {r.seeds} | {r.metric} | {r.base:.3g} | {r.full:.3g} | "
                 f"{r.rel:+.0f}% {'' if r.better else '(worse)'} | {r.p_holm:.4f}{star} |")
    L += ["", f"Significant after Holm (p<0.05): {(t.p_holm < 0.05).sum()} of {len(t)} "
              f"({(t.p_holm[t.better] < 0.05).sum()} in favour of the full system).", "",
          "## Safety totals (all configurations)", "",
          "| suite | scenario | config | runs | falls | not finished |", "|---|---|---|---|---|---|"]
    for (suite, sc, cfg), g in d.groupby(["suite", "scenario", "config"], sort=False):
        L.append(f"| {suite} | {sc} | {cfg} | {len(g)} | {int(g.falls.sum())} | {int((g.success == 0).sum())} |")
    la = os.path.join(P, "lookahead" + a.suffix, "swarm_eval.csv")
    if os.path.exists(la):
        q = pd.read_csv(la)
        L += ["", "## Lookahead (mean over seeds)", "", "| scenario | config | p99 tilt | urgent/min | min dist | time | falls | not finished |",
              "|---|---|---|---|---|---|---|---|"]
        for (sc, cfg), g in q.groupby(["scenario", "config"]):
            L.append(f"| {sc} | {cfg} | {g.p99_tilt.mean():.2f} | {g.urgent_per_robot_min.mean():.1f} | "
                     f"{g.min_dist.mean():.3f} | {g.time.mean():.1f} | {int(g.falls.sum())} | {int((g.success == 0).sum())} |")
    if os.path.exists(a.course):
        q = pd.read_csv(a.course)
        L += ["", "## Obstacle course (mean over seeds)", "",
              "| team | config | runs | falls | not finished | time | p99 tilt | max tilt | obstacle clearance | urgent/min |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for (tm, cfg), g in q.groupby(["team", "config"], sort=False):
            L.append(f"| {tm} | {cfg} | {len(g)} | {int(g.falls.sum())} | {int((g.success == 0).sum())} | "
                     f"{g.time.mean():.1f} | {g.p99_tilt.mean():.2f} | {g.max_tilt.max():.1f} | "
                     f"{g.min_clear_obstacle.min():.2f} | {g.urgent_per_robot_min.mean():.1f} |")
    out = os.path.join(P, "paper_tables.md")
    open(out, "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    print("\nwrote", out)


if __name__ == "__main__":
    main()
