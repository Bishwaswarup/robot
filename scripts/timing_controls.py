"""Is it the PHASE or just the WAIT? Paired comparison of the full system with its timing controls.

  full   release at the capability map's best phase within the window (the paper's system)
  sham   wait a uniformly random time from the same window (same mean wait, map ignored)
  worst  release at the map's worst phase

If the map's phase choice is what helps, full < sham < worst on urgent events and tilt. If full ~ sham,
the swarm-level gain comes from the short delay plus shaping, not from the phase choice.

Run:  python scripts/paper_runs.py --suites timing --robust      (the runs, 320)
      python scripts/timing_controls.py                           (this analysis)
Out:  results/paper/timing_robust/timing_controls.md
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import argparse
import numpy as np
import pandas as pd
from paper_tables import signflip_p, holm

ROOT = os.path.join(HERE, "..")
CFG = {"base": "reactive:limiter", "full": "predictive:phase+gaslc_equal:q",
       "sham": "predictive:sham+gaslc_equal:q", "worst": "predictive:worst+gaslc_equal:q"}
METRICS = [("urgent_per_robot_min", "urgent events / robot-min"), ("p99_tilt", "p99 tilt (deg)"),
           ("min_dist", "closest distance (m)")]
PAIRS = [("full", "sham"), ("full", "worst"), ("sham", "worst"), ("sham", "base")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(ROOT, "results", "paper", "timing_robust"))
    a = ap.parse_args()
    d = pd.read_csv(os.path.join(a.dir, "swarm_eval.csv"))
    inv = {v: k for k, v in CFG.items()}
    d["c"] = d.config.map(inv)
    L = ["# Timing controls: map-best phase vs random wait (sham) vs map-worst phase", "",
         "Paired by seed; exact two-sided sign-flip p; Holm over every test in the table.", "",
         "## Means", "", "| scenario | config | runs | urgent/min | p99 tilt | closest dist | mean wait (s) | falls | "
         "not finished |", "|---|---|---|---|---|---|---|---|---|"]
    for sc, g in d.groupby("scenario", sort=False):
        for c in ("base", "full", "sham", "worst"):
            q = g[g.c == c]
            if len(q):
                L.append(f"| {sc} | {c} | {len(q)} | {q.urgent_per_robot_min.mean():.1f} | {q.p99_tilt.mean():.2f} | "
                         f"{q.min_dist.mean():.3f} | {q.mean_wait.mean():.3f} | {int(q.falls.sum())} | "
                         f"{int((q.success == 0).sum())} |")
    rows = []
    for sc, g in d.groupby("scenario", sort=False):
        for x, y in PAIRS:
            A = g[g.c == x].set_index("seed").sort_index()
            B = g[g.c == y].set_index("seed").sort_index()
            s = A.index.intersection(B.index)
            if not len(s):
                continue
            for m, lab in METRICS:
                diff = (A.loc[s, m] - B.loc[s, m]).values
                rows.append(dict(scenario=sc, pair=f"{x} - {y}", metric=lab, n=len(s), diff=diff.mean(),
                                 rel=100 * diff.mean() / B.loc[s, m].mean(), p=signflip_p(diff)))
    t = pd.DataFrame(rows)
    t["p_holm"] = holm(t.p)
    L += ["", "## Paired differences", "", "| scenario | pair | metric | seeds | difference | relative | p (Holm) |",
          "|---|---|---|---|---|---|---|"]
    for r in t.itertuples():
        star = "**" if r.p_holm < 0.01 else ("*" if r.p_holm < 0.05 else "")
        L.append(f"| {r.scenario} | {r.pair} | {r.metric} | {r.n} | {r.diff:+.3g} | {r.rel:+.0f}% | {r.p_holm:.4f}{star} |")
    L += ["", "Reading: phase choice matters if 'full - sham' is negative (urgent, tilt) and significant; "
              "a dose-response full < sham < worst strengthens it. 'full - sham' ~ 0 means the gain is the delay."]
    out = os.path.join(a.dir, "timing_controls.md")
    open(out, "w").write("\n".join(L) + "\n")
    t.to_csv(os.path.join(a.dir, "timing_controls.csv"), index=False)
    print("\n".join(L))
    print("\nwrote", out)


if __name__ == "__main__":
    main()
