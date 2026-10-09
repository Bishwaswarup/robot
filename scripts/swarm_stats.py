"""Paired statistics for swarm runs (same scenario + seed = same initial conditions).

For each scenario and each config vs. the baseline config, per metric:
  mean paired difference, 95% bootstrap CI, and a two-sided sign-flip permutation p-value
  (exact-ish, no scipy needed). Success: number of seeds where only one of the two succeeded.

Run:  python scripts/swarm_stats.py results/paper/main/swarm_eval.csv [--baseline reactive:limiter]
Out:  <same folder>/swarm_stats.txt
"""
import argparse
import csv
import os
import numpy as np

# (metric, better direction: +1 higher is better, -1 lower is better, unit)
METRICS = [("min_dist", +1, "m"), ("urgent_frac", -1, ""), ("rr_contacts", -1, ""), ("ro_contacts", -1, ""), ("falls", -1, ""),
           ("p99_tilt", -1, "deg"), ("max_tilt", -1, "deg"), ("urgent_per_robot_min", -1, "/min"),
           ("form_err", -1, "m"), ("lag_mean", -1, "m"), ("arrive_mean", -1, "s"), ("time", -1, "s")]


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return np.nan


def paired_test(diff, n_boot=4000, n_perm=20000, seed=0):
    diff = np.asarray(diff, float)
    diff = diff[np.isfinite(diff)]
    if len(diff) < 2:
        return np.nan, (np.nan, np.nan), np.nan, len(diff)
    rng = np.random.default_rng(seed)
    boot = rng.choice(diff, (n_boot, len(diff))).mean(1)
    obs = abs(diff.mean())
    signs = rng.choice([-1.0, 1.0], (n_perm, len(diff)))
    p = (1 + np.sum(np.abs((signs * diff).mean(1)) >= obs - 1e-12)) / (n_perm + 1)
    return diff.mean(), (np.percentile(boot, 2.5), np.percentile(boot, 97.5)), p, len(diff)


def analyse(csv_path, baseline="reactive:limiter", out=None):
    rows = list(csv.DictReader(open(csv_path)))
    lines = []
    P = lambda s="": (print(s), lines.append(s))
    P("=" * 100)
    P(f"PAIRED STATISTICS vs baseline '{baseline}'  (pairs = same scenario + seed)")
    P("  diff = config - baseline;  95% bootstrap CI;  p = sign-flip permutation test (two-sided)")
    P("  * p < 0.05   ** p < 0.01   (no multiple-comparison correction applied - see Holm note)")
    P("=" * 100)
    for sc in dict.fromkeys(r["scenario"] for r in rows):
        R = [r for r in rows if r["scenario"] == sc]
        base = {r["seed"]: r for r in R if r["config"] == baseline}
        if not base:
            continue
        P(f"\n{sc}  (n seeds = {len(base)})")
        for cfg in dict.fromkeys(r["config"] for r in R):
            if cfg == baseline:
                continue
            other = {r["seed"]: r for r in R if r["config"] == cfg}
            seeds = [s for s in base if s in other]
            sb = sum(int(_f(base[s]["success"]) == 1) for s in seeds)
            so = sum(int(_f(other[s]["success"]) == 1) for s in seeds)
            P(f"  {cfg}   success {so}/{len(seeds)} vs {sb}/{len(seeds)}")
            for m, better, unit in METRICS:
                if m not in R[0]:
                    continue
                diff = [_f(other[s][m]) - _f(base[s][m]) for s in seeds]
                mean, (lo, hi), p, n = paired_test(diff)
                if not np.isfinite(mean):
                    continue
                star = "**" if p < 0.01 else ("*" if p < 0.05 else "  ")
                good = "better" if mean * better > 0 else ("worse" if mean * better < 0 else "same")
                P(f"     {m:22s} {mean:+8.3f} {unit:4s} CI [{lo:+.3f}, {hi:+.3f}]  p={p:.4f} {star} ({good})")
    P("\nHolm note: with many metrics/configs, report the key pre-registered comparisons")
    P("(urgent/min, min_dist, p99_tilt, success) and correct those, or treat the rest as exploratory.")
    out = out or os.path.join(os.path.dirname(csv_path), "swarm_stats.txt")
    with open(out, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("wrote", out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--baseline", default="reactive:limiter")
    a = ap.parse_args()
    analyse(a.csv, a.baseline)
