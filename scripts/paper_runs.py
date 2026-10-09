"""All swarm experiments for the paper, in three suites (each with paired statistics).

  main       4 scenarios x 6 configs x 20 seeds  - ablation of prediction / timing / shaping / queueing
  lookahead  swap8 + gate, lookahead 0.18 / 0.36 / 0.54 / 0.72 s vs baseline, 10 seeds
  scaling    16 and 32 robots (swap16, swap32, formation16_pillars), 3 configs, 10 seeds
  obstacles  obstacle scenarios again WITH static-obstacle safety (0.1 m margin, tau_obs 2 s,
             urgent if TTC to an obstacle < 1.5 s) for every config, 15 seeds

Run:  python scripts/paper_runs.py --quick                 (~10-15 min, 2 seeds, small scaling)
      python scripts/paper_runs.py                         (all suites; long - overnight)
      python scripts/paper_runs.py --suites main           (one suite)
      python scripts/paper_runs.py --suites obstacles      (obstacle-safety re-run)
      python scripts/paper_runs.py --suites obstacles --turn-gate   (+ body-command envelope)
      python scripts/paper_runs.py --analyse-only          (re-make summaries + stats)
Out:  results/paper/<suite>/swarm_eval.csv, swarm_summary.txt, swarm_stats.txt, swarm_paths_*.png
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import csv
import itertools
import time
from multiprocessing import Pool

import swarm_eval as SE
import swarm_stats as ST

ROOT = os.path.join(HERE, "..")
BASE = "reactive:limiter"
SUITES = {
    "main": dict(
        scenarios=["swap4", "swap8", "formation9_pillars", "formation6_gate"],
        configs=[BASE,                                   # baseline (today)
                 "reactive:limiter:q",                   # queueing only
                 "predictive:limiter",                   # prediction only
                 "predictive:phase+limiter",             # prediction + phase timing
                 "predictive:phase+gaslc_equal",         # + gait-aware shaped limiter
                 "predictive:phase+gaslc_equal:q"],      # full system
        seeds=20, quick_seeds=2),
    "lookahead": dict(
        scenarios=["swap8", "formation6_gate"],
        configs=[BASE] + [f"predictive@{L}:phase+gaslc_equal:q" for L in (0.18, 0.36, 0.54, 0.72)],
        seeds=10, quick_seeds=2),
    "scaling": dict(
        scenarios=["swap16", "swap32", "formation16_pillars"],
        configs=[BASE, "predictive:limiter:q", "predictive:phase+gaslc_equal:q"],
        seeds=10, quick_seeds=1, quick_scenarios=["swap16"]),
    "obstacles": dict(
        scenarios=["formation9_pillars", "formation6_gate", "formation16_pillars"],
        configs=[BASE, "predictive:limiter:q", "predictive:phase+limiter:q", "predictive:phase+gaslc_equal:q"],
        seeds=15, quick_seeds=1, quick_scenarios=["formation6_gate"], job=SE.STATIC_SAFETY),
}


def suite_dir(name, quick, turn_gate):
    return os.path.join(ROOT, "results", "paper", name + ("_gate" if turn_gate else "") + ("_quick" if quick else ""))


def run_suite(name, spec, quick, workers, urgent_ttc, turn_gate=False):
    out = suite_dir(name, quick, turn_gate)
    os.makedirs(out, exist_ok=True)
    scen = spec.get("quick_scenarios", spec["scenarios"]) if quick else spec["scenarios"]
    seeds = spec["quick_seeds"] if quick else spec["seeds"]
    jobs = [dict(scenario=s, config=c, seed=sd, urgent_ttc=urgent_ttc, lookahead=0.36, record=int(sd == 0),
                 turn_gate=int(turn_gate), **spec.get("job", {}))
            for s, sd, c in itertools.product(scen, range(seeds), spec["configs"])]
    jobs.sort(key=lambda j: -len(SE.SCENARIOS[j["scenario"]]()["starts"]))
    print(f"\n[{name}] {len(jobs)} runs on {workers} workers -> {os.path.relpath(out, ROOT)}")
    t0, results = time.time(), []
    with Pool(workers) as pool:
        for i, r in enumerate(pool.imap_unordered(SE.run, jobs)):
            results.append(r)
            el = time.time() - t0
            print(f"  {i + 1}/{len(jobs)} {r['scenario']:20s} {r['config']:36s} seed {r['seed']:2d}: "
                  f"ok {r['success']} falls {r['falls']} contacts {r['rr_contacts']}  "
                  f"({el / 60:.1f} min, ~{el / (i + 1) * (len(jobs) - i - 1) / 60:.0f} min left)", flush=True)
    SE.plot_paths(results, out)
    rows = [{k: v for k, v in r.items() if k != "paths"} for r in results]
    path = os.path.join(out, "swarm_eval.csv")
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return out


def analyse(out):
    path = os.path.join(out, "swarm_eval.csv")
    rows = list(csv.DictReader(open(path)))
    SE.analyse(rows, out)
    ST.analyse(path, BASE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--suites", default="main,lookahead,scaling",
                    help="comma list of: " + ",".join(SUITES))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--urgent-ttc", type=float, default=0.8)
    ap.add_argument("--turn-gate", action="store_true",
                    help="body-command envelope for every robot (|wz|<=1, |vy|<=0.3, slow while turning); "
                         "results go to results/paper/<suite>_gate")
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()
    for name in a.suites.split(","):
        out = suite_dir(name, a.quick, a.turn_gate)
        if not a.analyse_only:
            out = run_suite(name, SUITES[name], a.quick, a.workers, a.urgent_ttc, a.turn_gate)
        analyse(out)


if __name__ == "__main__":
    main()
