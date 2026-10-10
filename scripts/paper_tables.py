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
import itertools
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
BASE, FULL = "reactive:limiter", "predictive:phase+gaslc_equal:q"
PRIMARY = [("urgent_per_robot_min", "urgent events / robot-min", -1),
           ("p99_tilt", "p99 tilt (deg)", -1),
           ("min_dist", "closest robot distance (m)", +1)]
ORDER = ["swap4", "swap8", "swap16", "swap32", "formation9_pillars", "formation6_gate", "formation16_pillars"]


def signflip_p(x, n_perm=20000, seed=0, max_exact=22):
    """Two-sided paired sign-flip permutation p-value for the mean difference.

    Exact (all 2^n sign patterns) for n <= max_exact, which covers every test in the paper (10 or 20
    seeds); otherwise Monte Carlo with the observed pattern counted, p = (b + 1) / (n_perm + 1).
    Zero differences are kept: flipping them changes nothing, so the result equals the test on the
    non-zero differences. Ties with the observed |sum| count as at least as extreme (conservative).
    With n non-zero differences the smallest attainable p is 2 / 2^n (two-sided)."""
    x = np.asarray(x, float)
    if np.allclose(x, 0):
        return 1.0
    obs, tol = abs(x.sum()), 1e-9 * max(1.0, abs(x).sum())
    n = len(x)
    if n <= max_exact:
        k = min(n, 16)                                 # enumerate 2^k at once, loop over the rest
        lo = np.array(np.meshgrid(*[[-1.0, 1.0]] * k)).reshape(k, -1).T @ x[n - k:]
        hits = 0
        for hs in itertools.product([-1.0, 1.0], repeat=n - k):
            hits += int(np.sum(np.abs(lo + np.dot(hs, x[:n - k])) >= obs - tol))
        return hits / 2.0 ** n
    signs = np.random.default_rng(seed).choice([-1.0, 1.0], size=(n_perm, n))
    return float((1 + np.sum(np.abs(signs @ x) >= obs - tol)) / (n_perm + 1))


def holm(p):
    p = np.asarray(p, float)
    order, m, out, run = np.argsort(p), len(p), np.empty(len(p)), 0.0
    for k, i in enumerate(order):
        run = max(run, (m - k) * p[i])
        out[i] = min(1.0, run)
    return out


def family(d, pairs, scenarios):
    """Paired tests of config x against config y (same seed) on the primary metrics, Holm within the family."""
    rows = []
    for sc in scenarios:
        g = d[d.scenario == sc]
        for (x, xl), (y, yl) in pairs:
            A = g[g.config == x].set_index("seed").sort_index()
            B = g[g.config == y].set_index("seed").sort_index()
            seeds = A.index.intersection(B.index)
            if not len(seeds):
                continue
            for m, label, good in PRIMARY:
                diff = (A.loc[seeds, m] - B.loc[seeds, m]).values
                rows.append(dict(scenario=sc, pair=f"{xl} - {yl}", metric=label, seeds=len(seeds),
                                 x=A.loc[seeds, m].mean(), y=B.loc[seeds, m].mean(), diff=diff.mean(),
                                 rel=100 * diff.mean() / B.loc[seeds, m].mean(), p=signflip_p(diff),
                                 better=bool(np.sign(diff.mean()) == good)))
    t = pd.DataFrame(rows)
    t["p_holm"] = holm(t.p)
    return t


def pooled(d, x, y, scenarios, metric):
    """Exploratory: per-seed relative differences pooled over scenarios (Monte Carlo sign-flip)."""
    r = []
    for sc in scenarios:
        g = d[d.scenario == sc]
        A = g[g.config == x].set_index("seed").sort_index()[metric]
        B = g[g.config == y].set_index("seed").sort_index()[metric]
        s = A.index.intersection(B.index)
        r += list(((A[s] - B[s]) / B[s].mean()).values)
    r = np.asarray(r)
    return 100 * r.mean(), signflip_p(r), len(r)


def family_md(t, title):
    L = ["", f"## {title}", "", f"Holm over the {len(t)} tests of this family.", "",
         "| scenario | pair | metric | seeds | x | y | change | p (Holm) |", "|---|---|---|---|---|---|---|---|"]
    for r in t.itertuples():
        star = "**" if r.p_holm < 0.01 else ("*" if r.p_holm < 0.05 else "")
        L.append(f"| {r.scenario} | {r.pair} | {r.metric} | {r.seeds} | {r.x:.3g} | {r.y:.3g} | {r.rel:+.0f}% | "
                 f"{r.p_holm:.4f}{star} |")
    return L


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
         f"Full = `{FULL}`, baseline = `{BASE}`; paired by seed; exact two-sided sign-flip permutation p; "
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
    # (B) contribution of phase timing: prediction + timing vs prediction alone (main suite)
    MAIN4 = ["swap4", "swap8", "formation9_pillars", "formation6_gate"]
    main_ = d[d.suite == "main"]
    tb = family(main_, [(("predictive:phase+limiter", "pred+timing"), ("predictive:limiter", "pred"))], MAIN4)
    tb.to_csv(os.path.join(P, "family_B_timing.csv"), index=False)
    L += family_md(tb, "Family B: prediction + timing vs prediction alone (main suite)")
    # (C) timing controls: is it the phase or the wait?
    tf = os.path.join(P, "timing" + a.suffix, "swarm_eval.csv")
    if os.path.exists(tf):
        q = pd.read_csv(tf)
        F, S, W = FULL, FULL.replace("phase+", "sham+"), FULL.replace("phase+", "worst+")
        tc = family(q, [((F, "full"), (S, "sham")), ((F, "full"), (W, "worst"))], MAIN4)
        tc.to_csv(os.path.join(P, "family_C_controls.csv"), index=False)
        L += family_md(tc, "Family C: full system vs random wait (sham) and worst phase (timing suite)")
        L += ["", "Exploratory, pooled over seeds and scenarios (relative per-seed differences; not part of a "
                  "corrected family):", "", "| pair | scenarios | metric | change | p | n |", "|---|---|---|---|---|---|"]
        for (x, xl), (y, yl) in [((F, "full"), (S, "sham")), ((F, "full"), (W, "worst")), ((S, "sham"), (W, "worst")),
                                 ((S, "sham"), (BASE, "base"))]:
            for scs, lab in ((MAIN4, "all 4"), (MAIN4[1:], "crowded 3")):
                for m, ml, _ in PRIMARY:
                    mu, pp, n = pooled(q, x, y, scs, m)
                    L.append(f"| {xl} - {yl} | {lab} | {ml} | {mu:+.1f}% | {pp:.4f} | {n} |")
        L += ["", "| scenario | config | mean hold (s) |", "|---|---|---|"]
        for (sc, cfg), g in q.groupby(["scenario", "config"], sort=False):
            L.append(f"| {sc} | {cfg} | {g.mean_wait.mean():.3f} |")
    # contacts (onsets) and falls of the full system and the baseline over every final suite
    L += ["", "## Contacts and falls, full system vs baseline (all final suites + course)", "",
          "| config | runs | falls | robot-robot contact onsets | robot-obstacle contact onsets |", "|---|---|---|---|---|"]
    tot = {}
    for suite in ("main", "scaling", "lookahead"):
        f = os.path.join(P, suite + a.suffix, "swarm_eval.csv")
        if os.path.exists(f):
            q = pd.read_csv(f)
            for key, sel in (("full", q.config.str.endswith("phase+gaslc_equal:q")), ("baseline", q.config == BASE)):
                g = q[sel]
                t_ = tot.setdefault(key, [0, 0, 0, 0])
                t_[0] += len(g); t_[1] += int(g.falls.sum()); t_[2] += int(g.rr_contacts.sum()); t_[3] += int(g.ro_contacts.sum())
    if os.path.exists(a.course):
        q = pd.read_csv(a.course)
        for key, cfg in (("full", FULL), ("baseline", BASE)):
            g = q[q.config == cfg]
            t_ = tot.setdefault(key, [0, 0, 0, 0])
            t_[0] += len(g); t_[1] += int(g.falls.sum()); t_[2] += int(g.rr_contacts.sum())
            t_[3] += int(g.obstacle_contacts.sum() + g.mover_contacts.sum())
    for key, (n, fa, rr, ro) in tot.items():
        L.append(f"| {key} | {n} | {fa} | {rr} | {ro} |")
    out = os.path.join(P, "paper_tables.md")
    open(out, "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    print("\nwrote", out)


if __name__ == "__main__":
    main()
