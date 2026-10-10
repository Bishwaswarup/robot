"""Does the capability map still work inside the swarm? Counterfactual replay of real swarm events.

The map was calibrated on isolated command steps from settled, straight walking (held-out R2 0.94-0.98),
but in the swarm only ~1% of large command changes look like that. This experiment checks the map in the
context where it is actually used.

  A  isolated steps     the calibration's held-out numbers (results/capability_summary.txt), for reference
  B  real sequences     full-system swarm runs; at a random sample of the command changes the commander
                        times (non-urgent, large), the whole simulation is snapshotted
  C  counterfactuals    from each snapshot the same change is released after each of 9 candidate waits
                        (0 .. 0.5 gait period, a subset of the commander's own grid) while every robot's
                        logged command stream continues as recorded; the focal robot's peak tilt over
                        the next 1.0 s is measured and compared with the map's prediction

Per event this gives a measured tilt for every candidate phase, so the map is scored both on absolute
error |tilt_pred - tilt_meas| and on what the commander actually uses it for: ranking the phases.
A replay of the commander's own choice must reproduce the original run (fidelity check).

Run:  python scripts/map_validation.py --quick        (~5 min)
      python scripts/map_validation.py                (4 scenarios x 5 seeds, ~30 events per run)
Out:  results/map_validation/{events.csv, options.csv, summary.txt}
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import copy
import time
import warnings
from multiprocessing import Pool

import numpy as np
import pandas as pd
import mujoco

import swarm_eval as SE
warnings.filterwarnings("ignore", message="An input array is constant")
from go1_bezier.commander import PhaseAwareCommander

ROOT = os.path.join(HERE, "..")
FULL = "predictive:phase+gaslc_equal:q"
WINDOW = 1.0          # s: tilt measured from the request over this window (covers 0.18 s wait + response)
EVERY = 4             # candidate waits = every 4th point of the commander's 1/64-period grid -> 9 options
TILT_FLAG = 15.0      # deg: "large tilt" stand-in for fall risk (no falls occur in the full system)


def _tilt(d, b):
    return float(np.degrees(np.arccos(np.clip(d.xmat[b][8], -1, 1))))


def collect_and_replay(job):
    """One full-system run: sample timed events, snapshot, then replay every candidate wait."""
    rng = np.random.default_rng([job["seed"], 99])
    st, log, events, shared = {}, [], [], []
    rel, last_rel = None, None                       # per robot: times at which a command change was released

    def memo():
        return {id(o): o for o in shared}

    def on_setup(m, d, ctrls, coms):
        nonlocal rel, last_rel
        st.update(m=m, d=d, ctrls=ctrls, coms=coms,
                  trunk=[m.body(f"go1_{i}/trunk").id for i in range(len(ctrls))])
        rel, last_rel = [[] for _ in coms], [c.last_release for c in coms]
        shared.extend([m, coms[0].cmap, coms[0].env, ctrls[0].acc_scale_fn])
        for i, com in enumerate(coms):
            def wrapped(vx, vy, wz, urgent=False, deadline=None, i=i, com=com):
                target = np.array([vx, vy, wz], float)
                idx, t = len(log), d.time
                log.append((idx, t, i, target, bool(urgent), deadline))
                if (deadline is not None and len(events) < job["cap"] and rng.random() < job["p"]
                        and com.timed(target, urgent, deadline)):
                    key = com._event_key(target)
                    grid = com.wait_grid(deadline)
                    waits, cost, tilt, fall = com.phase_costs(key, deadline, waits=grid[::EVERY], parts=True)
                    _, cost_all, _, _ = com.phase_costs(key, deadline, parts=True)
                    j_all = 0                                     # the commander's own choice (first optimum)
                    for j, c in enumerate(cost_all):
                        if np.isfinite(c) and c < cost_all[j_all] - 1e-9:
                            j_all = j
                    prof = np.array([com.cmap.predict("cmd_lim", float(np.clip(com.c.cmd_target[0], 0, 0.4)),
                                                      key[0], key[1], ph, "tilt") for ph in np.arange(64) / 64])
                    c, dlt = com.c, target - com.c.cmd_target
                    events.append(dict(
                        idx=idx, t=t, robot=i, target=target, deadline=deadline, key=key,
                        waits=waits, pred_cost=cost, pred_tilt=tilt, pred_fall=fall,
                        chosen_wait=float(grid[j_all]), phase_blind=float(np.nanmean(prof)),
                        phase=float(c.gait.phi), v_fwd=float(c.cmd_target[0]),
                        speed=float(np.hypot(*c.cmd_target[:2])), yaw_rate=float(abs(c.cmd_target[2])),
                        d_planar=float(np.hypot(*dlt[:2])), d_yaw=float(abs(dlt[2]) / 2),
                        settled=bool(np.allclose(c.cmd, c.cmd_target, atol=0.05)),
                        since_release=float(com.t - com.last_release),
                        n_recent=int(sum(1 for tr in rel[i] if t - tr < 0.5)),
                        in_domain=bool(com.in_domain(target)),
                        snap=copy.deepcopy((d, ctrls, coms), memo()), qpos_end=None))
                PhaseAwareCommander.request(com, vx, vy, wz, urgent=urgent, deadline=deadline)
            com.request = wrapped

    def on_step(m, d, k):
        for i, c in enumerate(st["coms"]):
            if c.last_release != last_rel[i]:
                last_rel[i] = c.last_release
                rel[i].append(d.time)
        for ev in events:
            if ev["qpos_end"] is None and d.time >= ev["t"] + WINDOW - 1e-9:
                ev["qpos_end"] = d.qpos.copy()
        return True

    run_job = dict(scenario=job["scenario"], config=FULL, seed=job["seed"], urgent_ttc=0.8, lookahead=0.36,
                   record=0, **SE.ROBUST)
    SE.run(run_job, on_step=on_step, on_setup=on_setup)
    m, trunk = st["m"], st["trunk"]
    dt = m.opt.timestep
    rows, opts = [], []
    for e_id, ev in enumerate(events):
        if ev["qpos_end"] is None:                                   # run ended inside the window
            continue
        later = [r for r in log if r[0] > ev["idx"] and r[1] <= ev["t"] + WINDOW + 1e-9]
        i, b = ev["robot"], trunk[ev["robot"]]
        cand = list(ev["waits"]) + [ev["chosen_wait"]]
        meas, fidelity = [], np.nan
        for j, w in enumerate(cand):
            d2, ctrls2, coms2 = copy.deepcopy(ev["snap"], memo())
            PhaseAwareCommander.request(coms2[i], *ev["target"], urgent=False, deadline=ev["deadline"],
                                        force_wait=w)
            ptr, peak, fell = 0, 0.0, False
            for _ in range(int(round(WINDOW / dt))):
                while ptr < len(later) and later[ptr][1] <= d2.time + 0.5 * dt:
                    _, _, r, tg, urg, dl = later[ptr]
                    PhaseAwareCommander.request(coms2[r], *tg, urgent=urg, deadline=dl)
                    ptr += 1
                for c in coms2:
                    c.step()
                mujoco.mj_step(m, d2)
                tl = _tilt(d2, b)
                peak = max(peak, tl)
                fell = fell or d2.xpos[b][2] < 0.15 or d2.xmat[b][8] < 0.5
            if j == len(cand) - 1:
                fidelity = float(np.abs(d2.qpos - ev["qpos_end"]).max())
            else:
                meas.append((peak, fell))
        meas_t = np.array([x[0] for x in meas])
        jb = int(np.nanargmin(np.where(np.isfinite(ev["pred_cost"]), ev["pred_cost"], np.inf)))
        jw = int(np.nanargmax(np.where(np.isfinite(ev["pred_cost"]), ev["pred_cost"], -np.inf)))
        eid = f"{job['scenario']}/{job['seed']}/{e_id}"
        base = {k: ev[k] for k in ("robot", "t", "phase", "v_fwd", "speed", "yaw_rate", "d_planar", "d_yaw",
                                   "settled", "since_release", "n_recent", "in_domain", "chosen_wait",
                                   "phase_blind")}
        rows.append(dict(event=eid, scenario=job["scenario"], seed=job["seed"], **base,
                         key=str(ev["key"][0]), size=ev["key"][1],
                         tilt_mapbest=meas_t[jb], tilt_mapworst=meas_t[jw], tilt_now=meas_t[0],
                         tilt_random=meas_t.mean(), tilt_oracle=meas_t.min(), tilt_oracle_worst=meas_t.max(),
                         rho=pd.Series(ev["pred_cost"]).corr(pd.Series(meas_t), method="spearman"),
                         fidelity=fidelity, any_fall=any(x[1] for x in meas)))
        for j, (w, (pk, fl)) in enumerate(zip(ev["waits"], meas)):
            opts.append(dict(event=eid, scenario=job["scenario"], option=j, wait=w,
                             pred_tilt=ev["pred_tilt"][j], pred_fall=ev["pred_fall"][j],
                             pred_cost=ev["pred_cost"][j], meas_tilt=pk, fell=fl))
    return rows, opts


# ---------------------------------------------------------------------------------------------- analysis
def _r2(y, yhat):
    y, yhat = np.asarray(y, float), np.asarray(yhat, float)
    ok = np.isfinite(y) & np.isfinite(yhat)
    y, yhat = y[ok], yhat[ok]
    return 1 - np.sum((y - yhat) ** 2) / np.sum((y - y.mean()) ** 2) if len(y) > 2 else np.nan


def _ci(x, n_boot=4000, seed=0):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    bs = np.random.default_rng(seed).choice(x, (n_boot, len(x))).mean(1)
    return x.mean(), np.percentile(bs, 2.5), np.percentile(bs, 97.5)


def _p(x, n_perm=20000, seed=0):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    s = np.random.default_rng(seed).choice([-1.0, 1.0], (n_perm, len(x)))
    return (1 + np.sum(np.abs(s @ x) >= abs(x.sum()) - 1e-9)) / (n_perm + 1)


def analyse(ev, op, out):
    L = []
    P = L.append
    P("CAPABILITY MAP INSIDE THE SWARM - counterfactual replay of real full-system events")
    P(f"{len(ev)} events from {ev.groupby(['scenario', 'seed']).ngroups} runs "
      f"({', '.join(f'{s} {n}' for s, n in ev.scenario.value_counts().items())}); "
      f"9 candidate waits each; tilt = peak over {WINDOW:.1f} s from the request")
    fid = ev.fidelity
    P(f"replay fidelity (commander's own choice vs the original run): max |dq| median {fid.median():.1e}, "
      f"max {fid.max():.1e}  ({(fid < 1e-6).mean():.0%} of events reproduce to 1e-6)")
    ind = ev.in_domain.mean()
    P(f"context: in map domain {ind:.0%}; settled {ev.settled.mean():.0%}; combined planar+yaw "
      f"{((ev.d_planar >= .15) & (ev.d_yaw >= .15)).mean():.0%}; previous release < 1 gait period ago "
      f"{(ev.since_release < 0.36).mean():.0%}; median command updates released in the last 0.5 s {ev.n_recent.median():.0f}")

    P("\nA  ISOLATED STEPS (calibration, from results/capability_summary.txt)")
    cs = os.path.join(ROOT, "results", "capability_summary.txt")
    if os.path.exists(cs):
        for line in open(cs):
            if "held-out repeat, cmd_lim" in line:
                P("   " + line.strip())

    P("\nC1 ABSOLUTE ACCURACY in context (all events x options): |tilt_pred - tilt_meas|")
    e = (op.pred_tilt - op.meas_tilt)
    P(f"   MAE {e.abs().mean():.2f} deg, median {e.abs().median():.2f}, bias (pred - meas) {e.mean():+.2f}; "
      f"R2 {_r2(op.meas_tilt, op.pred_tilt):.2f}  (measured tilt range {op.meas_tilt.min():.1f}-"
      f"{op.meas_tilt.max():.1f} deg)")
    pb = op.merge(ev[["event", "phase_blind"]], on="event")
    P(f"   phase-blind map (mean over phase): R2 {_r2(pb.meas_tilt, pb.phase_blind):.2f}, "
      f"MAE {(pb.phase_blind - pb.meas_tilt).abs().mean():.2f} deg")
    # within-event: does the predicted phase pattern explain the measured phase pattern?
    w = op.assign(pc=op.pred_tilt - op.groupby("event").pred_tilt.transform("mean"),
                  mc=op.meas_tilt - op.groupby("event").meas_tilt.transform("mean"))
    P(f"   within-event (phase effect only): R2 {_r2(w.mc, w.pc):.2f}; measured spread across the 9 waits "
      f"median {(ev.tilt_oracle_worst - ev.tilt_oracle).median():.2f} deg, predicted spread median "
      f"{op.groupby('event').pred_tilt.agg(lambda s: s.max() - s.min()).median():.2f} deg")

    P("\nC2 RANKING (what the commander uses): Spearman rho between predicted cost and measured tilt "
      "over the 9 waits, per event")
    P(f"   median rho {ev.rho.median():+.2f}; rho > 0 in {(ev.rho > 0).mean():.0%} of events; "
      f"mean {ev.rho.mean():+.2f}")

    P("\nC3 DECISION VALUE (measured tilt, deg; paired over events; mean [95% CI], sign-flip p)")
    P(f"   map-best {ev.tilt_mapbest.mean():.2f} | random wait {ev.tilt_random.mean():.2f} | release now "
      f"{ev.tilt_now.mean():.2f} | map-worst {ev.tilt_mapworst.mean():.2f} | oracle {ev.tilt_oracle.mean():.2f}")
    for name, a, b in (("map-best - random", ev.tilt_mapbest, ev.tilt_random),
                       ("map-best - now", ev.tilt_mapbest, ev.tilt_now),
                       ("map-best - map-worst", ev.tilt_mapbest, ev.tilt_mapworst),
                       ("map-best - oracle (regret)", ev.tilt_mapbest, ev.tilt_oracle)):
        mu, lo, hi = _ci(a - b)
        P(f"   {name:28s} {mu:+.2f} [{lo:+.2f}, {hi:+.2f}]  p {_p(a - b):.4f}")
    capt = (ev.tilt_random - ev.tilt_mapbest).sum() / max((ev.tilt_random - ev.tilt_oracle).sum(), 1e-9)
    P(f"   share of the achievable gain (random -> oracle) captured by the map: {capt:.0%}")

    P(f"\nC4 LARGE-TILT / FALL STAND-IN: options with peak tilt > {TILT_FLAG:.0f} deg: "
      f"{(op.meas_tilt > TILT_FLAG).mean():.1%}; falls in replays: {int(op.fell.sum())} of {len(op)} "
      f"(predicted fall prob. mean {op.pred_fall.mean():.3f})")

    P("\nC5 BY CONTEXT (where does the map stop being useful?)")
    P(f"   {'stratum':34s} {'n':>4s} {'MAE':>5s} {'R2':>5s} {'rho':>5s} {'best-rand':>9s} {'[95% CI]':>16s}")
    strata = [("all", ev.index == ev.index)]
    strata += [(f"in map domain = {v}", ev.in_domain == v) for v in (True, False)]
    strata += [("settled", ev.settled), ("not settled", ~ev.settled)]
    strata += [("prev. release < 0.36 s ago", ev.since_release < 0.36),
               ("prev. release >= 0.36 s ago", ev.since_release >= 0.36)]
    comb = (ev.d_planar >= .15) & (ev.d_yaw >= .15)
    strata += [("combined planar + yaw change", comb), ("single-axis change", ~comb)]
    for lo_, hi_ in ((0, 0.2), (0.2, 0.4), (0.4, 9)):
        strata.append((f"speed {lo_:.1f}-{hi_:.1f} m/s" if hi_ < 9 else f"speed >= {lo_:.1f} m/s",
                       (ev.speed >= lo_) & (ev.speed < hi_)))
    for lo_, hi_ in ((0.2, 0.3), (0.3, 0.5), (0.5, 9)):
        strata.append((f"change size {lo_:.1f}-{hi_:.1f}" if hi_ < 9 else f"change size >= {lo_:.1f}",
                       (ev["size"] >= lo_) & (ev["size"] < hi_)))
    strata += [(f"scenario {s}", ev.scenario == s) for s in sorted(ev.scenario.unique())]
    for name, mask in strata:
        sub = ev[mask]
        if len(sub) < 5:
            P(f"   {name:34s} {len(sub):4d}  (too few)")
            continue
        so = op[op.event.isin(sub.event)]
        mu, lo, hi = _ci(sub.tilt_mapbest - sub.tilt_random)
        P(f"   {name:34s} {len(sub):4d} {(so.pred_tilt - so.meas_tilt).abs().mean():5.2f} "
          f"{_r2(so.meas_tilt, so.pred_tilt):5.2f} {sub.rho.median():+5.2f} {mu:+9.2f} [{lo:+6.2f}, {hi:+6.2f}]")
    P("\nReading: the map supports phase timing in context if rho is clearly > 0 and map-best < random "
      "(CI below 0). If\nmap-best ~ random, the swarm-level gain comes from the delay/shaping, not from the phase choice.")
    txt = "\n".join(L)
    open(os.path.join(out, "summary.txt"), "w").write(txt + "\n")
    print(txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--scenarios", default="swap8,formation9_pillars,formation6_gate,swap16")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--cap", type=int, default=30, help="max events per run")
    ap.add_argument("--p", type=float, default=0.25, help="probability of sampling a timed event")
    ap.add_argument("--workers", type=int, default=os.cpu_count())
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "map_validation"))
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()
    out = a.out + ("_quick" if a.quick else "")
    os.makedirs(out, exist_ok=True)
    if not a.analyse_only:
        scen = a.scenarios.split(",")
        seeds, cap = (1, 8) if a.quick else (a.seeds, a.cap)
        if a.quick:
            scen = [s for s in scen if s != "swap16"]
        jobs = [dict(scenario=s, seed=sd, cap=cap, p=a.p) for s in scen for sd in range(seeds)]
        t0 = time.time()
        rows, opts = [], []
        with Pool(min(a.workers, len(jobs))) as pool:
            for r, o in pool.imap_unordered(collect_and_replay, jobs):
                rows += r
                opts += o
                print(f"  {len(rows)} events  ({time.time() - t0:.0f} s)", flush=True)
        pd.DataFrame(rows).to_csv(os.path.join(out, "events.csv"), index=False)
        pd.DataFrame(opts).to_csv(os.path.join(out, "options.csv"), index=False)
    analyse(pd.read_csv(os.path.join(out, "events.csv")), pd.read_csv(os.path.join(out, "options.csv")), out)


if __name__ == "__main__":
    main()
