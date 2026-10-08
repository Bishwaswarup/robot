"""Swarm test: ORCA + formation layer driving Go1s through the command-timing strategies.

Configurations are "layer:strategy[:q]":
  layer     reactive   | ORCA on the current situation, change requested immediately
            predictive | ORCA solved for the situation one lookahead ahead (own position
                         extrapolated with the current command); the change is requested with
                         a deadline = lookahead, so timing strategies can pick the gait phase.
                         If the CURRENT command is unsafe (TTC < urgent-ttc) it falls back to
                         reactive + urgent.
  strategy  any commander / limiter strategy (limiter, phase+limiter, phase+gaslc_equal, ...)
  q         bottleneck queueing: near obstacles the robot further back yields
            (ORCA responsibility 0.9 / 0.1 instead of 0.5 / 0.5)

Every robot: local sensing (neighbours within 4 m, measured velocities), ORCA at 10 Hz,
heading control, then  commander.request(vx, vy, wz, urgent)  ->  Bezier controller.
urgent = predicted time-to-collision with the CURRENT command < --urgent-ttc (skip waiting).

Strategies: raw | limiter | phase | phase_safe   (same scenario + seed for all -> paired)
Metrics: success, completion time, robot-robot / robot-obstacle contact onsets, min
centre distance, falls, tilt (mean, p99), and how many command changes were "big", how
many were inside the capability map's domain, how many were urgent.

Run:  python scripts/swarm_eval.py --quick          (~3 min)
      python scripts/swarm_eval.py                  (4 scenarios x 5 configs x 5 seeds)
Out:  results/swarm_eval.csv, results/swarm_summary.txt, results/swarm_paths_*.png
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import argparse
import csv
import itertools
import time
from multiprocessing import Pool

import warnings
import numpy as np
import mujoco
warnings.filterwarnings("ignore", message="Mean of empty slice")
from go1_bezier import load_team, make_arena, Go1BezierController, CapabilityMap, PushEnvelope
from go1_bezier.gasl import make_commander
from go1_bezier.commander import PhaseAwareCommander
from go1_bezier.swarm import SwarmAgent, goal_velocity, elastic_scale, queue_responsibility
from go1_bezier.scenarios import SCENARIOS, R_ROBOT, add_obstacles

ROOT = os.path.join(HERE, "..")
MAP = os.path.join(ROOT, "data", "capability_map_bezier.npz")
ENV = os.path.join(ROOT, "data", "push_envelope_bezier.npz")
STRATS = ("raw", "limiter", "phase", "phase_safe")
SWARM_DT = 0.1


def ttc(p, v, others, R):
    """Smallest time to collision of a disc at p moving with v against (p_j, v_j, r_j)."""
    best = np.inf
    for q, vq, rq in others:
        dp, dv = np.asarray(q) - p, np.asarray(vq) - v
        rr = R + rq
        a, b, c = dv @ dv, 2 * dp @ dv, dp @ dp - rr * rr
        if c < 0:
            return 0.0
        if a < 1e-9:
            continue
        disc = b * b - 4 * a * c
        if disc < 0:
            continue
        t = (-b - np.sqrt(disc)) / (2 * a)
        if t > 0:
            best = min(best, t)
    return best


def parse_config(cfg):
    parts = cfg.split(":")
    if len(parts) == 1:
        return "reactive", parts[0], 0
    return parts[0], parts[1], int(len(parts) > 2 and parts[2] == "q")


def run(job):
    layer, strategy, queue = parse_config(job["config"])
    job = dict(job, strategy=strategy)
    look = job.get("lookahead", 0.36)
    sc = SCENARIOS[job["scenario"]]()
    rng = np.random.default_rng(job["seed"])
    starts = [(x + rng.normal(0, 0.05), y + rng.normal(0, 0.05), yaw) for x, y, yaw in sc["starts"]]
    spec = add_obstacles(make_arena(), sc["obstacles"])
    m, d = load_team(starts, spec=spec, visual_meshes=False)
    N, dt = len(starts), m.opt.timestep
    cmap = CapabilityMap.load(MAP)
    ctrls = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(N)]
    for c in ctrls:
        c.gait.phi = rng.uniform()                        # robots are not phase-locked
    env = PushEnvelope.load(ENV) if os.path.exists(ENV) else None
    cache = {}
    coms = [make_commander(c, job["strategy"], cmap, env, cache) for c in ctrls]
    agents = [SwarmAgent(radius=R_ROBOT) for _ in range(N)]
    trunk = [m.body(f"go1_{i}/trunk").id for i in range(N)]
    root_of = np.array([m.body_rootid[m.geom_bodyid[g]] for g in range(m.ngeom)])
    robot_of_root = {trunk[i]: i for i in range(N)}
    floor = m.geom("floor").id
    for _ in range(250):                                  # settle standing
        for cm_ in coms:
            cm_.step()
        mujoco.mj_step(m, d)
    for c in ctrls:
        c.set_command(0, 0, 0)

    vel = np.zeros((N, 2))
    alpha = dt / 0.15
    fallen = np.zeros(N, bool)
    done_t, tick, n_steps = np.nan, int(round(SWARM_DT / dt)), int(sc["T"] / dt)
    tilts = [[] for _ in range(N)]
    dmin, rr_pairs, ro_pairs = np.inf, set(), set()
    rr_events = ro_events = 0
    paths = [[] for _ in range(N)] if job.get("record") else None
    t0 = d.time
    for k in range(n_steps):
        t = d.time - t0
        pos = np.array([d.xpos[b][:2] for b in trunk])
        if k % tick == 0:
            yaws = [c.yaw() for c in ctrls]
            for i in range(N):
                if fallen[i]:
                    continue
                if sc["plan"] is not None:
                    plan = sc["plan"]
                    lam = elastic_scale(pos[i], plan.heading, sc["obstacles"]) if job.get("elastic", 1) else 1.0
                    target, _ = plan.slot(i, t, lam)
                    heading = plan.heading
                    pref_now = (lambda q, i=i, lam=lam, plan=plan: plan.preferred_velocity(i, q, t, lam))
                    pref_ahead = (lambda q, i=i, lam=lam, plan=plan: plan.preferred_velocity(i, q, t + look, lam))
                    in_bn = lam < 0.6
                else:
                    target = np.asarray(sc["goals"][i])
                    heading = np.arctan2(*(target - pos[i])[::-1])
                    pref_now = pref_ahead = (lambda q, g=target: goal_velocity(q, g))
                    in_bn = False
                far = np.linalg.norm(target - pos[i]) > 1.0
                nbs = [(pos[j], vel[j] if not fallen[j] else np.zeros(2), R_ROBOT) for j in range(N) if j != i]
                if queue:
                    nbs = queue_responsibility(pos[i], heading, nbs, in_bn)
                applied = ctrls[i].cmd_target
                Rz = np.array([[np.cos(yaws[i]), -np.sin(yaws[i])], [np.sin(yaws[i]), np.cos(yaws[i])]])
                v_cmd = Rz @ applied[:2]
                urgent = ttc(pos[i], v_cmd, [n[:3] for n in nbs] + [(q, np.zeros(2), r) for q, r in sc["obstacles"]],
                             R_ROBOT) < job["urgent_ttc"]
                if layer == "predictive" and not urgent:
                    v_w = agents[i].predictive(pos[i], v_cmd, pref_ahead, nbs, sc["obstacles"], look, far)
                    deadline = look
                else:
                    v_w = agents[i].velocity(pos[i], pref_now(pos[i]), nbs, sc["obstacles"], far)
                    deadline = None
                cmd = agents[i].body_command(v_w, yaws[i])
                coms[i].request(*cmd, urgent=bool(urgent), deadline=deadline)
            if paths is not None and k % (5 * tick) == 0:
                for i in range(N):
                    paths[i].append(pos[i].tolist())
            # completion
            if np.isnan(done_t):
                if sc["plan"] is not None:
                    ok = (t * sc["plan"].speed >= sc["plan"].L) and all(
                        fallen[i] or np.linalg.norm(sc["plan"].slot(i, t)[0] - pos[i]) < 0.6 for i in range(N))
                else:
                    ok = all(fallen[i] or np.linalg.norm(np.asarray(sc["goals"][i]) - pos[i]) < 0.4 for i in range(N))
                if ok:
                    done_t = t
            # contacts + distances (10 Hz)
            for i in range(N):
                for j in range(i + 1, N):
                    dmin = min(dmin, np.linalg.norm(pos[i] - pos[j]))
            now_rr, now_ro = set(), set()
            for ci in range(d.ncon):
                g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                if floor in (g1, g2):
                    continue
                r1, r2 = robot_of_root.get(root_of[g1]), robot_of_root.get(root_of[g2])
                if r1 is not None and r2 is not None and r1 != r2:
                    now_rr.add((min(r1, r2), max(r1, r2)))
                elif (r1 is None) != (r2 is None):
                    now_ro.add(r1 if r1 is not None else r2)
            rr_events += len(now_rr - rr_pairs)
            ro_events += len(now_ro - ro_pairs)
            rr_pairs, ro_pairs = now_rr, now_ro
            if not np.isnan(done_t) and t > done_t + 1.0:
                break
        for i in range(N):
            if not fallen[i]:
                coms[i].step()
            else:
                ctrls[i].step()
        mujoco.mj_step(m, d)
        newpos = np.array([d.xpos[b][:2] for b in trunk])
        vel = (1 - alpha) * vel + alpha * (newpos - pos) / dt
        for i in range(N):
            up = d.xmat[trunk[i]][8]
            if k % 5 == 0 and not fallen[i]:
                tilts[i].append(np.degrees(np.arccos(np.clip(up, -1, 1))))
            if not fallen[i] and (d.xpos[trunk[i]][2] < 0.15 or up < 0.5):
                fallen[i] = True
                ctrls[i].set_stand()
    sim_t = d.time - t0
    allt = np.concatenate([np.array(x) for x in tilts if x])
    big = sum(cm_.n_big for cm_ in coms)
    res = dict(job, N=N, success=int(not np.isnan(done_t)), time=round(done_t, 2) if np.isfinite(done_t) else np.nan,
               falls=int(fallen.sum()), rr_contacts=rr_events, ro_contacts=ro_events, min_dist=round(dmin, 3),
               mean_tilt=round(float(allt.mean()), 3), p99_tilt=round(float(np.percentile(allt, 99)), 3),
               max_tilt=round(float(allt.max()), 2),
               big_per_robot_min=round(big / N / (sim_t / 60), 2),
               in_domain_frac=round(sum(cm_.n_in_domain for cm_ in coms) / max(big, 1), 3),
               urgent_frac=round(sum(cm_.n_urgent for cm_ in coms) / max(big, 1), 3),
               mean_wait=round(float(np.mean([w for cm_ in coms for (_, _, w, kk) in cm_.log if kk is not None] or [0])), 4))
    if paths is not None:
        res["paths"] = paths
    return res


def analyse(rows, out):
    lines = []
    P = lambda s="": (print(s), lines.append(s))
    f = lambda r, k: float(r[k]) if r[k] not in ("", "nan", None) else np.nan
    P("=" * 110)
    P("SWARM TEST  (ORCA + formation -> commander -> Bezier trot; mean over seeds)")
    P("=" * 110)
    for sc in [s for s in SCENARIOS if any(r["scenario"] == s for r in rows)]:
        P(f"\n{sc}")
        P(f"  {'config':30s} {'success':>7s} {'time':>7s} {'falls':>5s} {'robot-robot':>11s} {'robot-obst':>10s} "
          f"{'min dist':>8s} {'tilt':>6s} {'p99':>6s} {'max':>6s} | {'big/rob/min':>11s} {'in-domain':>9s} "
          f"{'urgent':>7s} {'wait':>6s}")
        for s in dict.fromkeys(r["config"] for r in rows):
            R = [r for r in rows if r["scenario"] == sc and r["config"] == s]
            if not R:
                continue
            g = lambda k: np.array([f(r, k) for r in R])
            P(f"  {s:30s} {100 * np.mean(g('success')):6.0f}% {np.nanmean(g('time')):6.1f}s {np.sum(g('falls')):5.0f} "
              f"{np.mean(g('rr_contacts')):11.1f} {np.mean(g('ro_contacts')):10.1f} {np.min(g('min_dist')):7.2f}m "
              f"{np.mean(g('mean_tilt')):5.2f}° {np.mean(g('p99_tilt')):5.1f}° {np.max(g('max_tilt')):5.1f}° | "
              f"{np.mean(g('big_per_robot_min')):11.1f} {100 * np.mean(g('in_domain_frac')):8.0f}% "
              f"{100 * np.mean(g('urgent_frac')):6.0f}% {np.mean(g('mean_wait')):5.3f}s")
    P("\n  big = command change >= 0.15 m/s (or 0.3 rad/s); in-domain = like the events the capability map")
    P("  was measured on (settled straight walking, single change <= 0.8); urgent = TTC below threshold.")
    with open(os.path.join(out, "swarm_summary.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


def plot_paths(results, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    for sc in SCENARIOS:
        R = [r for r in results if r["scenario"] == sc and "paths" in r]
        if not R:
            return
        order = list(dict.fromkeys(r["config"] for r in results))
        R = sorted(R, key=lambda r: order.index(r["config"]))
        fig, axes = plt.subplots(1, len(R), figsize=(4.2 * len(R), 4.2), squeeze=False)
        obst = SCENARIOS[sc]()["obstacles"]
        cmap = plt.get_cmap("tab10")
        for ax, r in zip(axes[0], R):
            for i, pth in enumerate(r["paths"]):
                pth = np.array(pth)
                if len(pth):
                    ax.plot(pth[:, 0], pth[:, 1], "-", lw=1.4, color=cmap(i % 10))
                    ax.plot(*pth[0], "o", ms=4, color=cmap(i % 10))
            for (x, y), rad in obst:
                ax.add_patch(plt.Circle((x, y), rad, color="#8a6a4a"))
            ax.set_aspect("equal")
            ax.set_title(f"{r['config']}\nfalls {r['falls']}, contacts {r['rr_contacts']}", fontsize=8)
            ax.grid(alpha=0.2)
        fig.suptitle(f"{sc} - robot paths (seed 0)", fontsize=10)
        fig.tight_layout()
        fn = os.path.join(out, f"swarm_paths_{sc}.png")
        fig.savefig(fn, dpi=110)
        plt.close(fig)
        print("wrote", fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--scenarios", default=",".join(SCENARIOS))
    ap.add_argument("--configs", default="reactive:limiter,reactive:limiter:q,predictive:limiter:q,"
                                         "predictive:phase+limiter:q,predictive:phase+gaslc_equal:q")
    ap.add_argument("--lookahead", type=float, default=0.36, help="s (one gait period)")
    ap.add_argument("--urgent-ttc", type=float, default=0.8)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "swarm_eval_quick.csv" if a.quick else "swarm_eval.csv")
    if a.analyse_only:
        rows = list(csv.DictReader(open(path)))
    else:
        scen = ["swap4", "swap8"] if a.quick else a.scenarios.split(",")
        seeds = 1 if a.quick else a.seeds
        jobs = [dict(scenario=s, config=cf, seed=sd, urgent_ttc=a.urgent_ttc, lookahead=a.lookahead,
                     record=int(sd == 0))
                for s, sd, cf in itertools.product(scen, range(seeds), a.configs.split(","))]
        jobs.sort(key=lambda j: -len(SCENARIOS[j["scenario"]]()["starts"]))   # big runs first
        print(f"{len(jobs)} swarm runs on {a.workers} workers")
        t0, results = time.time(), []
        with Pool(a.workers) as pool:
            for i, r in enumerate(pool.imap_unordered(run, jobs)):
                results.append(r)
                print(f"  {i + 1}/{len(jobs)} {r['scenario']:20s} {r['config']:30s} seed {r['seed']}: "
                      f"success {r['success']} falls {r['falls']} contacts {r['rr_contacts']}  "
                      f"({(time.time() - t0) / 60:.1f} min)", flush=True)
        plot_paths(results, a.out)
        rows = [{k: v for k, v in r.items() if k != "paths"} for r in results]
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
        print("wrote", path)
        rows = [{k: str(v) for k, v in r.items()} for r in rows]
    analyse(rows, a.out)


if __name__ == "__main__":
    main()
