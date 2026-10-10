"""Obstacle course: one robot and a formation, reactive vs. predictive control.

Course (go1_bezier/course.py): weaving pole slalom -> gate (1.9 m) -> chicane -> two timed
moving obstacles -> pillar field -> finish, ~55 m path, side walls. The team follows a waypoint path
(reference speed 0.3 m/s); every robot runs the same local stack as in swarm_eval:
formation slot (curvilinear along the path; the team narrows to single file where the known
course is narrow) -> ORCA (static discs + the moving obstacle + neighbours) -> heading
control -> commander -> Bezier trot. The moving obstacle is a neighbour with known velocity
that takes no responsibility (the robots do all the avoiding).

Configs "layer[@lookahead]:strategy[:q]" as in swarm_eval.py.
Metrics: success, total time, per-section times, falls, contacts (robot-robot, robot-obstacle,
robot-mover), min distances, tilt, formation error, lag, urgent changes per robot-minute.

Run:  python scripts/course_eval.py --quick          (1 seed, ~5-10 min)
      python scripts/course_eval.py --seeds 20 --speed 0.45   (paper version)
      python scripts/course_eval.py --speed 0.45     (faster reference: harder)
Out:  results/course/course_eval.csv, course_summary.txt, course_paths.png
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
import warnings
from multiprocessing import Pool

import numpy as np
import mujoco
from go1_bezier import load_team, make_arena, Go1BezierController, CapabilityMap, PushEnvelope
from go1_bezier.gasl import make_commander
from go1_bezier.swarm import lag_priority, SwarmAgent, elastic_scale, queue_responsibility
from go1_bezier.course import obstacle_course, add_course_geoms
from go1_bezier.scenarios import R_ROBOT
from swarm_eval import ttc, parse_config, parse_lookahead, MAP, ENV

warnings.filterwarnings("ignore", message="Mean of empty slice")
ROOT = os.path.join(HERE, "..")
SWARM_DT = 0.1


def run(job, on_step=None):
    """One course run. on_step(m, d, k, positions) is called after every physics step (used by
    examples/course_demo.py for the viewer); returning False stops the run."""
    layer, strategy, queue = parse_config(job["config"])
    look = parse_lookahead(job["config"], 0.36)
    course = obstacle_course(job["team"], speed=job.get("speed", 0.3))
    plan, static, movers = course["plan"], course["obstacles"], course["movers"]
    static_orca = [(q, r + 0.1) for q, r in static]          # 10 cm safety margin for tracking lag
    rng = np.random.default_rng(job["seed"])
    starts = [(x + rng.normal(0, 0.05), y + rng.normal(0, 0.05), yaw) for x, y, yaw in course["starts"]]
    spec = make_arena()
    mover_names = add_course_geoms(spec, course)
    m, d = load_team(starts, spec=spec, visual_meshes=on_step is not None)
    N, dt = len(starts), m.opt.timestep
    mocap_ids = [m.body_mocapid[m.body(nm).id] for nm in mover_names]
    mover_bodies = {m.body(nm).id: k for k, nm in enumerate(mover_names)}
    cmap = CapabilityMap.load(MAP)
    env = PushEnvelope.load(ENV) if os.path.exists(ENV) else None
    ctrls = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(N)]
    for c in ctrls:
        c.gait.phi = rng.uniform()
    cache = {}
    coms = [make_commander(c, strategy, cmap, env, cache) for c in ctrls]
    agents = [SwarmAgent(radius=R_ROBOT, tau_obs=job.get("tau_obs", 2.0), turn_gate=bool(job.get("turn_gate", 0)),
                         escape=bool(job.get("escape", 0)))
              for _ in range(N)]
    trunk = [m.body(f"go1_{i}/trunk").id for i in range(N)]
    root_of = np.array([m.body_rootid[m.geom_bodyid[g]] for g in range(m.ngeom)])
    robot_of_root = {trunk[i]: i for i in range(N)}
    floor = m.geom("floor").id

    def set_movers(t):
        for k, mv in enumerate(movers):
            p, _ = mv.state(t)
            d.mocap_pos[mocap_ids[k]] = [p[0], p[1], 0.4]

    set_movers(0.0)
    for _ in range(250):
        for cm_ in coms:
            cm_.step()
        mujoco.mj_step(m, d)
    for c in ctrls:
        c.set_command(0, 0, 0)

    vel = np.zeros((N, 2)); alpha = dt / 0.15
    s_track = np.array([plan.progress_window(d.xpos[b][:2], -2.0, 2.0) for b in trunk])
    fallen = np.zeros(N, bool)
    tick, n_steps = int(round(SWARM_DT / dt)), int(course["T"] / dt)
    tilts = [[] for _ in range(N)]
    dmin_rr = dmin_obs = dmin_mover = np.inf
    rr_pairs, ro_set, rm_set = set(), set(), set()
    rr_ev = ro_ev = rm_ev = 0
    form_err, lag = [], []
    sec_t = {name: [np.nan, np.nan] for name, _, _ in course["sections"]}
    done_t = np.nan
    paths = [[] for _ in range(N)] if job.get("record") else None
    mover_path = [] if job.get("record") else None
    fall_log = []                                # where/when robots fell, and what they touched just before
    last_obs, last_rob = np.full(N, -1e9), np.full(N, -1e9)
    t0 = d.time
    for k in range(n_steps):
        t = d.time - t0
        set_movers(t)
        pos = np.array([d.xpos[b][:2] for b in trunk])
        if k % tick == 0:
            mstate = [mv.state(t) for mv in movers]
            mover_nbs = [(p, v, mv.r + 0.15, 1.0) for (p, v), mv in zip(mstate, movers)]   # safety margin
            dyn_obs = static + [(p, mv.r) for (p, _), mv in zip(mstate, movers)]
            heading = plan.heading_at(t)
            yaws = [c.yaw() for c in ctrls]
            if job.get("escape"):                         # slot errors for lag priority
                lam_all = plan.team_scale(t) if N > 1 else 1.0
                errs = np.array([np.linalg.norm(plan.slot(j, t, lam_all)[0] - pos[j]) for j in range(N)])
                errs[fallen] = 0.7                       # neutral: fallen robots cannot yield
            for i in range(N):
                if fallen[i]:
                    continue
                lam = plan.team_scale(t) if N > 1 else 1.0       # shared, from the known course geometry
                target, _ = plan.slot(i, t, lam)
                s_track[i] = plan.progress_window(pos[i], s_track[i] - 0.5, s_track[i] + 1.5)
                pref_now = (lambda q, i=i, lam=lam: plan.preferred_velocity_path(i, q, t, s_track[i], lam))
                pref_ahead = (lambda q, i=i, lam=lam: plan.preferred_velocity_path(
                    i, q, t + look, plan.progress_window(q, s_track[i] - 0.5, s_track[i] + 1.5), lam))
                far = np.linalg.norm(target - pos[i]) > 1.0
                nbs = [(pos[j], vel[j] if not fallen[j] else np.zeros(2), R_ROBOT) for j in range(N) if j != i]
                if queue:
                    nbs = queue_responsibility(pos[i], plan.heading_s(plan.progress(pos[i])), nbs, lam < 0.6)
                if job.get("escape"):
                    nbs = lag_priority(errs[i], [errs[j] for j in range(N) if j != i], nbs)
                nbs = nbs + mover_nbs
                Rz = np.array([[np.cos(yaws[i]), -np.sin(yaws[i])], [np.sin(yaws[i]), np.cos(yaws[i])]])
                v_cmd = Rz @ ctrls[i].cmd_target[:2]
                # urgent if the CURRENT command hits a robot/mover within urgent_ttc, or a static
                # obstacle (with its safety margin) within static_ttc: static obstacles need the
                # longer horizon because the commanded velocity is slew-limited (ramp, not jump)
                urgent = (ttc(pos[i], v_cmd, [n[:3] for n in nbs], R_ROBOT) < job["urgent_ttc"] or
                          ttc(pos[i], v_cmd, [(q, np.zeros(2), r) for q, r in static_orca], R_ROBOT)
                          < job.get("static_ttc", 1.5))
                agents[i].escape_to = plan.point(s_track[i] + 1.5)     # back onto the path ahead
                if layer == "predictive" and not urgent:
                    v_w = agents[i].predictive(pos[i], v_cmd, pref_ahead, nbs, static_orca, look, far)
                    deadline = look
                else:
                    v_w = agents[i].velocity(pos[i], pref_now(pos[i]), nbs, static_orca, far)
                    deadline = None
                coms[i].request(*agents[i].body_command(v_w, yaws[i]), urgent=bool(urgent), deadline=deadline)
            alive = [i for i in range(N) if not fallen[i]]
            if alive:
                if N > 1:
                    lam_t = plan.team_scale(t)
                    form_err.append(float(np.mean([np.linalg.norm(plan.slot(i, t, lam_t)[0] - pos[i]) for i in alive])))
                lag.append(plan.lag(pos[alive], t, alive, plan.team_scale(t) if N > 1 else 1.0))
                x_first, x_last = float(np.max(pos[alive, 0])), float(np.min(pos[alive, 0]))
                for name, x0, x1 in course["sections"]:          # first robot in -> last robot out
                    if np.isnan(sec_t[name][0]) and x_first >= x0:
                        sec_t[name][0] = t
                    if np.isnan(sec_t[name][1]) and x_last >= x1:
                        sec_t[name][1] = t
            # distances, contacts
            for i in alive:
                for j in alive:
                    if j > i:
                        dmin_rr = min(dmin_rr, np.linalg.norm(pos[i] - pos[j]))
                for (q, r) in static:
                    if abs(q[0] - pos[i][0]) < 2 and abs(q[1] - pos[i][1]) < 2:
                        dmin_obs = min(dmin_obs, np.linalg.norm(np.asarray(q) - pos[i]) - r)
                for (p, _), mv in zip(mstate, movers):
                    dmin_mover = min(dmin_mover, np.linalg.norm(p - pos[i]) - mv.r)
            now_rr, now_ro, now_rm = set(), set(), set()
            for ci in range(d.ncon):
                g1, g2 = d.contact[ci].geom1, d.contact[ci].geom2
                if floor in (g1, g2):
                    continue
                r1, r2 = robot_of_root.get(root_of[g1]), robot_of_root.get(root_of[g2])
                if r1 is not None and r2 is not None and r1 != r2:
                    now_rr.add((min(r1, r2), max(r1, r2)))
                elif (r1 is None) != (r2 is None):
                    rob = r1 if r1 is not None else r2
                    other = root_of[g2] if r1 is not None else root_of[g1]
                    (now_rm if other in mover_bodies else now_ro).add(rob)
            rr_ev += len(now_rr - rr_pairs); ro_ev += len(now_ro - ro_set); rm_ev += len(now_rm - rm_set)
            rr_pairs, ro_set, rm_set = now_rr, now_ro, now_rm
            for r_ in now_ro | now_rm:
                last_obs[r_] = t
            for a_, b_ in now_rr:
                last_rob[a_] = last_rob[b_] = t
            if paths is not None and k % (5 * tick) == 0:
                for i in range(N):
                    paths[i].append(pos[i].tolist())
                mover_path.append([ms[0].tolist() for ms in mstate])
            lam_end = plan.team_scale(t) if N > 1 else 1.0
            if np.isnan(done_t) and plan.s_of(t) >= plan.L and all(
                    fallen[i] or np.linalg.norm(plan.slot(i, t, lam_end)[0] - pos[i]) < 0.6 for i in range(N)):
                done_t = t
            if np.isfinite(done_t) and t > done_t + 1.0:
                break
        for i in range(N):
            (coms[i].step() if not fallen[i] else ctrls[i].step())
        mujoco.mj_step(m, d)
        newpos = np.array([d.xpos[b][:2] for b in trunk])
        vel = (1 - alpha) * vel + alpha * (newpos - pos) / dt
        if on_step is not None and on_step(m, d, k, newpos) is False:
            break
        for i in range(N):
            up = d.xmat[trunk[i]][8]
            if k % 5 == 0 and not fallen[i]:
                tilts[i].append(np.degrees(np.arccos(np.clip(up, -1, 1))))
            if not fallen[i] and (d.xpos[trunk[i]][2] < 0.15 or up < 0.5):
                fallen[i] = True
                fall_log.append(f"r{i}@{d.time - t0:.1f}s({d.xpos[trunk[i]][0]:.2f},{d.xpos[trunk[i]][1]:.2f})"
                                f"{'/obs' if d.time - t0 - last_obs[i] < 1.5 else ''}"
                                f"{'/rob' if d.time - t0 - last_rob[i] < 1.5 else ''}")
                ctrls[i].set_stand()
    sim_t = d.time - t0
    allt = np.concatenate([np.array(x) for x in tilts if x])
    res = dict(job, N=N, fall_where=";".join(fall_log), success=int(np.isfinite(done_t)), time=round(done_t, 2) if np.isfinite(done_t) else np.nan,
               falls=int(fallen.sum()), rr_contacts=rr_ev, obstacle_contacts=ro_ev, mover_contacts=rm_ev,
               min_dist_robots=round(dmin_rr, 3) if np.isfinite(dmin_rr) else np.nan,
               min_clear_obstacle=round(dmin_obs, 3), min_clear_mover=round(dmin_mover, 3),
               mean_tilt=round(float(allt.mean()), 3), p99_tilt=round(float(np.percentile(allt, 99)), 3),
               max_tilt=round(float(allt.max()), 2),
               form_err=round(float(np.mean(form_err)), 4) if form_err else np.nan,
               lag_mean=round(float(np.mean(lag)), 3), lag_max=round(float(np.max(lag)), 3),
               urgent_per_robot_min=round(sum(c.n_urgent for c in coms) / N / (sim_t / 60), 2),
               mean_wait=round(float(np.mean([w for c in coms for (_, _, w, kk) in c.log if kk is not None] or [0])), 4))
    for name, (a, b) in sec_t.items():
        res[f"t_{name}"] = round(b - a, 2) if np.isfinite(a) and np.isfinite(b) else np.nan
    if paths is not None:
        res["paths"], res["mover_path"] = paths, mover_path
    return res


def analyse(rows, out):
    lines = []
    P = lambda s="": (print(s), lines.append(s))
    f = lambda r, k: float(r[k]) if r.get(k) not in ("", "nan", None) else np.nan
    secs = [k[2:] for k in rows[0] if k.startswith("t_")]
    P("=" * 120)
    P("OBSTACLE COURSE  (pole slalom -> gate -> chicane -> 2 timed moving obstacles -> pillars; mean over seeds)")
    P("=" * 120)
    for team in sorted({int(float(r["team"])) for r in rows}):
        P(f"\nteam of {team}")
        P(f"  {'config':36s} {'ok':>4s} {'time':>6s} {'falls':>5s} {'r-r':>4s} {'r-obs':>5s} {'r-mov':>5s} "
          f"{'d robots':>8s} {'clr obs':>7s} {'clr mov':>7s} {'p99':>5s} {'max':>5s} {'form':>5s} {'lag':>5s} "
          f"{'urg/min':>7s}")
        for cfg in dict.fromkeys(r["config"] for r in rows):
            R = [r for r in rows if int(float(r["team"])) == team and r["config"] == cfg]
            if not R:
                continue
            g = lambda k: np.array([f(r, k) for r in R])
            robot_dist = g("min_dist_robots")
            form_err = g("form_err")
            robot_dist = robot_dist[np.isfinite(robot_dist)]
            form_err = form_err[np.isfinite(form_err)]
            robot_dist_text = f"{np.min(robot_dist):7.2f}" if robot_dist.size else f"{'n/a':>7s}"
            form_err_text = f"{np.mean(form_err):4.2f}" if form_err.size else f"{'n/a':>4s}"
            P(f"  {cfg:36s} {100 * np.mean(g('success')):3.0f}% {np.nanmean(g('time')):5.0f}s {np.sum(g('falls')):5.0f} "
              f"{np.mean(g('rr_contacts')):4.1f} {np.mean(g('obstacle_contacts')):5.1f} {np.mean(g('mover_contacts')):5.1f} "
              f"{robot_dist_text}m {np.min(g('min_clear_obstacle')):6.2f}m "
              f"{np.min(g('min_clear_mover')):6.2f}m {np.mean(g('p99_tilt')):4.1f}° {np.max(g('max_tilt')):4.0f}° "
              f"{form_err_text}m {np.mean(g('lag_mean')):4.2f}m {np.mean(g('urgent_per_robot_min')):7.1f}")
        P(f"  section times (s, first robot in -> last robot out):")
        for cfg in dict.fromkeys(r["config"] for r in rows):
            R = [r for r in rows if int(float(r["team"])) == team and r["config"] == cfg]
            if R:
                P(f"    {cfg:36s} " + "  ".join(f"{s} {np.nanmean([f(r, 't_' + s) for r in R]):5.1f}" for s in secs))
    P("\n  r-r / r-obs / r-mov = contact onsets robot-robot / robot-static obstacle / robot-moving obstacle;")
    P("  clr = smallest clearance (centre to obstacle surface; a robot is ~0.33 m wide at the hips);")
    P("  form = mean distance robot to its slot; lag = how far robots trail their own slots along the path.")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "course_summary.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


def plot(results, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    R = [r for r in results if "paths" in r]
    if not R:
        return
    teams = sorted({r["team"] for r in R})
    cfgs = list(dict.fromkeys(r["config"] for r in R))
    fig, axes = plt.subplots(len(teams) * len(cfgs), 1, figsize=(15, 2.6 * len(teams) * len(cfgs)), squeeze=False)
    cmap = plt.get_cmap("tab10")
    for a, (team, cfg) in enumerate(itertools.product(teams, cfgs)):
        ax = axes[a, 0]
        course = obstacle_course(team)
        for (x, y), r in course["obstacles"]:
            ax.add_patch(plt.Circle((x, y), r, color="#8a6a4a", lw=0))
        for mv in course["movers"]:
            ax.plot([mv.a[0], mv.b[0]], [mv.a[1], mv.b[1]], ":", color="#c0392b", lw=1.5)
        W = course["plan"].W
        ax.plot(W[:, 0], W[:, 1], "--", color="#999999", lw=0.8)
        rr = [r for r in R if r["team"] == team and r["config"] == cfg]
        if rr:
            r = rr[0]
            for i, pth in enumerate(r["paths"]):
                pth = np.array(pth)
                if len(pth):
                    ax.plot(pth[:, 0], pth[:, 1], "-", lw=1.3, color=cmap(i % 10))
            ax.set_title(f"team {team} | {cfg} | ok {r['success']}, falls {r['falls']}, "
                         f"contacts r-r {r['rr_contacts']} r-obs {r['obstacle_contacts']} r-mov {r['mover_contacts']}",
                         fontsize=9)
        for name, x0, x1 in course["sections"]:
            ax.axvline(x0, color="#cccccc", lw=0.6)
            ax.text((x0 + x1) / 2, 4.5, name, ha="center", fontsize=8, color="#666666")
        ax.set_xlim(-6, 40); ax.set_ylim(-4.8, 5.0); ax.set_aspect("equal")
        ax.set_xticks(range(-4, 40, 2)); ax.tick_params(labelsize=7)
    fig.tight_layout()
    os.makedirs(out, exist_ok=True)
    fn = os.path.join(out, "course_paths.png")
    fig.savefig(fn, dpi=110)
    plt.close(fig)
    print("wrote", fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--teams", default="1,6")
    ap.add_argument("--configs", default="reactive:limiter,predictive:limiter:q,predictive:phase+gaslc_equal:q")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--speed", type=float, default=0.3, help="reference speed along the path (m/s)")
    ap.add_argument("--urgent-ttc", type=float, default=0.8)
    ap.add_argument("--turn-gate", action="store_true", help="body-command envelope: |wz|<=1, |vy|<=0.3, slow while turning")
    ap.add_argument("--robust", action="store_true", help="turn gate + local-minimum escape")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "course"))
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "course_eval_quick.csv" if a.quick else "course_eval.csv")
    if a.analyse_only:
        rows = list(csv.DictReader(open(path)))
    else:
        seeds = 1 if a.quick else a.seeds
        jobs = [dict(team=int(tm), config=c, seed=s, speed=a.speed, urgent_ttc=a.urgent_ttc, record=int(s == 0),
                     turn_gate=int(a.turn_gate or a.robust), escape=int(a.robust))
                for tm, s, c in itertools.product(a.teams.split(","), range(seeds), a.configs.split(","))]
        jobs.sort(key=lambda j: -j["team"])
        print(f"{len(jobs)} course runs on {a.workers} workers")
        t0, results = time.time(), []
        with Pool(a.workers) as pool:
            for i, r in enumerate(pool.imap_unordered(run, jobs)):
                results.append(r)
                el = time.time() - t0
                print(f"  {i + 1}/{len(jobs)} team {r['team']} {r['config']:36s} seed {r['seed']}: ok {r['success']} "
                      f"time {r['time']} falls {r['falls']} contacts {r['rr_contacts']}/{r['obstacle_contacts']}/"
                      f"{r['mover_contacts']}  ({el / 60:.1f} min, ~{el / (i + 1) * (len(jobs) - i - 1) / 60:.0f} min left)",
                      flush=True)
        plot(results, a.out)
        rows = [{k: v for k, v in r.items() if k not in ("paths", "mover_path")} for r in results]
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
        rows = [{k: str(v) for k, v in r.items()} for r in rows]
    analyse(rows, a.out)


if __name__ == "__main__":
    main()
