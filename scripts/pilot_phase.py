"""Pilot experiment: does the effect of a disturbance depend on WHEN in the gait
cycle it happens?

From a steady trot, at each of 16 gait phases phi, apply either
  * a sudden velocity-command change  (8 planar directions + yaw left/right, 4 sizes,
    with and without the command slew limiter), or
  * a push on the trunk  (0.1 s force, 8 directions, 4 sizes),
then record for the next 2 s:
  fell, peak tilt, recovery time, foot slip (horizontal travel of feet in contact),
  peak velocity-tracking error.
Each (condition, phi) is repeated `--reps` times with a small random perturbation of
the starting state (0.05 m/s, 0.1 rad/s). Two questions are reported separately:
  * is there a phase effect at all?   eta^2 (share of variance explained by phase)
                                      + permutation p-value. In a near-deterministic
                                      simulator this is almost always "yes".
  * is it big enough to matter?       phase SWING = (worst-phase mean - best-phase mean),
                                      absolute and relative to the mean, and the number
                                      of conditions where survival depends on phase.
Decide on the second, not the first.

Run:   python scripts/pilot_phase.py               (full, all CPU cores)
       python scripts/pilot_phase.py --quick       (small smoke test, ~1 min)
Output: results/pilot_phase.csv, results/pilot_summary.txt, results/pilot_*.png
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import argparse
import copy
import csv
import itertools
import time
from multiprocessing import Pool

import numpy as np
import mujoco
from go1_bezier import load_single, Go1BezierController, LEGS

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
N_PHASE = 16
SETTLE = 3.0          # s of steady walking before snapshot
WINDOW = 2.0          # s observed after the event
PUSH_DUR = 0.1        # s
TILT_OK, V_OK, W_OK = 5.0, 0.10, 0.25    # recovery thresholds (deg, m/s, rad/s)

# ---------------------------------------------------------------------------
# per-process simulation state (built once per worker, restored for every trial)
_W = {}


def _controller_state(c):
    return {k: copy.deepcopy(v) for k, v in c.__dict__.items() if k not in ("m", "d")}


def _set_controller_state(c, st):
    for k, v in st.items():
        setattr(c, k, copy.deepcopy(v))


def _init_worker(menagerie_env):
    if menagerie_env:
        os.environ["MENAGERIE_PATH"] = menagerie_env
    _W.clear()


def _baseline(v0, limiter):
    key = (v0, limiter)
    if key in _W:
        return _W[key]
    m, d = load_single()
    c = Go1BezierController(m, d, acc_max=(1.5, 1.0, 3.0) if limiter else None)
    c.set_command(v0, 0.0, 0.0)
    for _ in range(int(SETTLE / m.opt.timestep)):
        c.step(); mujoco.mj_step(m, d)
    st = np.empty(mujoco.mj_stateSize(m, mujoco.mjtState.mjSTATE_PHYSICS))
    mujoco.mj_getState(m, d, st, mujoco.mjtState.mjSTATE_PHYSICS)
    floor = m.geom("floor").id
    foot_geoms = [m.geom(l).id for l in LEGS]
    ctrl = _controller_state(c)
    # reference slip of undisturbed walking over the same window
    slip0 = 0.0
    for k in range(int(WINDOW / m.opt.timestep)):
        c.step(); mujoco.mj_step(m, d)
        if k % 2 == 0:
            slip0 += _slip_speeds(m, d, foot_geoms, floor).sum() * 2 * m.opt.timestep
    _W[key] = dict(m=m, d=d, c=c, phys=st, ctrl=ctrl, foot_geoms=foot_geoms, floor=floor,
                   sites=[m.site(l).id for l in LEGS], trunk=m.body("trunk").id,
                   slip0_mm=slip0 * 1000)
    return _W[key]


_VEL = np.zeros(6)
_F = np.zeros(6)
F_MIN = 10.0          # N: only count contacts actually carrying load


def _slip_speeds(m, d, foot_geoms, floor):
    """Horizontal speed of the foot material point at each loaded floor contact (m/s),
    0 if the foot is not carrying load. Rolling without slipping gives ~0 here."""
    out = np.zeros(4)
    for i in range(d.ncon):
        con = d.contact[i]
        for k, g in enumerate(foot_geoms):
            if (con.geom1 == g and con.geom2 == floor) or (con.geom2 == g and con.geom1 == floor):
                mujoco.mj_contactForce(m, d, i, _F)
                if _F[0] < F_MIN:
                    continue
                mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, _VEL, 0)
                w, v = _VEL[:3], _VEL[3:]
                vp = v + np.cross(w, con.pos - d.geom_xpos[g])
                out[k] = np.hypot(vp[0], vp[1])
    return out


def run_trial(t):
    """t: dict with kind, v0, limiter, phi, dir, mag, rep, seed. Returns metrics dict."""
    B = _baseline(t["v0"], t["limiter"])
    m, d, c = B["m"], B["d"], B["c"]
    mujoco.mj_setState(m, d, B["phys"], mujoco.mjtState.mjSTATE_PHYSICS)
    _set_controller_state(c, B["ctrl"])
    d.xfrc_applied[:] = 0
    rng = np.random.default_rng(t["seed"])
    # realistic state noise (estimator-level): gives run-to-run spread within a phase
    d.qvel[0:2] += rng.normal(0, 0.05, 2)
    d.qvel[3:6] += rng.normal(0, 0.10, 3)
    mujoco.mj_forward(m, d)
    dt = m.opt.timestep

    # walk until the gait clock reaches the target phase
    target = t["phi"]
    for _ in range(int(1.5 * c.gait.period / dt)):
        if abs(((c.gait.phi - target + 0.5) % 1.0) - 0.5) < 0.5 * c.gait.period ** -1 * dt * c.decimation + 1e-9:
            break
        c.step(); mujoco.mj_step(m, d)
    phi_actual = c.gait.phi

    yaw = c.yaw()
    Rz = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    v_goal = np.array([t["v0"], 0.0, 0.0])
    force = np.zeros(3)
    if t["kind"] == "cmd":
        if t["dir"] == "yaw+":
            v_goal[2] = 2.0 * t["mag"]
        elif t["dir"] == "yaw-":
            v_goal[2] = -2.0 * t["mag"]
        else:
            a = np.radians(float(t["dir"]))
            v_goal[:2] += t["mag"] * np.array([np.cos(a), np.sin(a)])
        c.set_command(*v_goal)
        v_goal = c.cmd_target.copy()               # after clipping to limits
    else:                                            # push, body-frame direction, N
        a = np.radians(float(t["dir"]))
        force[:2] = Rz @ (t["mag"] * np.array([np.cos(a), np.sin(a)]))

    n = int(WINDOW / dt)
    per = int(round(c.gait.period / dt))
    tilt = np.zeros(n); verr = np.zeros(n); werr = np.zeros(n)
    vbuf = np.zeros((per, 3))
    slip = np.zeros(4)
    fell = False
    for k in range(n):
        if t["kind"] == "push":
            d.xfrc_applied[B["trunk"], :3] = force if k * dt < PUSH_DUR else 0.0
        c.step(); mujoco.mj_step(m, d)
        up = d.xmat[B["trunk"]][8]
        tilt[k] = np.degrees(np.arccos(np.clip(up, -1, 1)))
        vbuf[k % per] = c.body_velocity()
        vavg = vbuf.mean(0) if k >= per else vbuf[:k + 1].mean(0)
        verr[k] = np.linalg.norm(vavg[:2] - v_goal[:2])
        werr[k] = abs(vavg[2] - v_goal[2])
        if k % 2 == 0:
            slip += _slip_speeds(m, d, B["foot_geoms"], B["floor"]) * 2 * dt   # slid distance
        if d.xpos[B["trunk"]][2] < 0.12 or up < 0.5:
            fell = True
            break
    d.xfrc_applied[:] = 0
    if fell:
        rec = np.nan
    else:
        bad = (tilt > TILT_OK) | (verr > V_OK) | (werr > W_OK)
        bad[:per] = True                              # need one full period of averaging
        idx = np.nonzero(bad)[0]
        rec = (idx[-1] + 1) * dt if len(idx) else 0.0
        if len(idx) and idx[-1] == n - 1:
            rec = np.inf                              # not recovered within window
    # slip = distance the feet slid while touching the floor, mm (sum and worst foot)
    return dict(t, phi_actual=round(phi_actual, 4), fell=int(fell),
                peak_tilt=round(float(tilt[:k + 1].max()), 3),
                recovery_s=rec if np.isfinite(rec) or np.isnan(rec) else WINDOW,
                recovered=int((not fell) and np.isfinite(rec)),
                slip_mm=round(float(slip.sum() * 1000), 2),
                excess_slip_mm=round(float(slip.sum() * 1000 - B["slip0_mm"]), 2),
                max_slip_mm=round(float(slip.max() * 1000), 2),
                peak_verr=round(float(verr[per:k + 1].max()) if k >= per else np.nan, 4))


# ---------------------------------------------------------------------------
def build_trials(quick=False, reps=3):
    phases = np.arange(N_PHASE) / N_PHASE
    dirs = [0, 45, 90, 135, 180, 225, 270, 315]
    v0s = [0.0, 0.4]
    cmd_mags, push_mags = [0.2, 0.4, 0.6, 0.8], [60, 120, 180, 240]   # m/s ; N for 0.1 s
    if quick:
        phases, dirs, v0s = phases[::4], [0, 90, 180], [0.4]
        cmd_mags, push_mags, reps = [0.4, 0.8], [120, 240], 2
    trials, seed = [], 0
    for v0, phi, rep in itertools.product(v0s, phases, range(reps)):
        for lim, dr, mg in itertools.product((True, False), dirs + ["yaw+", "yaw-"], cmd_mags):
            seed += 1
            trials.append(dict(kind="cmd", v0=v0, limiter=lim, phi=round(float(phi), 4),
                               dir=str(dr), mag=mg, rep=rep, seed=seed))
        for dr, mg in itertools.product(dirs, push_mags):
            seed += 1
            trials.append(dict(kind="push", v0=v0, limiter=True, phi=round(float(phi), 4),
                               dir=str(dr), mag=mg, rep=rep, seed=seed))
    return trials


def eta2_perm(values, groups, n_perm=500, rng=None):
    """Share of variance explained by group (phase) + permutation p-value."""
    values, groups = np.asarray(values, float), np.asarray(groups)
    ok = np.isfinite(values)
    values, groups = values[ok], groups[ok]
    if len(values) < 4 or values.var() < 1e-12:
        return np.nan, np.nan
    ug = np.unique(groups)

    def eta(g):
        mu = values.mean()
        ssb = sum((g == u).sum() * (values[g == u].mean() - mu) ** 2 for u in ug if (g == u).any())
        return ssb / ((values - mu) ** 2).sum()

    e = eta(groups)
    rng = rng or np.random.default_rng(0)
    null = np.array([eta(rng.permutation(groups)) for _ in range(n_perm)])
    return e, (1 + (null >= e).sum()) / (n_perm + 1)


def analyse(rows, out_dir):
    import collections
    rng = np.random.default_rng(1)
    lines = []
    P = lambda s="": (print(s), lines.append(s))
    groups = collections.defaultdict(list)
    for r in rows:
        groups[(r["kind"], r["v0"], r["limiter"], r["dir"], r["mag"])].append(r)

    P("=" * 78)
    P("PHASE DEPENDENCE  (eta2 = share of variance explained by gait phase;")
    P("                   p from permutation test against run-to-run variation)")
    P("=" * 78)
    summary = collections.defaultdict(list)
    for key, rs in sorted(groups.items(), key=lambda kv: str(kv[0])):
        kind, v0, lim, dr, mg = key
        ph = np.array([r["phi"] for r in rs])
        fell = np.array([r["fell"] for r in rs])
        tilt = np.array([r["peak_tilt"] for r in rs])
        rec = np.array([r["recovery_s"] for r in rs], float)
        slip = np.array([r["excess_slip_mm"] for r in rs])
        fr_by_phase = np.array([fell[ph == u].mean() for u in np.unique(ph)])

        def swing(vals):
            m_ = [np.nanmean(vals[(ph == u) & np.isfinite(vals)]) for u in np.unique(ph)
                  if ((ph == u) & np.isfinite(vals)).any()]
            if len(m_) < 2:
                return np.nan, np.nan
            return max(m_) - min(m_), (max(m_) - min(m_)) / max(abs(np.mean(m_)), 1e-9)
        e_t, p_t = eta2_perm(np.where(fell == 1, np.nan, tilt), ph, rng=rng)
        e_r, p_r = eta2_perm(rec, ph, rng=rng)
        e_s, p_s = eta2_perm(np.where(fell == 1, np.nan, slip), ph, rng=rng)
        tag = f"{kind:4s} v0={v0:.1f} {'lim' if lim else 'raw':3s}"
        sw_t, rs_t = swing(np.where(fell == 1, np.nan, tilt))
        sw_r, rs_r = swing(rec)
        sw_s, rs_s = swing(np.where(fell == 1, np.nan, slip))
        summary[tag].append(dict(dir=dr, mag=mg, fall_min=fr_by_phase.min(), fall_max=fr_by_phase.max(),
                                 fall_mean=fell.mean(), e_t=e_t, p_t=p_t, e_r=e_r, p_r=p_r,
                                 e_s=e_s, p_s=p_s, sw_t=sw_t, rs_t=rs_t, sw_r=sw_r, rs_r=rs_r,
                                 sw_s=sw_s, rs_s=rs_s,
                                 best_phi=float(np.unique(ph)[np.argmin(fr_by_phase)]),
                                 worst_phi=float(np.unique(ph)[np.argmax(fr_by_phase)])))

    med = lambda cs, k: np.nanmedian([c[k] for c in cs])
    sig = lambda cs, k: np.mean([(c[k] < 0.01) for c in cs if np.isfinite(c[k])]) if any(
        np.isfinite(c[k]) for c in cs) else np.nan
    P("\n1) IS IT BIG ENOUGH TO MATTER?  phase swing = worst-phase mean - best-phase mean")
    P(f"{'group':18s} {'cond':>4s} {'fall%':>6s} {'phase-gated':>11s} | {'tilt swing':>14s} | "
      f"{'recovery swing':>15s} | {'excess-slip swing':>15s}")
    for tag, cs in summary.items():
        gated = sum(1 for c in cs if c["fall_min"] < 0.34 and c["fall_max"] > 0.66)
        P(f"{tag:18s} {len(cs):4d} {100 * np.mean([c['fall_mean'] for c in cs]):5.1f}% "
          f"{gated:5d}/{len(cs):<5d} | {med(cs, 'sw_t'):5.2f} deg {100 * med(cs, 'rs_t'):4.0f}% | "
          f"{med(cs, 'sw_r'):6.2f} s {100 * med(cs, 'rs_r'):4.0f}% | "
          f"{med(cs, 'sw_s'):6.1f} mm {100 * med(cs, 'rs_s'):4.0f}%")
    P("   (medians over conditions; % = swing relative to the condition's mean;")
    P("    excess slip = foot sliding beyond what undisturbed walking produces in 2 s)")
    P("   phase-gated = survives at some phases (<34% falls) but falls at others (>66%)")

    P("\n2) IS THERE A PHASE EFFECT AT ALL?  eta2 = variance share explained by phase")
    P(f"{'group':18s} {'tilt eta2':>9s} {'p<.01':>6s} {'recov eta2':>10s} {'p<.01':>6s} "
      f"{'slip eta2':>9s} {'p<.01':>6s}")
    for tag, cs in summary.items():
        P(f"{tag:18s} {med(cs, 'e_t'):9.2f} {100 * sig(cs, 'p_t'):5.0f}% {med(cs, 'e_r'):10.2f} "
          f"{100 * sig(cs, 'p_r'):5.0f}% {med(cs, 'e_s'):9.2f} {100 * sig(cs, 'p_s'):5.0f}%")

    P("\nPhase-gated conditions (survival depends on timing):")
    gated_all = [(tag, c) for tag, cs in summary.items() for c in cs
                 if c["fall_min"] < 0.34 and c["fall_max"] > 0.66]
    for tag, c in gated_all[:20]:
        P(f"  {tag:18s} dir {c['dir']:>4s} size {c['mag']:>5}  falls {100 * c['fall_min']:.0f}% at "
          f"phi={c['best_phi']:.3f}  vs {100 * c['fall_max']:.0f}% at phi={c['worst_phi']:.3f}")
    if not gated_all:
        P("  none")

    P("\nLargest tilt swings (non-falling runs):")
    flat = [(tag, c) for tag, cs in summary.items() for c in cs if np.isfinite(c["sw_t"])]
    for tag, c in sorted(flat, key=lambda x: -x[1]["sw_t"])[:10]:
        P(f"  {tag:18s} dir {c['dir']:>4s} size {c['mag']:>5}  tilt swing {c['sw_t']:.1f} deg "
          f"({100 * c['rs_t']:.0f}% of mean)  recovery swing {c['sw_r']:.2f} s")

    P("\nSuggested reading (decide on section 1, not on p-values):")
    P("  * STRONG: phase-gated falls in a good share of push/cmd conditions, or tilt /")
    P("    recovery swings >~30% of the mean -> gait phase belongs in the swarm layer.")
    P("  * WEAK:  no phase-gated falls and swings <~10-15% -> phase washes out; model")
    P("    feasibility on speed/turn state (+ slew limits) only.")
    P("  * in between: phase matters only near the limits -> keep it as a safety margin")
    P("    term (e.g. shrink the feasible set at bad phases), not a headline variable.")
    with open(os.path.join(out_dir, "pilot_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return summary


def plot(rows, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed - skipping plots: pip install matplotlib)")
        return
    phases = sorted({r["phi"] for r in rows})
    for kind, lim in (("cmd", True), ("cmd", False), ("push", True)):
        sub = [r for r in rows if r["kind"] == kind and r["limiter"] == lim]
        if not sub:
            continue
        mags = sorted({r["mag"] for r in sub})
        v0s = sorted({r["v0"] for r in sub})
        fig, axes = plt.subplots(len(v0s), 3, figsize=(13, 3.6 * len(v0s)), squeeze=False)
        blues = plt.get_cmap("Blues")
        for i, v0 in enumerate(v0s):
            for j, (metric, ylab) in enumerate((("fell", "fall rate"), ("peak_tilt", "peak tilt (deg)"),
                                                ("recovery_s", "recovery time (s)"))):
                ax = axes[i, j]
                for mi, mg in enumerate(mags):
                    ys = []
                    for p in phases:
                        vals = np.array([r[metric] for r in sub if r["v0"] == v0 and r["mag"] == mg
                                         and r["phi"] == p and (metric == "fell" or not r["fell"])], float)
                        ys.append(np.nanmean(vals) if len(vals) else np.nan)
                    unit = "N" if kind == "push" else "m/s"
                    ax.plot(phases, ys, "-o", ms=4, lw=2, color=blues(0.35 + 0.65 * mi / max(1, len(mags) - 1)),
                            label=f"{mg} {unit}")
                ax.set_xlabel("gait phase at event")
                ax.set_ylabel(ylab)
                ax.set_title(f"v0 = {v0} m/s", fontsize=10)
                ax.grid(alpha=0.25)
                ax.spines[["top", "right"]].set_visible(False)
                if j == 0:
                    ax.legend(title="size", fontsize=8, frameon=False)
        title = {"cmd": "Sudden velocity-command change", "push": "Push on trunk (0.1 s)"}[kind]
        if kind == "cmd":
            title += " - slew limiter " + ("ON" if lim else "OFF")
        fig.suptitle(title + "  (mean over directions and repeats; phase 0-0.5: FR+RL in stance, "
                     "0.5-1: FL+RR in stance)", fontsize=10)
        fig.tight_layout()
        name = f"pilot_{kind}{'' if kind == 'push' else ('_lim' if lim else '_raw')}.png"
        fig.savefig(os.path.join(out_dir, name), dpi=130)
        plt.close(fig)
        print("wrote", os.path.join(out_dir, name))

        # direction x phase maps: averaging over directions can hide phase effects
        # (a push to the left at phase phi mirrors a push to the right at phi + 0.5)
        dirs = sorted({r["dir"] for r in sub}, key=lambda x: (not x.lstrip("-").isdigit(), float(x) if x.lstrip("-").isdigit() else 0, x))
        fig, axes = plt.subplots(len(v0s), 2, figsize=(12, 3.4 * len(v0s)), squeeze=False)
        for i, v0 in enumerate(v0s):
            for j, (metric, mg, lab) in enumerate((("fell", mags[-1], "fall rate"),
                                                  ("peak_tilt", mags[len(mags) // 2 - 1 if len(mags) > 1 else 0],
                                                   "peak tilt (deg), non-falling runs"))):
                M = np.full((len(dirs), len(phases)), np.nan)
                for a, dr in enumerate(dirs):
                    for b, p in enumerate(phases):
                        vals = [r[metric] for r in sub if r["v0"] == v0 and r["mag"] == mg and r["dir"] == dr
                                and r["phi"] == p and (metric == "fell" or not r["fell"])]
                        if vals:
                            M[a, b] = np.mean(vals)
                ax = axes[i, j]
                im = ax.imshow(M, aspect="auto", cmap="Blues", origin="lower",
                               vmin=0 if metric == "fell" else None, vmax=1 if metric == "fell" else None,
                               extent=(phases[0] - 0.5 / len(phases), phases[-1] + 0.5 / len(phases),
                                       -0.5, len(dirs) - 0.5))
                ax.set_yticks(range(len(dirs)))
                ax.set_yticklabels([d_ if not d_.lstrip("-").isdigit() else d_ + "°" for d_ in dirs])
                ax.set_xlabel("gait phase at event")
                ax.set_ylabel("direction (body frame)")
                unit = "N" if kind == "push" else "m/s"
                ax.set_title(f"v0 = {v0} m/s, size {mg} {unit}: {lab}", fontsize=10)
                fig.colorbar(im, ax=ax, fraction=0.04)
        fig.suptitle(title + "  - by direction", fontsize=10)
        fig.tight_layout()
        name = name.replace(".png", "_by_direction.png")
        fig.savefig(os.path.join(out_dir, name), dpi=130)
        plt.close(fig)
        print("wrote", os.path.join(out_dir, name))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="small smoke test")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    ap.add_argument("--analyse-only", action="store_true", help="re-analyse existing CSV")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, "pilot_phase.csv" if not args.quick else "pilot_phase_quick.csv")

    if args.analyse_only:
        with open(csv_path) as f:
            rows = []
            for r in csv.DictReader(f):
                for k in ("v0", "phi", "mag", "phi_actual", "peak_tilt", "recovery_s", "slip_mm",
                          "excess_slip_mm",
                          "max_slip_mm", "peak_verr"):
                    r[k] = float(r[k]) if r[k] not in ("", "nan") else np.nan
                for k in ("fell", "rep", "seed", "recovered"):
                    r[k] = int(r[k])
                r["limiter"] = r["limiter"] == "True"
                rows.append(r)
    else:
        trials = build_trials(args.quick, args.reps)
        print(f"{len(trials)} trials on {args.workers} workers ...")
        t0 = time.time()
        rows = []
        with Pool(args.workers, initializer=_init_worker,
                  initargs=(os.environ.get("MENAGERIE_PATH", ""),)) as pool:
            for i, r in enumerate(pool.imap_unordered(run_trial, trials, chunksize=8)):
                rows.append(r)
                if (i + 1) % max(1, len(trials) // 20) == 0:
                    el = time.time() - t0
                    print(f"  {i + 1}/{len(trials)}  {el / 60:.1f} min elapsed, "
                          f"~{el / (i + 1) * (len(trials) - i - 1) / 60:.1f} min left", flush=True)
        rows.sort(key=lambda r: r["seed"])
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {csv_path}  ({(time.time() - t0) / 60:.1f} min)")
    analyse(rows, args.out)
    plot(rows, args.out)


if __name__ == "__main__":
    main()
