"""Push-capability envelope J_max: the strongest push the Go1 survives, measured
directly by bisection (instead of interpolating between a few tested magnitudes).

For every  base speed v0  x  push direction (16, every 22.5 deg)  x  gait phase (16)
x repeat (3, each with its own small state noise), bisect the push force F
(0.1 s, so impulse = 0.1*F N*s) between F_LO and F_HI for two criteria:

    fall : robot falls                                    -> J_fall
    safe : robot falls OR peak tilt > --tilt-crit (20 deg) -> J_safe

J_max is reported as the largest force that PASSED (conservative); the first
failing force is kept too, so the resolution is visible. Cells that pass even at
F_HI or fail already at F_LO are flagged as censored.

Then:  * monotonicity check on random cells (should pass at 0.85*J, fail at 1.15*J)
       * agreement with the pilot data (results/pilot_phase.csv, if present)
       * maps in absolute direction and relative to the stance diagonal
       * data/push_envelope_bezier.npz for go1_bezier.capability.PushEnvelope

Run:   python scripts/push_envelope.py --quick     (~2 min smoke test)
       python scripts/push_envelope.py             (full: ~24k sims, all cores)
       python scripts/push_envelope.py --analyse-only
"""
import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import argparse
import csv
import zlib
import itertools
import time
import warnings
from multiprocessing import Pool

import numpy as np
from pilot_phase import run_trial, _init_worker          # same simulator + metrics as the pilot

warnings.filterwarnings("ignore", message="Mean of empty slice")
ROOT = os.path.join(HERE, "..")
F_LO, F_HI, STEPS = 60.0, 320.0, 6       # N; 6 bisection steps -> ~4 N resolution
STANCE_AXIS = (-34.0, 34.0)              # deg: forward end of the diagonal in stance
                                         #   phase 0-0.5: FR+RL  (FR at -34 deg)
                                         #   phase 0.5-1: FL+RR  (FL at +34 deg)
TILT_CRIT = 20.0


def _fails(res, criterion, tilt_crit):
    return bool(res["fell"]) or (criterion == "safe" and res["peak_tilt"] > tilt_crit)


def bisect(job):
    """job: dict(v0, dir, phi, rep, criterion, seed, tilt_crit). One full bisection."""
    base = dict(kind="push", v0=job["v0"], limiter=True, phi=job["phi"], dir=str(job["dir"]),
                rep=job["rep"], seed=job["seed"])           # same state noise for every probe
    probes = []

    def probe(F):
        r = run_trial(dict(base, mag=float(F)))
        f = _fails(r, job["criterion"], job["tilt_crit"])
        probes.append((round(F, 1), int(f), round(r["peak_tilt"], 1)))
        return f

    lo, hi = F_LO, F_HI
    if not probe(hi):
        jm, jf, cens = hi, np.nan, "above"          # survives even F_HI
    elif probe(lo):
        jm, jf, cens = 0.0, lo, "below"             # fails already at F_LO
    else:
        for _ in range(STEPS):
            mid = 0.5 * (lo + hi)
            if probe(mid):
                hi = mid
            else:
                lo = mid
        jm, jf, cens = lo, hi, ""
    return dict(job, j_max=round(jm, 2), j_fail=round(jf, 2) if np.isfinite(jf) else np.nan,
                censored=cens, n_probes=len(probes),
                probes=";".join(f"{F}:{f}:{t}" for F, f, t in probes))


def verify(job):
    """Monotonicity spot check: should pass at 0.85*J and fail at 1.15*J."""
    base = dict(kind="push", v0=job["v0"], limiter=True, phi=job["phi"], dir=str(job["dir"]),
                rep=job["rep"], seed=job["seed"] + 10_000_000)   # fresh noise
    lo = run_trial(dict(base, mag=0.85 * job["j_max"]))
    hi = run_trial(dict(base, mag=1.15 * job["j_fail"]))
    return dict(job, pass_low=int(not _fails(lo, job["criterion"], job["tilt_crit"])),
                fail_high=int(_fails(hi, job["criterion"], job["tilt_crit"])))


def build_jobs(quick, reps, criteria, tilt_crit, n_dir=16, n_phase=16):
    dirs = np.round(np.arange(n_dir) * 360.0 / n_dir, 2)
    phases = np.round(np.arange(n_phase) / n_phase, 4)
    v0s = [0.0, 0.4]
    if quick:
        dirs, phases, v0s, reps = dirs[::4], phases[::4], [0.4], 1
    jobs = []
    for k, (v0, th, ph, rep, crit) in enumerate(itertools.product(v0s, dirs, phases, range(reps), criteria)):
        seed = 1 + zlib.crc32(f"{v0}|{float(th)}|{float(ph)}|{rep}".encode()) % 1_000_000   # shared by both criteria
        jobs.append(dict(v0=v0, dir=float(th), phi=float(ph), rep=rep, criterion=crit,
                         seed=int(seed), tilt_crit=tilt_crit))
    return jobs


def run_pool(fn, jobs, workers, label):
    out, t0 = [], time.time()
    with Pool(workers, initializer=_init_worker, initargs=(os.environ.get("MENAGERIE_PATH", ""),)) as pool:
        for i, r in enumerate(pool.imap_unordered(fn, jobs, chunksize=2)):
            out.append(r)
            if (i + 1) % max(1, len(jobs) // 20) == 0:
                el = time.time() - t0
                print(f"  {label}: {i + 1}/{len(jobs)}  {el / 60:.1f} min, "
                      f"~{el / (i + 1) * (len(jobs) - i - 1) / 60:.1f} min left", flush=True)
    return out


# ---------------------------------------------------------------------------- analysis
def relative_coords(theta, phi):
    """Push angle relative to the forward end of the stance diagonal, and progress
    through that stance (0-1). The second half-cycle is mirrored onto the first
    (trot symmetry), so both halves share one map."""
    theta, phi = np.asarray(theta, float), np.asarray(phi, float)
    first = phi < 0.5
    th_r = np.where(first, theta - STANCE_AXIS[0], -(theta - STANCE_AXIS[1]))
    th_r = (th_r + 180.0) % 360.0 - 180.0
    psi = np.where(first, phi, phi - 0.5) / 0.5
    return th_r, psi


def analyse(rows, out_dir, map_path, tilt_crit, pilot_csv=None, lines=None):
    lines = [] if lines is None else lines
    P = lambda s="": (print(s), lines.append(s))
    import collections
    for r in rows:
        for k in ("v0", "dir", "phi", "j_max", "j_fail"):
            r[k] = float(r[k]) if r[k] not in ("", "nan", None) else np.nan
        r["rep"] = int(r["rep"])
    crits = sorted({r["criterion"] for r in rows})
    P("=" * 78)
    P(f"PUSH CAPABILITY ENVELOPE  (push = 0.1 s force; impulse = 0.1 x F N*s; robot ~12 kg)")
    P(f"  fall: J_fall = largest force not causing a fall")
    P(f"  safe: J_safe = largest force with no fall AND peak tilt <= {tilt_crit:.0f} deg")
    P("=" * 78)
    grids = {}
    for crit in crits:
        R = [r for r in rows if r["criterion"] == crit]
        v0s = sorted({r["v0"] for r in R})
        dirs = sorted({r["dir"] for r in R})
        phases = sorted({r["phi"] for r in R})
        G = np.full((len(v0s), len(dirs), len(phases)), np.nan)
        S = np.full_like(G, np.nan)
        cell = collections.defaultdict(list)
        for r in R:
            cell[(r["v0"], r["dir"], r["phi"])].append(r["j_max"])
        for a, v0 in enumerate(v0s):
            for b, th in enumerate(dirs):
                for c, ph in enumerate(phases):
                    v = cell.get((v0, th, ph), [])
                    if v:
                        G[a, b, c] = np.mean(v)
                        S[a, b, c] = np.std(v)
        grids[crit] = dict(G=G, S=S, v0=np.array(v0s), dirs=np.array(dirs), phases=np.array(phases))
        cens = collections.Counter(r["censored"] for r in R)
        P(f"\n[{crit}]  {len(R)} bisections; censored: {cens.get('above', 0)} survive F_HI={F_HI:.0f} N, "
          f"{cens.get('below', 0)} fail at F_LO={F_LO:.0f} N")
        for a, v0 in enumerate(v0s):
            g = G[a]
            worst = np.unravel_index(np.nanargmin(g), g.shape)
            best = np.unravel_index(np.nanargmax(g), g.shape)
            per_dir_swing = np.nanmax(g, 1) - np.nanmin(g, 1)
            P(f"  v0={v0:.1f}: J median {np.nanmedian(g):5.0f} N  (range {np.nanmin(g):.0f}-{np.nanmax(g):.0f})  "
              f"worst {g[worst]:.0f} N at dir {dirs[worst[0]]:.1f}, phi {phases[worst[1]]:.3f}; "
              f"best {g[best]:.0f} N at dir {dirs[best[0]]:.1f}, phi {phases[best[1]]:.3f}")
            P(f"          phase swing per direction (max-min over phase): median {np.nanmedian(per_dir_swing):.0f} N, "
              f"max {np.nanmax(per_dir_swing):.0f} N | repeat-to-repeat std: median {np.nanmedian(S[a]):.1f} N")
            phase_blind = np.nanmin(g, 1)          # what a phase-blind margin must assume
            P(f"          gain if phase is known: mean J over phases / worst-phase J = "
              f"{np.nanmean(np.nanmean(g, 1) / phase_blind):.2f}x")
        # mirror symmetry check on the measured envelope
        pairs = []
        for a in range(len(v0s)):
            for b, th in enumerate(dirs):
                mb = int(np.argmin(np.abs(((np.array(dirs) - (-th) % 360) + 180) % 360 - 180)))
                for c, ph in enumerate(phases):
                    mc = int(np.argmin(np.abs(((np.array(phases) - (ph + 0.5)) + 0.5) % 1 - 0.5)))
                    if np.isfinite(G[a, b, c]) and np.isfinite(G[a, mb, mc]):
                        pairs.append((G[a, b, c], G[a, mb, mc]))
        if len(pairs) > 3:
            pa = np.array(pairs)
            P(f"  mirror symmetry (theta -> -theta, phi -> phi+0.5): r = {np.corrcoef(pa.T)[0, 1]:.3f}, "
              f"mean |diff| {np.abs(pa[:, 0] - pa[:, 1]).mean():.1f} N")

    # agreement with the pilot (fall criterion, pilot forces 60/120/180/240)
    if pilot_csv and os.path.exists(pilot_csv) and "fall" in grids:
        g = grids["fall"]
        agree = n = 0
        with open(pilot_csv) as f:
            for r in csv.DictReader(f):
                if r["kind"] != "push":
                    continue
                v0, th, ph, F = float(r["v0"]), float(r["dir"]), round(float(r["phi"]) % 1, 4), float(r["mag"])
                a = np.where(np.isclose(g["v0"], v0))[0]
                b = np.where(np.isclose(g["dirs"], th))[0]
                c = np.where(np.isclose(g["phases"], ph))[0]
                if len(a) and len(b) and len(c) and np.isfinite(g["G"][a[0], b[0], c[0]]):
                    pred_fall = F > g["G"][a[0], b[0], c[0]]
                    agree += int(pred_fall == bool(int(r["fell"])))
                    n += 1
        if n:
            P(f"\nAgreement with pilot falls (pilot trial falls iff F > J_fall of its cell): "
              f"{100 * agree / n:.1f}% of {n} pilot push trials")

    support_axis_analysis(rows, P)

    # save for go1_bezier.capability.PushEnvelope
    out = {}
    for crit, g in grids.items():
        for k, v in g.items():
            out[f"{crit}/{k}"] = v
    out["tilt_crit"] = np.array(tilt_crit)
    os.makedirs(os.path.dirname(map_path), exist_ok=True)
    np.savez_compressed(map_path, **out)
    P(f"\nsaved {os.path.relpath(map_path, ROOT)}")
    with open(os.path.join(out_dir, "push_envelope_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    plot(rows, grids, out_dir, tilt_crit)


N_PSI = 8


def preferred_axis(thr, psi, J):
    """Per stance-progress bin: axis (deg, mod 180) along which J is largest, and the
    modulation amplitude (N), from the 2nd circular harmonic of J(theta_r)."""
    ang, amp = np.full(N_PSI, np.nan), np.full(N_PSI, np.nan)
    for b in range(N_PSI):
        m = (psi >= b / N_PSI) & (psi < (b + 1) / N_PSI)
        if m.sum() < 4:
            continue
        z = np.sum((J[m] - J[m].mean()) * np.exp(2j * np.radians(thr[m])))
        ang[b] = np.degrees(np.angle(z)) / 2
        amp[b] = 2 * abs(z) / m.sum()
    return ang, amp


def support_axis_analysis(rows, P):
    """Compact physical model: J(theta_r, psi) = J0(psi) + A(psi) cos 2(theta_r - alpha(psi)).
    alpha = the 'effective support axis'; compare with the stance diagonal (0 deg) and the
    next diagonal (+68 deg). Held-out (repeat 2) R2 vs. adding more harmonics and vs. the
    full 16x16 table."""
    P("\nEFFECTIVE SUPPORT AXIS  (axis of largest J vs. progress through stance;")
    P("   0 deg = current stance diagonal, +68 deg = the next diagonal (FL+RR seen from FR+RL))")
    for crit in sorted({r["criterion"] for r in rows}):
        for v0 in sorted({r["v0"] for r in rows}):
            R = [r for r in rows if r["criterion"] == crit and r["v0"] == v0]
            thr, psi = relative_coords([r["dir"] for r in R], [r["phi"] for r in R])
            J = np.array([r["j_max"] for r in R])
            rep = np.array([r["rep"] for r in R])
            ang, amp = preferred_axis(thr, psi, J)
            P(f"  {crit:4s} v0={v0:.1f}  alpha(psi): " +
              " ".join(f"{a:+4.0f}" for a in ang) + "  deg   amplitude: " + " ".join(f"{x:3.0f}" for x in amp) + " N")
            if (rep == 2).any() and (rep < 2).any():
                t = np.radians(thr)
                for ks, name in (((2,), "axis model (k=2)"), ((1, 2, 3, 4), "harmonics k=1..4")):
                    pred = np.full(len(J), np.nan)
                    for b in range(N_PSI):
                        m = (psi >= b / N_PSI) & (psi < (b + 1) / N_PSI)
                        tr, te = m & (rep < 2), m & (rep == 2)
                        F = lambda idx: np.column_stack([np.ones(idx.sum())] +
                                                        [f(k * t[idx]) for k in ks for f in (np.cos, np.sin)])
                        beta = np.linalg.lstsq(F(tr), J[tr], rcond=None)[0]
                        pred[te] = F(te) @ beta
                    te = rep == 2
                    r2 = 1 - np.nansum((J[te] - pred[te]) ** 2) / np.sum((J[te] - J[te].mean()) ** 2)
                    P(f"            held-out R2 {name:18s} {r2:.3f}")
                cell = {}
                for r in R:
                    if r["rep"] < 2:
                        cell.setdefault((r["dir"], r["phi"]), []).append(r["j_max"])
                y = np.array([r["j_max"] for r in R if r["rep"] == 2])
                pt = np.array([np.mean(cell[(r["dir"], r["phi"])]) for r in R if r["rep"] == 2])
                P(f"            held-out R2 {'full 16x16 table':18s} {1 - ((y - pt) ** 2).sum() / ((y - y.mean()) ** 2).sum():.3f}")


def plot(rows, grids, out_dir, tilt_crit):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed - skipping plots)")
        return
    lab = {"fall": "J_fall: largest force without falling (N)",
           "safe": f"J_safe: largest force, no fall and tilt <= {tilt_crit:.0f}° (N)"}
    for crit, g in grids.items():
        v0s = g["v0"]
        # (a) absolute direction x phase
        fig, axes = plt.subplots(1, len(v0s), figsize=(6 * len(v0s), 4.6), squeeze=False)
        vmin, vmax = np.nanmin(g["G"]), np.nanmax(g["G"])
        for j, v0 in enumerate(v0s):
            ax = axes[0, j]
            Z = g["G"][j]
            ph, th = g["phases"], g["dirs"]
            im = ax.imshow(Z, origin="lower", aspect="auto", cmap="Blues", vmin=vmin, vmax=vmax,
                           extent=(ph[0] - 0.5 / len(ph), ph[-1] + 0.5 / len(ph),
                                   th[0] - 180 / len(th), th[-1] + 180 / len(th)))
            cs = ax.contour(ph, th, Z, levels=[100, 150, 200, 250], colors="k", linewidths=0.7)
            ax.clabel(cs, fmt="%d N", fontsize=7)
            ax.axvline(0.5, color="k", lw=0.6, ls=":")
            ax.set_xlabel("gait phase at push (0-0.5 FR+RL, 0.5-1 FL+RR stance)")
            ax.set_ylabel("push direction, body frame (deg; 0 fwd, 90 left)")
            ax.set_yticks(np.arange(0, 360, 45))
            ax.set_title(f"v0 = {v0} m/s", fontsize=10)
        fig.colorbar(im, ax=axes, fraction=0.03, label=lab[crit])
        fig.suptitle(f"Push capability envelope ({crit}) - measured by bisection, mean of repeats", fontsize=11)
        fn = os.path.join(out_dir, f"envelope_{crit}_absolute.png")
        fig.savefig(fn, dpi=130, bbox_inches="tight")
        plt.close(fig)
        print("wrote", fn)

        # (b) relative to stance diagonal, both half-cycles pooled by mirror symmetry
        R = [r for r in rows if r["criterion"] == crit]
        fig, axes = plt.subplots(1, len(v0s), figsize=(6.5 * len(v0s), 5.0), squeeze=False)
        nb_t, nb_p = 16, 8
        for j, v0 in enumerate(v0s):
            Rv = [r for r in R if r["v0"] == v0]
            thr, psi = relative_coords([r["dir"] for r in Rv], [r["phi"] for r in Rv])
            J = np.array([r["j_max"] for r in Rv])
            ti = np.floor((thr + 180) / 360 * nb_t).astype(int) % nb_t
            pi = np.clip(np.floor(psi * nb_p).astype(int), 0, nb_p - 1)
            Z = np.full((nb_t, nb_p), np.nan)
            for a in range(nb_t):
                for b in range(nb_p):
                    m = (ti == a) & (pi == b)
                    if m.any():
                        Z[a, b] = J[m].mean()
            ax = axes[0, j]
            im = ax.imshow(Z, origin="lower", aspect="auto", cmap="Blues", vmin=vmin, vmax=vmax,
                           extent=(0, 1, -180, 180))
            ax.axhline(0, color="k", lw=0.6, ls=":")
            ax.axhline(68, color="k", lw=0.6, ls="--")
            ax.axhline(-112, color="k", lw=0.6, ls="--")
            ang, _ = preferred_axis(thr, psi, J)
            xc = (np.arange(N_PSI) + 0.5) / N_PSI
            for off in (0, -180, 180):
                y = ang + off
                ok = (y >= -180) & (y <= 180)
                ax.plot(xc[ok], y[ok], "o-", color="#f2a900", lw=1.8, ms=4)
            ax.set_ylim(-180, 180)
            ax.set_yticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])
            ax.set_xlabel("stance progress of supporting diagonal (0 touchdown, 1 lift-off)")
            ax.set_ylabel("push angle vs. stance diagonal (deg)")
            ax.set_title(f"v0 = {v0} m/s", fontsize=10)
        fig.colorbar(im, ax=axes, fraction=0.03, label=lab[crit])
        fig.suptitle(f"Push capability envelope ({crit}) relative to the supporting diagonal "
                     f"(both diagonals pooled by mirror symmetry)\n"
                     "dotted: stance diagonal (0°)   dashed: next diagonal (+68° / -112°)   "
                     "orange: axis of largest J (effective support axis)", fontsize=10, y=1.06)
        fn = os.path.join(out_dir, f"envelope_{crit}_relative.png")
        fig.savefig(fn, dpi=130, bbox_inches="tight")
        plt.close(fig)
        print("wrote", fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--criteria", default="fall,safe")
    ap.add_argument("--tilt-crit", type=float, default=TILT_CRIT)
    ap.add_argument("--verify", type=int, default=200, help="random cells for the monotonicity check")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tag = "_quick" if a.quick else ""
    csv_path = os.path.join(a.out, f"push_envelope{tag}.csv")
    map_path = os.path.join(ROOT, "data", f"push_envelope_bezier{tag}.npz")
    lines = []
    if a.analyse_only:
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
        old = os.path.join(a.out, "push_envelope_summary.txt")
        if os.path.exists(old):                    # keep the monotonicity result of the run
            with open(old) as f:
                lines += [l.rstrip("\n") for l in f if l.startswith("MONOTONICITY")][:1]
    else:
        jobs = build_jobs(a.quick, a.reps, a.criteria.split(","), a.tilt_crit)
        print(f"{len(jobs)} bisections (~{len(jobs) * (STEPS + 2)} simulations) on {a.workers} workers")
        rows = run_pool(bisect, jobs, a.workers, "bisect")
        rows.sort(key=lambda r: (r["criterion"], r["v0"], r["dir"], r["phi"], r["rep"]))
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("wrote", csv_path)
        cand = [r for r in rows if not r["censored"]]
        if a.verify and cand:
            rng = np.random.default_rng(0)
            pick = [cand[i] for i in rng.choice(len(cand), min(a.verify, len(cand)), replace=False)]
            ver = run_pool(verify, pick, a.workers, "verify")
            pl, fh = np.mean([v["pass_low"] for v in ver]), np.mean([v["fail_high"] for v in ver])
            msg = (f"MONOTONICITY CHECK ({len(ver)} random cells, fresh noise): passes at 0.85*J_max "
                   f"{100 * pl:.0f}%, fails at 1.15*J_fail {100 * fh:.0f}%  (both ~100% = clean threshold)")
            print(msg)
            lines.append(msg)
        rows = [dict(r) for r in rows]
    analyse(rows, a.out, map_path, a.tilt_crit, os.path.join(ROOT, "results", "pilot_phase.csv"), lines)


if __name__ == "__main__":
    main()
