"""Locomotion capability map: how badly does a disturbance hurt the robot, given
WHEN in the gait cycle it happens?

Built from scripts/pilot_phase.py data. For each event type
    "cmd_lim"  sudden velocity-command change, slew limiter ON
    "cmd_raw"  sudden velocity-command change, slew limiter OFF
    "push"     0.1 s push on the trunk
it stores, on a grid of (base speed v0, size, direction, gait phase), the mean of
    tilt (peak, deg), fall (probability), recovery (s), slip (excess foot slip, mm).

Trot symmetry is used to pool data: a left-right mirror maps direction theta -> -theta
and swaps the diagonal leg pairs, i.e. phase phi -> phi + 0.5. (Checked on the pilot
data: r = 0.993 between mirrored cells.)

Queries interpolate linearly: periodic in phase and direction, clamped in v0 and size.

    cmap = CapabilityMap.load("data/capability_map_bezier.npz")
    cmap.predict("cmd_raw", v0=0.4, direction=90, size=0.6, phase=0.3)
    cmap.best_phase("cmd_raw", 0.4, 90, 0.6, current_phase=0.3, max_wait=0.5)
"""
import numpy as np

METRICS = ("tilt", "fall", "recovery", "slip")
KINDS = ("cmd_lim", "cmd_raw", "push")
PLANAR_DIRS = np.arange(0, 360, 45)          # deg, body frame (0 = forward, 90 = left)


def _kind_of(row_kind, limiter):
    return "push" if row_kind == "push" else ("cmd_lim" if limiter else "cmd_raw")


class CapabilityMap:
    def __init__(self, grids, axes):
        self.grids = grids            # kind -> metric -> array [v0, size, dir(10), phase]
        self.axes = axes              # kind -> dict(v0=..., size=..., dirs=[...], phase=...)

    # ------------------------------------------------------------------ build
    @classmethod
    def from_rows(cls, rows, symmetrize=True):
        """rows: iterable of dicts / DataFrame records from pilot_phase.csv."""
        import collections
        acc = collections.defaultdict(list)
        axes = {}
        for r in rows:
            k = _kind_of(r["kind"], str(r["limiter"]) in ("True", "1", "true") or r["limiter"] is True)
            d = str(r["dir"])
            vals = (float(r["peak_tilt"]), float(r["fell"]),
                    float(r["recovery_s"]) if str(r["recovery_s"]) not in ("", "nan") else np.nan,
                    float(r["excess_slip_mm"]) if not int(r["fell"]) else np.nan)
            vals = (vals[0] if not int(r["fell"]) else np.nan,) + vals[1:]   # tilt of survivors
            phi = round(float(r["phi"]) % 1.0, 4)
            keys = [(k, float(r["v0"]), float(r["mag"]), d, phi)]
            if symmetrize:
                md = ("yaw-" if d == "yaw+" else "yaw+") if d.startswith("yaw") else str((-int(d)) % 360)
                keys.append((k, float(r["v0"]), float(r["mag"]), md, round((phi + 0.5) % 1.0, 4)))
            for key in keys:
                acc[key].append(vals)
                ax = axes.setdefault(k, dict(v0=set(), size=set(), dirs=set(), phase=set()))
                ax["v0"].add(key[1]); ax["size"].add(key[2]); ax["dirs"].add(key[3]); ax["phase"].add(key[4])
        grids = {}
        for k, ax in axes.items():
            ax["v0"] = np.array(sorted(ax["v0"]))
            ax["size"] = np.array(sorted(ax["size"]))
            ax["phase"] = np.array(sorted(ax["phase"]))
            ax["dirs"] = [str(x) for x in PLANAR_DIRS if str(x) in ax["dirs"]] + \
                         [y for y in ("yaw+", "yaw-") if y in ax["dirs"]]
            shape = (len(ax["v0"]), len(ax["size"]), len(ax["dirs"]), len(ax["phase"]))
            grids[k] = {m: np.full(shape, np.nan) for m in METRICS}
            for a, v0 in enumerate(ax["v0"]):
                for b, sz in enumerate(ax["size"]):
                    for c, dr in enumerate(ax["dirs"]):
                        for e, ph in enumerate(ax["phase"]):
                            v = np.array(acc.get((k, v0, sz, dr, ph), []), float)
                            if len(v):
                                for mi, m in enumerate(METRICS):
                                    col = v[:, mi]
                                    if m == "fall":
                                        grids[k][m][a, b, c, e] = col.mean()
                                    elif np.isfinite(col).any():
                                        grids[k][m][a, b, c, e] = np.nanmean(col)
        return cls(grids, axes)

    @classmethod
    def from_csv(cls, path, symmetrize=True):
        import csv
        with open(path) as f:
            return cls.from_rows(list(csv.DictReader(f)), symmetrize)

    def save(self, path):
        out = {}
        for k in self.grids:
            for m in METRICS:
                out[f"{k}/{m}"] = self.grids[k][m]
            ax = self.axes[k]
            out[f"{k}/v0"], out[f"{k}/size"], out[f"{k}/phase"] = ax["v0"], ax["size"], ax["phase"]
            out[f"{k}/dirs"] = np.array(ax["dirs"])
        np.savez_compressed(path, **out)

    @classmethod
    def load(cls, path):
        z = np.load(path)
        grids, axes = {}, {}
        for k in KINDS:
            if f"{k}/v0" not in z:
                continue
            grids[k] = {m: z[f"{k}/{m}"] for m in METRICS}
            axes[k] = dict(v0=z[f"{k}/v0"], size=z[f"{k}/size"], phase=z[f"{k}/phase"],
                           dirs=[str(x) for x in z[f"{k}/dirs"]])
        return cls(grids, axes)

    # ------------------------------------------------------------------ query
    @staticmethod
    def _lin(axis, x):
        """indices + weights for clamped linear interpolation on a sorted axis."""
        if len(axis) == 1:
            return [(0, 1.0)]
        x = float(np.clip(x, axis[0], axis[-1]))
        i = int(np.clip(np.searchsorted(axis, x) - 1, 0, len(axis) - 2))
        w = (x - axis[i]) / (axis[i + 1] - axis[i])
        return [(i, 1 - w), (i + 1, w)]

    @staticmethod
    def _per(n, x):
        """periodic linear interpolation on n equally spaced points over one period (x in units of period)."""
        u = (x % 1.0) * n
        i = int(np.floor(u)) % n
        w = u - np.floor(u)
        return [(i, 1 - w), ((i + 1) % n, w)]

    def _dir_terms(self, kind, direction):
        dirs = self.axes[kind]["dirs"]
        if isinstance(direction, str):
            return [(dirs.index(direction), 1.0)]
        planar = [i for i, d in enumerate(dirs) if not d.startswith("yaw")]
        return [(planar[i], w) for i, w in self._per(len(planar), float(direction) / 360.0)]

    def predict(self, kind, v0, direction, size, phase, metric=None):
        """direction: degrees in body frame (0 fwd, 90 left) or 'yaw+'/'yaw-'.
        Returns dict of metrics (or one float if metric is given)."""
        ax, G = self.axes[kind], self.grids[kind]
        terms = [(a, b, c, e, wa * wb * wc * we)
                 for a, wa in self._lin(ax["v0"], v0)
                 for b, wb in self._lin(ax["size"], size)
                 for c, wc in self._dir_terms(kind, direction)
                 for e, we in self._per(len(ax["phase"]), phase)]
        out = {}
        for m in ((metric,) if metric else METRICS):
            num = den = 0.0
            for a, b, c, e, w in terms:
                v = G[m][a, b, c, e]
                if w > 0 and np.isfinite(v):
                    num += w * v
                    den += w
            out[m] = num / den if den > 0 else np.nan
        return out[metric] if metric else out

    def phase_profile(self, kind, v0, direction, size, metric="tilt", n=64):
        ph = np.arange(n) / n
        return ph, np.array([self.predict(kind, v0, direction, size, p, metric) for p in ph])

    def best_phase(self, kind, v0, direction, size, current_phase, max_wait=0.5, metric="tilt",
                   n=64, fall_weight=None):
        """Best phase to release the event within the next `max_wait` gait periods.
        Returns (phase, wait_in_periods, predicted_metric). If fall_weight is given the
        cost is metric + fall_weight * fall probability."""
        waits = np.linspace(0, max_wait, max(2, int(n * max_wait) + 1))
        best = None
        for w in waits:
            p = (current_phase + w) % 1.0
            pr = self.predict(kind, v0, direction, size, p)
            cost = pr[metric] + (fall_weight * pr["fall"] if fall_weight else 0.0)
            if np.isfinite(cost) and (best is None or cost < best[3] - 1e-9):
                best = (p, w, pr[metric], cost)
        return best[:3]
