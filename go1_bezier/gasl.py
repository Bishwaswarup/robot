"""Gait-aware slew limiter (GASL).

The uniform limiter lets the commanded velocity change by at most acc_max*dt per
control step, whatever the gait is doing. GASL scales the planar limit with the
robot's measured disturbance tolerance for the CURRENT gait phase and the direction
of the requested acceleration:

    s(theta, phi) built from the push envelope J_safe(v0, theta, phi)
    (d'Alembert: commanding an acceleration a in direction theta loads the legs like a
     push of m*a in that direction, so phases/directions that tolerate pushes well
     should also tolerate acceleration well)

    mode "fast" : s = clip(J / min_phi J, 1, k_max)   -> never below the uniform limit
                  (uniform at the robot's worst phase for that direction), faster elsewhere
    mode "equal": s = J / mean_phi J                  -> same average limit as uniform,
                  redistributed from weak to strong phases

The scale is evaluated every control step, so a ramp speeds up through good phases
and slows down through bad ones. Yaw keeps the uniform limit (no yaw envelope yet).

    lim = GaitAwareLimiter(PushEnvelope.load(...), mode="fast")
    ctrl.acc_scale_fn = lim
"""
import numpy as np


class GaitAwareLimiter:
    def __init__(self, envelope=None, mode="fast", k_max=2.0, criterion="safe", phase_lead=0.0,
                 n_dir=36, n_phase=64, cmap=None, source="push", cmd_size=0.4, uniform=1.0):
        """source="push": tolerance J = push envelope J_safe(theta, phi).
        source="cmd" : tolerance J = 1 / (tilt per m/s of an instantaneous velocity step at
                       phase phi in direction theta), from the capability map ('cmd_raw').
                       Under linear superposition the tilt of a ramp ~ integral of
                       a(t) * K(theta, phi(t)) dt, so acceleration belongs where K is small.
        mode "uniform": constant scale `uniform` (baseline family for fair comparison)."""
        self.mode, self.k_max, self.phase_lead = mode, k_max, phase_lead
        self.n_dir, self.n_phase = n_dir, n_phase
        dirs = np.arange(n_dir) * 360.0 / n_dir
        phases = np.arange(n_phase) / n_phase
        if source == "push":
            self.v0s = np.asarray(envelope.grids[criterion]["v0"], float)
            tol = lambda v0, th, p: envelope.j_max(v0, th, p, criterion)
        else:
            self.v0s = np.asarray(cmap.axes["cmd_raw"]["v0"], float)
            tol = lambda v0, th, p: cmd_size / max(cmap.predict("cmd_raw", v0, th, cmd_size, p, "tilt"), 1e-3)
        S = np.ones((len(self.v0s), n_dir, n_phase))
        for a, v0 in enumerate(self.v0s):
            for b, th in enumerate(dirs):
                if mode == "uniform":
                    S[a, b] = uniform
                    continue
                J = np.array([tol(v0, th, p) for p in phases])
                if mode == "fast":
                    S[a, b] = np.clip(J / np.nanmin(J), 1.0, k_max)
                elif mode == "equal":
                    S[a, b] = J / np.nanmean(J)
                elif mode.startswith("x"):                     # mean-normalised, times k
                    S[a, b] = float(mode[1:]) * J / np.nanmean(J)
                else:
                    raise ValueError(mode)
        self.S = S

    def scale(self, v0, direction_deg, phase):
        a = int(np.argmin(np.abs(self.v0s - np.clip(v0, self.v0s.min(), self.v0s.max()))))
        u = (direction_deg % 360.0) / 360.0 * self.n_dir
        b0 = int(u) % self.n_dir
        wb = u - int(u)
        q = ((phase + self.phase_lead) % 1.0) * self.n_phase
        c0 = int(q) % self.n_phase
        wc = q - int(q)
        S = self.S[a]
        b1, c1 = (b0 + 1) % self.n_dir, (c0 + 1) % self.n_phase
        return ((1 - wb) * ((1 - wc) * S[b0, c0] + wc * S[b0, c1]) +
                wb * ((1 - wc) * S[b1, c0] + wc * S[b1, c1]))

    def __call__(self, ctrl, err):
        if abs(err[0]) + abs(err[1]) < 1e-9:
            return 1.0, 1.0
        th = np.degrees(np.arctan2(err[1], err[0]))
        return float(self.scale(ctrl.cmd[0], th, ctrl.gait.phi)), 1.0


def parse_strategy(strategy):
    """Limiter-family strategy names (all release immediately):
         limiter*1.4             uniform limiter scaled 1.4x
         gasl_fast, gasl_equal   push-envelope shaped
         gaslc_equal, gaslc_x1.4 step-sensitivity (capability map) shaped, mean scale 1 / 1.4
         ...@0.125               phase lead (fraction of a gait period)
    Returns GaitAwareLimiter kwargs, or None for the commander's own strategies."""
    base, _, lead = strategy.partition("@")
    lead = float(lead) if lead else 0.0
    if base.startswith("limiter*"):
        return dict(mode="uniform", uniform=float(base.split("*")[1]))
    if base.startswith("gaslc_"):
        return dict(mode=base.split("_", 1)[1], source="cmd", phase_lead=lead)
    if base.startswith("gasl_"):
        return dict(mode=base.split("_", 1)[1], source="push", phase_lead=lead)
    return None


def make_commander(ctrl, strategy, cmap, envelope, cache=None):
    """PhaseAwareCommander for any strategy name (limiter families get a cached limiter)."""
    from .commander import PhaseAwareCommander
    kw = parse_strategy(strategy)
    if kw is None:
        return PhaseAwareCommander(ctrl, cmap, strategy, envelope=envelope)
    cache = {} if cache is None else cache
    if strategy not in cache:
        cache[strategy] = GaitAwareLimiter(envelope, cmap=cmap, **kw)
    return PhaseAwareCommander(ctrl, cmap, "gasl_fast", envelope=envelope, gasl=cache[strategy])
