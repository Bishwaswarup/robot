"""Phase-aware command timing: the layer between the swarm controller and one robot.

The swarm controller calls  request(vx, vy, wz)  whenever it wants (e.g. 10-20 Hz).
Small changes go straight through. A LARGE change is held back for at most
`max_wait` gait periods and released at the gait phase where the capability map
predicts the smallest disturbance (peak tilt + fall risk). `urgent=True` bypasses
the wait (e.g. imminent collision).

Strategies (for experiments):
    "raw"            no slew limiter, release immediately
    "limiter"        uniform slew limiter, release immediately   (old default)
    "phase"          no slew limiter, release at the best phase  (proposed)
    "phase+limiter"  slew limiter, released at the best phase
    "phase_safe"     like "phase" inside the map's domain; events outside it (combined
                     planar+yaw jumps, > 0.8 m/s, from non-straight or unsettled walking)
                     fall back to the uniform slew limiter
    "phase_soft"     best phase + a soft limiter (2x the default acceleration limits)
    "gasl_fast"      no waiting; gait-aware slew limiter, never below the uniform limit
    "gasl_equal"     no waiting; gait-aware slew limiter, same average limit as uniform
                     (both need envelope=PushEnvelope; see go1_bezier/gasl.py)

    cmap = CapabilityMap.load("data/capability_map_bezier.npz")
    com  = PhaseAwareCommander(ctrl, cmap, strategy="phase")
    com.request(0.6, 0.2, 0.0)          # any rate
    com.step()                          # every physics step (instead of ctrl.step())
    com.safe_push_force(90)             # optional, needs a PushEnvelope

Map conventions: direction of the velocity change in the body frame (0 = forward,
90 = left), size = |change| in m/s; pure yaw changes use 'yaw+'/'yaw-' with
size = |dwz| / 2 (as in the pilot). The map was measured at forward base speeds
0 and 0.4 m/s; other base velocities use the forward component, clamped.
"""
import numpy as np

STRATEGIES = ("raw", "limiter", "phase", "phase+limiter", "phase_safe", "phase_soft",
              "gasl_fast", "gasl_equal")
IMMEDIATE = ("raw", "limiter", "gasl_fast", "gasl_equal")      # never wait
LIMITS = (1.5, 1.0, 3.0)          # the controller's default slew limits
SOFT = (3.0, 2.0, 6.0)


class PhaseAwareCommander:
    def __init__(self, ctrl, cmap=None, strategy="phase", max_wait=0.5, big_change=0.15,
                 fall_weight=50.0, envelope=None, gasl=None):
        base, _, lead = strategy.partition("@")          # e.g. "gasl_fast@0.125" = phase lead
        assert base in STRATEGIES, strategy
        self.c, self.cmap, self.env = ctrl, cmap, envelope
        self.strategy = strategy = base
        self.max_wait = max_wait            # gait periods
        self.big_change = big_change        # m/s (planar) or rad/s/2 (yaw): below this, apply now
        self.fall_weight = fall_weight      # deg of tilt per unit fall probability in the cost
        ctrl.acc_max = {"raw": None, "phase": None, "phase_safe": None,
                        "phase_soft": np.asarray(SOFT, float)}.get(strategy, np.asarray(LIMITS, float))
        self.pending = None                 # (target, release_time)
        self.pending_key = None
        ctrl.acc_scale_fn = gasl if gasl is not None else None
        if strategy.startswith("gasl"):
            if gasl is None:
                from .gasl import GaitAwareLimiter
                assert envelope is not None, "gasl strategies need envelope=PushEnvelope(...)"
                gasl = GaitAwareLimiter(envelope, mode=strategy.split("_")[1],
                                        phase_lead=float(lead) if lead else 0.0)
            ctrl.acc_scale_fn = gasl
        self.last_release = -1e9
        self.n_fallback = 0
        self.n_requests = self.n_big = self.n_in_domain = self.n_urgent = 0
        self.t = 0.0
        self.log = []                       # (t_request, t_release, wait_s, key)

    # ------------------------------------------------------------------ API
    def request(self, vx, vy, wz, urgent=False, deadline=None):
        """deadline (s): the change is needed no later than this (planned manoeuvre from a
        predictive layer); timing strategies may wait up to min(max_wait, deadline)."""
        target = np.array([vx, vy, wz], float)
        ref = self.pending[0] if self.pending is not None else self.c.cmd_target
        if np.allclose(target, ref, atol=1e-3):
            return                                           # nothing new
        self.n_requests += 1
        key = self._event_key(target)
        if key is not None:                                  # statistics for every strategy
            self.n_big += 1
            self.n_in_domain += int(self.in_domain(target))
            self.n_urgent += int(urgent)
        if self.pending is not None and key is not None and not urgent and \
                self._similar(key, self.pending_key):
            self.pending = (target, self.pending[1])         # same manoeuvre, refined: keep timing
            return
        if self.strategy in IMMEDIATE or urgent or self.cmap is None:
            self._release(target, 0.0)
            return
        if key is None:                                      # small change
            self._release(target, 0.0)
            return
        if self.strategy == "phase_safe" and deadline is None:
            if not self.in_domain(target):                   # unknown territory -> be careful
                self.n_fallback += 1
                self.c.acc_max = np.asarray(LIMITS, float)
                self._release(target, 0.0)
                return
            self.c.acc_max = None
        kind = "cmd_raw" if self.strategy in ("phase", "phase_safe") else "cmd_lim"
        v0 = float(np.clip(self.c.cmd_target[0], 0.0, 0.4))
        direction, size = key
        best = None
        max_wait = self.max_wait if deadline is None else min(self.max_wait, deadline / self.c.gait.period)
        for w in np.linspace(0.0, max_wait, max(2, int(64 * max_wait) + 1)):
            p = (self.c.gait.phi + w) % 1.0
            pr = self.cmap.predict(kind, v0, direction, size, p)
            cost = pr["tilt"] + self.fall_weight * (pr["fall"] if np.isfinite(pr["fall"]) else 0.0)
            if np.isfinite(cost) and (best is None or cost < best[1] - 1e-9):
                best = (w, cost)
        wait_s = (best[0] if best else 0.0) * self.c.gait.period
        self.pending = (target, self.t + wait_s)
        self.pending_key = key
        self.log.append((self.t, self.t + wait_s, wait_s, key))
        if wait_s <= 0.0:
            self._flush()

    def step(self):
        """Call once per physics step (replaces ctrl.step())."""
        if self.pending is not None and self.t >= self.pending[1] - 1e-12:
            self._flush()
        self.c.step()
        self.t += self.c.m.opt.timestep

    @property
    def waiting(self):
        return self.pending is not None

    def in_domain(self, target):
        """Is this event like the ones the capability map was measured on? Straight,
        settled walking at 0-0.4 m/s and a single planar OR yaw change <= 0.8."""
        base, d = self.c.cmd, target - self.c.cmd_target
        settled = np.allclose(self.c.cmd, self.c.cmd_target, atol=0.05) and \
            (self.t - self.last_release) > self.c.gait.period
        straight = -0.05 <= base[0] <= 0.45 and abs(base[1]) < 0.1 and abs(base[2]) < 0.3
        planar, yaw = float(np.hypot(d[0], d[1])), abs(d[2]) / 2.0
        single = not (planar >= self.big_change and yaw >= self.big_change)
        return settled and straight and single and planar <= 0.85 and yaw <= 0.85

    def safe_push_force(self, direction, criterion="safe"):
        """Largest 0.1 s push (N) the robot tolerates right now from `direction` (deg, body frame)."""
        if self.env is None:
            return np.nan
        v0 = float(np.clip(self.c.cmd_target[0], 0.0, 0.4))
        return self.env.j_max(v0, direction, self.c.gait.phi, criterion)

    # ------------------------------------------------------------------ internals
    @staticmethod
    def _similar(a, b, ang=30.0, size=0.15):
        if a is None or b is None or isinstance(a[0], str) != isinstance(b[0], str):
            return False
        if isinstance(a[0], str):
            return a[0] == b[0] and abs(a[1] - b[1]) <= size
        da = abs((a[0] - b[0] + 180.0) % 360.0 - 180.0)
        return da <= ang and abs(a[1] - b[1]) <= size

    def _event_key(self, target):
        d = target - self.c.cmd_target
        planar = float(np.hypot(d[0], d[1]))
        if planar >= self.big_change:
            return float(np.degrees(np.arctan2(d[1], d[0])) % 360.0), float(np.clip(planar, 0.2, 0.8))
        if abs(d[2]) / 2.0 >= self.big_change:
            return ("yaw+" if d[2] > 0 else "yaw-"), float(np.clip(abs(d[2]) / 2.0, 0.2, 0.8))
        return None

    def _release(self, target, wait_s):
        self.pending = None
        self.c.set_command(*target)
        self.last_release = self.t
        if wait_s == 0.0 and self.strategy in IMMEDIATE:
            self.log.append((self.t, self.t, 0.0, None))

    def _flush(self):
        target, _ = self.pending
        self.pending = None
        self.c.set_command(*target)
        self.last_release = self.t
