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

STRATEGIES = ("raw", "limiter", "phase", "phase+limiter")
LIMITS = (1.5, 1.0, 3.0)          # the controller's default slew limits


class PhaseAwareCommander:
    def __init__(self, ctrl, cmap=None, strategy="phase", max_wait=0.5, big_change=0.15,
                 fall_weight=50.0, envelope=None):
        assert strategy in STRATEGIES, strategy
        self.c, self.cmap, self.env = ctrl, cmap, envelope
        self.strategy = strategy
        self.max_wait = max_wait            # gait periods
        self.big_change = big_change        # m/s (planar) or rad/s/2 (yaw): below this, apply now
        self.fall_weight = fall_weight      # deg of tilt per unit fall probability in the cost
        ctrl.acc_max = None if strategy in ("raw", "phase") else np.asarray(LIMITS, float)
        self.pending = None                 # (target, release_time)
        self.t = 0.0
        self.log = []                       # (t_request, t_release, wait_s, key)

    # ------------------------------------------------------------------ API
    def request(self, vx, vy, wz, urgent=False):
        target = np.array([vx, vy, wz], float)
        ref = self.pending[0] if self.pending is not None else self.c.cmd_target
        if np.allclose(target, ref, atol=1e-3):
            return                                           # nothing new
        if self.strategy in ("raw", "limiter") or urgent or self.cmap is None:
            self._release(target, 0.0)
            return
        key = self._event_key(target)
        if key is None:                                      # small change
            self._release(target, 0.0)
            return
        kind = "cmd_raw" if self.strategy == "phase" else "cmd_lim"
        v0 = float(np.clip(self.c.cmd_target[0], 0.0, 0.4))
        direction, size = key
        best = None
        for w in np.linspace(0.0, self.max_wait, int(64 * self.max_wait) + 1):
            p = (self.c.gait.phi + w) % 1.0
            pr = self.cmap.predict(kind, v0, direction, size, p)
            cost = pr["tilt"] + self.fall_weight * (pr["fall"] if np.isfinite(pr["fall"]) else 0.0)
            if np.isfinite(cost) and (best is None or cost < best[1] - 1e-9):
                best = (w, cost)
        wait_s = (best[0] if best else 0.0) * self.c.gait.period
        self.pending = (target, self.t + wait_s)
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

    def safe_push_force(self, direction, criterion="safe"):
        """Largest 0.1 s push (N) the robot tolerates right now from `direction` (deg, body frame)."""
        if self.env is None:
            return np.nan
        v0 = float(np.clip(self.c.cmd_target[0], 0.0, 0.4))
        return self.env.j_max(v0, direction, self.c.gait.phi, criterion)

    # ------------------------------------------------------------------ internals
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
        if wait_s == 0.0 and self.strategy in ("raw", "limiter"):
            self.log.append((self.t, self.t, 0.0, None))

    def _flush(self):
        target, _ = self.pending
        self.pending = None
        self.c.set_command(*target)
