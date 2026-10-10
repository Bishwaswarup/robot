"""Decentralised swarm layer: goal / formation preference + ORCA + heading control.

Each robot runs the same code with only LOCAL information: its own state, the
positions/velocities of neighbours within `sense_radius`, and nearby obstacles.
The only shared item in formation mode is the plan of a virtual reference point
(start, goal, speed, start time), i.e. a common clock, not a central controller.

    agent = SwarmAgent(radius=0.38, v_max=0.6)
    v_world = agent.velocity(p, v, v_pref, neighbours, obstacles)
    vx, vy, wz = agent.body_command(v_world, yaw)       # -> commander.request(...)
"""
import numpy as np
from .orca import orca_velocity


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class SwarmAgent:
    def __init__(self, radius=0.38, v_max=0.6, tau=2.0, tau_obs=1.0, dt=0.1, sense_radius=4.0,
                 max_neighbours=10, keep_right_deg=15.0, k_heading=2.0, w_max=1.2, vy_max=0.35,
                 turn_gate=False, escape=False):
        self.r, self.v_max, self.tau, self.tau_obs, self.dt = radius, v_max, tau, tau_obs, dt
        self.sense_radius, self.max_nb = sense_radius, max_neighbours
        self.keep_right = np.radians(keep_right_deg)
        self.k_heading, self.w_max, self.vy_max = k_heading, w_max, vy_max
        # turn_gate: keep body commands inside the envelope the gait was stress-tested on
        # (|wz| <= 1.0, |vy| <= 0.3) and slow down while the heading error is large
        # (turn first, then walk) - avoids fast sideways walking + max yaw rate together
        self.turn_gate = turn_gate
        if turn_gate:
            self.w_max, self.vy_max = min(w_max, 1.0), min(vy_max, 0.3)
        self.v_prev = np.zeros(2)          # ORCA uses the last chosen velocity (reciprocity)
        self.heading_ref = None
        # escape: local-minimum breaker. If ORCA keeps returning ~0 while the robot wants to move
        # (pressed against a wall between it and its slot) for > 1 s, follow the obstacle for 2 s:
        # preferred velocity turned 75 deg towards `escape_to` (caller sets it each tick, e.g. the
        # formation reference point / path ahead); the side flips if the robot is still stuck.
        self.escape, self.escape_to = escape, None
        self.stall_t, self.detour_t, self.detour_side, self.n_detours = 0.0, 0.0, 1.0, 0
        self.progress = 0.3

    def velocity(self, p, v_pref, neighbours, obstacles, far_from_goal=True):
        """neighbours: list of (p_j, v_j, r_j); obstacles: list of (p_o, r_o)."""
        p = np.asarray(p, float)
        nb = sorted(((np.linalg.norm(np.asarray(n[0]) - p), n) for n in neighbours), key=lambda x: x[0])
        nb = [n for dist, n in nb if dist < self.sense_radius][: self.max_nb]
        ob = [(q, rq) for q, rq in obstacles if np.linalg.norm(np.asarray(q) - p) - rq < self.sense_radius]
        v_pref = np.asarray(v_pref, float)
        if far_from_goal and np.linalg.norm(v_pref) > 1e-3:      # keep right: breaks symmetric deadlocks
            v_pref = rot(-self.keep_right) @ v_pref
        pref_hat = v_pref / max(np.linalg.norm(v_pref), 1e-9)      # direction BEFORE any detour
        pref_speed = np.linalg.norm(v_pref)
        if self.escape and self.detour_t > 0:
            v_pref = rot(self.detour_side * np.radians(75)) @ v_pref
            self.detour_t -= self.dt
        v = orca_velocity(p, self.v_prev, self.r, v_pref, self.v_max, nb, ob,
                          self.tau, self.tau_obs, self.dt)
        if self.escape:
            # stalled = little PROGRESS along the preferred direction over ~1 s (an EMA of
            # v . pref_hat), not just low speed: a robot jittering in front of an obstacle counts
            self.progress += (float(v @ pref_hat) - self.progress) * self.dt / 1.0
            stalled = pref_speed > 0.3 and self.progress < 0.05
            self.stall_t = self.stall_t + self.dt if stalled else 0.0
            if self.detour_t <= 0 and not stalled:
                self.n_detours = 0
            if self.stall_t > 1.0 and self.detour_t <= 0:
                side = 1.0
                if self.escape_to is not None:
                    to = np.asarray(self.escape_to, float) - p
                    side = 1.0 if pref_hat[0] * to[1] - pref_hat[1] * to[0] >= 0 else -1.0
                self.detour_side = side if self.n_detours % 2 == 0 else -side
                self.detour_t, self.stall_t = 2.0, 0.0
                self.progress = 0.3                                   # give the detour time to work
                self.n_detours += 1
        self.v_prev = v
        return v

    def body_command(self, v_world, yaw, heading=None):
        """World velocity -> (vx, vy, wz) in the body frame. Heading follows the motion
        direction (Go1 walks best forward), or `heading` if given."""
        speed = np.linalg.norm(v_world)
        if heading is None:
            if speed > 0.15:
                self.heading_ref = np.arctan2(v_world[1], v_world[0])
            heading = yaw if self.heading_ref is None else self.heading_ref
        err = wrap(heading - yaw)
        wz = float(np.clip(self.k_heading * err, -self.w_max, self.w_max))
        if self.turn_gate:      # full speed below 30 deg heading error, 30 % at >= 90 deg
            v_world = np.asarray(v_world, float) * float(np.interp(abs(err), [np.radians(30), np.radians(90)], [1.0, 0.3]))
        vb = rot(-yaw) @ v_world
        return float(vb[0]), float(np.clip(vb[1], -self.vy_max, self.vy_max)), wz


    def predictive(self, p, v_cmd, v_pref_fn, neighbours, obstacles, lookahead, far_from_goal=True):
        """ORCA solved for the situation `lookahead` seconds ahead: own position extrapolated
        with the current command, neighbours with their measured velocities. Returns the
        velocity the robot will need then -> can be requested with a deadline (planned)
        instead of reactively. v_pref_fn(p_future) gives the preferred velocity there."""
        p_f = np.asarray(p, float) + np.asarray(v_cmd, float) * lookahead
        nb_f = [(np.asarray(n[0]) + np.asarray(n[1]) * lookahead,) + tuple(n[1:]) for n in neighbours]
        self.v_prev = np.asarray(v_cmd, float)
        return self.velocity(p_f, v_pref_fn(p_f), nb_f, obstacles, far_from_goal)


def lag_priority(err_self, err_nbs, neighbours, settled=0.4, late=1.0, yield_resp=0.9):
    """Late robots get right of way: a robot already at its slot/goal (error < settled) takes
    most of the avoidance (moves aside) for a neighbour that is still far from its own
    (error > late), and vice versa. Otherwise the existing responsibility is kept (0.5 default).
    Fixes 'late robot cannot get through the settled formation' deadlocks."""
    out = []
    for n, e in zip(neighbours, err_nbs):
        resp = n[3] if len(n) > 3 else 0.5
        if err_self < settled and e > late:
            resp = yield_resp
        elif err_self > late and e < settled:
            resp = 1.0 - yield_resp
        out.append((n[0], n[1], n[2], resp))
    return out


def queue_responsibility(p, heading, neighbours, in_bottleneck, ahead=0.3, yield_resp=0.9, lead_resp=0.1):
    """Priority at bottlenecks: the robot further back yields (takes most of the avoidance),
    the one in front keeps going. Outside bottlenecks everything stays reciprocal (0.5)."""
    if not in_bottleneck:
        return neighbours
    fwd = np.array([np.cos(heading), np.sin(heading)])
    out = []
    for n in neighbours:
        s = (np.asarray(n[0]) - np.asarray(p)) @ fwd
        resp = yield_resp if s > ahead else (lead_resp if s < -ahead else 0.5)
        out.append((n[0], n[1], n[2], resp))
    return out


class FormationPlan:
    """Virtual reference point moving from `start` to `goal` at `speed` (shared clock);
    robot i's slot = reference + R(heading) @ offsets[i]."""

    def __init__(self, start, goal, offsets, speed=0.35, t0=0.0, k_p=0.8, v_max=0.6):
        self.start, self.goal = np.asarray(start, float), np.asarray(goal, float)
        self.offsets = np.asarray(offsets, float)
        d = self.goal - self.start
        self.L = np.linalg.norm(d)
        self.dir = d / self.L
        self.heading = np.arctan2(d[1], d[0])
        self.speed, self.t0, self.k_p, self.v_max = speed, t0, k_p, v_max

    def reference(self, t):
        s = np.clip((t - self.t0) * self.speed, 0.0, self.L)
        v = self.dir * self.speed if s < self.L else np.zeros(2)
        return self.start + s * self.dir, v

    def slot(self, i, t, lateral_scale=1.0):
        ref, v = self.reference(t)
        off = self.offsets[i] * np.array([1.0, lateral_scale])
        return ref + rot(self.heading) @ off, v

    def preferred_velocity(self, i, p, t, lateral_scale=1.0):
        target, v_ref = self.slot(i, t, lateral_scale)
        v = v_ref + self.k_p * (target - np.asarray(p, float))
        n = np.linalg.norm(v)
        return v * (self.v_max / n) if n > self.v_max else v


def goal_velocity(p, goal, v_max=0.6, slow_radius=0.6):
    d = np.asarray(goal, float) - np.asarray(p, float)
    n = np.linalg.norm(d)
    if n < 1e-6:
        return np.zeros(2)
    return d / n * v_max * min(1.0, n / slow_radius)


def elastic_scale(p, heading, obstacles, look=2.5, half_angle=np.radians(70), min_scale=0.1):
    """Formation elasticity from LOCAL sensing: shrink the lateral slot offsets when
    obstacles are close ahead (1 = nominal formation, min_scale = near single file)."""
    p = np.asarray(p, float)
    fwd = np.array([np.cos(heading), np.sin(heading)])
    d_min = np.inf
    for q, r in obstacles:
        rel = np.asarray(q, float) - p
        dist = np.linalg.norm(rel) - r
        if dist < look and rel @ fwd > np.cos(half_angle) * np.linalg.norm(rel):
            d_min = min(d_min, dist)
    if not np.isfinite(d_min):
        return 1.0
    return float(np.clip((d_min - 0.3) / (look - 0.3), min_scale, 1.0))
