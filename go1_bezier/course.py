"""Obstacle course for one robot or a formation.

Sections along x (the formation follows a waypoint path at constant speed):
  -3-3    start (room for a team to fall into single file)
   3-9    slalom: three poles on the centre line, path weaves +-1.3 m around them
   9-12   gate in a wall (1.9 m opening)
  12-20   chicane: two staggered walls, path swings to y=+2.4 then y=-2.4
  20-26   crossing: two obstacles moving across the course in opposite directions (0.4 m/s),
          TIMED from the reference speed so they cut across the team's path (the first one
          just in front of the leading robot, the second one in front of the last row)
  26-31   random pillar field
  31-38   finish straight (room for the team to re-form)
Side walls at y = +-4.2 keep everything inside the course.

    course = obstacle_course(n_robots=6)
    course["plan"]       PathPlan (formation reference along the waypoints)
    course["obstacles"]  static discs ((x, y), r)
    course["movers"]     [Mover]   dynamic disc obstacles
    course["sections"]   [(name, x_start, x_end)]
"""
import numpy as np
from .swarm import rot

WALL_R, WALL_STEP = 0.22, 0.4


def _wall(p0, p1, r=WALL_R, step=WALL_STEP):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / step)) + 1)
    return [((float(x), float(y)), r) for x, y in np.linspace(p0, p1, n)]


class Mover:
    """Disc that moves back and forth between a and b at constant speed."""

    def __init__(self, a, b, speed=0.4, radius=0.3, phase=0.0):
        self.a, self.b = np.asarray(a, float), np.asarray(b, float)
        self.speed, self.r = speed, radius
        self.L = np.linalg.norm(self.b - self.a)
        self.phase = phase

    @classmethod
    def timed(cls, a, b, t_cross, speed=0.4, radius=0.3, reverse=False):
        """Mover between a and b that passes the MIDPOINT at time t_cross, heading a->b
        (or b->a with reverse=True)."""
        mv = cls(a, b, speed, radius)
        mv.phase = ((1.5 if reverse else 0.5) - t_cross * speed / mv.L) % 2.0
        return mv

    def state(self, t):
        s = (t * self.speed / self.L + self.phase) % 2.0
        u = s if s < 1.0 else 2.0 - s
        d = (self.b - self.a) / self.L * self.speed * (1 if s < 1.0 else -1)
        return self.a + u * (self.b - self.a), d


class PathPlan:
    """Formation reference moving along a polyline at `speed`. Slots are CURVILINEAR:
    offset (dx, dy) means dx metres along the path from the reference (negative = behind)
    and dy metres to the left of the path there, so the team bends through turns like a
    train instead of swinging a rigid formation around. Same interface as
    swarm.FormationPlan."""

    def __init__(self, waypoints, offsets, speed=0.3, t0=0.0, k_p=0.8, v_max=0.6, smooth=0.8,
                 queue_gap=0.95):
        self.W = np.asarray(waypoints, float)
        seg = np.diff(self.W, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.r_[0.0, np.cumsum(self.seg_len)]
        self.L = self.cum[-1]
        self.offsets = np.asarray(offsets, float)
        self.speed, self.t0, self.k_p, self.v_max, self.smooth = speed, t0, k_p, v_max, smooth
        self.start, self.goal = self.W[0], self.W[-1]
        # queue order when the team collapses to single file: front row first, left to right
        order = sorted(range(len(self.offsets)), key=lambda i: (-self.offsets[i][0], -self.offsets[i][1]))
        self.queue = np.empty(len(self.offsets), int)
        self.queue[order] = np.arange(len(order))
        self.queue_gap = queue_gap
        self.s_grid, self.lam_grid = None, None

    def set_corridor(self, obstacles, clearance=0.75, step=0.25):
        """Precompute how wide the team may be along the path: lam(s) = fraction of the
        nominal lateral offsets that keeps every slot `clearance` m from static obstacles.
        (Static course geometry is known to all robots; moving obstacles are left to ORCA.)"""
        max_dy = max(1e-6, float(np.max(np.abs(self.offsets[:, 1]))))
        P = np.array([q for q, _ in obstacles]); R = np.array([r for _, r in obstacles])
        self.s_grid = np.arange(0.0, self.L + step, step)
        lam = np.ones(len(self.s_grid))
        for k, s in enumerate(self.s_grid):
            p, h = self.point(s), self.heading_s(s)
            nrm = np.array([-np.sin(h), np.cos(h)])
            best = 0.0
            for cand in np.linspace(1.0, 0.0, 9):
                ok = True
                for side in (-1, 1):
                    q = p + side * nrm * max_dy * cand
                    if np.any(np.linalg.norm(P - q, axis=1) - R < clearance):
                        ok = False
                        break
                if ok:
                    best = cand
                    break
            lam[k] = best
        self.lam_grid = lam
        return self

    def team_scale(self, t, ahead=3.0):
        """Shared lateral scale for the whole team at time t: the narrowest point between the
        tail of the team and `ahead` metres in front of the reference (collapse early, stay
        collapsed until the last robot is through)."""
        if self.lam_grid is None:
            return 1.0
        s = self.s_of(t)
        tail = max(abs(self.offsets[:, 0]).max(), (len(self.offsets) - 1) * self.queue_gap) + 1.0
        m = (self.s_grid >= s - tail) & (self.s_grid <= s + ahead)
        return float(self.lam_grid[m].min()) if m.any() else 1.0

    def point(self, s):
        s = float(np.clip(s, 0.0, self.L))
        k = int(np.clip(np.searchsorted(self.cum, s, side="right") - 1, 0, len(self.seg_len) - 1))
        u = (s - self.cum[k]) / self.seg_len[k]
        return self.W[k] + u * (self.W[k + 1] - self.W[k])

    def s_of(self, t):
        return float(np.clip((t - self.t0) * self.speed, 0.0, self.L))

    def heading_s(self, s):
        d = self.point(s + self.smooth) - self.point(s - self.smooth)
        return float(np.arctan2(d[1], d[0]))

    def heading_at(self, t):
        return self.heading_s(self.s_of(t))

    def reference(self, t):
        s = self.s_of(t)
        h = self.heading_s(s)
        v = self.speed * np.array([np.cos(h), np.sin(h)]) if s < self.L else np.zeros(2)
        return self.point(s), v

    def slot(self, i, t, lateral_scale=1.0):
        """lateral_scale 1 = nominal formation; 0 = single file (queue order, queue_gap apart)."""
        s_ref = self.s_of(t)
        dx, dy = self.offsets[i]
        lam = float(np.clip(lateral_scale, 0.0, 1.0))
        s_i = s_ref + lam * dx - (1.0 - lam) * self.queue[i] * self.queue_gap
        h = self.heading_s(s_i)
        base = self.point(s_i)
        if s_i < 0:                                   # behind the start: extend backwards
            base = self.W[0] + s_i * np.array([np.cos(h), np.sin(h)])
        elif s_i > self.L:
            base = self.W[-1] + (s_i - self.L) * np.array([np.cos(h), np.sin(h)])
        normal = np.array([-np.sin(h), np.cos(h)])
        v = self.speed * np.array([np.cos(h), np.sin(h)]) if s_ref < self.L else np.zeros(2)
        return base + normal * dy * lam, v

    def preferred_velocity(self, i, p, t, lateral_scale=1.0):
        target, v_ref = self.slot(i, t, lateral_scale)
        v = v_ref + self.k_p * (target - np.asarray(p, float))
        n = np.linalg.norm(v)
        return v * (self.v_max / n) if n > self.v_max else v

    def progress_window(self, p, s_lo, s_hi, step=0.1):
        """Arc length of the closest path point to p, searched only in [s_lo, s_hi]
        (track each robot's progress over time; avoids jumping to a neighbouring leg)."""
        ss = np.arange(max(0.0, s_lo), min(self.L, s_hi) + step, step)
        if len(ss) == 0:
            return float(np.clip(s_lo, 0.0, self.L))
        pts = np.array([self.point(x) for x in ss])
        return float(ss[np.argmin(np.linalg.norm(pts - np.asarray(p, float), axis=1))])

    def preferred_velocity_path(self, i, p, t, s_robot, lateral_scale=1.0, carrot=1.0):
        """Follow the PATH to the slot instead of a straight line (no corner cutting through
        walls): aim at a carrot `carrot` m ahead of the robot's own progress, at its lateral
        offset; close to the slot, plain slot tracking."""
        lam = float(np.clip(lateral_scale, 0.0, 1.0))
        s_slot = self.slot_s(i, t, lam)
        if s_slot - s_robot <= carrot:
            return self.preferred_velocity(i, p, t, lam)
        s_c = s_robot + carrot
        h = self.heading_s(s_c)
        target = self.point(s_c) + np.array([-np.sin(h), np.cos(h)]) * self.offsets[i][1] * lam
        d = target - np.asarray(p, float)
        n = np.linalg.norm(d)
        return d / n * self.v_max if n > 1e-6 else np.zeros(2)

    def progress(self, p):
        """Arc length of the closest point of the path to p."""
        p = np.asarray(p, float)
        best, s_best = np.inf, 0.0
        for k in range(len(self.seg_len)):
            a, b = self.W[k], self.W[k + 1]
            u = np.clip((p - a) @ (b - a) / self.seg_len[k] ** 2, 0, 1)
            d = np.linalg.norm(a + u * (b - a) - p)
            if d < best:
                best, s_best = d, self.cum[k] + u * self.seg_len[k]
        return s_best

    def slot_s(self, i, t, lateral_scale=1.0):
        lam = float(np.clip(lateral_scale, 0.0, 1.0))
        return self.s_of(t) + lam * self.offsets[i][0] - (1.0 - lam) * self.queue[i] * self.queue_gap

    def lag(self, positions, t, idx=None, lateral_scale=1.0):
        """Mean distance (m, along the path) by which robots trail their own slots."""
        idx = range(len(positions)) if idx is None else idx
        return float(np.mean([self.slot_s(i, t, lateral_scale) - self.progress(p) for i, p in zip(idx, positions)]))


def team_offsets(n):
    if n == 1:
        return [(0.0, 0.0)]
    if n == 6:
        return [(-dx, dy) for dx in (0.0, 1.2) for dy in (-1.2, 0.0, 1.2)]
    if n == 9:
        return [(-dx, dy) for dx in (0.0, 1.2, 2.4) for dy in (-1.2, 0.0, 1.2)]
    if n == 4:
        return [(-dx, dy) for dx in (0.0, 1.2) for dy in (-0.6, 0.6)]
    raise ValueError(n)


SLALOM_POLES = [(4.0, 0.0), (6.0, 0.0), (8.0, 0.0)]
SLALOM_AMP = 1.3


def obstacle_course(n_robots=1, speed=0.3, lag=0.6, mover_speed=0.4):
    """lag: expected distance (m) robots trail their slots, used only to time the movers."""
    obs = []
    obs += _wall((-5.5, 4.2), (39.5, 4.2)) + _wall((-5.5, -4.2), (39.5, -4.2))      # side walls
    obs += [(q, 0.25) for q in SLALOM_POLES]                                          # slalom poles
    obs += _wall((10.5, -4.2), (10.5, -0.95 - WALL_R)) + _wall((10.5, 0.95 + WALL_R), (10.5, 4.2))  # gate
    obs += _wall((14.0, -4.2), (14.0, 0.6))                                           # chicane wall 1
    obs += _wall((17.5, -0.6), (17.5, 4.2))                                           # chicane wall 2
    rng = np.random.default_rng(11)                                                   # pillar field
    field = []
    while len(field) < 7:
        q = (rng.uniform(26.5, 30.5), rng.uniform(-3.2, 3.2))
        if all(np.hypot(q[0] - f[0], q[1] - f[1]) > 1.9 for f, _ in field):
            field.append((q, 0.3))
    obs += field
    A = SLALOM_AMP
    waypoints = [(-3.0, 0.0), (3.0, 0.0), (4.0, -A), (6.0, A), (8.0, -A), (9.2, 0.0), (11.5, 0.0),
                 (12.6, 0.0), (13.2, 2.4), (15.75, 2.4), (15.75, 0.0), (15.75, -2.4), (17.5, -2.4),
                 (18.6, 0.0), (38.0, 0.0)]
    offs = team_offsets(n_robots)
    plan = PathPlan(waypoints, offs, speed=speed).set_corridor(obs)
    # movers: timed from the plan so they really cut across the team. Last leg is the straight
    # y=0 from x=18.6, so s(x) = cum[-2] + (x - 18.6).
    s_x = lambda x: plan.cum[-2] + (x - 18.6)
    depth = max(0.0, -min(o[0] for o in offs))          # front row -> last row (m along the path)
    lead = 1.0                                          # mover crosses y=0 when the robot is 1 m short
    t1 = (s_x(22.5) - lead + lag) / speed               # first: in front of the leading row
    t2 = (s_x(25.0) - lead + lag + depth) / speed       # second: in front of the last row
    movers = [Mover.timed((22.5, -3.4), (22.5, 3.4), t1, mover_speed),
              Mover.timed((25.0, -3.4), (25.0, 3.4), t2, mover_speed, reverse=True)]
    starts = [(ox - 3.0, oy, 0.0) for ox, oy in offs]
    sections = [("slalom", 3.0, 9.0), ("gate", 9.0, 12.0), ("chicane", 12.0, 20.0),
                ("crossing", 20.0, 26.0), ("pillars", 26.0, 31.0), ("finish", 31.0, 33.0)]
    return dict(name=f"course{n_robots}", starts=starts, goals=None, plan=plan, obstacles=obs,
                movers=movers, sections=sections, T=plan.L / speed + 25.0)


def add_course_geoms(spec, course):
    """Static cylinders + one mocap body per mover. Returns mocap body names."""
    import mujoco
    for k, ((x, y), r) in enumerate(course["obstacles"]):
        spec.worldbody.add_geom(name=f"obs{k}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                                size=[r, 0.35, 0], pos=[x, y, 0.35], rgba=[0.75, 0.45, 0.2, 1])
    names = []
    for k, mv in enumerate(course["movers"]):
        b = spec.worldbody.add_body(name=f"mover{k}", mocap=True, pos=[*mv.a, 0.4])
        b.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[mv.r, 0.4, 0], rgba=[0.85, 0.2, 0.2, 1])
        names.append(f"mover{k}")
    return names
