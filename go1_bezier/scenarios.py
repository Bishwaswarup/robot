"""Swarm test scenarios: robot starts, goals / formation plan, static disc obstacles.

Each scenario is a dict:
    starts     list of (x, y, yaw)
    goals      list of (x, y)            (goal mode)  or  None
    plan       FormationPlan              (formation mode) or None
    obstacles  list of ((x, y), r)
    T          time limit (s)
"""
import numpy as np
from .swarm import FormationPlan

R_ROBOT = 0.38


def swap(n, radius):
    a = 2 * np.pi * np.arange(n) / n + np.pi / n            # avoid axis-aligned starts
    p = radius * np.c_[np.cos(a), np.sin(a)]
    starts = [(x, y, np.arctan2(-y, -x)) for x, y in p]
    return dict(name=f"swap{n}", starts=starts, goals=[tuple(-q) for q in p], plan=None,
                obstacles=[], T=12 + 2.6 * radius / 0.5 * 1.3)


def formation_pillars():
    offs = [(-dx, dy) for dx in (0.0, 1.4, 2.8) for dy in (-1.4, 0.0, 1.4)]
    plan = FormationPlan(start=(0, 0), goal=(14, 0), offsets=offs, speed=0.35)
    starts = [(ox, oy, 0.0) for ox, oy in offs]
    pillars = [((5.0, 0.8), 0.3), ((6.5, -1.2), 0.3), ((8.0, 0.3), 0.3), ((9.5, 1.6), 0.3), ((9.2, -2.2), 0.3)]
    return dict(name="formation9_pillars", starts=starts, goals=None, plan=plan, obstacles=pillars, T=60.0)


def formation_gate(gap=1.9, wall_x=7.0, half_len=6.0):
    offs = [(-dx, dy) for dx in (0.0, 1.3) for dy in (-1.3, 0.0, 1.3)]
    plan = FormationPlan(start=(0, 0), goal=(14, 0), offsets=offs, speed=0.3)
    starts = [(ox, oy, 0.0) for ox, oy in offs]
    wall = [((wall_x, y), 0.2) for y in np.arange(-half_len, half_len + 1e-9, 0.3) if abs(y) > gap / 2 + 0.2]
    return dict(name="formation6_gate", starts=starts, goals=None, plan=plan, obstacles=wall, T=70.0)


SCENARIOS = {
    "swap4": lambda: swap(4, 2.5),
    "swap8": lambda: swap(8, 4.0),
    "formation9_pillars": formation_pillars,
    "formation6_gate": formation_gate,
}


def add_obstacles(spec, obstacles):
    import mujoco
    for k, ((x, y), r) in enumerate(obstacles):
        spec.worldbody.add_geom(name=f"obs{k}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                                size=[r, 0.35, 0], pos=[x, y, 0.35], rgba=[0.75, 0.45, 0.2, 1])
    return spec
