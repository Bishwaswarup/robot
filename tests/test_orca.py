import numpy as np
from go1_bezier.orca import orca_velocity


def _simulate(starts, goals, obstacles=(), r=0.35, vmax=0.6, T=40.0, dt=0.1, keep_right_deg=15.0):
    rng = np.random.default_rng(0)                  # RVO2-style symmetry breaking
    p = np.array(starts, float); g = np.array(goals, float); v = np.zeros_like(p)
    dmin = np.inf; omin = np.inf
    for _ in range(int(T / dt)):
        new = []
        for i in range(len(p)):
            to = g[i] - p[i]; d = np.linalg.norm(to)
            pref = to / d * min(vmax, d) if d > 1e-6 else np.zeros(2)
            if d > 1.0:                             # keep-right bias breaks dense deadlocks
                c, sn = np.cos(np.radians(keep_right_deg)), np.sin(np.radians(keep_right_deg))
                pref = np.array([c * pref[0] + sn * pref[1], -sn * pref[0] + c * pref[1]])
            ang = rng.uniform(0, 2 * np.pi)
            pref = pref + 1e-2 * rng.uniform() * np.array([np.cos(ang), np.sin(ang)])
            nb = [(p[j], v[j], r) for j in range(len(p)) if j != i]
            new.append(orca_velocity(p[i], v[i], r, pref, vmax, nb, obstacles, dt=dt))
        v = np.array(new); p = p + v * dt
        for i in range(len(p)):
            for j in range(i + 1, len(p)):
                dmin = min(dmin, np.linalg.norm(p[i] - p[j]))
            for po, ro in obstacles:
                omin = min(omin, np.linalg.norm(p[i] - np.array(po)) - ro)
    return p, dmin, omin


def test_head_on_pair_avoids_and_arrives():
    p, dmin, _ = _simulate([(-3, 0.01), (3, 0)], [(3, 0), (-3, 0)])
    assert dmin >= 0.7 - 0.02
    assert np.allclose(p, [(3, 0), (-3, 0)], atol=0.1)


def test_circle_swap_eight_agents():
    n = 8
    a = 2 * np.pi * np.arange(n) / n
    starts = 4 * np.c_[np.cos(a), np.sin(a)]
    p, dmin, _ = _simulate(starts, -starts, T=60)
    assert dmin >= 0.7 - 0.05
    assert np.allclose(p, -starts, atol=0.15)


def test_static_obstacle_respected():
    p, _, omin = _simulate([(-3, 0)], [(3, 0)], obstacles=[((0, 0), 0.5)])
    assert omin >= 0.35 - 0.02
    assert np.allclose(p, [(3, 0)], atol=0.1)


def test_responsibility_changes_who_yields():
    from go1_bezier.swarm import queue_responsibility
    me, other = np.array([0.0, 0.0]), np.array([0.9, 0.0])
    nb = [(other, np.zeros(2), 0.35)]
    q = queue_responsibility(me, 0.0, nb, in_bottleneck=True)
    assert q[0][3] == 0.9                              # I am behind -> I yield
    q2 = queue_responsibility(other, 0.0, [(me, np.zeros(2), 0.35)], in_bottleneck=True)
    assert q2[0][3] == 0.1                             # the other is in front -> keeps going
    assert queue_responsibility(me, 0.0, nb, in_bottleneck=False) == nb


def test_predictive_reacts_earlier_than_reactive():
    from go1_bezier.swarm import SwarmAgent
    p, v = np.array([0.0, 0.0]), np.array([0.5, 0.0])
    nb = [(np.array([2.2, 0.0]), np.array([-0.5, 0.0]), 0.35)]   # head-on, 2.2 m away
    a, b = SwarmAgent(radius=0.35, keep_right_deg=0), SwarmAgent(radius=0.35, keep_right_deg=0)
    a.v_prev = v.copy()
    pref = lambda q: np.array([0.5, 0.0])
    v_react = a.velocity(p, pref(p), nb, [])
    v_pred = b.predictive(p, v, pref, nb, [], lookahead=0.36)
    assert abs(v_pred[1]) >= abs(v_react[1]) - 1e-9


def test_turn_gate_limits_body_command():
    import numpy as np
    from go1_bezier.swarm import SwarmAgent
    free, gated = SwarmAgent(), SwarmAgent(turn_gate=True)
    v = np.array([0.0, 0.6])                       # wants to go +y, robot faces +x (90 deg error)
    vx0, vy0, wz0 = free.body_command(v, 0.0)
    vx1, vy1, wz1 = gated.body_command(v, 0.0)
    assert abs(wz0) == 1.2 and abs(vy0) == 0.35    # old limits
    assert abs(wz1) <= 1.0 + 1e-9 and abs(vy1) <= 0.6 * 0.3 + 1e-9   # slowed to 30 % while turning
    a, b = SwarmAgent(turn_gate=True), SwarmAgent()
    assert np.allclose(a.body_command(np.array([0.5, 0.0]), 0.0), b.body_command(np.array([0.5, 0.0]), 0.0))


def test_escape_breaks_wall_deadlock():
    import numpy as np
    from go1_bezier.swarm import SwarmAgent
    wall = [((1.0, y), 0.22) for y in np.arange(-3.0, 0.01, 0.4)]     # wall below y=0, opening above
    a = SwarmAgent(escape=True)
    a.escape_to = np.array([2.0, 1.5])                                 # goal side: through the opening
    p = np.array([0.35, -1.5])
    moved = 0.0
    for _ in range(60):                                                # 6 s, pref straight into the wall
        v = a.velocity(p, np.array([0.5, 0.0]), [], wall, far_from_goal=False)
        p = p + v * 0.1
        moved = max(moved, p[1] + 1.5)
    assert moved > 0.5                                                 # slid along the wall towards the gap


def test_lag_priority_settled_robot_yields():
    import numpy as np
    from go1_bezier.swarm import lag_priority
    nbs = [(np.zeros(2), np.zeros(2), 0.38), (np.ones(2), np.zeros(2), 0.38, 0.1), (np.ones(2) * 2, np.zeros(2), 0.38)]
    out = lag_priority(0.1, [2.0, 0.5, 0.2], nbs)          # I am settled; nb0 is late, nb1 neutral, nb2 settled
    assert out[0][3] == 0.9 and out[1][3] == 0.1 and out[2][3] == 0.5
    out = lag_priority(1.5, [0.2, 2.0, 0.6], nbs)          # I am late
    assert out[0][3] == pytest_approx(0.1) and out[1][3] == 0.1 and out[2][3] == 0.5


def pytest_approx(x):
    import pytest
    return pytest.approx(x)
