import numpy as np
from go1_bezier.course import PathPlan, obstacle_course, Mover


def test_path_plan_curvilinear_slots_and_queue():
    plan = PathPlan([(0, 0), (10, 0), (10, 10)], [(0, 0), (0, 1.0), (-1.0, 0)], speed=1.0, smooth=0.01)
    ref, _ = plan.reference(15.0)                    # 5 m up the second leg
    assert np.allclose(ref, (10, 5), atol=1e-6)
    left, _ = plan.slot(1, 15.0)                     # 1 m to the LEFT of the path heading +y
    assert np.allclose(left, (9, 5), atol=1e-6)
    behind, _ = plan.slot(2, 15.0)                   # 1 m BEHIND along the path
    assert np.allclose(behind, (10, 4), atol=1e-6)
    q = [plan.slot(i, 15.0, 0.0)[0] for i in range(3)]   # single file: all on the path, distinct
    assert all(abs(p[0] - 10) < 1e-6 for p in q)
    assert len({round(p[1], 3) for p in q}) == 3


def test_course_is_feasible_and_mover_bounces():
    c = obstacle_course(6)
    plan = c["plan"]
    pts = [plan.point(s) for s in np.arange(0, plan.L, 0.25)]
    # the path itself keeps >= 0.4 m clearance from every wall disc (centre poles excepted: r 0.25)
    walls = [(q, r) for q, r in c["obstacles"] if abs(r - 0.22) < 1e-9]
    clear = min(np.linalg.norm(np.asarray(q) - p) - r for p in pts for q, r in walls)
    assert clear > 0.5
    assert plan.lam_grid.min() >= 0.0 and plan.lam_grid.max() <= 1.0
    mv = Mover((0, -1), (0, 1), speed=1.0)
    assert np.allclose(mv.state(0.0)[0], (0, -1)) and np.allclose(mv.state(2.0)[0], (0, 1))
    assert np.allclose(mv.state(4.0)[0], (0, -1))


def test_slalom_weaves_and_movers_are_timed():
    c = obstacle_course(1, speed=0.45)
    plan = c["plan"]
    ys = [plan.point(s)[1] for s in np.arange(0, 20, 0.1)]
    assert max(ys) > 1.0 and min(ys) < -1.0            # path swings both sides of the poles
    poles = [q for q, r in c["obstacles"] if abs(r - 0.25) < 1e-9]
    pts = [plan.point(s) for s in np.arange(0, plan.L, 0.05)]
    assert min(np.linalg.norm(np.asarray(q) - p) for p in pts for q in poles) > 0.7
    # each mover passes y=0 within ~2 s of the reference reaching ~1-2 m before it
    for mv, x in zip(c["movers"], (22.5, 25.0)):
        ts = np.arange(0, c["T"], 0.05)
        cross = [t for t in ts if abs(mv.state(t)[0][1]) < 0.011]
        assert any(x - 2.5 < plan.reference(t)[0][0] < x + 1.0 for t in cross)
    m2 = Mover.timed((0, -1), (0, 1), t_cross=3.0, speed=0.5)
    assert np.allclose(m2.state(3.0)[0], (0, 0), atol=1e-9) and m2.state(3.0)[1][1] > 0
    m3 = Mover.timed((0, -1), (0, 1), t_cross=3.0, speed=0.5, reverse=True)
    assert np.allclose(m3.state(3.0)[0], (0, 0), atol=1e-9) and m3.state(3.0)[1][1] < 0
