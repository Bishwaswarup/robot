"""Physics checks (need mujoco_menagerie next to the repo or MENAGERIE_PATH)."""
import numpy as np
import mujoco
from conftest import needs_model
from go1_bezier import load_single, load_team, Go1BezierController, PoseSequencer, LEGS
from go1_bezier.tricks import build_tricks


def _walk(cmd, seconds=3.0, **kw):
    m, d = load_single()
    c = Go1BezierController(m, d, **kw)
    for _ in range(250):
        c.step(); mujoco.mj_step(m, d)
    c.set_command(*cmd)
    vs = []
    n = int(seconds / m.opt.timestep)
    for k in range(n):
        c.step(); mujoco.mj_step(m, d)
        assert d.qpos[2] > 0.15, "robot fell"
        if k > n // 2:
            vs.append(c.body_velocity())
    return np.mean(vs, 0)


@needs_model
def test_tracks_forward_lateral_yaw():
    assert abs(_walk((0.5, 0, 0))[0] - 0.5) < 0.05
    assert abs(_walk((0, 0.25, 0))[1] - 0.25) < 0.05
    assert abs(_walk((0, 0, 0.8))[2] - 0.8) < 0.1
    assert abs(_walk((-0.3, 0, 0))[0] + 0.3) < 0.05


@needs_model
def test_tricks_each_paw_lifts_alone():
    m, d = load_single()
    for _ in range(500):
        mujoco.mj_step(m, d)
    kfs = build_tricks()
    seq = PoseSequencer(m, d, kfs)
    seq.start()
    sites = [m.site(l).id for l in LEGS]
    peak = {}
    while not seq.done:
        seq.step(); mujoco.mj_step(m, d)
        if not seq.done and kfs[seq.k].wave:
            leg = next(iter(kfs[seq.k].wave))
            z = np.array([d.site_xpos[s][2] for s in sites])
            p = peak.setdefault(leg, np.zeros(4))
            np.maximum(p, z, out=p)
    assert set(peak) == {"FR", "FL", "RL", "RR"}
    for leg, z in peak.items():
        i = LEGS.index(leg)
        assert z[i] > 0.15, f"{leg} not lifted"
        assert np.delete(z, i).max() < 0.05, f"another foot left the ground while waving {leg}"
    assert d.xmat[m.body("trunk").id][8] > 0.95, "robot not upright at the end"


@needs_model
def test_team_loads_and_stands():
    m, d = load_team([(0, 0, 0), (1.5, 0, 0), (0, 1.5, 1.0)])
    cs = [Go1BezierController(m, d, prefix=f"go1_{i}/") for i in range(3)]
    for _ in range(500):
        for c in cs:
            c.step()
        mujoco.mj_step(m, d)
    assert all(c.base_pose()[0][2] > 0.2 for c in cs)
