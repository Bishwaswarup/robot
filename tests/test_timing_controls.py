"""Release rules of the phase-aware commander (best / sham-random / worst) and the replay hooks."""
import copy
import os
import numpy as np
import mujoco
from conftest import needs_model
from go1_bezier import load_team, Go1BezierController, CapabilityMap, PushEnvelope
from go1_bezier.gasl import make_commander, parse_strategy

ROOT = os.path.join(os.path.dirname(__file__), "..")
CMAP = CapabilityMap.load(os.path.join(ROOT, "data", "capability_map_bezier.npz"))
ENV = PushEnvelope.load(os.path.join(ROOT, "data", "push_envelope_bezier.npz"))


def test_strategy_names_map_to_rules():
    assert parse_strategy("phase+gaslc_equal")["_rule"] == "best"
    assert parse_strategy("sham+gaslc_equal")["_rule"] == "random"
    assert parse_strategy("worst+gaslc_equal")["_rule"] == "worst"


def _walker(strategy, seed=0):
    m, d = load_team([(0, 0, 0)])
    c = Go1BezierController(m, d, prefix="go1_0/")
    com = make_commander(c, strategy, CMAP, ENV, {}, rng=np.random.default_rng(seed))
    c.set_command(0.3, 0, 0)
    for _ in range(1500):
        com.step()
        mujoco.mj_step(m, d)
    return m, d, com


def _wait_after_request(com, target=(0.3, 0.3, 0.0), deadline=0.36):
    assert com.timed(target, False, deadline)
    com.request(*target, deadline=deadline)
    return 0.0 if com.pending is None else (com.pending[1] - com.t) / com.c.gait.period


@needs_model
def test_best_and_worst_follow_the_map():
    _, _, com = _walker("phase+gaslc_equal")
    key = com._event_key(np.array([0.3, 0.3, 0.0]))
    waits, cost = com.phase_costs(key, 0.36)
    w = _wait_after_request(com)
    assert np.isclose(w, waits[np.nanargmin(cost)]) or np.isclose(cost[np.argmin(np.abs(waits - w))], np.nanmin(cost))
    _, _, com = _walker("worst+gaslc_equal")
    waits, cost = com.phase_costs(key, 0.36)
    w = _wait_after_request(com)
    assert np.isclose(cost[np.argmin(np.abs(waits - w))], np.nanmax(cost))


@needs_model
def test_sham_waits_are_random_from_the_same_grid():
    ws = []
    for seed in range(12):
        _, _, com = _walker("sham+gaslc_equal", seed)
        grid = com.wait_grid(0.36)
        w = _wait_after_request(com)
        assert np.isclose(grid, w, atol=1e-9).any() or w == 0.0
        ws.append(w)
    assert len(set(np.round(ws, 6))) > 3                     # not a fixed phase


@needs_model
def test_forced_wait_and_snapshot_replay_are_deterministic():
    m, d, com = _walker("phase+gaslc_equal")
    snap = copy.deepcopy((d, com), {id(m): m, id(CMAP): CMAP, id(ENV): ENV, id(com.c.acc_scale_fn): com.c.acc_scale_fn})
    ends = []
    for _ in range(2):
        d2, com2 = copy.deepcopy(snap, {id(m): m, id(CMAP): CMAP, id(ENV): ENV,
                                        id(com.c.acc_scale_fn): com.c.acc_scale_fn})
        com2.request(0.3, 0.3, 0.0, deadline=0.36, force_wait=0.25)
        assert np.isclose(com2.pending[1] - com2.t, 0.25 * com2.c.gait.period)
        for _ in range(300):
            com2.step()
            mujoco.mj_step(m, d2)
        ends.append(d2.qpos.copy())
    assert np.array_equal(ends[0], ends[1])
    assert d.time == snap[0].time                            # the original is untouched
