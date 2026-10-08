import numpy as np
from go1_bezier.capability import CapabilityMap


def _rows():
    rows = []
    for v0 in (0.0, 0.4):
        for mag in (0.2, 0.8):
            for d in [str(x) for x in range(0, 360, 45)] + ["yaw+", "yaw-"]:
                for k in range(16):
                    phi = k / 16
                    th = 0.0 if d.startswith("yaw") else np.radians(float(d))
                    # mirror-symmetric toy truth: depends on cos(theta) and on theta-phase coupling
                    tilt = 2 + 5 * mag + np.cos(th) + np.sin(th) * np.sin(2 * np.pi * phi)
                    rows.append(dict(kind="cmd", limiter="False", v0=v0, mag=mag, dir=d, phi=phi,
                                     peak_tilt=tilt, fell=0, recovery_s=1.0, excess_slip_mm=10.0, rep=0))
    return rows


def test_grid_values_and_interpolation():
    cm = CapabilityMap.from_rows(_rows())
    # on-grid value reproduced
    t = cm.predict("cmd_raw", 0.4, 90, 0.8, 0.25, "tilt")
    assert abs(t - (2 + 4 + 0 + 1)) < 1e-9
    # size interpolation is linear
    mid = cm.predict("cmd_raw", 0.0, 0, 0.5, 0.0, "tilt")
    assert abs(mid - (2 + 2.5 + 1)) < 1e-9
    # phase is periodic
    assert abs(cm.predict("cmd_raw", 0.0, 45, 0.2, 0.1, "tilt") - cm.predict("cmd_raw", 0.0, 45, 0.2, 1.1, "tilt")) < 1e-9


def test_best_phase_finds_minimum():
    cm = CapabilityMap.from_rows(_rows())
    ph, wait, val = cm.best_phase("cmd_raw", 0.4, 90, 0.8, current_phase=0.0, max_wait=1.0)
    assert abs(ph - 0.75) < 1 / 32 and abs(val - 5.0) < 0.1


def test_save_load_roundtrip(tmp_path):
    cm = CapabilityMap.from_rows(_rows())
    p = tmp_path / "m.npz"
    cm.save(p)
    cm2 = CapabilityMap.load(p)
    assert cm.predict("cmd_raw", 0.2, 30, 0.5, 0.3) == cm2.predict("cmd_raw", 0.2, 30, 0.5, 0.3)


def test_push_envelope_interpolation(tmp_path):
    from go1_bezier.capability import PushEnvelope
    dirs = np.arange(16) * 22.5
    phases = np.arange(16) / 16
    G = np.zeros((2, 16, 16))
    for a in range(2):
        for b, th in enumerate(dirs):
            G[a, b] = 200 + 50 * np.cos(np.radians(th)) + 20 * np.cos(2 * np.pi * phases) - 10 * a
    p = tmp_path / "env.npz"
    np.savez(p, **{"safe/G": G, "safe/S": 0 * G, "safe/v0": np.array([0.0, 0.4]),
                   "safe/dirs": dirs, "safe/phases": phases, "tilt_crit": np.array(20.0)})
    env = PushEnvelope.load(p)
    assert abs(env.j_max(0.0, 0, 0.0) - 270) < 1e-9                 # on grid
    assert abs(env.j_max(0.2, 0, 0.0) - 265) < 1e-9                 # v0 interpolation
    assert abs(env.j_max(0.0, 360, 1.0) - env.j_max(0.0, 0, 0.0)) < 1e-9   # periodic
    assert env.worst_case(0.0, 180) < env.j_max(0.0, 180, 0.0)
    assert abs(env.impulse_max(0.0, 0, 0.0) - 27.0) < 1e-9


def test_gait_aware_limiter_modes():
    import os
    from go1_bezier.capability import PushEnvelope
    from go1_bezier.gasl import GaitAwareLimiter
    root = os.path.join(os.path.dirname(__file__), "..", "data")
    env = PushEnvelope.load(os.path.join(root, "push_envelope_bezier.npz"))
    cm = CapabilityMap.load(os.path.join(root, "capability_map_bezier.npz"))
    u = GaitAwareLimiter(env, mode="uniform", uniform=1.35, n_dir=8, n_phase=16)
    assert np.allclose(u.S, 1.35)
    for src in ("push", "cmd"):
        e = GaitAwareLimiter(env, cmap=cm, source=src, mode="equal", n_dir=8, n_phase=16)
        assert np.allclose(e.S.mean(axis=2), 1.0)               # same mean limit as uniform
        f = GaitAwareLimiter(env, cmap=cm, source=src, mode="fast", n_dir=8, n_phase=16)
        assert f.S.min() >= 1.0 - 1e-9 and f.S.max() <= 2.0 + 1e-9
