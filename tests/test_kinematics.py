import numpy as np
from go1_bezier.kinematics import leg_ik, leg_fk, body_ik, HIP_OFFSETS, L_ABD


def test_ik_fk_roundtrip():
    rng = np.random.default_rng(0)
    for leg in range(4):
        for _ in range(300):
            q = np.array([rng.uniform(-0.5, 0.5), rng.uniform(0.2, 1.5), rng.uniform(-2.5, -1.0)])
            assert np.allclose(leg_ik(leg_fk(q, leg), leg), q, atol=1e-9)


def test_home_pose_is_menagerie_keyframe():
    # Menagerie "home": (0, 0.9, -1.8) on every leg
    for leg in range(4):
        p = leg_fk(np.array([0, 0.9, -1.8]), leg)
        assert np.allclose(leg_ik(p, leg), [0, 0.9, -1.8], atol=1e-9)


def test_vectorised_matches_per_leg():
    rng = np.random.default_rng(1)
    for _ in range(100):
        feet = HIP_OFFSETS + np.c_[rng.uniform(-.1, .1, 4), L_ABD + rng.uniform(-.03, .03, 4),
                                   rng.uniform(-.3, -.2, 4)]
        ref = np.concatenate([leg_ik(feet[i] - HIP_OFFSETS[i], i) for i in range(4)])
        assert np.allclose(body_ik(feet), ref)
