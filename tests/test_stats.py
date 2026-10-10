"""Paired sign-flip permutation test and Holm correction used for the paper tables/figures."""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from paper_tables import signflip_p, holm


def test_two_sided_minimum_is_two_over_2n():
    for n in (10, 20):
        assert signflip_p(-np.arange(1.0, n + 1)) == 2 / 2 ** n      # all same sign: only +/- observed


def test_zero_differences_drop_out():
    x = np.r_[0.0, -np.arange(1.0, 10)]
    assert signflip_p(x) == signflip_p(x[1:]) == 2 / 2 ** 9


def test_symmetric_and_null():
    x = np.array([1.0, -2.0, 3.0, 0.5, -0.1, 2.0])
    assert signflip_p(x) == signflip_p(-x)
    assert signflip_p(np.zeros(5)) == 1.0


def test_holm_step_down():
    p = np.array([2 / 1024] * 2 + [0.5])
    assert np.allclose(holm(p), [3 * 2 / 1024, 3 * 2 / 1024, 0.5])  # ties share the first multiplier
    assert np.isclose(holm([2 / 1024])[0], 2 / 1024)
