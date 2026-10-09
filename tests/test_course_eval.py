import warnings

import pytest

from scripts.course_eval import analyse, plot


def _single_robot_result():
    return {
        "team": 1,
        "config": "reactive:limiter",
        "success": 1,
        "time": 102.7,
        "falls": 0,
        "rr_contacts": 0,
        "obstacle_contacts": 0,
        "mover_contacts": 0,
        "min_dist_robots": float("nan"),
        "min_clear_obstacle": 0.21,
        "min_clear_mover": 1.83,
        "p99_tilt": 5.2,
        "max_tilt": 15,
        "form_err": float("nan"),
        "lag_mean": 0.53,
        "urgent_per_robot_min": 14.0,
    }


def test_analyse_handles_inapplicable_single_robot_metrics(tmp_path, capsys):
    out = tmp_path / "missing" / "course"

    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        analyse([_single_robot_result()], str(out))

    output = capsys.readouterr().out
    assert "n/a" in output
    assert (out / "course_summary.txt").is_file()


def test_plot_creates_missing_output_directory(tmp_path):
    pytest.importorskip("matplotlib")
    out = tmp_path / "missing" / "course"
    result = {
        **_single_robot_result(),
        "paths": [[[0, 0], [1, 1]]],
    }

    plot([result], str(out))

    assert (out / "course_paths.png").is_file()
