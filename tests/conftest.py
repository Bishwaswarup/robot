import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from go1_bezier.sim_utils import DEFAULT_MENAGERIE  # noqa: E402

HAS_MODEL = os.path.exists(os.path.join(DEFAULT_MENAGERIE, "unitree_go1", "scene.xml"))
needs_model = pytest.mark.skipif(not HAS_MODEL, reason="mujoco_menagerie not found")
