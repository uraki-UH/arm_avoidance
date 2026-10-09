"""MID-360入力選択・取付TFのlaunch検証。"""

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

pytest.importorskip("launch")
spec = importlib.util.spec_from_file_location(
    "viewer_launch", Path(__file__).resolve().parents[1] / "launch/gng_viewer_bridge.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_disabled():
    assert module.mid360_input({"enable_input": False}, "robot") == ("", [])


def test_mount_prefix_and_no_double_pitch():
    with patch.object(module, "Node") as node:
        topic, actions = module.mid360_input({"enable_input": True}, "topo_dual_arm_max_long")
    assert topic == "/sensors/mid360/points"
    assert len(actions) == 1
    args = node.call_args.kwargs["arguments"]
    assert args[args.index("--frame-id") + 1] == "topo_dual_arm_max_long/chest_lidar_link"
    assert args[args.index("--child-frame-id") + 1] == "mid360_link"
    assert float(args[args.index("--pitch") + 1]) == 0


def test_external_tf_owner():
    assert module.mid360_input({"enable_input": True, "enable_mount_tf": False}, "robot") == (
        "/sensors/mid360/points", [])


@pytest.mark.parametrize("extra", [
    {"points_topic": "relative"},
    {"pos": [0, 0, float("nan")]},
    {"rot_deg": [0, 0]},
    {"parent_frame_id": "/wrong"},
    {"frame_id": "robot/chest_lidar_link"},
])
def test_invalid(extra):
    with pytest.raises(Exception):
        module.mid360_input(dict(enable_input=True, **extra), "robot")
