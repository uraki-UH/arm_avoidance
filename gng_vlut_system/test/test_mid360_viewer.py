"""MID-360入力選択・取付TFのlaunch検証。"""

import importlib.util
import math
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

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
    assert topic == "/livox/lidar"
    assert len(actions) == 1
    args = node.call_args.kwargs["arguments"]
    assert args[args.index("--frame-id") + 1] == "topo_dual_arm_max_long/chest_lidar_link"
    assert args[args.index("--child-frame-id") + 1] == "mid360_link"
    assert float(args[args.index("--pitch") + 1]) == 0


def test_external_tf_owner():
    assert module.mid360_input({"enable_input": True, "enable_mount_tf": False}, "robot") == (
        "/livox/lidar", [])


def test_long_sensor_axes():
    workspace = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((workspace / 'gng_vlut_system/config/topo_dual_arm_max_long.yaml').read_text())
    config = params['/**']['ros__parameters']['mid360']
    standalone = yaml.safe_load((workspace / 'integrations/mid360/mid360.yaml').read_text())
    # 計測+Xはコネクタの反対側。取付リンクの+Yに対応
    assert config['rot_deg'] == standalone['rot_deg'] == [0, 0, 90]
    with patch.object(module, 'Node') as node:
        module.mid360_input(config, 'topo_dual_arm_max_long')
    args = node.call_args.kwargs['arguments']
    assert float(args[args.index('--yaw') + 1]) == pytest.approx(math.pi / 2)
    assert float(args[args.index('--pitch') + 1]) == 0
    assert float(args[args.index('--roll') + 1]) == 0


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
