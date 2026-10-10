"""従来runtime引数を現行Viewer起動へ接続する互換入口。"""

import math
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import load_root_parameters, resolve_package_path, namespaced_frame, is_enabled


def launch_setup(context, *args, **kwargs):
    def value(name):
        return LaunchConfiguration(name).perform(context).strip()

    for name in ("safety_margin", "tag", "mode"):
        if value(name):
            raise ValueError(f"{name}: 旧安全監視専用オプションは現行Viewer入口に接続できません")

    if is_enabled(value("enable_safety_monitor")):
        raise ValueError(
            "旧runtimeのsafety_monitor_nodeは現行ビルドの対象外です。"
            "安全監視の接続確認前の起動はできません。"
            "表示専用の場合だけenable_safety_monitor:=falseを指定してください")
    if is_enabled(value("enable_joint_state_publisher")):
        raise ValueError("外部実測の表示専用入口には初回姿勢配信を追加できません")
    share = Path(get_package_share_directory("gng_vlut_system"))
    params_file = resolve_package_path(value("params_file"), get_package_share_directory)
    if not Path(params_file).is_file() and Path(params_file).parent == Path("."):
        params_file = str(share / "config" / params_file)
    params = load_root_parameters(params_file)
    robot_name = value("robot_name") or str(params.get("robot_name", "topo_dual_arm_max_long"))
    state_topic = value("state_topic") or f"/{robot_name}/joint_states"
    arguments = {
        "params_file": params_file,
        "robot_name": robot_name,
        "dir": value("dir") or value("data_directory"),
        "id": value("id") or value("experiment_id"),
        "urdf_path": value("urdf_path"),
        "resource_root_dir": value("resource_root_dir"),
        "mesh_root_dir": value("mesh_root_dir"),
        "joint_control_backend": "external",
        "state_topic": state_topic,
        "enable_joint_state_publisher": "false",
        "robot_base_frame": value("base_frame"),
    }
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/gng_viewer_bridge.launch.py")),
        launch_arguments=arguments.items())]
    calibration = [value("sensor_" + axis) for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
    if any(calibration):
        values = [float(item or "0") for item in calibration]
        if not all(math.isfinite(item) for item in values):
            raise ValueError("sensor位置・角度には有限値が必要です")
        frame = namespaced_frame(robot_name, value("base_frame") or params.get("frame_id", "base_link"))
        child = value("sensor_frame_id")
        for frame_name in (frame, child):
            if not frame_name or frame_name.startswith("/") or any(char.isspace() for char in frame_name):
                raise ValueError("sensor較正のTF名が不正です")
        if frame == child:
            raise ValueError("sensor較正の親子TFが同一です")
        node_arguments = [item for key, item_value in zip(("x", "y", "z", "roll", "pitch", "yaw"), values)
                          for item in ("--" + key, str(item_value))]
        node_arguments += ["--frame-id", frame, "--child-frame-id", child]
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                            name="sensor_calibration_publisher", arguments=node_arguments))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("gng_vlut_system"))
    defaults = {
        "params_file": str(share / "config/topo_dual_arm_max_long.yaml"),
        "robot_name": "", "id": "", "experiment_id": "", "dir": "", "data_directory": "",
        "urdf_path": "", "resource_root_dir": "", "mesh_root_dir": "", "base_frame": "",
        "state_topic": "", "enable_safety_monitor": "false", "enable_joint_state_publisher": "false",
        "safety_margin": "", "tag": "", "mode": "",
        "sensor_x": "", "sensor_y": "", "sensor_z": "", "sensor_roll": "", "sensor_pitch": "", "sensor_yaw": "",
        "sensor_frame_id": "camera_link",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
