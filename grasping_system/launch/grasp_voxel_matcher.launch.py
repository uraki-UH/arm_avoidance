import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_package_uri(value):
    if not value.startswith("package://"):
        return str(Path(value).expanduser())

    package_path = value.removeprefix("package://")
    package_name, separator, relative_path = package_path.partition("/")
    if not package_name or not separator or not relative_path:
        raise ValueError(f"invalid package URI: {value}")
    return os.path.join(get_package_share_directory(package_name), relative_path)


def _load_volume_graph_config(params_file):
    if not params_file:
        return None

    path = Path(params_file).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"parameter file does not exist: {path}")
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError("parameter file must contain a YAML mapping")

    wildcard = loaded.get("/**", {})
    parameters = wildcard.get("ros__parameters", {}) if isinstance(wildcard, dict) else {}
    if not isinstance(parameters, dict):
        raise ValueError("/**.ros__parameters must be a YAML mapping")

    volume_graph = parameters.get("gripper_volume_graph", {})
    if not isinstance(volume_graph, dict):
        raise ValueError("gripper_volume_graph must be a YAML mapping")
    if not volume_graph.get("enable_matcher_launch", False):
        return None

    definitions_file = str(volume_graph.get("definitions_file", "")).strip()
    if not definitions_file:
        raise ValueError(
            "gripper_volume_graph.definitions_file is required when "
            "enable_matcher_launch is true"
        )
    robot_name = str(parameters.get("robot_name", "")).strip()
    if not robot_name:
        raise ValueError("robot_name is required when enable_matcher_launch is true")
    return {
        "definitions_file": _resolve_package_uri(definitions_file),
        "tf_prefix": robot_name,
        "cache_directory": str(volume_graph.get("cache_directory", "")).strip(),
        "cache_mode": str(volume_graph.get("cache_mode", "use")).strip() or "use",
    }


def _launch_matcher(context):
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    parameters = [params_file] if params_file else []
    volume_graph = _load_volume_graph_config(params_file)
    actions = []

    if volume_graph:
        package_share = get_package_share_directory("grasping_system")
        graph_launch = os.path.join(
            package_share, "launch", "gripper_volume_graph.launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(graph_launch),
                launch_arguments={
                    "grippers_file": volume_graph["definitions_file"],
                    "tf_prefix": volume_graph["tf_prefix"],
                    "cache_directory": volume_graph["cache_directory"],
                    "cache_mode": volume_graph["cache_mode"],
                }.items(),
            )
        )

    actions.extend([
        Node(
            package="grasping_system",
            executable="grasp_voxel_matcher_node",
            name=LaunchConfiguration("node_name"),
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_marker_bridge_node",
            name="grasp_pose_marker_bridge_node",
            output="screen",
            parameters=parameters,
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument(
                "node_name", default_value="left_grasp_voxel_matcher"
            ),
            OpaqueFunction(function=_launch_matcher),
        ]
    )
