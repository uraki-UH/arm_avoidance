import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_root_params(params_file_path: str) -> dict:
    if not params_file_path or not os.path.exists(params_file_path):
        return {}

    try:
        with open(params_file_path, "r", encoding="utf-8") as f:
            params_yaml = yaml.safe_load(f) or {}
    except Exception:
        return {}

    if not isinstance(params_yaml, dict):
        return {}

    root_candidates = []
    for key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(key)
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate.get("ros__parameters", {})
        if isinstance(candidate, dict):
            root_candidates.append(candidate)

    if root_candidates:
        return root_candidates[0]
    return params_yaml if isinstance(params_yaml, dict) else {}


def _get_nested(data: dict, path: list[str], default=None):
    cur = data
    for part in path:
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _setup_launch(context, *args, **kwargs):
    del args, kwargs

    params_file = LaunchConfiguration("params_file").perform(context)
    root_params = _load_root_params(params_file)

    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    if not robot_name:
        robot_name = str(root_params.get("robot_name", "topoarm"))

    robot_description_file = LaunchConfiguration("robot_description_file").perform(context).strip()
    if not robot_description_file:
        robot_description_file = str(root_params.get("robot_description_file", root_params.get("robot_urdf_path", "")))

    resource_root_dir = LaunchConfiguration("resource_root_dir").perform(context).strip()
    if not resource_root_dir:
        resource_root_dir = str(root_params.get("resource_root_dir", ""))

    mesh_root_dir = LaunchConfiguration("mesh_root_dir").perform(context).strip()
    if not mesh_root_dir:
        mesh_root_dir = str(root_params.get("mesh_root_dir", ""))

    voxel_link_names = LaunchConfiguration("voxel_link_names").perform(context).strip()

    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context).strip()
    if not joint_state_topic:
        joint_state_topic = str(root_params.get("joint_state_topic", "")).strip()
    if not joint_state_topic:
        joint_state_topic = f"/{robot_name}/joint_states"
    elif not joint_state_topic.startswith("/"):
        joint_state_topic = f"/{robot_name}/{joint_state_topic}"

    stream_topic = LaunchConfiguration("stream_topic").perform(context).strip()
    if not stream_topic:
        stream_topic = str(root_params.get("stream_topic", "/viewer/internal/stream/robot"))

    frame_id = LaunchConfiguration("frame_id").perform(context).strip()
    if not frame_id:
        frame_id = str(root_params.get("frame_id", "base_link"))

    robot_voxel_size = _get_nested(root_params, ["robot", "voxel_size"], 0.005)
    robot_inflation = _get_nested(root_params, ["robot", "inflation"], 0.0)

    node_parameters = {
        "robot_name": robot_name,
        "robot_description_file": robot_description_file,
        "resource_root_dir": resource_root_dir,
        "mesh_root_dir": mesh_root_dir,
        "joint_state_topic": joint_state_topic,
        "stream_topic": stream_topic,
        "frame_id": frame_id,
        "voxel_link_names": voxel_link_names,
        "voxel_size": float(LaunchConfiguration("voxel_size").perform(context) or robot_voxel_size),
        "voxel_padding": float(LaunchConfiguration("voxel_padding").perform(context) or robot_inflation),
        "max_spheres": int(LaunchConfiguration("max_spheres").perform(context) or 64),
        "min_points_per_sphere": int(LaunchConfiguration("min_points_per_sphere").perform(context) or 12),
        "min_gain_ratio": float(LaunchConfiguration("min_gain_ratio").perform(context) or 0.15),
        "capsule_min_chain_spheres": int(LaunchConfiguration("capsule_min_chain_spheres").perform(context) or 5),
        "capsule_axis_ratio_threshold": float(LaunchConfiguration("capsule_axis_ratio_threshold").perform(context) or 5.0),
        "capsule_radius_cv_threshold": float(LaunchConfiguration("capsule_radius_cv_threshold").perform(context) or 0.45),
    }

    return [
        Node(
            package="gng_vlut_system",
            executable="voxel_spherized_robot_viewer_node",
            output="screen",
            parameters=[params_file, node_parameters],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("robot_name", default_value=""),
            DeclareLaunchArgument("robot_description_file", default_value=""),
            DeclareLaunchArgument("resource_root_dir", default_value=""),
            DeclareLaunchArgument("mesh_root_dir", default_value=""),
            DeclareLaunchArgument("joint_state_topic", default_value=""),
            DeclareLaunchArgument("stream_topic", default_value=""),
            DeclareLaunchArgument("frame_id", default_value=""),
            DeclareLaunchArgument("voxel_link_names", default_value=""),
            DeclareLaunchArgument("voxel_size", default_value="0.005"),
            DeclareLaunchArgument("voxel_padding", default_value=""),
            DeclareLaunchArgument("max_spheres", default_value="64"),
            DeclareLaunchArgument("min_points_per_sphere", default_value="12"),
            DeclareLaunchArgument("min_gain_ratio", default_value="0.15"),
            DeclareLaunchArgument("capsule_min_chain_spheres", default_value="5"),
            DeclareLaunchArgument("capsule_axis_ratio_threshold", default_value="5.0"),
            DeclareLaunchArgument("capsule_radius_cv_threshold", default_value="0.45"),
            OpaqueFunction(function=_setup_launch),
        ]
    )
