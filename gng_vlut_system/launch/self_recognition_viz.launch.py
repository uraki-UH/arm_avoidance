import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from launch_config import (
    resolve_package_path,
    read_vlut_voxel_size,
    root_parameters as _root_parameters,
    parameter_file_data,
    node_parameter_source,
)


def resolve_package_uri(raw_path: str) -> str:
    return resolve_package_path(raw_path, get_package_share_directory)


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    
    # 機体設定と明示引数による自己認識対象の決定
    config = parameter_file_data(params_file)
    parameter_source = node_parameter_source(params_file, config)
    root_params = _root_parameters(config)
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or str(
        root_params.get("robot_name", "topo_dual_arm_max_long"))
    yaml_urdf_path = str(root_params.get("urdf_path", "")).strip()
    yaml_vlut_resolution = float(root_params.get("gng", {}).get("vlut_resolution", 0.0))

    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    if not urdf_path and yaml_urdf_path:
        urdf_path = yaml_urdf_path
    if not urdf_path:
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly."
        )
    robot_urdf = resolve_package_uri(urdf_path)
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(f"Robot description file does not exist: {robot_urdf}")

    vlut_file = ""
    if params_file and os.path.exists(params_file):
        root_params = _root_parameters(config if 'config' in locals() else {})
        gng_params = root_params.get("gng", {}) if isinstance(root_params, dict) else {}
        if isinstance(gng_params, dict):
            data_directory = str(gng_params.get("data_directory", "")).strip()
            experiment_id = str(gng_params.get("experiment_id", "")).strip()
            vlut_filename = str(gng_params.get("vlut_filename", "vlut.bin")).strip()
            if data_directory and experiment_id and vlut_filename:
                # 左右で共通のボクセル幅。独立モデルでは左腕のヘッダを参照
                profile_dir = ("left_arm" if str(gng_params.get("enable_independent_arms", False)).lower()
                               in ("true", "1", "yes", "on") else "")
                vlut_file = os.path.join(data_directory, experiment_id, profile_dir, vlut_filename)

    self_recognition_resolution = read_vlut_voxel_size(vlut_file)
    if self_recognition_resolution is None or self_recognition_resolution <= 0.0:
        self_recognition_resolution = yaml_vlut_resolution

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    node_params = {}
    if robot_urdf:
        node_params["urdf_path"] = robot_urdf
    if self_recognition_resolution and self_recognition_resolution > 0.0:
        # VLUT のセルサイズへ自己認識側を追従
        node_params["self_recognition.resolution"] = self_recognition_resolution
    
    # コマンドライン引数を辞書に追加（明示的に指定された場合のみ、適切な型でYAMLを上書きするようにする）
    def add_if_not_empty(name, config_name, type_func=None):
        val = LaunchConfiguration(config_name).perform(context)
        if val:
            try:
                node_params[name] = type_func(val) if type_func else val
            except ValueError:
                node_params[name] = val

    def add_override_param(name, config_name, type_func=None, aliases=None):
        val = LaunchConfiguration(config_name).perform(context)
        if not val:
            return
        try:
            parsed = type_func(val) if type_func else val
        except ValueError:
            parsed = val
        node_params[name] = parsed
        if aliases:
            for alias in aliases:
                node_params[alias] = parsed

    add_if_not_empty("marker_frame_id", "marker_frame_id")
    add_if_not_empty("joint_topic", "joint_topic")
    add_if_not_empty("robot.voxel_size", "voxel_size", float)
    add_if_not_empty("robot.inflation", "inflation", float)
    add_if_not_empty("update_hz", "update_hz", float)
    add_if_not_empty("publish_self_mask", "publish_self_mask") # boolは文字列でも解釈されることが多いが
    add_if_not_empty("publish_link_voxels", "publish_link_voxels")
    add_if_not_empty("publish_link_aabb", "publish_link_aabb")
    add_if_not_empty("display_mode", "display_mode", int)
    add_override_param("root_link", "root_link", aliases=["self_recognition.root_link"])
    add_override_param("leaf_link", "leaf_link", aliases=["self_recognition.leaf_link"])
    add_if_not_empty("target_frame_id", "target_frame_id")
    add_override_param("mask_topic", "mask_topic", aliases=["self_recognition.mask_topic"])

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(parameter_source)
    final_params_list.append(node_params)

    # 名前空間の決定 (既に上でYAML等から決定済み)

    return [
        # ロボットモデルの展開 (名前空間付き)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
                "urdf_path": robot_urdf,
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
            }.items()
        ),
        # 自己認識可視化ノード (名前空間付き)
        Node(
            package="gng_vlut_system",
            executable="self_recognition_viz_node",
            name="self_recognition_viz_node",
            namespace=robot_name, # 名前空間を適用
            output="screen",
            parameters=final_params_list,
            # トピックのリマップ（名前空間外の/joint_statesを参照したい場合などに対応）
            remappings=[
                ("/joint_states", f"/{robot_name}/joint_states"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ]
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value=""),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "topo_dual_arm_max_long.yaml")),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("marker_frame_id", default_value="world"),
        DeclareLaunchArgument("joint_topic", default_value="joint_states"),
        DeclareLaunchArgument("voxel_size", default_value="0.02"),
        DeclareLaunchArgument("inflation", default_value="0.0"),
        DeclareLaunchArgument("update_hz", default_value="10.0"),
        DeclareLaunchArgument("publish_self_mask", default_value="true"),
        DeclareLaunchArgument("publish_link_voxels", default_value="true"),
        DeclareLaunchArgument("publish_link_aabb", default_value="true"),
        DeclareLaunchArgument("display_mode", default_value="link_local"),
        DeclareLaunchArgument("root_link", default_value=""),
        DeclareLaunchArgument("leaf_link", default_value=""),
        DeclareLaunchArgument("target_frame_id", default_value="world"),
        DeclareLaunchArgument("mask_topic", default_value="/self_recognition/voxel_mask"),
        OpaqueFunction(function=launch_setup),
    ])
