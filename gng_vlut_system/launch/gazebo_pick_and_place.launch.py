import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_gazebo_config(config_path: str) -> dict:
    if not config_path or not os.path.isfile(config_path):
        raise FileNotFoundError(f"Gazebo設定ファイルがありません: {config_path}")
    with open(config_path, "r", encoding="utf-8") as config_stream:
        config_root = yaml.safe_load(config_stream) or {}
    config = config_root.get("gazebo_pick_and_place", {})
    if not isinstance(config, dict):
        raise ValueError("gazebo_pick_and_placeはmappingで指定してください")
    return config


def get_mapping(config: dict, key: str) -> dict:
    value = config.get(key, {})
    if not isinstance(value, dict):
        raise ValueError(f"Gazebo設定の{key}はmappingで指定してください")
    return value


def format_launch_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return " ".join(str(item) for item in value)
    return str(value)


def select_launch_value(context, argument_name: str, config_value) -> str:
    override = LaunchConfiguration(argument_name).perform(context).strip()
    return override if override else format_launch_value(config_value)


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("gng_vlut_system")
    config = load_gazebo_config(
        LaunchConfiguration("gazebo_params_file").perform(context).strip())
    lidar = get_mapping(config, "world_lidar")
    scan = get_mapping(config, "scan")

    world_file = select_launch_value(context, "world_file", config.get("world_file", ""))
    if not world_file:
        raise ValueError("world_fileにはworldファイル名が必要です")
    world_path = world_file if os.path.isabs(world_file) else os.path.join(
        package_share, "worlds", world_file)
    if not os.path.isfile(world_path):
        raise FileNotFoundError(f"Gazebo worldファイルがありません: {world_path}")

    default_robot_params = os.path.join(package_share, "config", "ToPoDualArm.yaml")
    launch_arguments = {
        "robot_name": select_launch_value(
            context, "robot_name", config.get("robot_name", "ToPoDualArm")),
        "params_file": default_robot_params,
        "gui": select_launch_value(context, "gui", config.get("gui", True)),
        "world": world_path,
        "spawn_z": select_launch_value(context, "spawn_z", config.get("spawn_z", 0.0)),
        "fixed_base_link": select_launch_value(
            context, "fixed_base_link", config.get("fixed_base_link", "")),
        "enable_world_lidar": select_launch_value(
            context, "enable_world_lidar", lidar.get("enable", False)),
        "lidar_params_file": os.path.join(package_share, "config", "gazebo_lidar.yaml"),
        "world_lidar_model_name": select_launch_value(
            context, "world_lidar_model_name", lidar.get("model_name", "environment_lidar")),
        "world_lidar_frame_id": select_launch_value(
            context, "world_lidar_frame_id", lidar.get("frame_id", "world_lidar_link")),
        "world_lidar_xyz": select_launch_value(
            context, "world_lidar_xyz", lidar.get("xyz", [1.5, 0.0, 1.2])),
        "world_lidar_rpy": select_launch_value(
            context, "world_lidar_rpy", lidar.get("rpy", [0.0, 0.0, 0.0])),
        "world_lidar_topic": select_launch_value(
            context, "world_lidar_topic", lidar.get("topic", "/lidar/points")),
    }
    scan_argument_names = {
        "update_hz": "lidar_update_hz",
        "num_horizontal_samples": "num_lidar_horizontal_samples",
        "num_vertical_samples": "num_lidar_vertical_samples",
        "min_horizontal_angle": "min_lidar_horizontal_angle",
        "max_horizontal_angle": "max_lidar_horizontal_angle",
        "min_vertical_angle": "min_lidar_vertical_angle",
        "max_vertical_angle": "max_lidar_vertical_angle",
        "min_range": "min_lidar_range",
        "max_range": "max_lidar_range",
        "noise_mean": "lidar_noise_mean",
        "noise_std_dev": "lidar_noise_std_dev",
    }
    for config_key, argument_name in scan_argument_names.items():
        launch_arguments[argument_name] = select_launch_value(
            context, config_key, scan.get(config_key, ""))

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", "robot_gazebo_sim.launch.py")),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument(
            "gazebo_params_file",
            default_value=os.path.join(package_share, "config", "gazebo_pick_and_place.yaml"),
        ),
        DeclareLaunchArgument("robot_name", default_value=""),
        DeclareLaunchArgument("gui", default_value=""),
        DeclareLaunchArgument("world_file", default_value=""),
        DeclareLaunchArgument("spawn_z", default_value=""),
        DeclareLaunchArgument("fixed_base_link", default_value=""),
        DeclareLaunchArgument("enable_world_lidar", default_value=""),
        DeclareLaunchArgument("world_lidar_model_name", default_value=""),
        DeclareLaunchArgument("world_lidar_frame_id", default_value=""),
        DeclareLaunchArgument("world_lidar_xyz", default_value=""),
        DeclareLaunchArgument("world_lidar_rpy", default_value=""),
        DeclareLaunchArgument("world_lidar_topic", default_value=""),
        DeclareLaunchArgument("update_hz", default_value=""),
        DeclareLaunchArgument("num_horizontal_samples", default_value=""),
        DeclareLaunchArgument("num_vertical_samples", default_value=""),
        DeclareLaunchArgument("min_horizontal_angle", default_value=""),
        DeclareLaunchArgument("max_horizontal_angle", default_value=""),
        DeclareLaunchArgument("min_vertical_angle", default_value=""),
        DeclareLaunchArgument("max_vertical_angle", default_value=""),
        DeclareLaunchArgument("min_range", default_value=""),
        DeclareLaunchArgument("max_range", default_value=""),
        DeclareLaunchArgument("noise_mean", default_value=""),
        DeclareLaunchArgument("noise_std_dev", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
