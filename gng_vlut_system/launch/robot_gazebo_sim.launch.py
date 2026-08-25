import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_root_params(params_file: str) -> dict:
    if not params_file or not os.path.exists(params_file):
        return {}
    try:
        with open(params_file, "r", encoding="utf-8") as f:
            params_yaml = yaml.safe_load(f) or {}
    except Exception:
        return {}

    for root_key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(root_key, {})
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate["ros__parameters"]
        if isinstance(candidate, dict):
            return candidate
    return {}


def configure_gazebo_paths(context, *args, **kwargs):
    del args, kwargs
    params = load_root_params(LaunchConfiguration("params_file").perform(context).strip())
    resource_root = LaunchConfiguration("resource_root_dir").perform(context).strip()
    mesh_root = LaunchConfiguration("mesh_root_dir").perform(context).strip()
    urdf_path = LaunchConfiguration("urdf_path").perform(context).strip()

    resource_root = resource_root or str(params.get("resource_root_dir", "")).strip()
    mesh_root = mesh_root or str(params.get("mesh_root_dir", "")).strip()
    urdf_path = urdf_path or str(params.get("urdf_path", "")).strip()
    if not resource_root and mesh_root:
        resource_root = os.path.dirname(mesh_root.rstrip("/"))
    if not resource_root and urdf_path and not urdf_path.startswith("package://"):
        resource_root = os.path.dirname(urdf_path)
    if not resource_root:
        return []

    existing_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    gazebo_share = "/usr/share/gazebo-11"
    resource_paths = [resource_root]
    if os.path.isdir(gazebo_share):
        resource_paths.append(gazebo_share)
    if existing_path:
        resource_paths.append(existing_path)

    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_model_path = os.path.join(gazebo_share, "models")
    model_paths = [resource_root]
    if os.path.isdir(gazebo_model_path):
        model_paths.append(gazebo_model_path)
    if existing_model_path:
        model_paths.append(existing_model_path)

    return [
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", os.pathsep.join(resource_paths)),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.pathsep.join(model_paths)),
        # The default empty world only needs Gazebo's bundled models. Avoid a
        # blocking download attempt when the container has no model DB access.
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("resource_root_dir", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("enable_safety_monitor", default_value="true"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("static_model", default_value="false"),
        DeclareLaunchArgument("fixed_base_link", default_value=""),
        DeclareLaunchArgument("follow_tf_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_ref", default_value="world"),
        DeclareLaunchArgument("follow_tf_reference_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_update_hz", default_value="20.0"),
        DeclareLaunchArgument("follow_tf_service_name", default_value="/gazebo/set_entity_state"),
        DeclareLaunchArgument("publish_initial_joint_state", default_value="false"),
        DeclareLaunchArgument("joint_state_topic", default_value=""),
        DeclareLaunchArgument("enable_gazebo_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("joint_state_update_hz", default_value="50.0"),
        DeclareLaunchArgument("enable_lidar", default_value="false"),
        DeclareLaunchArgument(
            "lidar_params_file", default_value=os.path.join(pkg_share, "config", "gazebo_lidar.yaml")
        ),
        DeclareLaunchArgument("lidar_parent_link", default_value=""),
        DeclareLaunchArgument("lidar_link", default_value=""),
        DeclareLaunchArgument("lidar_xyz", default_value=""),
        DeclareLaunchArgument("lidar_rpy", default_value=""),
        DeclareLaunchArgument("lidar_topic", default_value=""),
        DeclareLaunchArgument("lidar_frame_id", default_value=""),
        DeclareLaunchArgument("lidar_transformed_topic", default_value=""),
        DeclareLaunchArgument("lidar_target_frame", default_value=""),
        DeclareLaunchArgument("lidar_update_hz", default_value=""),
        DeclareLaunchArgument("num_lidar_horizontal_samples", default_value=""),
        DeclareLaunchArgument("num_lidar_vertical_samples", default_value=""),
        DeclareLaunchArgument("min_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_horizontal_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("max_lidar_vertical_angle", default_value=""),
        DeclareLaunchArgument("min_lidar_range", default_value=""),
        DeclareLaunchArgument("max_lidar_range", default_value=""),
        DeclareLaunchArgument("lidar_noise_mean", default_value=""),
        DeclareLaunchArgument("lidar_noise_std_dev", default_value=""),
        DeclareLaunchArgument("enable_world_lidar", default_value="false"),
        DeclareLaunchArgument("world_lidar_model_name", default_value=""),
        DeclareLaunchArgument("world_lidar_frame_id", default_value=""),
        DeclareLaunchArgument("world_lidar_xyz", default_value=""),
        DeclareLaunchArgument("world_lidar_rpy", default_value=""),
        DeclareLaunchArgument("world_lidar_topic", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(gazebo_ros_share, "worlds", "empty.world"),
        ),

        OpaqueFunction(function=configure_gazebo_paths),

        # 1. Start Gazebo Server and Client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "gui": LaunchConfiguration("gui"),
            }.items()
        ),

        # 2. Spawn Robot Model (URDF + Mesh Streaming)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "resource_root_dir": LaunchConfiguration("resource_root_dir"),
                "publish_initial_joint_state": LaunchConfiguration("publish_initial_joint_state"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            }.items()
        ),

        # 3. Spawn Robot Entity into Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_gazebo_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
                "spawn_z": LaunchConfiguration("spawn_z"),
                "static_model": LaunchConfiguration("static_model"),
                "fixed_base_link": LaunchConfiguration("fixed_base_link"),
                "follow_tf_frame": LaunchConfiguration("follow_tf_frame"),
                "follow_tf_ref": LaunchConfiguration("follow_tf_ref"),
                "follow_tf_reference_frame": LaunchConfiguration("follow_tf_reference_frame"),
                "follow_tf_update_hz": LaunchConfiguration("follow_tf_update_hz"),
                "follow_tf_service_name": LaunchConfiguration("follow_tf_service_name"),
                "enable_gazebo_joint_state_publisher": LaunchConfiguration(
                    "enable_gazebo_joint_state_publisher"
                ),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "joint_state_update_hz": LaunchConfiguration("joint_state_update_hz"),
                "enable_lidar": LaunchConfiguration("enable_lidar"),
                "lidar_params_file": LaunchConfiguration("lidar_params_file"),
                "lidar_parent_link": LaunchConfiguration("lidar_parent_link"),
                "lidar_link": LaunchConfiguration("lidar_link"),
                "lidar_xyz": LaunchConfiguration("lidar_xyz"),
                "lidar_rpy": LaunchConfiguration("lidar_rpy"),
                "lidar_topic": LaunchConfiguration("lidar_topic"),
                "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
                "lidar_transformed_topic": LaunchConfiguration("lidar_transformed_topic"),
                "lidar_target_frame": LaunchConfiguration("lidar_target_frame"),
                "lidar_update_hz": LaunchConfiguration("lidar_update_hz"),
                "num_lidar_horizontal_samples": LaunchConfiguration("num_lidar_horizontal_samples"),
                "num_lidar_vertical_samples": LaunchConfiguration("num_lidar_vertical_samples"),
                "min_lidar_horizontal_angle": LaunchConfiguration("min_lidar_horizontal_angle"),
                "max_lidar_horizontal_angle": LaunchConfiguration("max_lidar_horizontal_angle"),
                "min_lidar_vertical_angle": LaunchConfiguration("min_lidar_vertical_angle"),
                "max_lidar_vertical_angle": LaunchConfiguration("max_lidar_vertical_angle"),
                "min_lidar_range": LaunchConfiguration("min_lidar_range"),
                "max_lidar_range": LaunchConfiguration("max_lidar_range"),
                "lidar_noise_mean": LaunchConfiguration("lidar_noise_mean"),
                "lidar_noise_std_dev": LaunchConfiguration("lidar_noise_std_dev"),
            }.items()
        ),

        # 4. 環境固定LiDARモデルの追加
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gazebo_world_lidar.launch.py")),
            condition=IfCondition(LaunchConfiguration("enable_world_lidar")),
            launch_arguments={
                "lidar_params_file": LaunchConfiguration("lidar_params_file"),
                "world_lidar_model_name": LaunchConfiguration("world_lidar_model_name"),
                "world_lidar_frame_id": LaunchConfiguration("world_lidar_frame_id"),
                "world_lidar_xyz": LaunchConfiguration("world_lidar_xyz"),
                "world_lidar_rpy": LaunchConfiguration("world_lidar_rpy"),
                "world_lidar_topic": LaunchConfiguration("world_lidar_topic"),
                "lidar_update_hz": LaunchConfiguration("lidar_update_hz"),
                "num_lidar_horizontal_samples": LaunchConfiguration("num_lidar_horizontal_samples"),
                "num_lidar_vertical_samples": LaunchConfiguration("num_lidar_vertical_samples"),
                "min_lidar_horizontal_angle": LaunchConfiguration("min_lidar_horizontal_angle"),
                "max_lidar_horizontal_angle": LaunchConfiguration("max_lidar_horizontal_angle"),
                "min_lidar_vertical_angle": LaunchConfiguration("min_lidar_vertical_angle"),
                "max_lidar_vertical_angle": LaunchConfiguration("max_lidar_vertical_angle"),
                "min_lidar_range": LaunchConfiguration("min_lidar_range"),
                "max_lidar_range": LaunchConfiguration("max_lidar_range"),
                "lidar_noise_mean": LaunchConfiguration("lidar_noise_mean"),
                "lidar_noise_std_dev": LaunchConfiguration("lidar_noise_std_dev"),
            }.items(),
        ),

        # 5. GNG/VLUT Monitorの起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_monitor.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "experiment_id": LaunchConfiguration("experiment_id"),
                "enable_safety_monitor": LaunchConfiguration("enable_safety_monitor"),
                "safety_margin": LaunchConfiguration("safety_margin"),
            }.items()
        ),
    ])
