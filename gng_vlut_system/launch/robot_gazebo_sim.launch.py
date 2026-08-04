import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
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
        DeclareLaunchArgument("spawn_z", default_value="0.5"),
        DeclareLaunchArgument("static_model", default_value="false"),
        DeclareLaunchArgument("fixed_base_link", default_value=""),
        DeclareLaunchArgument("follow_tf_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_ref", default_value="world"),
        DeclareLaunchArgument("follow_tf_reference_frame", default_value=""),
        DeclareLaunchArgument("follow_tf_update_hz", default_value="20.0"),
        DeclareLaunchArgument("follow_tf_service_name", default_value="/gazebo/set_entity_state"),
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
            }.items()
        ),

        # 4. Start GNG/VLUT Monitor
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
