import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """左グリッパの把持ボクセルテンプレート照合。"""
    package_share = get_package_share_directory("grasping_system")
    graph_launch = os.path.join(package_share, "launch", "gripper_volume_graph.launch.py")
    matcher_launch = os.path.join(package_share, "launch", "grasp_voxel_matcher.launch.py")
    gripper_definitions = os.path.join(
        package_share, "config", "ToPoDualArm_gripper_volumes.yaml"
    )

    matcher_params_file = LaunchConfiguration("matcher_params_file")
    gripper_tf_prefix = LaunchConfiguration("gripper_tf_prefix")
    gripper_cache_directory = LaunchConfiguration("gripper_cache_directory")
    gripper_cache_mode = LaunchConfiguration("gripper_cache_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "matcher_params_file",
            default_value="/ros2_ws/src/gng_vlut_system/config/ToPoDualArm.yaml",
        ),
        DeclareLaunchArgument("gripper_tf_prefix", default_value="ToPoDualArm"),
        DeclareLaunchArgument(
            "gripper_cache_directory",
            default_value=(
                "/ros2_ws/src/gng_vlut_system/gng_results/ToPoDualArm10000/"
                "gripper_volume_cache"
            ),
        ),
        DeclareLaunchArgument("gripper_cache_mode", default_value="use"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(graph_launch),
            launch_arguments={
                "grippers_file": gripper_definitions,
                "tf_prefix": gripper_tf_prefix,
                "cache_directory": gripper_cache_directory,
                "cache_mode": gripper_cache_mode,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(matcher_launch),
            launch_arguments={
                "params_file": matcher_params_file,
                "node_name": "left_grasp_voxel_matcher",
            }.items(),
        ),
    ])
