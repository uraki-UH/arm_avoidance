import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="旧単腕後処理ツール用の専用YAML。robot_urdf_path・data_directory・experiment_idの明示設定",
    )

    node = Node(
        package="gng_vlut_system",
        executable="offline_gng_main6_island_pruning",
        name="offline_gng_main6_island_pruning",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        node,
    ])
