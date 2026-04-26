import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_safety")
    
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml"),
        description="Path to the ROS 2 parameters file for the GNG nodes.",
    )

    node = Node(
        package="gng_safety",
        executable="offline_gng_status_updater",
        name="offline_gng_status_updater",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        node,
    ])
