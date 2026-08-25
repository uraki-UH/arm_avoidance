import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")
    default_params_file = os.path.join(
        package_share, "config", "pointcloud_human_candidate_detection.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        Node(
            package="gng_vlut_system",
            executable="pointcloud_human_candidate_detector_node",
            name="pointcloud_human_candidate_detector_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
