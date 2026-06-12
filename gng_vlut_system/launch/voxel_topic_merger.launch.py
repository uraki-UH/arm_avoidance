import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="ToPoDualArm",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "voxel_merger.yaml"),
        ),
        Node(
            package="gng_vlut_system",
            executable="voxel_topic_merger_node",
            name="voxel_topic_merger_node",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
