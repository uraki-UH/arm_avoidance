import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    config_path = os.path.join(pkg_share, "config", "topoarm_bridge.yaml")

    return LaunchDescription([
        Node(
            package="gng_vlut_system",
            executable="robot_bridge_node",
            name="robot_bridge_node",
            parameters=[config_path],
            output="screen"
        )
    ])
