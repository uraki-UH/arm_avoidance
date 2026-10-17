import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

rviz_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ais_gng.rviz")

def generate_launch_description():
    visualizar = Node(
            package="ais_gng",
            executable="visualizar",
            output="screen")

    rviz2 = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_path])

    return LaunchDescription([
        visualizar,
        rviz2
    ])