import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    rviz_config_default = os.path.join(pkg_share, "config", "gng_safety.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config_default),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "urdf_path": LaunchConfiguration("urdf_path"),
            }.items()
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
