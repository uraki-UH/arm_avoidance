import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    dual_cfg = os.path.join(
        get_package_share_directory("gng_vlut_system"),
        "config",
        "topodual.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=dual_cfg,
        description="Path to the ROS 2 parameters file for the dual-arm GNG trainer.",
    )

    robot_urdf_path_arg = DeclareLaunchArgument(
        "robot_urdf_path",
        default_value="",
        description="Override for the robot URDF path.",
    )
    experiment_id_arg = DeclareLaunchArgument(
        "experiment_id",
        default_value="",
        description="Override for the experiment ID.",
    )
    vlut_only_arg = DeclareLaunchArgument(
        "vlut_only",
        default_value="false",
        description="Set to 'true' to skip GNG training and only generate the VLUT.",
    )

    trainer_node = Node(
        package="gng_vlut_system",
        executable="offline_urdf_trainer",
        name="offline_urdf_trainer",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        robot_urdf_path_arg,
        experiment_id_arg,
        vlut_only_arg,
        trainer_node,
    ])
