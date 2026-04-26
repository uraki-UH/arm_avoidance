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
        default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml"),
        description="Path to the ROS 2 parameters file for the GNG nodes.",
    )

    # Allow overriding specific parameters from the command line
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
            # Pass through overrides
            {
                "robot_urdf_path": LaunchConfiguration("robot_urdf_path"),
                "experiment_id": LaunchConfiguration("experiment_id"),
                "vlut_only": LaunchConfiguration("vlut_only"),
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        robot_urdf_path_arg,
        experiment_id_arg,
        vlut_only_arg,
        trainer_node,
    ])
