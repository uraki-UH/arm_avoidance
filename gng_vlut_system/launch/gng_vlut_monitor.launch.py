import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    experiment_id = LaunchConfiguration("experiment_id").perform(context)
    if not experiment_id:
        experiment_id = robot_name
    
    data_dir = LaunchConfiguration("data_directory").perform(context)
    gng_path = os.path.join(data_dir, experiment_id, "gng.bin")
    vlut_path = os.path.join(data_dir, experiment_id, "vlut.bin")

    return [
        Node(
            package="gng_vlut_system",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "gng_model_path": gng_path,
                    "vlut_path": vlut_path,
                    "safety_margin": LaunchConfiguration("safety_margin"),
                    "base_frame": LaunchConfiguration("frame_id"),
                    "experiment_id": experiment_id,
                },
            ]
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml")),
        DeclareLaunchArgument("data_directory", default_value="gng_results"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        OpaqueFunction(function=launch_setup)
    ])
