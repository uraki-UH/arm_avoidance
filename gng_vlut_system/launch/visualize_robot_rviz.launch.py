import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("joint_state_source", default_value="rviz"),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_runtime.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "experiment_id": LaunchConfiguration("experiment_id"),
                "joint_state_source": LaunchConfiguration("joint_state_source"),
                "enable_safety_monitor": "false",
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_model_spawner.launch.py")),
            launch_arguments={
                "rviz_config": os.path.join(pkg_share, "rviz", "gng_monitor.rviz"),
            }.items()
        )
    ])
