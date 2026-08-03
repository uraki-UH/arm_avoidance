import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("resource_root_dir", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("enable_safety_monitor", default_value="true"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value=""), # Optional world file

        # 1. Start Gazebo Server and Client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")),
            launch_arguments={
                "world": LaunchConfiguration("world"),
                "gui": LaunchConfiguration("gui"),
            }.items()
        ),

        # 2. Spawn Robot Model (URDF + Mesh Streaming)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "resource_root_dir": LaunchConfiguration("resource_root_dir"),
                "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
            }.items()
        ),

        # 3. Spawn Robot Entity into Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_gazebo_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "urdf_path": LaunchConfiguration("urdf_path"),
            }.items()
        ),

        # 4. Start GNG/VLUT Monitor
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_monitor.launch.py")),
            condition=None, # We'll use the argument inside or a condition here
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "experiment_id": LaunchConfiguration("experiment_id"),
                "enable_safety_monitor": LaunchConfiguration("enable_safety_monitor"),
                "safety_margin": LaunchConfiguration("safety_margin"),
            }.items()
        ),
    ])
