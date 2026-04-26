import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("joint_state_source", default_value="rviz"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("enable_safety_monitor", default_value="true"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),

        # 1. Spawn Robot (Visuals & TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={"robot_name": LaunchConfiguration("robot_name")}.items()
        ),

        # 2. GNG/VLUT Monitor (Logic)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_monitor.launch.py")),
            condition=IfCondition(LaunchConfiguration("enable_safety_monitor")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "experiment_id": LaunchConfiguration("experiment_id"),
                "frame_id": LaunchConfiguration("frame_id"),
                "safety_margin": LaunchConfiguration("safety_margin"),
            }.items()
        ),

        # 3. Joint State Providers (Select only one)
        # 'rviz' mode: Use UDP listener for Topoarm
        Node(
            package="gng_vlut_system",
            executable="topoarm_udp_bridge",
            name="topoarm_udp_bridge",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("joint_state_source"), "' == 'rviz'"])),
            parameters=[{"base_frame": LaunchConfiguration("frame_id"), "output_topic": "/joint_states"}]
        ),
        
        # 'real' mode: Use bidirectional Robot Bridge
        Node(
            package="gng_vlut_system",
            executable="robot_bridge_node",
            name="robot_bridge_node",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("joint_state_source"), "' == 'real'"])),
            parameters=[{"joint_state_topic": "/joint_states"}]
        ),
    ])
