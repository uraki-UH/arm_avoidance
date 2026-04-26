import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    # Default paths
    robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
    resource_root = os.path.join(pkg_share, "urdf")
    mesh_root = os.path.join(resource_root, "meshes", "topoarm")
    
    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml")),
        DeclareLaunchArgument("robot_description_file", default_value=robot_desc_default),
        DeclareLaunchArgument("joint_state_source", default_value="rviz"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("enable_safety_monitor", default_value="false"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        DeclareLaunchArgument("gng_model_path", default_value=""),
        DeclareLaunchArgument("vlut_path", default_value=""),
        DeclareLaunchArgument("experiment_id", default_value="default_experiment"),

        # --- Nodes ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(['xacro ', LaunchConfiguration('robot_description_file')])}]
        ),

        Node(
            package="gng_vlut_system",
            executable="robot_description_player",
            name="robot_description_player",
            parameters=[{
                "robot_description_file": LaunchConfiguration("robot_description_file"),
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
            }]
        ),

        Node(
            package="gng_vlut_system",
            executable="topoarm_joint_state_player",
            name="topoarm_joint_state_player",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("joint_state_source"), "' == 'rviz'"])),
            parameters=[{"base_frame": LaunchConfiguration("frame_id"), "output_topic": "/joint_states_rviz"}]
        ),

        Node(
            package="gng_vlut_system",
            executable="robot_bridge_node",
            name="robot_bridge_node",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("joint_state_source"), "' == 'real'"])),
            parameters=[{"joint_state_topic": "/joint_states_real"}]
        ),

        Node(
            package="gng_vlut_system",
            executable="joint_state_mux_node",
            name="joint_state_mux_node",
            parameters=[{
                "rviz_topic": "/joint_states_rviz",
                "real_topic": "/joint_states_real",
                "output_topic": "/joint_states",
                "active_source": LaunchConfiguration("joint_state_source"),
            }]
        ),

        Node(
            package="gng_vlut_system",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            condition=IfCondition(LaunchConfiguration("enable_safety_monitor")),
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "gng_model_path": LaunchConfiguration("gng_model_path"),
                    "vlut_path": LaunchConfiguration("vlut_path"),
                    "safety_margin": LaunchConfiguration("safety_margin"),
                    "robot_urdf_path": LaunchConfiguration("robot_description_file"),
                    "experiment_id": LaunchConfiguration("experiment_id"),
                },
            ]
        ),
    ])
