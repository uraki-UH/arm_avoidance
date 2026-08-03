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
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("resource_root_dir", default_value=""),
        DeclareLaunchArgument("mesh_root_dir", default_value=""),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("publish_initial_joint_state", default_value="true"),
        DeclareLaunchArgument("joint_state_topic", default_value="viewer_joint_states"),
        DeclareLaunchArgument("robot_description_topic", default_value="rviz_robot_description"),
        DeclareLaunchArgument("fixed_frame", default_value="world"),
        DeclareLaunchArgument("robot_base_frame", default_value="ToPoDualArm/base_footprint"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config_default),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "params_file": LaunchConfiguration("params_file"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "resource_root_dir": LaunchConfiguration("resource_root_dir"),
                "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
                "publish_initial_joint_state": LaunchConfiguration("publish_initial_joint_state"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "robot_description_topic": LaunchConfiguration("robot_description_topic"),
            }.items()
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="robot_world_tf_publisher",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                LaunchConfiguration("fixed_frame"),
                LaunchConfiguration("robot_base_frame"),
            ],
        ),

        Node(
            package="gng_vlut_system",
            executable="rviz_robot_visual_marker_node",
            name="rviz_robot_visual_marker_node",
            namespace=LaunchConfiguration("robot_name"),
            parameters=[{
                "robot_name": LaunchConfiguration("robot_name"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "resource_root_dir": LaunchConfiguration("resource_root_dir"),
                "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
                "topic_name": "rviz_robot_visual_markers",
                "robot_description_topic": LaunchConfiguration("robot_description_topic"),
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
