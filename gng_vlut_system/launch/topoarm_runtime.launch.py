import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _write_resolved_robot_description(robot_description_file: str) -> str:
    text = Path(robot_description_file).read_text(encoding="utf-8")
    package_prefix = "package://topoarm_description/meshes/topoarm/"
    replacement = "file://" + os.path.join(
        get_package_share_directory("gng_safety"),
        "urdf",
        "topoarm_description",
        "meshes",
        "topoarm",
    ) + "/"
    resolved_text = text.replace(package_prefix, replacement)
    resolved_path = Path("/tmp/gng_safety_resolved_topoarm.urdf")
    resolved_path.write_text(resolved_text, encoding="utf-8")
    return str(resolved_path)


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")

    robot_description_file_default = os.path.join(package_share, "temp_robot.urdf")
    robot_mesh_root_dir_default = os.path.join(
        package_share, "urdf", "topoarm_description", "meshes", "topoarm"
    )

    robot_description_topic = DeclareLaunchArgument(
        "robot_description_topic",
        default_value="robot_description",
        description="Topic used by the robot_description_player",
    )
    udp_port = DeclareLaunchArgument(
        "udp_port",
        default_value="12345",
        description="UDP port that feeds the topoarm joint state player",
    )
    joint_state_source = DeclareLaunchArgument(
        "joint_state_source",
        default_value="rviz",
        description="Which source should drive /joint_states: rviz or real",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world",
        description="Fixed frame for RViz-oriented publishers",
    )
    enable_safety_monitor = DeclareLaunchArgument(
        "enable_safety_monitor",
        default_value="false",
        description="Whether to start safety_monitor_node",
    )
    gng_model_path = DeclareLaunchArgument(
        "gng_model_path",
        default_value="",
        description="Optional GNG model override; defaults come from gng_results/config.txt",
    )
    vlut_path = DeclareLaunchArgument(
        "vlut_path",
        default_value="",
        description="Optional VLUT override; defaults come from gng_results/config.txt",
    )
    gng_results_config_path = DeclareLaunchArgument(
        "gng_results_config_path",
        default_value="gng_results/config.txt",
        description="Path to the GNG results config file",
    )

    robot_description_file_resolved = _write_resolved_robot_description(robot_description_file_default)
    robot_description_text = Path(robot_description_file_resolved).read_text(encoding="utf-8")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_text,
        }],
    )

    robot_description_player = Node(
        package="gng_safety",
        executable="robot_description_player",
        name="robot_description_player",
        output="screen",
        parameters=[{
            "robot_description_file": robot_description_file_resolved,
            "mesh_root_dir": robot_mesh_root_dir_default,
            "topic_name": LaunchConfiguration("robot_description_topic"),
            "poll_ms": 1000,
            "republish_ms": 1000,
        }],
    )

    topoarm_joint_state_player = Node(
        package="gng_safety",
        executable="topoarm_joint_state_player",
        name="topoarm_joint_state_player",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("joint_state_source"),
            "' == 'rviz'",
        ])),
        parameters=[{
            "udp_port": LaunchConfiguration("udp_port"),
            "base_frame": LaunchConfiguration("frame_id"),
            "output_topic": "/joint_states_rviz",
        }],
    )

    robot_bridge_node = Node(
        package="gng_safety",
        executable="robot_bridge_node",
        name="robot_bridge_node",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("joint_state_source"),
            "' == 'real'",
        ])),
        parameters=[{
            "joint_state_topic": "/joint_states_real",
        }],
    )

    joint_state_mux_node = Node(
        package="gng_safety",
        executable="joint_state_mux_node",
        name="joint_state_mux_node",
        output="screen",
        parameters=[{
            "rviz_topic": "/joint_states_rviz",
            "real_topic": "/joint_states_real",
            "output_topic": "/joint_states",
            "active_source": LaunchConfiguration("joint_state_source"),
        }],
    )

    self_recognition_viz_node = Node(
        package="gng_safety",
        executable="self_recognition_viz_node",
        name="self_recognition_viz_node",
        output="screen",
        parameters=[{
            "robot_urdf_path": robot_description_file_resolved,
        }],
    )

    safety_monitor_node = Node(
        package="gng_safety",
        executable="safety_monitor_node",
        name="safety_monitor_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_safety_monitor")),
        parameters=[{
            "gng_results_config_path": LaunchConfiguration("gng_results_config_path"),
            "gng_model_path": LaunchConfiguration("gng_model_path"),
            "vlut_path": LaunchConfiguration("vlut_path"),
            "lidar_pos": [0.0, 0.0, 1.0],
            "lidar_rot": [0.0, 0.0, 0.0],
            "robot_pos": [0.0, 0.0, 0.0],
            "robot_rot": [0.0, 0.0, 0.0],
        }],
    )

    return LaunchDescription([
        robot_description_topic,
        udp_port,
        joint_state_source,
        frame_id,
        enable_safety_monitor,
        gng_results_config_path,
        gng_model_path,
        vlut_path,
        robot_state_publisher,
        robot_description_player,
        topoarm_joint_state_player,
        robot_bridge_node,
        joint_state_mux_node,
        self_recognition_viz_node,
        safety_monitor_node,
    ])
