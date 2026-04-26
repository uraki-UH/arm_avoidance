import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    gng_vlut_system_share = get_package_share_directory("gng_vlut_system")
    
    # Default to the topoarm xacro
    robot_description_file_default = os.path.join(
        gng_vlut_system_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro"
    )
    resource_root_dir_default = os.path.join(gng_vlut_system_share, "urdf")
    robot_mesh_root_dir_default = os.path.join(
        resource_root_dir_default, "meshes", "topoarm"
    )
    
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(gng_vlut_system_share, "config", "gng_safety_params.yaml"),
        description="Path to the ROS 2 parameters file for the GNG nodes.",
    )

    robot_description_file_arg = DeclareLaunchArgument(
        "robot_description_file",
        default_value=robot_description_file_default,
        description="URDF/Xacro file for the robot",
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
        description="Optional GNG model override; defaults come from params file",
    )
    vlut_path = DeclareLaunchArgument(
        "vlut_path",
        default_value="",
        description="Optional VLUT override; defaults come from params file",
    )
    experiment_id = DeclareLaunchArgument(
        "experiment_id",
        default_value="default_experiment",
    )
    data_directory = DeclareLaunchArgument(
        "data_directory",
        default_value="gng_results",
    )

    # Use xacro to expand the robot description
    robot_description_content = Command([
        'xacro ', LaunchConfiguration('robot_description_file')
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
        }],
    )

    robot_description_player = Node(
        package="gng_safety",
        executable="robot_description_player",
        name="robot_description_player",
        output="screen",
        parameters=[{
            "robot_description_file": LaunchConfiguration("robot_description_file"),
            "resource_root_dir": resource_root_dir_default,
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
            "robot_urdf_path": LaunchConfiguration("robot_description_file"),
        }],
    )

    safety_monitor_node = Node(
        package="gng_safety",
        executable="safety_monitor_node",
        name="safety_monitor_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_safety_monitor")),
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "gng_model_path": LaunchConfiguration("gng_model_path"),
                "vlut_path": LaunchConfiguration("vlut_path"),
                "lidar_pos": [0.0, 0.0, 1.0],
                "lidar_rot": [0.0, 0.0, 0.0],
                "robot_pos": [0.0, 0.0, 0.0],
                "robot_rot": [0.0, 0.0, 0.0],
                "experiment_id": LaunchConfiguration("experiment_id"),
                "data_directory": LaunchConfiguration("data_directory"),
                "gng_model_filename": "gng.bin",
                "vlut_filename": "vlut.bin",
                "robot_urdf_path": LaunchConfiguration("robot_description_file"),
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        robot_description_file_arg,
        robot_description_topic,
        udp_port,
        joint_state_source,
        frame_id,
        enable_safety_monitor,
        gng_model_path,
        vlut_path,
        experiment_id,
        data_directory,
        robot_state_publisher,
        robot_description_player,
        topoarm_joint_state_player,
        robot_bridge_node,
        joint_state_mux_node,
        self_recognition_viz_node,
        safety_monitor_node,
    ])
