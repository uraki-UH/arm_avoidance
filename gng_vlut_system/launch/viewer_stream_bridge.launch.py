from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")

    # Arguments for joint state source
    joint_state_source = DeclareLaunchArgument(
        "joint_state_source",
        default_value="rviz",
        description="Which source should drive /joint_states: rviz, real, or udp",
    )
    udp_port = DeclareLaunchArgument(
        "udp_port",
        default_value="12345",
        description="UDP port that feeds the topoarm joint state player",
    )

    # Arguments for robot_arm_bridge_node
    end_effector_name = DeclareLaunchArgument(
        "end_effector_name",
        default_value="",
        description="Optional end-effector link name for the kinematic chain",
    )
    joint_state_topic = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/joint_states",
        description="JointState topic driving the robot arm pose",
    )
    stream_topic = DeclareLaunchArgument(
        "stream_topic",
        default_value="/viewer/internal/stream/robot_arm",
        description="Viewer bridge stream topic",
    )
    publish_hz = DeclareLaunchArgument(
        "publish_hz",
        default_value="20.0",
        description="Robot arm publish rate",
    )

    # Arguments for safety_monitor_node (passed to topoarm_runtime)
    enable_safety_monitor = DeclareLaunchArgument(
        "enable_safety_monitor",
        default_value="false",
        description="Whether to start safety_monitor_node",
    )
    experiment_id = DeclareLaunchArgument(
        "experiment_id",
        default_value="",
        description="ID of the experiment to load GNG/VLUT data from",
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

    # Include the simplified topoarm_viewer_bridge.launch.py
    topoarm_viewer_bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "robot_model_spawner.launch.py"])
        ),
        launch_arguments={
            "robot_description_file": LaunchConfiguration("robot_description_file"),
            "resource_root_dir": LaunchConfiguration("resource_root_dir"),
            "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
            "frame_id": LaunchConfiguration("frame_id"),
        }.items(),
    )

    # Conditionally include udp_joint_state_player.launch.py
    udp_joint_state_player_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "udp_joint_state_player.launch.py"])
        ),
        launch_arguments={
            "udp_port": LaunchConfiguration("udp_port"),
            "frame_id": LaunchConfiguration("frame_id"),
        }.items(),
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration("joint_state_source"),
            "' == 'rviz'", # or 'udp'
        ])),
    )

    # Launch robot_arm_bridge_node
    robot_arm_bridge_node = Node(
        package="gng_safety",
        executable="robot_arm_bridge_node",
        name="robot_arm_bridge_node",
        output="screen",
        parameters=[{
            "robot_description_file": LaunchConfiguration("robot_description_file"),
            "resource_root_dir": LaunchConfiguration("resource_root_dir"),
            "mesh_root_dir": LaunchConfiguration("mesh_root_dir"),
            "end_effector_name": LaunchConfiguration("end_effector_name"),
            "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            "stream_topic": LaunchConfiguration("stream_topic"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": LaunchConfiguration("publish_hz"),
        }],
    )

    return LaunchDescription([
        joint_state_source,
        udp_port,
        end_effector_name,
        joint_state_topic,
        stream_topic,
        publish_hz,
        enable_safety_monitor,
        experiment_id,
        gng_model_path,
        vlut_path,
        
        topoarm_viewer_bridge_include,
        udp_joint_state_player_include,
        robot_arm_bridge_node,
    ])
