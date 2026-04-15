import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("fuzz_gng_result_player")
    rviz_config_path = os.path.join(package_dir, "config", "fuzz_gng_result_player_arm.rviz")

    experiment_id = DeclareLaunchArgument(
        "experiment_id",
        default_value="topoarm_real_v1",
        description="Experiment id used under gng_results/<experiment_id>",
    )
    data_directory = DeclareLaunchArgument(
        "data_directory",
        default_value="/home/fuzzrobo/uraki_ws/fuzz_arm/gng_results",
        description="Directory that contains offline_urdf_trainer outputs",
    )
    phase2_suffix = DeclareLaunchArgument(
        "phase2_suffix",
        default_value="_phase2",
        description="Suffix used for the saved GNG binary",
    )
    config_file = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Optional config.txt path with key=value overrides",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world",
        description="Frame id used in TopologicalMap header",
    )
    poll_ms = DeclareLaunchArgument(
        "poll_ms",
        default_value="250",
        description="Polling interval for reloading the result file",
    )
    publish_edges = DeclareLaunchArgument(
        "publish_edges",
        default_value="false",
        description="Whether to show GNG edges in RViz",
    )
    republish_ms = DeclareLaunchArgument(
        "republish_ms",
        default_value="1000",
        description="How often to republish the latest MarkerArray",
    )
    udp_port = DeclareLaunchArgument(
        "udp_port",
        default_value="12345",
        description="UDP port that receives arm joint commands",
    )
    robot_description_xacro_file = DeclareLaunchArgument(
        "robot_description_xacro_file",
        default_value="/home/fuzzrobo/uraki_ws/fuzz_arm/topoarm_description/urdf/topoarm.urdf.xacro",
        description="TopoArm xacro used by robot_state_publisher",
    )
    robot_model_file = DeclareLaunchArgument(
        "robot_model_file",
        default_value="/home/fuzzrobo/uraki_ws/fuzz_arm/temp_robot.urdf",
        description="URDF file used by RobotModel display",
    )

    result_player = Node(
        package="fuzz_gng_result_player",
        executable="gng_result_player",
        name="gng_result_player",
        output="screen",
        parameters=[{
            "experiment_id": LaunchConfiguration("experiment_id"),
            "data_directory": LaunchConfiguration("data_directory"),
            "phase2_suffix": LaunchConfiguration("phase2_suffix"),
            "config_file": LaunchConfiguration("config_file"),
            "frame_id": LaunchConfiguration("frame_id"),
            "poll_ms": LaunchConfiguration("poll_ms"),
            "publish_edges": LaunchConfiguration("publish_edges"),
            "republish_ms": LaunchConfiguration("republish_ms"),
        }],
    )

    arm_player = Node(
        package="fuzz_gng_result_player",
        executable="udp_joint_state_player",
        name="udp_joint_state_player",
        output="screen",
        parameters=[{
            "udp_port": LaunchConfiguration("udp_port"),
            "base_frame": "world",
            "publish_gripper_mirror": True,
        }],
    )

    robot_description_player = Node(
        package="fuzz_gng_result_player",
        executable="robot_description_player",
        name="robot_description_player",
        output="screen",
        parameters=[{
            "robot_description_file": LaunchConfiguration("robot_model_file"),
            "mesh_root_dir": "/home/fuzzrobo/uraki_ws/fuzz_arm/topoarm_description/meshes/topoarm",
            "topic_name": "robot_description",
            "poll_ms": 1000,
            "republish_ms": 1000,
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro", " ", LaunchConfiguration("robot_description_xacro_file")]),
                value_type=str,
            ),
        }],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path, "-f", LaunchConfiguration("frame_id")],
        output="screen",
    )

    return LaunchDescription([
        experiment_id,
        data_directory,
        phase2_suffix,
        config_file,
        frame_id,
        poll_ms,
        publish_edges,
        republish_ms,
        udp_port,
        robot_description_xacro_file,
        robot_model_file,
        result_player,
        arm_player,
        robot_description_player,
        robot_state_publisher,
        rviz2,
    ])
