import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_vlut_system")

    rviz_config_default = os.path.join(package_share, "config", "rviz", "gng_safety.rviz")

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
        description="Fixed frame for RViz",
    )
    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2",
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
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_default,
        description="RViz config file",
    )
    experiment_id = DeclareLaunchArgument(
        "experiment_id",
        default_value="standalone_train",
        description="ID of the experiment to load GNG/VLUT data from.",
    )
    data_directory = DeclareLaunchArgument(
        "data_directory",
        default_value="gng_results",
        description="Root directory where GNG/VLUT data is stored.",
    )

    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gng_vlut_system"), "launch", "topoarm_runtime.launch.py")
        ),
        launch_arguments={
            "frame_id": LaunchConfiguration("frame_id"),
            "joint_state_source": LaunchConfiguration("joint_state_source"),
            "robot_description_topic": LaunchConfiguration("robot_description_topic"),
            "udp_port": LaunchConfiguration("udp_port"),
            "enable_safety_monitor": LaunchConfiguration("enable_safety_monitor"),
            "gng_model_path": LaunchConfiguration("gng_model_path"),
            "vlut_path": LaunchConfiguration("vlut_path"),
            "experiment_id": LaunchConfiguration("experiment_id"),
            "data_directory": LaunchConfiguration("data_directory"),
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config"), "-f", LaunchConfiguration("frame_id")],
        output="log",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        robot_description_topic,
        udp_port,
        joint_state_source,
        frame_id,
        use_rviz,
        enable_safety_monitor,
        gng_model_path,
        vlut_path,
        rviz_config,
        experiment_id, # 追加
        data_directory, # 追加
        runtime,
        rviz2,
    ])
