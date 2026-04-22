import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")

    robot_description_file_default = os.path.join(package_share, "temp_robot.urdf")
    robot_mesh_root_dir_default = os.path.join(
        package_share, "urdf", "real_model", "topoarm_description", "meshes", "topoarm"
    )

    robot_description_topic = DeclareLaunchArgument(
        "robot_description_topic",
        default_value="robot_description",
        description="Topic used by gazebo_ros/spawn_entity.py",
    )
    gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="false",
        description="Whether to start the Gazebo GUI",
    )
    spawn_entity_name = DeclareLaunchArgument(
        "spawn_entity_name",
        default_value="topoarm",
        description="Name used for the spawned Gazebo entity",
    )
    spawn_x = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Gazebo spawn X position",
    )
    spawn_y = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Gazebo spawn Y position",
    )
    spawn_z = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.02",
        description="Gazebo spawn Z position",
    )
    spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description="Gazebo spawn yaw",
    )

    robot_description_player = Node(
        package="gng_safety",
        executable="robot_description_player",
        name="robot_description_player",
        output="screen",
        parameters=[{
            "robot_description_file": robot_description_file_default,
            "mesh_root_dir": robot_mesh_root_dir_default,
            "topic_name": LaunchConfiguration("robot_description_topic"),
            "poll_ms": 1000,
            "republish_ms": 1000,
        }],
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "server_required": "true",
        }.items(),
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        additional_env={
            "QT_QPA_PLATFORM": "xcb",
        },
        condition=IfCondition(LaunchConfiguration("gazebo_gui")),
    )

    gazebo_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="gazebo_spawn_entity",
        output="screen",
        arguments=[
            "-topic",
            LaunchConfiguration("robot_description_topic"),
            "-entity",
            LaunchConfiguration("spawn_entity_name"),
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )

    return LaunchDescription([
        robot_description_topic,
        gazebo_gui,
        spawn_entity_name,
        spawn_x,
        spawn_y,
        spawn_z,
        spawn_yaw,
        robot_description_player,
        gazebo_server,
        gazebo_client,
        gazebo_spawn_entity,
    ])
