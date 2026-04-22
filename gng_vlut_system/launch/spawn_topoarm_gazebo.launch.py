import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _write_resolved_robot_description(
    robot_description_file: str,
    resource_root_dir: str,
    output_name: str,
) -> str:
    text = Path(robot_description_file).read_text(encoding="utf-8")
    replacement = "file://" + resource_root_dir.rstrip("/") + "/"
    resolved_text = text
    pos = 0
    while True:
        pos = resolved_text.find("package://", pos)
        if pos == -1:
            break
        subpath_start = resolved_text.find("/", pos + len("package://"))
        if subpath_start == -1:
            break
        resolved_text = (
            resolved_text[:pos]
            + replacement
            + resolved_text[subpath_start + 1 :]
        )
        pos += len(replacement)
    resolved_path = Path("/tmp") / output_name
    resolved_path.write_text(resolved_text, encoding="utf-8")
    return str(resolved_path)


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")
    resource_root_dir_default = os.path.join(package_share, "urdf", "topoarm_description")
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
    robot_description_file = _write_resolved_robot_description(
        os.path.join(package_share, "temp_robot.urdf"),
        resource_root_dir_default,
        "gng_safety_resolved_topoarm_gazebo.urdf",
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
            "-file",
            robot_description_file,
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
        gazebo_gui,
        spawn_entity_name,
        spawn_x,
        spawn_y,
        spawn_z,
        spawn_yaw,
        gazebo_server,
        gazebo_client,
        gazebo_spawn_entity,
    ])
