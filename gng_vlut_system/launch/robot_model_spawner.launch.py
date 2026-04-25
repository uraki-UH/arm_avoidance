from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("gng_safety")
    robot_description_file_default = PathJoinSubstitution([package_share, "temp_robot.urdf"])
    resource_root_dir_default = PathJoinSubstitution([package_share, "urdf", "topoarm_description"])
    robot_mesh_root_dir_default = PathJoinSubstitution([resource_root_dir_default, "meshes", "topoarm"])

    robot_description_file = DeclareLaunchArgument(
        "robot_description_file",
        default_value=robot_description_file_default,
        description="URDF/Xacro file used for the robot arm bridge",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world",
        description="Frame id used in the streamed robot arm payload",
    )
    
    resource_root_dir = DeclareLaunchArgument(
        "resource_root_dir",
        default_value=resource_root_dir_default,
        description="Root directory that contains urdf/ and meshes/",
    )
    mesh_root_dir = DeclareLaunchArgument(
        "mesh_root_dir",
        default_value=robot_mesh_root_dir_default,
        description="Mesh directory under the resource root",
    )
    
    
    runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory("gng_safety"), "launch", "topoarm_runtime.launch.py"])
        ),
        launch_arguments={
            "frame_id": LaunchConfiguration("frame_id"),
        }.items(),
    )

    return LaunchDescription([
        robot_description_file,
        frame_id,
        mesh_root_dir,
        runtime,
    ])
