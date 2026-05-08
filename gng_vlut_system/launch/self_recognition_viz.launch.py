import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_robot_description_path(pkg_share: str, raw_path: str) -> str:
    if not raw_path:
        return os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")

    if raw_path.startswith("package://gng_vlut_system/"):
        return os.path.join(pkg_share, raw_path[len("package://gng_vlut_system/"):])

    if raw_path.startswith("package://"):
        pkg_and_path = raw_path[len("package://"):]
        _, _, rel_path = pkg_and_path.partition("/")
        if rel_path:
            return os.path.join(pkg_share, rel_path)

    return raw_path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_urdf = resolve_robot_description_path(
        pkg_share,
        LaunchConfiguration("robot_description_file").perform(context),
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "robot_description_file": robot_urdf,
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
            }.items()
        ),
        Node(
            package="gng_vlut_system",
            executable="self_recognition_viz_node",
            name="self_recognition_viz_node",
            output="screen",
            parameters=[{
                "robot_urdf_path": robot_urdf,
                "marker_frame_id": LaunchConfiguration("marker_frame_id"),
                "joint_topic": LaunchConfiguration("joint_topic"),
                "voxel_size": LaunchConfiguration("voxel_size"),
                "update_hz": LaunchConfiguration("update_hz"),
                "publish_self_mask": LaunchConfiguration("publish_self_mask"),
                "publish_link_voxels": LaunchConfiguration("publish_link_voxels"),
                "publish_link_aabb": LaunchConfiguration("publish_link_aabb"),
                "display_mode": LaunchConfiguration("display_mode"),
            }],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="",
        ),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("marker_frame_id", default_value="world"),
        DeclareLaunchArgument("joint_topic", default_value="/joint_states"),
        DeclareLaunchArgument("voxel_size", default_value="0.02"),
        DeclareLaunchArgument("update_hz", default_value="10.0"),
        DeclareLaunchArgument("publish_self_mask", default_value="true"),
        DeclareLaunchArgument("publish_link_voxels", default_value="true"),
        DeclareLaunchArgument("publish_link_aabb", default_value="true"),
        DeclareLaunchArgument("display_mode", default_value="link_local"),
        OpaqueFunction(function=launch_setup),
    ])
