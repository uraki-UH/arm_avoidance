import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml"),
        description="Path to the ROS 2 parameters file for the GNG nodes.",
    )
    gng_model_path = DeclareLaunchArgument(
        "gng_model_path",
        default_value="",
        description="Optional GNG model override; defaults come from the params file",
    )
    vlut_path = DeclareLaunchArgument(
        "vlut_path",
        default_value="",
        description="Optional VLUT override; defaults come from the params file",
    )
    publish_hz = DeclareLaunchArgument(
        "publish_hz",
        default_value="5.0",
        description="Topological map publish rate",
    )
    edge_mode = DeclareLaunchArgument(
        "edge_mode",
        default_value="-1",
        description="Edge source: -1 auto, 0 angle, 1 coord",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="world",
        description="Frame id for the published topological map",
    )
    occupied_voxels_topic = DeclareLaunchArgument(
        "occupied_voxels_topic",
        default_value="/occupied_voxels",
        description="Topic providing occupied voxel ids",
    )
    danger_voxels_topic = DeclareLaunchArgument(
        "danger_voxels_topic",
        default_value="/danger_voxels",
        description="Topic providing danger voxel ids",
    )

    bridge_node = Node(
        package="gng_vlut_system",
        executable="topofuzzy_bridge_node",
        name="topofuzzy_bridge_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "gng_model_path": LaunchConfiguration("gng_model_path"),
                "vlut_path": LaunchConfiguration("vlut_path"),
                "publish_hz": LaunchConfiguration("publish_hz"),
                "edge_mode": LaunchConfiguration("edge_mode"),
                "frame_id": LaunchConfiguration("frame_id"),
                "occupied_voxels_topic": LaunchConfiguration("occupied_voxels_topic"),
                "danger_voxels_topic": LaunchConfiguration("danger_voxels_topic"),
            },
        ],
    )

    return LaunchDescription([
        params_file_arg,
        gng_model_path,
        vlut_path,
        publish_hz,
        edge_mode,
        frame_id,
        occupied_voxels_topic,
        danger_voxels_topic,
        bridge_node,
    ])
