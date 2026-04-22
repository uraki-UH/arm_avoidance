from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gng_results_config_path = DeclareLaunchArgument(
        "gng_results_config_path",
        default_value="gng_results/config.txt",
        description="Path to the GNG results config file",
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
        package="gng_safety",
        executable="topofuzzy_bridge_node",
        name="topofuzzy_bridge_node",
        output="screen",
        parameters=[{
            "gng_results_config_path": LaunchConfiguration("gng_results_config_path"),
            "gng_model_path": LaunchConfiguration("gng_model_path"),
            "vlut_path": LaunchConfiguration("vlut_path"),
            "publish_hz": LaunchConfiguration("publish_hz"),
            "edge_mode": LaunchConfiguration("edge_mode"),
            "frame_id": LaunchConfiguration("frame_id"),
            "occupied_voxels_topic": LaunchConfiguration("occupied_voxels_topic"),
            "danger_voxels_topic": LaunchConfiguration("danger_voxels_topic"),
        }],
    )

    return LaunchDescription([
        gng_results_config_path,
        gng_model_path,
        vlut_path,
        publish_hz,
        edge_mode,
        frame_id,
        occupied_voxels_topic,
        danger_voxels_topic,
        bridge_node,
    ])
