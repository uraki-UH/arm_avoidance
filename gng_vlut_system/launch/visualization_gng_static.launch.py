from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("topic_name", default_value="topological_map_vis"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        Node(
            package="gng_vlut_system",
            executable="visualization_gng_static_node",
            name="visualization_gng_static_node",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "topic_name": LaunchConfiguration("topic_name"),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
    ])
