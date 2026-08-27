from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_hypothesis_node(context):
    template_id = LaunchConfiguration("template_id").perform(context)
    template_topic = f"/{template_id}/topological_map_static"
    return [Node(
        package="gng_vlut_system",
        executable="object_match_hypothesis_publisher_node",
        name="object_match_hypothesis_publisher_node",
        output="screen",
        parameters=[{
            "template_id": template_id,
            "hypothesis_id": LaunchConfiguration("hypothesis_id"),
            "environment_topological_map_topic": LaunchConfiguration("environment_topological_map_topic"),
            "template_topological_map_topic": template_topic,
            "environment_cluster_id": LaunchConfiguration("environment_cluster_id"),
            "grid_cell_size": LaunchConfiguration("grid_cell_size"),
            "score": LaunchConfiguration("score"),
            "yaw_deg": LaunchConfiguration("yaw_deg"),
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_hz": LaunchConfiguration("publish_hz"),
            "aabb_line_width": LaunchConfiguration("aabb_line_width"),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("template_id"),
        DeclareLaunchArgument("hypothesis_id", default_value="candidate_0"),
        DeclareLaunchArgument("environment_topological_map_topic", default_value="/topological_map"),
        DeclareLaunchArgument("environment_cluster_id", default_value="-1"),
        DeclareLaunchArgument("grid_cell_size", default_value="0.03"),
        DeclareLaunchArgument("score", default_value="0.0"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("frame_id", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value="1.0"),
        DeclareLaunchArgument("aabb_line_width", default_value="0.004"),
        OpaqueFunction(function=create_hypothesis_node),
    ])
