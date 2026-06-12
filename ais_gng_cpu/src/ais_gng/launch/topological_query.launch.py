from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/topological_map/merged",
    )
    output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/reachable_handle_nodes",
    )
    summary_topic = DeclareLaunchArgument(
        "summary_topic",
        default_value="/reachable_handle_nodes/summary",
    )
    relation_mode = DeclareLaunchArgument(
        "relation_mode",
        default_value="graph_edges",
    )
    semantic_label = DeclareLaunchArgument(
        "semantic_label",
        default_value="1",
    )
    max_euclidean_distance = DeclareLaunchArgument(
        "max_euclidean_distance",
        default_value="0.5",
    )
    max_hops = DeclareLaunchArgument(
        "max_hops",
        default_value="-1",
    )
    include_seed_nodes = DeclareLaunchArgument(
        "include_seed_nodes",
        default_value="true",
    )

    return LaunchDescription([
        input_topic,
        output_topic,
        summary_topic,
        relation_mode,
        semantic_label,
        max_euclidean_distance,
        max_hops,
        include_seed_nodes,
        Node(
            package="ais_gng",
            executable="topological_query_node",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "summary_topic": LaunchConfiguration("summary_topic"),
                "relation_mode": LaunchConfiguration("relation_mode"),
                "semantic_label": LaunchConfiguration("semantic_label"),
                "max_euclidean_distance": LaunchConfiguration("max_euclidean_distance"),
                "max_hops": LaunchConfiguration("max_hops"),
                "include_seed_nodes": LaunchConfiguration("include_seed_nodes"),
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "INFO"],
        )
    ])
