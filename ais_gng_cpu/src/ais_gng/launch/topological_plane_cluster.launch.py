"""Extract and visualize broad planar patches from the GNG graph.

Only topics are launch-time choices.  The density-normalized geometric
profile stays in YAML so that a normal command is short and reproducible.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ais_gng"), "config", "topological_plane_cluster.yaml"]
        ),
        description="Advanced density-normalized plane-clustering profile.",
    )
    input_topic = DeclareLaunchArgument("input_topic", default_value="/topological_map")
    output_topic = DeclareLaunchArgument(
        "output_topic", default_value="/topological_planar_clusters"
    )
    return LaunchDescription(
        [
            params_file,
            input_topic,
            output_topic,
            Node(
                package="ais_gng",
                executable="topological_plane_cluster_node",
                name="topological_plane_cluster_node",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    },
                ],
                output="screen",
            ),
        ]
    )
