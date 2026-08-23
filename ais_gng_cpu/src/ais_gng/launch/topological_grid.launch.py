"""Launch the topological voxel pre-filter with a small operational surface.

The normal invocation changes only the three topic names and the output grid
resolution. Algorithmic settings belong to the YAML profile so that a launch
command is reproducible and does not turn into an undocumented parameter dump.
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
            [FindPackageShare("ais_gng"), "config", "topological_grid.yaml"]
        ),
        description="Advanced algorithm profile YAML.",
    )
    input_topic = DeclareLaunchArgument(
        "input_topic", default_value="/topological_map/merged"
    )
    pointcloud_topic = DeclareLaunchArgument(
        "pointcloud_topic", default_value="/downsampling/grasp_support"
    )
    output_topic = DeclareLaunchArgument(
        "output_topic", default_value="/topological_grid_voxels"
    )
    grid_size = DeclareLaunchArgument(
        "grid_size", default_value="0.01", description="Voxel edge length in metres."
    )

    return LaunchDescription(
        [
            params_file,
            input_topic,
            pointcloud_topic,
            output_topic,
            grid_size,
            Node(
                package="ais_gng",
                executable="topological_grid_node",
                name="topological_grid_node",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "grid_size": LaunchConfiguration("grid_size"),
                    },
                ],
                output="screen",
            ),
        ]
    )
