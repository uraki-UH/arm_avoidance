from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    node_name = DeclareLaunchArgument(
        "node_name",
        default_value="topological_grid_node",
    )
    input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/topological_map/merged",
    )
    pointcloud_topic = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="/downsampling/unknown",
        description="Point cloud used as current occupancy support.",
    )
    pointcloud_timeout_sec = DeclareLaunchArgument(
        "pointcloud_timeout_sec",
        default_value="0.5",
        description="Maximum wait for a point cloud with the same frame and timestamp.",
    )
    output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/topological_grid_voxels",
    )
    summary_topic = DeclareLaunchArgument(
        "summary_topic",
        default_value="/topological_grid_voxels/summary",
    )
    grid_size = DeclareLaunchArgument(
        "grid_size",
        default_value="0.02",
        description="Voxel edge length in meters.",
    )
    origin_x = DeclareLaunchArgument(
        "origin_x",
        default_value="0.0",
    )
    origin_y = DeclareLaunchArgument(
        "origin_y",
        default_value="0.0",
    )
    origin_z = DeclareLaunchArgument(
        "origin_z",
        default_value="0.0",
    )
    origin_shift_half = DeclareLaunchArgument(
        "origin_shift_half",
        default_value="false",
    )
    x_shift = DeclareLaunchArgument(
        "x_shift",
        default_value="42",
    )
    y_shift = DeclareLaunchArgument(
        "y_shift",
        default_value="21",
    )
    z_shift = DeclareLaunchArgument(
        "z_shift",
        default_value="0",
    )
    offset = DeclareLaunchArgument(
        "offset",
        default_value="1000000",
    )
    included_labels = DeclareLaunchArgument(
        "included_labels",
        default_value="UNKNOWN_OBJECT",
        description=(
            "Comma-separated labels eligible for object-candidate voxels. "
            "Use an empty string to allow every non-excluded label."
        ),
    )
    excluded_labels = DeclareLaunchArgument(
        "excluded_labels",
        default_value="SAFE_TERRAIN,HUMAN,CAR",
        description=(
            "Comma-separated TopologicalMap label names or numeric values excluded "
            "from voxelization. Use an empty string to include every label."
        ),
    )
    require_input_points = DeclareLaunchArgument(
        "require_input_points",
        default_value="true",
        description="Require a matching current point-cloud cell for each candidate.",
    )
    minimum_input_points_per_voxel = DeclareLaunchArgument(
        "minimum_input_points_per_voxel",
        default_value="1",
        description="Minimum current input-point support in each voxel.",
    )
    neighbor_radius_cells = DeclareLaunchArgument(
        "neighbor_radius_cells",
        default_value="1",
        description="Chebyshev radius used to count neighboring candidate voxels.",
    )
    minimum_observations = DeclareLaunchArgument(
        "minimum_observations",
        default_value="3",
        description="Consecutive point-supported updates required before publication.",
    )
    maximum_missed_updates = DeclareLaunchArgument(
        "maximum_missed_updates",
        default_value="2",
        description="Connected-cell misses tolerated before confirmation history resets.",
    )
    isolated_minimum_observations = DeclareLaunchArgument(
        "isolated_minimum_observations",
        default_value="5",
        description="Consecutive updates required for a voxel with no neighbors.",
    )
    isolated_maximum_missed_updates = DeclareLaunchArgument(
        "isolated_maximum_missed_updates",
        default_value="0",
        description="Isolated-cell misses tolerated before confirmation history resets.",
    )

    return LaunchDescription([
        node_name,
        input_topic,
        pointcloud_topic,
        pointcloud_timeout_sec,
        output_topic,
        summary_topic,
        grid_size,
        origin_x,
        origin_y,
        origin_z,
        origin_shift_half,
        x_shift,
        y_shift,
        z_shift,
        offset,
        included_labels,
        excluded_labels,
        require_input_points,
        minimum_input_points_per_voxel,
        neighbor_radius_cells,
        minimum_observations,
        maximum_missed_updates,
        isolated_minimum_observations,
        isolated_maximum_missed_updates,
        Node(
            package="ais_gng",
            executable="topological_grid_node",
            name=LaunchConfiguration("node_name"),
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "pointcloud_timeout_sec": LaunchConfiguration("pointcloud_timeout_sec"),
                "output_topic": LaunchConfiguration("output_topic"),
                "summary_topic": LaunchConfiguration("summary_topic"),
                "grid_size": LaunchConfiguration("grid_size"),
                "origin_x": LaunchConfiguration("origin_x"),
                "origin_y": LaunchConfiguration("origin_y"),
                "origin_z": LaunchConfiguration("origin_z"),
                "origin_shift_half": LaunchConfiguration("origin_shift_half"),
                "x_shift": LaunchConfiguration("x_shift"),
                "y_shift": LaunchConfiguration("y_shift"),
                "z_shift": LaunchConfiguration("z_shift"),
                "offset": LaunchConfiguration("offset"),
                "included_labels": LaunchConfiguration("included_labels"),
                "excluded_labels": LaunchConfiguration("excluded_labels"),
                "require_input_points": LaunchConfiguration("require_input_points"),
                "minimum_input_points_per_voxel": LaunchConfiguration(
                    "minimum_input_points_per_voxel"
                ),
                "neighbor_radius_cells": LaunchConfiguration("neighbor_radius_cells"),
                "minimum_observations": LaunchConfiguration("minimum_observations"),
                "maximum_missed_updates": LaunchConfiguration("maximum_missed_updates"),
                "isolated_minimum_observations": LaunchConfiguration(
                    "isolated_minimum_observations"
                ),
                "isolated_maximum_missed_updates": LaunchConfiguration(
                    "isolated_maximum_missed_updates"
                ),
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "INFO"],
        ),
    ])
