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
        description="Combined direct and edge-inferred object voxels.",
    )
    edge_inferred_topic = DeclareLaunchArgument(
        "edge_inferred_topic",
        default_value="",
        description="Edge-only output; empty derives <output_topic>/edge_inferred.",
    )
    edge_inference_enabled = DeclareLaunchArgument(
        "edge_inference_enabled",
        default_value="true",
        description="Fill cells between stable direct voxels connected by GNG edges.",
    )
    edge_max_length = DeclareLaunchArgument(
        "edge_max_length",
        default_value="0.10",
        description="Maximum voxel-center edge length eligible for filling, in meters.",
    )
    summary_topic = DeclareLaunchArgument(
        "summary_topic",
        default_value="/topological_grid_voxels/summary",
    )
    grid_size = DeclareLaunchArgument(
        "grid_size",
        default_value="0.01",
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
    history_window_size = DeclareLaunchArgument(
        "history_window_size",
        default_value="100",
        description="Number of synchronized updates retained per voxel.",
    )
    minimum_label_history_count = DeclareLaunchArgument(
        "minimum_label_history_count",
        default_value="3",
        description="Label occurrences required within the history window.",
    )
    minimum_point_input_history_count = DeclareLaunchArgument(
        "minimum_point_input_history_count",
        default_value="3",
        description="Point-cloud input updates required within the history window.",
    )
    isolated_minimum_label_history_count = DeclareLaunchArgument(
        "isolated_minimum_label_history_count",
        default_value="5",
        description="Label occurrences required for a voxel with no neighbors.",
    )
    isolated_minimum_point_input_history_count = DeclareLaunchArgument(
        "isolated_minimum_point_input_history_count",
        default_value="5",
        description="Point-cloud input updates required for a voxel with no neighbors.",
    )
    maximum_missing_label_updates = DeclareLaunchArgument(
        "maximum_missing_label_updates",
        default_value="10",
        description="Updates to retain an active voxel after its eligible label disappears.",
    )

    return LaunchDescription([
        node_name,
        input_topic,
        pointcloud_topic,
        pointcloud_timeout_sec,
        output_topic,
        edge_inferred_topic,
        edge_inference_enabled,
        edge_max_length,
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
        excluded_labels,
        require_input_points,
        minimum_input_points_per_voxel,
        neighbor_radius_cells,
        history_window_size,
        minimum_label_history_count,
        minimum_point_input_history_count,
        isolated_minimum_label_history_count,
        isolated_minimum_point_input_history_count,
        maximum_missing_label_updates,
        Node(
            package="ais_gng",
            executable="topological_grid_node",
            name=LaunchConfiguration("node_name"),
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "pointcloud_timeout_sec": LaunchConfiguration("pointcloud_timeout_sec"),
                "output_topic": LaunchConfiguration("output_topic"),
                "edge_inferred_topic": LaunchConfiguration("edge_inferred_topic"),
                "edge_inference_enabled": LaunchConfiguration("edge_inference_enabled"),
                "edge_max_length": LaunchConfiguration("edge_max_length"),
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
                "excluded_labels": LaunchConfiguration("excluded_labels"),
                "require_input_points": LaunchConfiguration("require_input_points"),
                "minimum_input_points_per_voxel": LaunchConfiguration(
                    "minimum_input_points_per_voxel"
                ),
                "neighbor_radius_cells": LaunchConfiguration("neighbor_radius_cells"),
                "history_window_size": LaunchConfiguration("history_window_size"),
                "minimum_label_history_count": LaunchConfiguration(
                    "minimum_label_history_count"
                ),
                "minimum_point_input_history_count": LaunchConfiguration(
                    "minimum_point_input_history_count"
                ),
                "isolated_minimum_label_history_count": LaunchConfiguration(
                    "isolated_minimum_label_history_count"
                ),
                "isolated_minimum_point_input_history_count": LaunchConfiguration(
                    "isolated_minimum_point_input_history_count"
                ),
                "maximum_missing_label_updates": LaunchConfiguration(
                    "maximum_missing_label_updates"
                ),
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "INFO"],
        ),
    ])
