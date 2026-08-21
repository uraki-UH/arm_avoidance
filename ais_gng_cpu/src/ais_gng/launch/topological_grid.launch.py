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
        description=(
            "Connected direct, edge-inferred, and triangle-inferred grasp-candidate voxels."
        ),
    )
    delta_topic = DeclareLaunchArgument(
        "delta_topic",
        default_value="",
        description="Changed voxel labels only; empty derives <output_topic>/delta.",
    )
    isolated_topic = DeclareLaunchArgument(
        "isolated_topic",
        default_value="",
        description=(
            "Visualization-only isolated voxels excluded from grasp candidates; "
            "empty derives <output_topic>/isolated."
        ),
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
    inferred_require_input_points = DeclareLaunchArgument(
        "inferred_require_input_points",
        default_value="true",
        description="Emit inferred edge/triangle cells only where current points exist.",
    )
    triangle_inferred_topic = DeclareLaunchArgument(
        "triangle_inferred_topic",
        default_value="",
        description="Triangle-only output; empty derives <output_topic>/triangle_inferred.",
    )
    triangle_inference_enabled = DeclareLaunchArgument(
        "triangle_inference_enabled",
        default_value="true",
        description="Fill cells intersected by validated GNG three-edge triangle faces.",
    )
    triangle_max_edge_length = DeclareLaunchArgument(
        "triangle_max_edge_length",
        default_value="0.05",
        description="Maximum length of every accepted triangle edge, in meters.",
    )
    triangle_min_area = DeclareLaunchArgument(
        "triangle_min_area",
        default_value="0.000001",
        description="Minimum accepted triangle area, in square meters.",
    )
    triangle_min_aspect_ratio = DeclareLaunchArgument(
        "triangle_min_aspect_ratio",
        default_value="0.05",
        description="Minimum twice-area divided by squared longest edge.",
    )
    triangle_max_normal_angle_deg = DeclareLaunchArgument(
        "triangle_max_normal_angle_deg",
        default_value="45.0",
        description="Maximum angle between triangle and valid node normals.",
    )
    triangle_min_point_support_ratio = DeclareLaunchArgument(
        "triangle_min_point_support_ratio",
        default_value="0.0",
        description="Minimum fraction of triangle cells containing current input points.",
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
        description="Require current input-point support selected by point_support_mode.",
    )
    point_support_mode = DeclareLaunchArgument(
        "point_support_mode",
        default_value="auto",
        description=(
            "Point support source: auto prefers GNG node inpcl_ids and falls back to "
            "metric radius; radius, node_input_ids, and same_cell force one behavior."
        ),
    )
    point_support_radius_m = DeclareLaunchArgument(
        "point_support_radius_m",
        default_value="0.02",
        description="Metric point-cloud support radius used by radius and auto modes.",
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
    neighbor_radius_m = DeclareLaunchArgument(
        "neighbor_radius_m",
        default_value="0.02",
        description=(
            "Physical Chebyshev neighbor radius in meters; set 0 to use "
            "neighbor_radius_cells."
        ),
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
        default_value="2",
        description="Updates to retain an active voxel after its eligible label disappears.",
    )
    node_identity_retention_enabled = DeclareLaunchArgument(
        "node_identity_retention_enabled",
        default_value="false",
        description="Retain non-isolated cells while their source node generation remains nearby.",
    )
    node_identity_max_displacement = DeclareLaunchArgument(
        "node_identity_max_displacement",
        default_value="0.02",
        description="Maximum source-node displacement that retains its previous cell, in meters.",
    )
    node_identity_history_migration_enabled = DeclareLaunchArgument(
        "node_identity_history_migration_enabled",
        default_value="true",
        description="Move temporal history with a uniquely identified GNG node.",
    )
    history_reset_on_time_regression = DeclareLaunchArgument(
        "history_reset_on_time_regression",
        default_value="false",
        description="Clear temporal history when rosbag or source timestamps move backward.",
    )
    history_reset_node_count_ratio = DeclareLaunchArgument(
        "history_reset_node_count_ratio",
        default_value="0.5",
        description="Clear history when node count falls below this fraction; 0 disables it.",
    )

    return LaunchDescription([
        node_name,
        input_topic,
        pointcloud_topic,
        pointcloud_timeout_sec,
        output_topic,
        delta_topic,
        isolated_topic,
        edge_inferred_topic,
        edge_inference_enabled,
        edge_max_length,
        inferred_require_input_points,
        triangle_inferred_topic,
        triangle_inference_enabled,
        triangle_max_edge_length,
        triangle_min_area,
        triangle_min_aspect_ratio,
        triangle_max_normal_angle_deg,
        triangle_min_point_support_ratio,
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
        point_support_mode,
        point_support_radius_m,
        minimum_input_points_per_voxel,
        neighbor_radius_cells,
        neighbor_radius_m,
        history_window_size,
        minimum_label_history_count,
        minimum_point_input_history_count,
        isolated_minimum_label_history_count,
        isolated_minimum_point_input_history_count,
        maximum_missing_label_updates,
        node_identity_retention_enabled,
        node_identity_max_displacement,
        node_identity_history_migration_enabled,
        history_reset_on_time_regression,
        history_reset_node_count_ratio,
        Node(
            package="ais_gng",
            executable="topological_grid_node",
            name=LaunchConfiguration("node_name"),
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "pointcloud_timeout_sec": LaunchConfiguration("pointcloud_timeout_sec"),
                "output_topic": LaunchConfiguration("output_topic"),
                "delta_topic": LaunchConfiguration("delta_topic"),
                "isolated_topic": LaunchConfiguration("isolated_topic"),
                "edge_inferred_topic": LaunchConfiguration("edge_inferred_topic"),
                "edge_inference_enabled": LaunchConfiguration("edge_inference_enabled"),
                "edge_max_length": LaunchConfiguration("edge_max_length"),
                "inferred_require_input_points": LaunchConfiguration(
                    "inferred_require_input_points"
                ),
                "triangle_inferred_topic": LaunchConfiguration("triangle_inferred_topic"),
                "triangle_inference_enabled": LaunchConfiguration(
                    "triangle_inference_enabled"
                ),
                "triangle_max_edge_length": LaunchConfiguration(
                    "triangle_max_edge_length"
                ),
                "triangle_min_area": LaunchConfiguration("triangle_min_area"),
                "triangle_min_aspect_ratio": LaunchConfiguration(
                    "triangle_min_aspect_ratio"
                ),
                "triangle_max_normal_angle_deg": LaunchConfiguration(
                    "triangle_max_normal_angle_deg"
                ),
                "triangle_min_point_support_ratio": LaunchConfiguration(
                    "triangle_min_point_support_ratio"
                ),
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
                "point_support_mode": LaunchConfiguration("point_support_mode"),
                "point_support_radius_m": LaunchConfiguration(
                    "point_support_radius_m"
                ),
                "minimum_input_points_per_voxel": LaunchConfiguration(
                    "minimum_input_points_per_voxel"
                ),
                "neighbor_radius_cells": LaunchConfiguration("neighbor_radius_cells"),
                "neighbor_radius_m": LaunchConfiguration("neighbor_radius_m"),
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
                "node_identity_retention_enabled": LaunchConfiguration(
                    "node_identity_retention_enabled"
                ),
                "node_identity_max_displacement": LaunchConfiguration(
                    "node_identity_max_displacement"
                ),
                "node_identity_history_migration_enabled": LaunchConfiguration(
                    "node_identity_history_migration_enabled"
                ),
                "history_reset_on_time_regression": LaunchConfiguration(
                    "history_reset_on_time_regression"
                ),
                "history_reset_node_count_ratio": LaunchConfiguration(
                    "history_reset_node_count_ratio"
                ),
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "INFO"],
        ),
    ])
