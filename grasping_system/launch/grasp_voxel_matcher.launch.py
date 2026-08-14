from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _argument(name, default_value, description=""):
    return DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
    )


def generate_launch_description():
    arguments = [
        _argument("node_name", "grasp_voxel_matcher_node"),
        _argument("object_voxels_topic", "/topological_grid_voxels"),
        _argument(
            "object_delta_topic",
            "",
            "Changed object voxels; empty derives <object_voxels_topic>/delta.",
        ),
        _argument(
            "environment_voxels_topic",
            "",
            "Optional full-environment grid used for forbidden-volume checks; empty reuses object voxels.",
        ),
        _argument(
            "environment_delta_topic",
            "",
            "Changed environment voxels; empty derives <environment_voxels_topic>/delta.",
        ),
        _argument(
            "incremental_matching_enabled",
            "true",
            "Use revisioned VoxelLabelDelta updates after the initial Voxel snapshot.",
        ),
        _argument("required_graph_topic", "grip_V_topological_map"),
        _argument("undersize_graph_topic", "grip_minV_topological_map"),
        _argument("forbidden_graph_topic", "grip_baseV_topological_map"),
        _argument("candidate_topic", "/grasp_voxel_candidates"),
        _argument("summary_topic", "/grasp_voxel_candidates/summary"),
        _argument("minimum_required_occupancy_ratio", "0.1"),
        _argument("minimum_required_hits", "3"),
        _argument("minimum_outside_undersize_hits", "1"),
        _argument("maximum_forbidden_hits", "0"),
        _argument("maximum_anchor_voxels", "500"),
        _argument("maximum_candidates", "50"),
        _argument("yaw_samples", "12"),
        _argument("roll", "0.0"),
        _argument("pitch", "0.0"),
        _argument("yaw_offset", "0.0"),
        _argument(
            "update_period_ms",
            "500",
            "Matching period in milliseconds; object updates are coalesced between runs.",
        ),
    ]

    parameters = {
        name: LaunchConfiguration(name)
        for name in (
            "object_voxels_topic",
            "object_delta_topic",
            "environment_voxels_topic",
            "environment_delta_topic",
            "incremental_matching_enabled",
            "required_graph_topic",
            "undersize_graph_topic",
            "forbidden_graph_topic",
            "candidate_topic",
            "summary_topic",
            "minimum_required_occupancy_ratio",
            "minimum_required_hits",
            "minimum_outside_undersize_hits",
            "maximum_forbidden_hits",
            "maximum_anchor_voxels",
            "maximum_candidates",
            "yaw_samples",
            "roll",
            "pitch",
            "yaw_offset",
            "update_period_ms",
        )
    }

    return LaunchDescription(
        arguments
        + [
            Node(
                package="grasping_system",
                executable="grasp_voxel_matcher_node",
                name=LaunchConfiguration("node_name"),
                output="screen",
                parameters=[parameters],
            )
        ]
    )
