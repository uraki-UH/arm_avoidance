from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/topo_points"),
        DeclareLaunchArgument("pose_topic", default_value="/grasp_pose_candidates"),
        DeclareLaunchArgument("score_topic", default_value="/grasp_pose_scores"),
        DeclareLaunchArgument("marker_topic", default_value="/grasp_pose_markers"),
        DeclareLaunchArgument("target_frame_id", default_value="world"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("approach_offset", default_value="0.06"),
        DeclareLaunchArgument("max_candidates", default_value="256"),
        DeclareLaunchArgument("min_points_per_voxel", default_value="1"),
        Node(
            package="gng_vlut_system",
            executable="grasp_pose_candidate_producer_node",
            name="grasp_pose_candidate_producer_node",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "score_topic": LaunchConfiguration("score_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "target_frame_id": LaunchConfiguration("target_frame_id"),
                "voxel_size": LaunchConfiguration("voxel_size"),
                "approach_offset": LaunchConfiguration("approach_offset"),
                "max_candidates": LaunchConfiguration("max_candidates"),
                "min_points_per_voxel": LaunchConfiguration("min_points_per_voxel"),
            }],
        ),
    ])
