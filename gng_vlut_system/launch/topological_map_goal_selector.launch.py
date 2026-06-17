from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    script = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector_node.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "topological_map_topic",
            default_value="/ToPoDualArm/topological_map_static",
        ),
        DeclareLaunchArgument("output_topic", default_value="/selected_topological_map"),
        DeclareLaunchArgument(
            "marker_topic",
            default_value="/selected_topological_map_markers",
        ),
        DeclareLaunchArgument("candidate_count", default_value="8"),
        DeclareLaunchArgument("non_collision_only", default_value="true"),
        DeclareLaunchArgument("orientation_weight", default_value="0.25"),
        DeclareLaunchArgument("target_pose_topic", default_value=""),
        DeclareLaunchArgument("target_point_topic", default_value=""),
        DeclareLaunchArgument(
            "target_pose_array_topic",
            default_value="/grasp_pose_candidates",
        ),
        DeclareLaunchArgument(
            "target_score_topic",
            default_value="/grasp_pose_scores",
        ),
        DeclareLaunchArgument(
            "goal_candidate_ids_topic",
            default_value="/selected_goal_candidate_ids",
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                script,
                "--topological-map-topic",
                LaunchConfiguration("topological_map_topic"),
                "--output-topic",
                LaunchConfiguration("output_topic"),
                "--marker-topic",
                LaunchConfiguration("marker_topic"),
                "--candidate-count",
                LaunchConfiguration("candidate_count"),
                "--orientation-weight",
                LaunchConfiguration("orientation_weight"),
                "--non-collision-only",
                LaunchConfiguration("non_collision_only"),
                "--target-pose-topic",
                LaunchConfiguration("target_pose_topic"),
                "--target-point-topic",
                LaunchConfiguration("target_point_topic"),
                "--target-pose-array-topic",
                LaunchConfiguration("target_pose_array_topic"),
                "--target-score-topic",
                LaunchConfiguration("target_score_topic"),
                "--goal-candidate-ids-topic",
                LaunchConfiguration("goal_candidate_ids_topic"),
            ],
            output="screen",
        ),
    ])
