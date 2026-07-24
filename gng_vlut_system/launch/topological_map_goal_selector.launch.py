from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def _truthy(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    script = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector_node.py"]
    ).perform(context)

    topological_map_topic = LaunchConfiguration("topological_map_topic").perform(context)
    output_topic = LaunchConfiguration("output_topic").perform(context)
    marker_topic = LaunchConfiguration("marker_topic").perform(context)
    candidate_count = LaunchConfiguration("candidate_count").perform(context)
    non_collision_only = LaunchConfiguration("non_collision_only").perform(context)
    orientation_weight = LaunchConfiguration("orientation_weight").perform(context)
    target_pose_topic = LaunchConfiguration("target_pose_topic").perform(context)
    target_point_topic = LaunchConfiguration("target_point_topic").perform(context)
    target_pose_array_topic = LaunchConfiguration("target_pose_array_topic").perform(context)
    target_score_topic = LaunchConfiguration("target_score_topic").perform(context)
    goal_candidate_ids_topic = LaunchConfiguration("goal_candidate_ids_topic").perform(context)
    node_feature_topic = LaunchConfiguration("node_feature_topic").perform(context)
    manipulability_weight = LaunchConfiguration("manipulability_weight").perform(context)
    allow_untransformed_target = LaunchConfiguration("allow_untransformed_target").perform(context)

    cmd = [
        "python3",
        script,
        "--topological-map-topic",
        topological_map_topic,
        "--output-topic",
        output_topic,
        "--marker-topic",
        marker_topic,
        "--candidate-count",
        candidate_count,
        "--orientation-weight",
        orientation_weight,
        "--target-pose-array-topic",
        target_pose_array_topic,
        "--target-score-topic",
        target_score_topic,
        "--goal-candidate-ids-topic",
        goal_candidate_ids_topic,
    ]

    if node_feature_topic.strip():
        cmd.extend(["--node-feature-topic", node_feature_topic])
    if manipulability_weight.strip():
        cmd.extend(["--manipulability-weight", manipulability_weight])

    if _truthy(non_collision_only):
        cmd.append("--non-collision-only")
    else:
        cmd.append("--no-non-collision-only")

    if _truthy(allow_untransformed_target):
        cmd.append("--allow-untransformed-target")
    else:
        cmd.append("--no-allow-untransformed-target")

    if target_pose_topic.strip():
        cmd.extend(["--target-pose-topic", target_pose_topic])
    if target_point_topic.strip():
        cmd.extend(["--target-point-topic", target_point_topic])

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
        )
    ]


def generate_launch_description():
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
        DeclareLaunchArgument(
            "node_feature_topic",
            default_value="/ToPoDualArm/topological_node_features",
        ),
        DeclareLaunchArgument("manipulability_weight", default_value="0.25"),
        DeclareLaunchArgument("allow_untransformed_target", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
