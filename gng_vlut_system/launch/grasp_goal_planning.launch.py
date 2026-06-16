from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    selector_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector.launch.py"]
    )
    avoidance_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_avoidance.launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("topological_map_topic", default_value="/ToPoDualArm/topological_map_static"),
        DeclareLaunchArgument("output_topic", default_value="/selected_topological_map"),
        DeclareLaunchArgument("marker_topic", default_value="/selected_topological_map_markers"),
        DeclareLaunchArgument("goal_candidate_ids_topic", default_value="/selected_goal_candidate_ids"),
        DeclareLaunchArgument("candidate_count", default_value="8"),
        DeclareLaunchArgument("safe_only", default_value="true"),
        DeclareLaunchArgument("orientation_weight", default_value="0.25"),
        DeclareLaunchArgument("target_pose_topic", default_value=""),
        DeclareLaunchArgument("target_point_topic", default_value=""),
        DeclareLaunchArgument("target_pose_array_topic", default_value="/grasp_pose_candidates"),
        DeclareLaunchArgument("target_score_topic", default_value="/grasp_pose_scores"),
        DeclareLaunchArgument("joint_topic", default_value="/ToPoDualArm/joint_states"),
        DeclareLaunchArgument("trajectory_topic", default_value="/ToPoDualArm/planned_topological_map"),
        DeclareLaunchArgument("candidate_trajectory_topic", default_value="/ToPoDualArm/candidate_topological_map"),
        DeclareLaunchArgument("publish_hz", default_value="20.0"),
        DeclareLaunchArgument("avoid_collisions", default_value="true"),
        DeclareLaunchArgument("avoid_danger", default_value="true"),
        DeclareLaunchArgument("strict_goal_collision_check", default_value="false"),
        DeclareLaunchArgument("replan_on_path_collision", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(selector_launch),
            launch_arguments={
                "topological_map_topic": LaunchConfiguration("topological_map_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "candidate_count": LaunchConfiguration("candidate_count"),
                "safe_only": LaunchConfiguration("safe_only"),
                "orientation_weight": LaunchConfiguration("orientation_weight"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "target_point_topic": LaunchConfiguration("target_point_topic"),
                "target_pose_array_topic": LaunchConfiguration("target_pose_array_topic"),
                "target_score_topic": LaunchConfiguration("target_score_topic"),
                "goal_candidate_ids_topic": LaunchConfiguration("goal_candidate_ids_topic"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(avoidance_launch),
            launch_arguments={
                "joint_topic": LaunchConfiguration("joint_topic"),
                "topological_map_topic": LaunchConfiguration("topological_map_topic"),
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "candidate_trajectory_topic": LaunchConfiguration("candidate_trajectory_topic"),
                "publish_hz": LaunchConfiguration("publish_hz"),
                "avoid_collisions": LaunchConfiguration("avoid_collisions"),
                "avoid_danger": LaunchConfiguration("avoid_danger"),
                "strict_goal_collision_check": LaunchConfiguration("strict_goal_collision_check"),
                "replan_on_path_collision": LaunchConfiguration("replan_on_path_collision"),
                "goal_candidate_ids_topic": LaunchConfiguration("goal_candidate_ids_topic"),
            }.items(),
        ),
    ])
