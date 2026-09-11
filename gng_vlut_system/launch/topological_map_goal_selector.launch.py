from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def launch_setup(context, *args, **kwargs):
    script = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector_node.py"]).perform(context)
    cmd = ["python3", script]
    for name in ("topological_map_topic", "output_topic", "marker_topic", "candidate_topic",
                 "candidate_count", "orientation_weight", "goal_candidate_ids_topic",
                 "node_feature_topic", "manipulability_weight", "goal_update_hz"):
        value = LaunchConfiguration(name).perform(context)
        cmd.extend(["--" + name.replace("_", "-"), value])
    is_non_collision_only = LaunchConfiguration("non_collision_only").perform(context).lower() in ("1", "true", "yes", "on")
    cmd.append("--non-collision-only" if is_non_collision_only else "--no-non-collision-only")
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topological_map_topic", default_value="/ToPoDualArm/topological_map_static"),
        DeclareLaunchArgument("output_topic", default_value="/selected_topological_map"),
        DeclareLaunchArgument("marker_topic", default_value="/selected_topological_map_markers"),
        DeclareLaunchArgument("candidate_topic", default_value="/grasp_pose_cands"),
        DeclareLaunchArgument("goal_update_hz", default_value="5.0"),
        DeclareLaunchArgument("candidate_count", default_value="8"),
        DeclareLaunchArgument("non_collision_only", default_value="true"),
        DeclareLaunchArgument("orientation_weight", default_value="0.25"),
        DeclareLaunchArgument("goal_candidate_ids_topic", default_value="/selected_goal_candidate_ids"),
        DeclareLaunchArgument("node_feature_topic", default_value="/ToPoDualArm/topological_node_features"),
        DeclareLaunchArgument("manipulability_weight", default_value="0.25"),
        OpaqueFunction(function=launch_setup),
    ])
