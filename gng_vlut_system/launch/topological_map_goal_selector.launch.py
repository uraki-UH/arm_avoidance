from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    params = {}
    for value_type, names in (
        (str, ("topological_map_topic", "output_topic", "candidate_topic", "goal_candidate_ids_topic", "node_feature_topic")),
        (int, ("candidate_count",)),
        (float, ("orientation_weight", "manipulability_weight", "goal_update_hz")),
        (bool, ("non_collision_only",)),
    ):
        for name in names:
            params[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(package="gng_vlut_system", executable="topological_map_goal_selector_node",
                 parameters=[params], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topological_map_topic", default_value="/ToPoDualArm/Tmap_static"),
        DeclareLaunchArgument("output_topic", default_value="/selected_Tmap"),
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
