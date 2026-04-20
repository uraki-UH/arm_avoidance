from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _to_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _node_from_context(context, *args, **kwargs):
    _ = (args, kwargs)
    params_file = LaunchConfiguration("params_file").perform(context)

    optional_overrides = {}

    def set_if_given(param_key: str, arg_key: str, caster):
        raw = LaunchConfiguration(arg_key).perform(context)
        if raw == "":
            return
        optional_overrides[param_key] = caster(raw)

    set_if_given("input_topic", "input_topic", str)
    set_if_given("output_topic", "output_topic", str)
    set_if_given("rewrite_label", "rewrite_label", _to_bool)
    set_if_given("rewrite_label_inferred", "rewrite_label_inferred", _to_bool)
    set_if_given(
        "preserve_structural_labels", "preserve_structural_labels", _to_bool
    )
    set_if_given("score.min_nodes", "score_min_nodes", int)
    set_if_given(
        "score.set_reliability_from_score", "set_reliability_from_score", _to_bool
    )
    set_if_given("score.human.threshold", "human_threshold", float)
    set_if_given("score.human.bias", "human_bias", float)
    set_if_given("score.human.speed_weight", "human_speed_weight", float)
    set_if_given(
        "score.human.reliability_weight", "human_reliability_weight", float
    )
    set_if_given("score.human.match_weight", "human_match_weight", float)
    set_if_given(
        "score.human.node_count_weight", "human_node_count_weight", float
    )
    set_if_given("score.human.inv_volume_weight", "human_inv_volume_weight", float)
    set_if_given("score.car.threshold", "car_threshold", float)
    set_if_given("score.car.threshold_for_small", "car_threshold_for_small", float)
    set_if_given("score.car.threshold_for_big", "car_threshold_for_big", float)
    set_if_given("score.car.bias", "car_bias", float)
    set_if_given("score.car.speed_weight", "car_speed_weight", float)
    set_if_given("score.car.reliability_weight", "car_reliability_weight", float)
    set_if_given("score.car.match_weight", "car_match_weight", float)
    set_if_given("score.car.node_count_weight", "car_node_count_weight", float)
    set_if_given("score.car.volume_weight", "car_volume_weight", float)

    node_parameters = [params_file]
    if optional_overrides:
        node_parameters.append(optional_overrides)

    return [
        Node(
            package="fuzzy_classifier",
            executable="cluster_relabel_node",
            name="fuzzy_cluster_relabel",
            output="screen",
            parameters=node_parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("fuzzy_classifier"), "config", "relabel.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML parameter file for cluster_relabel_node",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="",
                description="Override input_topic only when non-empty",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="",
                description="Override output_topic only when non-empty",
            ),
            DeclareLaunchArgument("rewrite_label", default_value=""),
            DeclareLaunchArgument("rewrite_label_inferred", default_value=""),
            DeclareLaunchArgument("preserve_structural_labels", default_value=""),
            DeclareLaunchArgument("score_min_nodes", default_value=""),
            DeclareLaunchArgument("set_reliability_from_score", default_value=""),
            DeclareLaunchArgument("human_threshold", default_value=""),
            DeclareLaunchArgument("human_bias", default_value=""),
            DeclareLaunchArgument("human_speed_weight", default_value=""),
            DeclareLaunchArgument("human_reliability_weight", default_value=""),
            DeclareLaunchArgument("human_match_weight", default_value=""),
            DeclareLaunchArgument("human_node_count_weight", default_value=""),
            DeclareLaunchArgument("human_inv_volume_weight", default_value=""),
            DeclareLaunchArgument("car_threshold", default_value=""),
            DeclareLaunchArgument("car_threshold_for_small", default_value=""),
            DeclareLaunchArgument("car_threshold_for_big", default_value=""),
            DeclareLaunchArgument("car_bias", default_value=""),
            DeclareLaunchArgument("car_speed_weight", default_value=""),
            DeclareLaunchArgument("car_reliability_weight", default_value=""),
            DeclareLaunchArgument("car_match_weight", default_value=""),
            DeclareLaunchArgument("car_node_count_weight", default_value=""),
            DeclareLaunchArgument("car_volume_weight", default_value=""),
            OpaqueFunction(function=_node_from_context),
        ]
    )
