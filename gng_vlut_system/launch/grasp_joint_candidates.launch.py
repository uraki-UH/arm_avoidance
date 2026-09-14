import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    params_file = LaunchConfiguration("params_file").perform(context)
    explicit_urdf_path = LaunchConfiguration("urdf_path").perform(context)
    node_feature_topic = f"/{robot_name}/topological_node_features"
    manipulability_weight = LaunchConfiguration("manipulability_weight").perform(context)

    def resolve_urdf_path(params_file_value: str, explicit_urdf_path_value: str) -> str:
        if explicit_urdf_path_value:
            return explicit_urdf_path_value
        if params_file_value and os.path.exists(params_file_value):
            try:
                with open(params_file_value, "r", encoding="utf-8") as f:
                    params_yaml = yaml.safe_load(f) or {}

                def find_urdf_path(value):
                    if not isinstance(value, dict):
                        return None
                    for key in ("urdf_path", "robot_urdf_path", "robot_description_file"):
                        candidate = value.get(key)
                        if candidate:
                            return str(candidate).strip()
                    for child in value.values():
                        found = find_urdf_path(child)
                        if found:
                            return found
                    return None

                found = find_urdf_path(params_yaml)
                if found:
                    return found
            except Exception:
                pass
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Pass urdf_path explicitly or add urdf_path to the params file."
        )

    resolved_urdf_path = resolve_urdf_path(params_file, explicit_urdf_path)

    selector_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector.launch.py"]
    )
    planner_params = {"urdf_path": resolved_urdf_path}
    for value_type, names in (
        (str, ("joint_topic", "topological_map_topic", "trajectory_topic",
               "candidate_trajectory_topic", "candidate_metrics_topic", "goal_candidate_ids_topic")),
        (float, ("publish_hz", "goal_rot_manip_weight", "goal_joint_limit_weight")),
        (bool, ("avoid_collisions", "avoid_danger", "allow_danger_goal",
                "strict_goal_collision_check", "allow_zero_initial_joint_state",
                "publish_candidate_robot_preview")),
    ):
        for name in names:
            planner_params[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(selector_launch),
            launch_arguments={
                "topological_map_topic": LaunchConfiguration("topological_map_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "candidate_count": LaunchConfiguration("candidate_count"),
                "non_collision_only": LaunchConfiguration("non_collision_only"),
                "orientation_weight": LaunchConfiguration("orientation_weight"),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "goal_update_hz": LaunchConfiguration("goal_update_hz"),
                "goal_candidate_ids_topic": LaunchConfiguration("goal_candidate_ids_topic"),
                "node_feature_topic": node_feature_topic,
                "manipulability_weight": manipulability_weight,
            }.items(),
        ),
        Node(
            package="gng_vlut_system",
            executable="topological_map_path_planner_node",
            name="topological_map_path_planner_node",
            namespace=robot_name,
            output="screen",
            parameters=[params_file, planner_params],
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm2.yaml")),
        DeclareLaunchArgument("topological_map_topic", default_value="/ToPoDualArm/Tmap_static"),
        DeclareLaunchArgument("output_topic", default_value="/selected_Tmap"),
        DeclareLaunchArgument("goal_candidate_ids_topic", default_value="/selected_goal_candidate_ids"),
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("candidate_count", default_value="8"),
        DeclareLaunchArgument("non_collision_only", default_value="true"),
        DeclareLaunchArgument("orientation_weight", default_value="0.0"),
        DeclareLaunchArgument("candidate_topic", default_value="/grasp_pose_cands"),
        DeclareLaunchArgument("goal_update_hz", default_value="5.0"),
        DeclareLaunchArgument("node_feature_topic", default_value="/ToPoDualArm/topological_node_features"),
        DeclareLaunchArgument("manipulability_weight", default_value="0.25"),
        DeclareLaunchArgument("joint_topic", default_value="/ToPoDualArm/joint_states"),
        DeclareLaunchArgument("trajectory_topic", default_value="/ToPoDualArm/plan_Tmap"),
        DeclareLaunchArgument("candidate_trajectory_topic", default_value="/ToPoDualArm/cand_Tmap"),
        DeclareLaunchArgument("candidate_metrics_topic", default_value="/ToPoDualArm/grasp_candidate_metrics"),
        DeclareLaunchArgument("publish_candidate_robot_preview", default_value="true"),
        DeclareLaunchArgument("publish_hz", default_value="20.0"),
        DeclareLaunchArgument("avoid_collisions", default_value="true",
                              description="衝突ノードを経路から除外するか"),
        DeclareLaunchArgument("avoid_danger", default_value="true",
                              description="危険ノード（障害物近傍）を経路から除外するか"),
        DeclareLaunchArgument("allow_danger_goal", default_value="true",
                              description="危険ノードでも最終ゴールとしては許可するか"),
        # --- ゴール姿勢スコアリング ---
        # ゴールノードの選択スコア = ホップ数 + 0.5*関節距離
        #                          + goal_rot_manip_weight * log(回転可操作性 条件数)
        #                          - goal_joint_limit_weight * 関節限界余裕 [0,1]
        # 回転可操作性 条件数: 1=理想(等方)、大きいほど手首特異点に近い
        # goal_rot_manip_weight=0 で無効化（旧動作に戻る）
        DeclareLaunchArgument("goal_rot_manip_weight", default_value="1.0",
                              description="ゴール姿勢の回転可操作性ペナルティ重み。大きいほど手首ねじれ姿勢が選ばれにくくなる"),
        DeclareLaunchArgument("goal_joint_limit_weight", default_value="0.5",
                              description="ゴール姿勢の関節限界余裕ボーナス重み。大きいほど関節限界から遠い姿勢が優先される"),
        DeclareLaunchArgument("strict_goal_collision_check", default_value="false"),
        DeclareLaunchArgument("allow_zero_initial_joint_state", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
