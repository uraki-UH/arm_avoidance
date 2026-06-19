import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def safe_float(value, default):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def safe_int(value, default):
    try:
        if value is None or value == "":
            return default
        return int(value)
    except Exception:
        return default


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")

    robot_name = LaunchConfiguration("robot_name").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    topological_map_topic = LaunchConfiguration("topological_map_topic").perform(context)
    trajectory_topic = LaunchConfiguration("trajectory_topic").perform(context)
    candidate_trajectory_topic = LaunchConfiguration("candidate_trajectory_topic").perform(context)
    joint_topic = LaunchConfiguration("joint_topic").perform(context)
    trial_mode = LaunchConfiguration("trial_mode").perform(context)
    trial_goal_interval_sec = LaunchConfiguration("trial_goal_interval_sec").perform(context)
    trial_safe_only = LaunchConfiguration("trial_safe_only").perform(context)
    trial_return_home = LaunchConfiguration("trial_return_home").perform(context)
    trial_auto_advance_goal = LaunchConfiguration("trial_auto_advance_goal").perform(context)
    trial_seed = LaunchConfiguration("trial_seed").perform(context)
    avoid_danger = LaunchConfiguration("avoid_danger").perform(context)
    replan_on_path_collision = LaunchConfiguration("replan_on_path_collision").perform(context)
    allow_zero_initial_joint_state = LaunchConfiguration("allow_zero_initial_joint_state").perform(context)
    goal_candidate_ids_topic = LaunchConfiguration("goal_candidate_ids_topic").perform(context)
    target_topic = LaunchConfiguration("target_topic").perform(context)
    robot_base_frame = LaunchConfiguration("robot_base_frame").perform(context)
    control_claim_priority = LaunchConfiguration("control_claim_priority").perform(context)
    control_claim_mode = LaunchConfiguration("control_claim_mode").perform(context)
    control_claim_enabled = LaunchConfiguration("control_claim_enabled").perform(context)
    right_arm_oscillation_enabled = LaunchConfiguration("right_arm_oscillation_enabled").perform(context)
    right_arm_wobble_amp_m = LaunchConfiguration("right_arm_wobble_amp_m").perform(context)
    right_arm_wobble_period_sec = LaunchConfiguration("right_arm_wobble_period_sec").perform(context)
    right_arm_wobble_z_scale = LaunchConfiguration("right_arm_wobble_z_scale").perform(context)
    right_arm_search_step_m = LaunchConfiguration("right_arm_search_step_m").perform(context)
    right_arm_search_max_m = LaunchConfiguration("right_arm_search_max_m").perform(context)
    right_arm_publish_hz = LaunchConfiguration("right_arm_publish_hz").perform(context)

    def resolve_urdf_path(params_file_value: str, explicit_urdf_path: str) -> str:
        if explicit_urdf_path:
            return explicit_urdf_path
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
        return "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro"

    urdf_path = resolve_urdf_path(params_file, urdf_path)

    if not robot_base_frame.strip():
        robot_base_frame = f"{robot_name}/base_link"

    node_params = {}
    if urdf_path:
        node_params["urdf_path"] = urdf_path
    if gng_model_path:
        node_params["gng_model_path"] = gng_model_path
    if topological_map_topic:
        node_params["topological_map_topic"] = topological_map_topic
    if trajectory_topic:
        node_params["trajectory_topic"] = trajectory_topic
    if candidate_trajectory_topic:
        node_params["candidate_trajectory_topic"] = candidate_trajectory_topic
    if joint_topic:
        node_params["joint_topic"] = joint_topic
    if trial_mode:
        node_params["trial_mode"] = trial_mode.lower() in ("1", "true", "yes", "on")
    if trial_goal_interval_sec:
        node_params["trial_goal_interval_sec"] = safe_float(trial_goal_interval_sec, 4.0)
    if trial_safe_only:
        node_params["trial_safe_only"] = trial_safe_only.lower() in ("1", "true", "yes", "on")
    if trial_return_home:
        node_params["trial_return_home"] = trial_return_home.lower() in ("1", "true", "yes", "on")
    if trial_auto_advance_goal:
        node_params["trial_auto_advance_goal"] = trial_auto_advance_goal.lower() in ("1", "true", "yes", "on")
    if trial_seed:
        node_params["trial_seed"] = safe_int(trial_seed, 0)
    if avoid_danger:
        node_params["avoid_danger"] = avoid_danger.lower() in ("1", "true", "yes", "on")
    if replan_on_path_collision:
        node_params["replan_on_path_collision"] = replan_on_path_collision.lower() in ("1", "true", "yes", "on")
    if allow_zero_initial_joint_state:
        node_params["allow_zero_initial_joint_state"] = (
            allow_zero_initial_joint_state.lower() in ("1", "true", "yes", "on")
        )
    if goal_candidate_ids_topic:
        node_params["goal_candidate_ids_topic"] = goal_candidate_ids_topic
    if robot_base_frame:
        node_params["robot_base_frame"] = robot_base_frame
    if target_topic:
        node_params["target_topic"] = target_topic
    if control_claim_priority:
        node_params["control_claim_priority"] = safe_int(control_claim_priority, 10)
    if control_claim_mode:
        node_params["control_claim_mode"] = safe_int(control_claim_mode, 1)
    if control_claim_enabled:
        node_params["control_claim_enabled"] = control_claim_enabled.lower() in ("1", "true", "yes", "on")

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(params_file)
    final_params_list.append(node_params)

    nodes = [
        Node(
            package="gng_vlut_system",
            executable="topological_map_avoidance_node",
            name="topological_map_avoidance_node",
            namespace=robot_name,
            output="screen",
            parameters=final_params_list,
        )
    ]

    if right_arm_oscillation_enabled and right_arm_oscillation_enabled.lower() in ("1", "true", "yes", "on"):
        nodes.append(
            Node(
                package="gng_vlut_system",
                executable="right_arm_oscillator_node",
                name="right_arm_oscillator_node",
                namespace=robot_name,
                output="screen",
                parameters=[{
                    "urdf_path": urdf_path or "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro",
                    "root_link": "base_link",
                    "leaf_link": "right_end_effector_link",
                    "publish_hz": safe_float(right_arm_publish_hz, 20.0),
                    "wobble_amp_m": safe_float(right_arm_wobble_amp_m, 0.03),
                    "wobble_period_sec": safe_float(right_arm_wobble_period_sec, 6.0),
                    "wobble_z_scale": safe_float(right_arm_wobble_z_scale, 0.6),
                    "search_step_m": safe_float(right_arm_search_step_m, 0.02),
                    "search_max_m": safe_float(right_arm_search_max_m, 0.60),
                }],
            )
        )
    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("urdf_path",
                              default_value="package://topoarm_description/urdf/topo_dual_arm.urdf.xacro"),
        DeclareLaunchArgument("gng_model_path",
                              default_value=""),
        DeclareLaunchArgument("topological_map_topic",
                              default_value="/ToPoDualArm/topological_map_static"),
        DeclareLaunchArgument("joint_topic", default_value="/ToPoDualArm/joint_states"),
        DeclareLaunchArgument("trajectory_topic",
                              default_value="/ToPoDualArm/planned_topological_map"),
        DeclareLaunchArgument("candidate_trajectory_topic",
                              default_value="/ToPoDualArm/candidate_topological_map"),
        DeclareLaunchArgument("trial_mode", default_value="false"),
        DeclareLaunchArgument("trial_goal_interval_sec", default_value="4.0"),
        DeclareLaunchArgument("trial_safe_only", default_value="true"),
        DeclareLaunchArgument("trial_return_home", default_value="false"),
        DeclareLaunchArgument("trial_auto_advance_goal", default_value="false"),
        DeclareLaunchArgument("trial_seed", default_value="0"),
        DeclareLaunchArgument("avoid_danger", default_value="true"),
        DeclareLaunchArgument("replan_on_path_collision", default_value="true"),
        DeclareLaunchArgument("allow_zero_initial_joint_state", default_value="true"),
        DeclareLaunchArgument("goal_candidate_ids_topic", default_value="/selected_goal_candidate_ids"),
        DeclareLaunchArgument("target_topic", default_value=""),
        DeclareLaunchArgument("robot_base_frame", default_value=""),
        DeclareLaunchArgument("control_claim_priority", default_value="10"),
        DeclareLaunchArgument("control_claim_mode", default_value="1"),
        DeclareLaunchArgument("control_claim_enabled", default_value="true"),
        DeclareLaunchArgument("right_arm_oscillation_enabled", default_value="false"),
        DeclareLaunchArgument("right_arm_wobble_amp_m", default_value="0.03"),
        DeclareLaunchArgument("right_arm_wobble_period_sec", default_value="6.0"),
        DeclareLaunchArgument("right_arm_wobble_z_scale", default_value="0.6"),
        DeclareLaunchArgument("right_arm_search_step_m", default_value="0.02"),
        DeclareLaunchArgument("right_arm_search_max_m", default_value="0.60"),
        DeclareLaunchArgument("right_arm_publish_hz", default_value="20.0"),
        OpaqueFunction(function=launch_setup),
    ])
