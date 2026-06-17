import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    explicit_urdf_path = LaunchConfiguration("urdf_path").perform(context)

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
        return "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro"

    resolved_urdf_path = resolve_urdf_path(params_file, explicit_urdf_path)

    selector_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_goal_selector.launch.py"]
    )
    avoidance_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "topological_map_avoidance.launch.py"]
    )
    robot_spawn_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "robot_spawn.launch.py"]
    )
    virtual_joint_state_driver_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "virtual_joint_state_driver.launch.py"]
    )
    joint_state_relay_launch = PathJoinSubstitution(
        [ThisLaunchFileDir(), "virtual_joint_state_driver.launch.py"]
    )
    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="grasp_goal_planning_world_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "ToPoDualArm/base_link"],
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(selector_launch),
            launch_arguments={
                "topological_map_topic": LaunchConfiguration("topological_map_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "candidate_count": LaunchConfiguration("candidate_count"),
                "non_collision_only": LaunchConfiguration("non_collision_only"),
                "orientation_weight": LaunchConfiguration("orientation_weight"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "target_point_topic": LaunchConfiguration("target_point_topic"),
                "target_pose_array_topic": LaunchConfiguration("target_pose_array_topic"),
                "target_score_topic": LaunchConfiguration("target_score_topic"),
                "goal_candidate_ids_topic": LaunchConfiguration("goal_candidate_ids_topic"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "urdf_path": resolved_urdf_path,
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
                "joint_state_topic": LaunchConfiguration("joint_topic"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(avoidance_launch),
            launch_arguments={
                "params_file": LaunchConfiguration("params_file"),
                "joint_topic": LaunchConfiguration("planner_joint_topic"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(virtual_joint_state_driver_launch),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "target_topic": "target_joint_states",
                "state_topic": "/ToPoDualArm/joint_states",
                "output_topic": LaunchConfiguration("planner_joint_topic"),
                "publish_hz": LaunchConfiguration("virtual_joint_state_publish_hz"),
                "max_joint_velocity": LaunchConfiguration("virtual_joint_state_max_joint_velocity"),
                "position_tolerance": LaunchConfiguration("virtual_joint_state_position_tolerance"),
                "use_wraparound": LaunchConfiguration("virtual_joint_state_use_wraparound"),
                "hold_when_no_target": LaunchConfiguration("virtual_joint_state_hold_when_no_target"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint_state_relay_launch),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "node_name": "grasp_joint_state_relay_node",
                "target_topic": LaunchConfiguration("planner_joint_topic"),
                "state_topic": LaunchConfiguration("planner_joint_topic"),
                "output_topic": "/ToPoDualArm/joint_states",
                "publish_hz": LaunchConfiguration("public_joint_state_publish_hz"),
                "max_joint_velocity": LaunchConfiguration("virtual_joint_state_max_joint_velocity"),
                "position_tolerance": LaunchConfiguration("virtual_joint_state_position_tolerance"),
                "use_wraparound": LaunchConfiguration("virtual_joint_state_use_wraparound"),
                "hold_when_no_target": LaunchConfiguration("virtual_joint_state_hold_when_no_target"),
            }.items(),
        ),
        static_world_tf,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm2.yaml")),
        DeclareLaunchArgument("topological_map_topic", default_value="/ToPoDualArm/topological_map_static"),
        DeclareLaunchArgument("output_topic", default_value="/selected_topological_map"),
        DeclareLaunchArgument("marker_topic", default_value="/selected_topological_map_markers"),
        DeclareLaunchArgument("goal_candidate_ids_topic", default_value="/selected_goal_candidate_ids"),
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="true"),
        DeclareLaunchArgument("planner_joint_topic", default_value="/ToPoDualArm/grasp_joint_states"),
        DeclareLaunchArgument("candidate_count", default_value="8"),
        DeclareLaunchArgument("non_collision_only", default_value="true"),
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
        DeclareLaunchArgument("avoid_danger", default_value="false"),
        DeclareLaunchArgument("strict_goal_collision_check", default_value="false"),
        DeclareLaunchArgument("replan_on_path_collision", default_value="false"),
        DeclareLaunchArgument("virtual_joint_state_publish_hz", default_value="50.0"),
        DeclareLaunchArgument("virtual_joint_state_max_joint_velocity", default_value="0.6"),
        DeclareLaunchArgument("virtual_joint_state_position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("virtual_joint_state_use_wraparound", default_value="true"),
        DeclareLaunchArgument("virtual_joint_state_hold_when_no_target", default_value="true"),
        DeclareLaunchArgument("public_joint_state_publish_hz", default_value="50.0"),
        DeclareLaunchArgument("publish_world_tf", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
