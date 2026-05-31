import os

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
    robot_urdf = LaunchConfiguration("robot_urdf_path").perform(context)
    topological_map_topic = LaunchConfiguration("topological_map_topic").perform(context)
    trajectory_topic = LaunchConfiguration("trajectory_topic").perform(context)
    trial_mode = LaunchConfiguration("trial_mode").perform(context)
    trial_goal_interval_sec = LaunchConfiguration("trial_goal_interval_sec").perform(context)
    trial_safe_only = LaunchConfiguration("trial_safe_only").perform(context)
    trial_seed = LaunchConfiguration("trial_seed").perform(context)

    node_params = {}
    if robot_urdf:
        node_params["robot_urdf_path"] = robot_urdf
    if gng_model_path:
        node_params["gng_model_path"] = gng_model_path
    if topological_map_topic:
        node_params["topological_map_topic"] = topological_map_topic
    if trajectory_topic:
        node_params["trajectory_topic"] = trajectory_topic
    if trial_mode:
        node_params["trial_mode"] = trial_mode.lower() in ("1", "true", "yes", "on")
    if trial_goal_interval_sec:
        node_params["trial_goal_interval_sec"] = safe_float(trial_goal_interval_sec, 4.0)
    if trial_safe_only:
        node_params["trial_safe_only"] = trial_safe_only.lower() in ("1", "true", "yes", "on")
    if trial_seed:
        node_params["trial_seed"] = safe_int(trial_seed, 0)

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(params_file)
    final_params_list.append(node_params)

    return [
        Node(
            package="gng_vlut_system",
            executable="topological_map_avoidance_node",
            name="topological_map_avoidance_node",
            namespace=robot_name,
            output="screen",
            parameters=final_params_list,
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("params_file",
                              default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("robot_urdf_path",
                              default_value="package://topoarm_description/urdf/topo_dual_arm.urdf.xacro"),
        DeclareLaunchArgument("gng_model_path",
                              default_value=""),
        DeclareLaunchArgument("topological_map_topic",
                              default_value="/ToPoDualArm/topological_map_static"),
        DeclareLaunchArgument("trajectory_topic",
                              default_value="/ToPoDualArm/planned_topological_map"),
        DeclareLaunchArgument("trial_mode", default_value="false"),
        DeclareLaunchArgument("trial_goal_interval_sec", default_value="4.0"),
        DeclareLaunchArgument("trial_safe_only", default_value="true"),
        DeclareLaunchArgument("trial_seed", default_value="0"),
        OpaqueFunction(function=launch_setup),
    ])
