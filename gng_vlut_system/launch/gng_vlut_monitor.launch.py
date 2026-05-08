import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    experiment_id = LaunchConfiguration("id").perform(context)
    if not experiment_id:
        experiment_id = LaunchConfiguration("experiment_id").perform(context)
    if not experiment_id:
        experiment_id = robot_name

    data_dir = LaunchConfiguration("dir").perform(context)
    if not data_dir:
        data_dir = LaunchConfiguration("data_directory").perform(context)
    yaml_data_dir = data_dir
    yaml_exp_id = experiment_id
    gng_model_filename = "gng.bin"
    vlut_filename = "vlut.bin"
    if params_file and os.path.exists(params_file):
        try:
            with open(params_file, "r", encoding="utf-8") as f:
                params_yaml = yaml.safe_load(f) or {}
            for node_key in ("offline_urdf_trainer", "gng_safety"):
                ros_params = params_yaml.get(node_key, {}).get("ros__parameters", {})
                if ros_params:
                    yaml_data_dir = ros_params.get("data_directory", yaml_data_dir)
                    yaml_exp_id = ros_params.get("experiment_id", yaml_exp_id)
                    gng_model_filename = ros_params.get("gng_model_filename", gng_model_filename)
                    vlut_filename = ros_params.get("vlut_filename", vlut_filename)
                    break
        except Exception:
            pass

    if not data_dir:
        data_dir = yaml_data_dir
    if not experiment_id:
        experiment_id = yaml_exp_id
    if not os.path.isabs(data_dir):
        data_dir = os.path.join(pkg_share, data_dir)

    gng_path = os.path.join(data_dir, experiment_id, gng_model_filename)
    vlut_path = os.path.join(data_dir, experiment_id, vlut_filename)
    gng_tag = LaunchConfiguration("tag").perform(context)
    gng_mode = LaunchConfiguration("mode").perform(context)

    return [
        Node(
            package="gng_vlut_system",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "gng_model_path": gng_path,
                    "vlut_path": vlut_path,
                    "safety_margin": LaunchConfiguration("safety_margin"),
                    "base_frame": LaunchConfiguration("frame_id"),
                    "experiment_id": experiment_id,
                    "tag": gng_tag,
                    "mode": gng_mode,
                    "gng_model_filename": gng_model_filename,
                    "vlut_filename": vlut_filename,
                },
            ],
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("id", default_value=""),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("dir", default_value="gng_results"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml")),
        DeclareLaunchArgument("data_directory", default_value=""),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        DeclareLaunchArgument("tag", default_value="dynamic"),
        DeclareLaunchArgument("mode", default_value="dynamic"),
        OpaqueFunction(function=launch_setup)
    ])
