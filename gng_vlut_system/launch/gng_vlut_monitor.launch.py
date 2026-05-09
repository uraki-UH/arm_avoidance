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

    # YAMLファイルからデフォルト値を読み込む試行
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
    
    # 上書き用パラメータの準備
    override_params = {
        "gng_model_path": gng_path,
        "vlut_path": vlut_path,
        "safety_margin": LaunchConfiguration("safety_margin"),
        "base_frame": LaunchConfiguration("frame_id"),
        "experiment_id": experiment_id,
        "tag": LaunchConfiguration("tag"),
        "mode": LaunchConfiguration("mode"),
        "gng_model_filename": gng_model_filename,
        "vlut_filename": vlut_filename,
    }

    return [
        # 安全監視ノード (Safety Monitor)
        # GNGとVLUTを用いて、ロボットの自己干渉や周囲との接触危険度を判定します。
        Node(
            package="gng_vlut_system",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            parameters=[
                params_file,
                override_params
            ],
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm", description="ロボットの名前"),
        DeclareLaunchArgument("id", default_value="", description="実験ID (experiment_id)"),
        DeclareLaunchArgument("experiment_id", default_value="", description="(旧) 実験ID"),
        DeclareLaunchArgument("dir", default_value="gng_results", description="GNGデータのディレクトリ"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml"), description="設定YAMLファイル"),
        DeclareLaunchArgument("data_directory", default_value="", description="(旧) データディレクトリ"),
        DeclareLaunchArgument("frame_id", default_value="base_link", description="基準座標系"),
        DeclareLaunchArgument("safety_margin", default_value="0.05", description="安全マージン [m]"),
        DeclareLaunchArgument("tag", default_value="dynamic", description="GNGレイヤータグ"),
        DeclareLaunchArgument("mode", default_value="dynamic", description="動作モード"),
        OpaqueFunction(function=launch_setup)
    ])
