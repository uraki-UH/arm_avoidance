import os
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    enable_safety_monitor = LaunchConfiguration("enable_safety_monitor").perform(context).lower() in ("true", "1", "yes", "on")

    try:
        robot_desc_pkg = get_package_share_directory("topoarm_description")
        candidate = os.path.join(robot_desc_pkg, "urdf", "topo_dual_arm.urdf.xacro")
        if not os.path.exists(candidate):
            candidate = os.path.join(robot_desc_pkg, "urdf", "topoarm.urdf.xacro")
        robot_desc_default = candidate if os.path.exists(candidate) else ""
        resource_root = robot_desc_pkg
        mesh_root = os.path.join(robot_desc_pkg, "meshes")
    except PackageNotFoundError:
        robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
        resource_root = os.path.join(pkg_share, "urdf")
        mesh_root = os.path.join(resource_root, "meshes", "topoarm")

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    # 明示的に指定された項目のみを上書き対象とする
    viewer_params = {}
    if robot_desc_default:
        viewer_params["robot_description_file"] = robot_desc_default
    if resource_root:
        viewer_params["resource_root_dir"] = resource_root
    if mesh_root:
        viewer_params["mesh_root_dir"] = mesh_root
    
    viewer_params.update({
        "joint_state_topic": f"/{robot_name}/joint_states",
        "stream_topic": "/viewer/internal/stream/robot",
        "frame_id": LaunchConfiguration("base_frame"),
        "publish_hz": 20.0,
    })

    return [
        # 1. ロボットモデルの展開 (Digital Twin / TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
                launch_arguments={
                    "robot_name": LaunchConfiguration("robot_name"),
                    "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
                }.items()
            ),

        # 2. センサー位置の静的TF配信 (キャリブレーション用)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_calibration_publisher',
            arguments=[
                LaunchConfiguration("sensor_x"), LaunchConfiguration("sensor_y"), LaunchConfiguration("sensor_z"),
                LaunchConfiguration("sensor_yaw"), LaunchConfiguration("sensor_pitch"), LaunchConfiguration("sensor_roll"),
                LaunchConfiguration("base_frame"), LaunchConfiguration("sensor_frame_id")
            ]
        ),

        # 3. GNG 座標変換ノード (Sensor -> Base Frame 高速変換)
        Node(
            package="pointcloud_transformer_cpp",
            executable="gng_transformer_node_cpp",
            name="gng_transformer",
            parameters=[{
                "target_frame": LaunchConfiguration("base_frame"),
                "input_topic": "/gng_map",
                "output_topic": "/topological_map_transformed",
                "filter_radius": 3.0
            }]
        ),

        # 4. 安全監視・判定ノード (中核ロジック)
        *(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "gng_vlut_monitor.launch.py")),
                    launch_arguments={
                        "robot_name": LaunchConfiguration("robot_name"),
                        "id": LaunchConfiguration("id"),
                        "experiment_id": LaunchConfiguration("experiment_id"),
                        "dir": LaunchConfiguration("dir"),
                        "data_directory": LaunchConfiguration("data_directory"),
                        "params_file": LaunchConfiguration("params_file"),
                        "frame_id": LaunchConfiguration("base_frame"),
                        "safety_margin": LaunchConfiguration("safety_margin"),
                        "tag": LaunchConfiguration("tag"),
                        "mode": LaunchConfiguration("mode"),
                    }.items()
                )
            ]
            if enable_safety_monitor and os.path.exists(gng_path) and os.path.exists(vlut_path)
            else []
        ),

        # 5. 可視化ブリッジ (React Viewer用)
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge",
            parameters=[viewer_params]
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        # --- 基本設定 ---
        DeclareLaunchArgument("robot_name", default_value="topoarm"),
        DeclareLaunchArgument("id", default_value=""),
        DeclareLaunchArgument("experiment_id", default_value=""),
        DeclareLaunchArgument("dir", default_value="gng_results"),
        DeclareLaunchArgument("data_directory", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml")),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("tag", default_value="dynamic"),
        DeclareLaunchArgument("mode", default_value="dynamic"),
        
        # --- 安全設定 ---
        DeclareLaunchArgument("enable_safety_monitor", default_value="true"),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("safety_margin", default_value="0.05"),
        
        # --- センサーキャリブレーション（実測値をここに入力） ---
        DeclareLaunchArgument("sensor_x", default_value="0.5"),
        DeclareLaunchArgument("sensor_y", default_value="0.0"),
        DeclareLaunchArgument("sensor_z", default_value="1.0"),
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        DeclareLaunchArgument("sensor_yaw", default_value="0.0"),
        DeclareLaunchArgument("sensor_frame_id", default_value="camera_link"),
        OpaqueFunction(function=launch_setup)
    ])
