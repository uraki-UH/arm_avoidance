import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_package_uri(raw_path: str) -> str:
    if not raw_path.startswith("package://"):
        return raw_path

    pkg_and_path = raw_path[len("package://"):]
    pkg_name, _, rel_path = pkg_and_path.partition("/")
    if not pkg_name or not rel_path:
        return raw_path

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except Exception:
        return raw_path
    return os.path.join(pkg_share, rel_path)


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
    params_file = LaunchConfiguration("params_file").perform(context)
    data_dir = LaunchConfiguration("dir").perform(context)
    exp_id = LaunchConfiguration("id").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    vlut_path = LaunchConfiguration("vlut_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    robot_base_frame = LaunchConfiguration("robot_base_frame").perform(context)
    arm_leaf_link_names = LaunchConfiguration("arm_leaf_link_names").perform(context)
    gng_frame_id = LaunchConfiguration("gng_frame_id").perform(context)
    gng_source_frame_id = LaunchConfiguration("gng_source_frame_id").perform(context)
    publish_hz_str = LaunchConfiguration("publish_hz").perform(context)
    publish_hz = safe_float(publish_hz_str, 30.0)
    topic_name = LaunchConfiguration("topic_name").perform(context)
    node_feature_topic = LaunchConfiguration("node_feature_topic").perform(context)
    edge_mode = LaunchConfiguration("edge_mode").perform(context)
    enable_joint_state_publisher = LaunchConfiguration("enable_joint_state_publisher").perform(context)

    yaml_data_dir = data_dir
    yaml_exp_id = exp_id
    yaml_robot_name = ""
    yaml_urdf_path = ""
    gng_model_filename = "gng.bin"
    vlut_filename = "vlut.bin"
    yaml_resource_root_dir = ""
    yaml_mesh_root_dir = ""
    if params_file and os.path.exists(params_file):
        try:
            with open(params_file, "r", encoding="utf-8") as f:
                params_yaml = yaml.safe_load(f) or {}
            
            # robot_name を全階層から探す
            def find_robot_name(d):
                if not isinstance(d, dict): return None
                if 'robot_name' in d.get('ros__parameters', {}):
                    return d['ros__parameters']['robot_name']
                if 'ros__parameters' in d:
                    return d['ros__parameters'].get('robot_name')
                for v in d.values():
                    res = find_robot_name(v)
                    if res: return res
                return None
            
            extracted_name = find_robot_name(params_yaml)
            if extracted_name:
                yaml_robot_name = extracted_name

            root_ros_params = {}
            for root_key in ('/**', 'ros__parameters'):
                candidate = params_yaml.get(root_key, {})
                if isinstance(candidate, dict) and 'ros__parameters' in candidate:
                    candidate = candidate.get('ros__parameters', {})
                if isinstance(candidate, dict):
                    root_ros_params = candidate
                    break

            if isinstance(root_ros_params, dict):
                gng_ns = root_ros_params.get('gng', {}) if isinstance(root_ros_params.get('gng', {}), dict) else {}
                yaml_data_dir = gng_ns.get('data_directory', yaml_data_dir)
                yaml_exp_id = gng_ns.get('experiment_id', yaml_exp_id)
                gng_model_filename = gng_ns.get('gng_model_filename', gng_model_filename)
                vlut_filename = gng_ns.get('vlut_filename', vlut_filename)
                yaml_resource_root_dir = root_ros_params.get('resource_root_dir', yaml_resource_root_dir)
                yaml_mesh_root_dir = root_ros_params.get('mesh_root_dir', yaml_mesh_root_dir)
                candidate_robot_description = root_ros_params.get('urdf_path', '')
                if candidate_robot_description is not None:
                    yaml_urdf_path = str(candidate_robot_description).strip()

            for node_key in ("offline_urdf_trainer", "gng_safety", "viewer_ws_gateway"):
                ros_params = params_yaml.get(node_key, {}).get("ros__parameters", {})
                if ros_params:
                    gng_ns = ros_params.get("gng", {}) if isinstance(ros_params.get("gng", {}), dict) else {}
                    yaml_data_dir = gng_ns.get("data_directory", yaml_data_dir)
                    yaml_exp_id = gng_ns.get("experiment_id", yaml_exp_id)
                    gng_model_filename = gng_ns.get("gng_model_filename", gng_model_filename)
                    vlut_filename = gng_ns.get("vlut_filename", vlut_filename)
                    break

        except Exception:
            pass

    # 名前空間の決定（YAML優先、コマンドライン指定があればそちら）
    robot_name_default = LaunchConfiguration("robot_name").perform(context)
    if robot_name_default and robot_name_default != "ToPoDualArm":
        robot_name = robot_name_default
    else:
        robot_name = yaml_robot_name

    if not data_dir or data_dir == "gng_results":
        data_dir = yaml_data_dir
    if not exp_id or exp_id == "topoarm":
        exp_id = yaml_exp_id

    # YAML に古い絶対パスが入っていても、この workspace で実在しないなら
    # package share 配下の相対ディレクトリとして扱い直す。
    if data_dir and os.path.isabs(data_dir) and not os.path.exists(data_dir):
        data_dir = os.path.basename(data_dir) or data_dir
    if not urdf_path and yaml_urdf_path:
        urdf_path = yaml_urdf_path
    if not urdf_path:
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly."
        )
    urdf_path = resolve_package_uri(urdf_path)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(
            f"Robot description file does not exist: {urdf_path}. "
            "Pass urdf_path explicitly or install the matching description package."
        )

    resource_root = yaml_resource_root_dir
    mesh_root = yaml_mesh_root_dir

    def resolve_result_path(path: str, default_filename: str) -> str:
        if path:
            if os.path.isabs(path):
                if os.path.exists(path):
                    return path
                # 既存の絶対パスを優先しつつ、存在しない場合は basename を相対候補として扱う。
                path = os.path.basename(path)
            if path.startswith("gng_results/") or "/" in path:
                return os.path.join(pkg_share, path)
        filename = path or default_filename
        return os.path.join(pkg_share, data_dir, exp_id, filename)

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    # YAMLの値を上書き（消去）しないよう、明示的に指定された（空でない）パラメータのみを抽出
    common_params = {}
    if robot_name:
        common_params["robot_name"] = robot_name
    if urdf_path:
        common_params["urdf_path"] = urdf_path
    if arm_leaf_link_names:
        common_params["robot.arm_leaf_link_names"] = arm_leaf_link_names
    
    # 座標系(frame_id)などは明示的に指定された場合のみ上書き
    def add_if_not_empty(name, config_name):
        val = LaunchConfiguration(config_name).perform(context)
        if val:
            common_params[name] = val

    add_if_not_empty("frame_id", "robot_base_frame")
    add_if_not_empty("publish_hz", "publish_hz")
    
    if resource_root:
        common_params["resource_root_dir"] = resource_root
    if mesh_root:
        common_params["mesh_root_dir"] = mesh_root
    # Viewer 用の関節状態は制御系の /joint_states と分離する。
    # ここが public /joint_states と混ざると、初期姿勢や GUI 由来の部分集合が
    # そのまま制御系に流れ込んで不安定になる。
    viewer_joint_state_topic = f"/{robot_name}/viewer_joint_states"
    common_params["joint_state_topic"] = viewer_joint_state_topic

    # 内部ストリーム用のトピック名
    stream_topic = "/viewer/internal/stream/robot"

    viewer_bridge_params = []
    if params_file and os.path.exists(params_file):
        viewer_bridge_params.append(params_file)
    
    # 上書き用辞書を追加（ROS 2では後から追加したパラメータがYAMLを上書きする）
    if common_params:
        viewer_bridge_params.append(common_params)
    
    # 内部ストリーム用のトピック名を常にセット（これはノード内部で必須のパラメータ）
    has_stream_topic = any("stream_topic" in p if isinstance(p, dict) else False for p in viewer_bridge_params)
    if not has_stream_topic:
        if not viewer_bridge_params or not isinstance(viewer_bridge_params[-1], dict):
             viewer_bridge_params.append({})
        viewer_bridge_params[-1]["stream_topic"] = stream_topic
    else:
        for p in viewer_bridge_params:
            if isinstance(p, dict) and "stream_topic" in p:
                p["stream_topic"] = stream_topic

    return [
        # 0. ロボット本体の召喚 (TF / robot_state_publisher / optional joint_state_publisher)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
                "enable_joint_state_publisher": enable_joint_state_publisher,
                "publish_initial_joint_state": "true",
                "urdf_path": urdf_path,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "joint_state_topic": viewer_joint_state_topic,
            }.items()
        ),

        # 0.5 source claim / command を統合する mux
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "joint_state_mux.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
            }.items()
        ),

        # 0.6 仮想的な関節追従ドライバ（mux の target_joint_states -> joint_states）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "virtual_joint_state_driver.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
                "target_topic": f"/{robot_name}/target_joint_states",
                "state_topic": viewer_joint_state_topic,
                "output_topic": viewer_joint_state_topic,
            }.items()
        ),

        # 1. GNGブリッジ (Topofuzzy)
        Node(
            package="gng_vlut_system",
            executable="topofuzzy_bridge_node",
            name="topofuzzy_bridge_node",
            namespace=robot_name,
            parameters=[
                params_file,
                {
                    "gng_model_path": resolve_result_path(gng_model_path, gng_model_filename),
                    "vlut_path": resolve_result_path(vlut_path, vlut_filename),
                    "gng.data_directory": data_dir,
                    "gng.experiment_id": exp_id,
                    "publish_hz": publish_hz,
                    "topic_name": topic_name,
                    "node_feature_topic": node_feature_topic,
                    "edge_mode": safe_int(edge_mode, 1),
                    # robot_name namespace 配下の相対トピックを購読する。
                    "occupied_voxels_topic": "occupied_voxels",
                    "danger_voxels_topic": "danger_voxels",
                    "grasp.state_topic": LaunchConfiguration("grasp_state_topic"),
                    "grasp.applied_state_topic": LaunchConfiguration("grasp_applied_state_topic"),
                    "urdf_path": urdf_path,
                },
                # 座標系などは指定がある場合のみ上書き
                {k: v for k, v in {
                    "frame_id": gng_frame_id,
                    "source_frame_id": gng_source_frame_id,
                    "robot.arm_leaf_link_names": arm_leaf_link_names,
                }.items() if v}
            ]
        ),

        # 2. ロボットビューアブリッジ
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge_node",
            namespace=robot_name,
            parameters=viewer_bridge_params,
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("dir", default_value="gng_results"),
        DeclareLaunchArgument("id", default_value=""),
        DeclareLaunchArgument("gng_model_path", default_value=""),
        DeclareLaunchArgument("vlut_path", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "topoarm_dual.yaml")),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("robot_base_frame", default_value=""),
        DeclareLaunchArgument("arm_leaf_link_names", default_value=""),
        DeclareLaunchArgument("gng_frame_id", default_value=""),
        DeclareLaunchArgument("gng_source_frame_id", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value=""),
        DeclareLaunchArgument("topic_name", default_value="topological_map_static"),
        DeclareLaunchArgument("node_feature_topic", default_value="topological_node_features"),
        DeclareLaunchArgument("grasp_state_topic", default_value="grasp_state"),
        DeclareLaunchArgument("grasp_applied_state_topic", default_value="grasp_state_applied"),
        DeclareLaunchArgument("edge_mode", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])
