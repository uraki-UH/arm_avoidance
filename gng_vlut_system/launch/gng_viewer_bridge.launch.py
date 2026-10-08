import os
import json
import math
import struct
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


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


def read_vlut_voxel_size(vlut_file: str):
    if not vlut_file:
        return None
    try:
        with open(vlut_file, "rb") as stream:
            header = stream.read(12)
    except OSError:
        return None
    if len(header) != 12:
        return None
    file_id, version, voxel_size = struct.unpack("<IIf", header)
    if file_id != int.from_bytes(b"VLUT", byteorder="big") or version < 1:
        return None
    if not math.isfinite(voxel_size) or voxel_size <= 0.0:
        return None
    return voxel_size



def read_vlut_bounds(vlut_file: str):
    # VLUT全占有範囲。TCP位置だけで切り落とさない環境ROIの基準
    with open(vlut_file, "rb") as stream:
        header = stream.read(36)
    if len(header) != 36:
        raise ValueError("VLUTの占有範囲ヘッダが不足しています")
    file_id, version, _, *bounds = struct.unpack("<II7f", header)
    if file_id != int.from_bytes(b"VLUT", byteorder="big") or version < 2:
        raise ValueError("占有範囲を持つVLUTが必要です")
    if not all(math.isfinite(value) for value in bounds) or any(bounds[idx] > bounds[idx+3] for idx in range(3)):
        raise ValueError("VLUTの占有範囲が不正です")
    return bounds[:3], bounds[3:]


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


def safe_bool(value, default):
    if isinstance(value, bool):
        return value
    if value is None or value == "":
        return default
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    return default

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    joint_control_backend = LaunchConfiguration("joint_control_backend").perform(context)
    yaml_joint_control_backend = "viewer"
    params_file = resolve_package_uri(LaunchConfiguration("params_file").perform(context))
    # config内のファイル名指定と、明示的な設定パスの検証
    if params_file and not os.path.isfile(params_file):
        if not os.path.dirname(params_file):
            params_file = os.path.join(pkg_share, "config", params_file)
        if not os.path.isfile(params_file):
            raise FileNotFoundError(f"設定ファイルが見つかりません: {params_file}")
    data_dir = LaunchConfiguration("dir").perform(context)
    exp_id = LaunchConfiguration("id").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    vlut_path = LaunchConfiguration("vlut_path").perform(context)
    has_result_override = bool(
        (data_dir and data_dir != "gng_results") or exp_id or gng_model_path or vlut_path
    )
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
    enable_joint_state_publisher_arg = LaunchConfiguration(
        "enable_joint_state_publisher"
    ).perform(context)
    direct_joint_tracking = LaunchConfiguration("direct_joint_tracking").perform(context)
    enable_self_recognition_viz_arg = LaunchConfiguration(
        "enable_self_recognition_viz"
    ).perform(context)

    yaml_data_dir = data_dir
    yaml_exp_id = exp_id
    yaml_robot_name = ""
    yaml_urdf_path = ""
    gng_model_filename = "gng.bin"
    vlut_filename = "vlut.bin"
    yaml_resource_root_dir = ""
    yaml_mesh_root_dir = ""
    yaml_gripper_volume_enabled = False
    yaml_gripper_volume_config_file = ""
    yaml_gripper_volume_cache_directory = ""
    yaml_gripper_volume_cache_mode = "use"
    yaml_enable_self_recognition_viz = False
    yaml_enable_environment_self_filter = False
    yaml_enable_joint_state_publisher = True
    yaml_enable_dynamixel_current_pose = False
    yaml_enable_environment_voxelization = False
    yaml_environment_voxelization = {}
    yaml_voxel_idx = {}
    yaml_dynamixel_mapping_file = ""
    yaml_enable_realsense_mount_tf = False
    yaml_realsense_mount_config = "package://gng_vlut_system/config/realsense_mount.yaml"
    yaml_vlut_resolution = 0.0
    yaml_enable_independent_arms = False
    yaml_gng_sampling = {}
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
                yaml_joint_control_backend = root_ros_params.get('joint_control_backend', 'viewer')
                # 学習の出力先とは独立したViewer用の保存済みモデル。明示引数を優先
                viewer_params = root_ros_params.get('viewer', {})
                if isinstance(viewer_params, dict) and not has_result_override:
                    gng_model_path = str(viewer_params.get('gng_model_path') or '')
                    vlut_path = str(viewer_params.get('vlut_path') or '')
                yaml_enable_environment_voxelization = safe_bool(
                    root_ros_params.get('enable_environment_voxelization'), False
                )
                yaml_environment_voxelization = root_ros_params.get('environment_voxelization', {})
                yaml_voxel_idx = root_ros_params.get('voxel_idx_shift', {})
                yaml_enable_dynamixel_current_pose = safe_bool(
                    root_ros_params.get('enable_dynamixel_current_pose'), False
                )
                yaml_dynamixel_mapping_file = root_ros_params.get('dynamixel_mapping_file', '')
                yaml_enable_realsense_mount_tf = safe_bool(
                    root_ros_params.get('enable_realsense_mount_tf'), False
                )
                yaml_realsense_mount_config = root_ros_params.get(
                    'realsense_mount_config', yaml_realsense_mount_config
                )
                gng_ns = root_ros_params.get('gng', {}) if isinstance(root_ros_params.get('gng', {}), dict) else {}
                yaml_enable_independent_arms = safe_bool(gng_ns.get('enable_independent_arms'), False)
                yaml_gng_sampling = root_ros_params.get('gng_params', {})
                yaml_data_dir = gng_ns.get('data_directory', yaml_data_dir)
                yaml_exp_id = gng_ns.get('experiment_id', yaml_exp_id)
                gng_model_filename = gng_ns.get('gng_model_filename', gng_model_filename)
                vlut_filename = gng_ns.get('vlut_filename', vlut_filename)
                yaml_vlut_resolution = safe_float(
                    gng_ns.get("vlut_resolution"), yaml_vlut_resolution
                )
                yaml_resource_root_dir = root_ros_params.get('resource_root_dir', yaml_resource_root_dir)
                yaml_mesh_root_dir = root_ros_params.get('mesh_root_dir', yaml_mesh_root_dir)
                candidate_robot_description = root_ros_params.get('urdf_path', '')
                if candidate_robot_description is not None:
                    yaml_urdf_path = str(candidate_robot_description).strip()
                gripper_volume_ns = root_ros_params.get('gripper_volume_graph', {})
                if isinstance(gripper_volume_ns, dict):
                    yaml_gripper_volume_enabled = safe_bool(
                        gripper_volume_ns.get('enabled'), yaml_gripper_volume_enabled
                    )
                    candidate_config_file = gripper_volume_ns.get('definitions_file', '')
                    if candidate_config_file is not None:
                        yaml_gripper_volume_config_file = str(candidate_config_file).strip()
                    candidate_cache_directory = gripper_volume_ns.get('cache_directory', '')
                    if candidate_cache_directory is not None:
                        yaml_gripper_volume_cache_directory = str(
                            candidate_cache_directory
                        ).strip()
                    candidate_cache_mode = gripper_volume_ns.get('cache_mode', '')
                    if candidate_cache_mode is not None and str(candidate_cache_mode).strip():
                        yaml_gripper_volume_cache_mode = str(candidate_cache_mode).strip()

                self_recognition_ns = root_ros_params.get('self_recognition', {})
                if isinstance(self_recognition_ns, dict):
                    yaml_enable_self_recognition_viz = safe_bool(
                        self_recognition_ns.get('enable_self_recognition_viz'),
                        yaml_enable_self_recognition_viz,
                    )
                    yaml_enable_environment_self_filter = safe_bool(
                        self_recognition_ns.get('enable_environment_self_filter'),
                        yaml_enable_environment_self_filter,
                    )
                if 'enable_self_recognition_viz' in root_ros_params:
                    yaml_enable_self_recognition_viz = safe_bool(
                        root_ros_params.get('enable_self_recognition_viz'),
                        yaml_enable_self_recognition_viz,
                    )
                if 'enable_joint_state_publisher' in root_ros_params:
                    yaml_enable_joint_state_publisher = safe_bool(
                        root_ros_params.get('enable_joint_state_publisher'),
                        yaml_enable_joint_state_publisher,
                    )

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

    # 関節出力先の優先順位: 明示launch引数、機体YAML、従来のviewer。
    joint_control_backend = joint_control_backend or yaml_joint_control_backend
    if joint_control_backend not in ("viewer", "dynamixel", "external"):
        raise ValueError("joint_control_backendはviewer・dynamixel・externalのいずれかが必要です")

    enable_independent_arms = safe_bool(
        LaunchConfiguration("enable_independent_arms").perform(context), yaml_enable_independent_arms
    )

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

    enable_joint_state_publisher = (
        safe_bool(enable_joint_state_publisher_arg, yaml_enable_joint_state_publisher)
        if enable_joint_state_publisher_arg else yaml_enable_joint_state_publisher
    )
    enable_dynamixel_current_pose = joint_control_backend == 'viewer' and safe_bool(
        LaunchConfiguration('enable_dynamixel_current_pose').perform(context),
        yaml_enable_dynamixel_current_pose,
    )
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

    def resolve_result_path(path: str, default_filename: str) -> str:
        if path:
            if os.path.isabs(path):
                # 指定モデルの欠落時に、別の保存済みモデルへ切り替わることの防止
                return path
            if path.startswith("gng_results/") or "/" in path:
                return os.path.join(pkg_share, path)
        filename = path or default_filename
        if data_dir and os.path.isabs(data_dir):
            return os.path.join(data_dir, exp_id, filename)
        return os.path.join(pkg_share, data_dir, exp_id, filename)

    enable_self_recognition_viz = safe_bool(
        enable_self_recognition_viz_arg, yaml_enable_self_recognition_viz
    )
    vlut_file = resolve_result_path(vlut_path, vlut_filename)
    self_recognition_resolution = read_vlut_voxel_size(vlut_file)
    if self_recognition_resolution is None or self_recognition_resolution <= 0.0:
        self_recognition_resolution = yaml_vlut_resolution

    resource_root = yaml_resource_root_dir
    mesh_root = yaml_mesh_root_dir

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    # YAMLの値を上書き（消去）しないよう、明示的に指定された（空でない）パラメータのみを抽出
    common_params = {"enable_viewer_status": True}
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
    # 外部実測入力の直接購読。仮想姿勢との混在防止
    state_topic = LaunchConfiguration("state_topic").perform(context)
    if state_topic and joint_control_backend != "external":
        raise ValueError("state_topicの直接指定にはjoint_control_backend:=externalが必要です")
    viewer_joint_state_topic = state_topic or f"/{robot_name}/viewer_joint_states"
    if state_topic:
        enable_joint_state_publisher = False
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

    # 学習前のURDF表示と、学習済みGNG・VLUT配信の分離
    gng_file = resolve_result_path(gng_model_path, gng_model_filename)
    missing_result_files = [path for path in (gng_file, vlut_file) if not os.path.isfile(path)]
    independent_profiles = []
    independent_bounds = None
    if enable_independent_arms:
        if gng_model_path or vlut_path:
            raise ValueError("独立学習モデルはdir・idで指定してください。単体モデルの指定にはenable_independent_arms:=falseが必要です")
        models_dir = os.path.join(data_dir if os.path.isabs(data_dir) else os.path.join(pkg_share, data_dir), exp_id)
        manifest_path = os.path.join(models_dir, "independent_arms.json")
        missing_result_files = []
        if not os.path.isfile(manifest_path):
            missing_result_files.append(manifest_path)
        else:
            with open(manifest_path, encoding="utf-8") as stream:
                manifest = json.load(stream)
            if manifest.get("version") != 1 or manifest.get("mode") != "independent_arms":
                raise ValueError("独立学習モデルのmanifest形式が不正です")
            profiles = manifest.get("profiles", [])
            if len(profiles) != 2 or {item["name"] for item in profiles} != {"left_arm", "right_arm"}:
                raise ValueError("left_armとright_armの独立学習モデルが必要です")
            for profile in profiles:
                metadata_path = os.path.join(models_dir, profile["metadata"])
                with open(metadata_path, encoding="utf-8") as stream:
                    metadata = json.load(stream)
                if metadata.get("mode") != "independent_arm" or metadata.get("profile") != profile["name"]:
                    raise ValueError("独立学習モデルのprofileが一致しません")
                metadata_dir = os.path.dirname(metadata_path)
                entry = dict(metadata, metadata_path=metadata_path,
                             gng_path=os.path.join(metadata_dir, metadata["gng_file"]),
                             vlut_path=os.path.join(metadata_dir, metadata["vlut_file"]))
                independent_profiles.append(entry)
                missing_result_files.extend(path for path in (entry["gng_path"], entry["vlut_path"]) if not os.path.isfile(path))
            sizes = [read_vlut_voxel_size(item["vlut_path"]) for item in independent_profiles]
            if not missing_result_files:
                if sizes[0] is None or sizes[0] != sizes[1]:
                    raise ValueError("左右VLUTのボクセル幅が一致しません")
                self_recognition_resolution = sizes[0]
                bounds = [read_vlut_bounds(item["vlut_path"]) for item in independent_profiles]
                independent_bounds = {
                    "min": [min(item[0][idx] for item in bounds) for idx in range(3)],
                    "max": [max(item[1][idx] for item in bounds) for idx in range(3)],
                }
    has_learning_data = not missing_result_files

    actions = [
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        # 0. ロボット本体の起動（TF / robot_state_publisher / 初回姿勢配信）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            condition=IfCondition(LaunchConfiguration("enable_robot_state_publisher")),
            launch_arguments={
                "robot_name": robot_name,
                "enable_joint_state_publisher": "false",
                "publish_initial_joint_state": (
                    "true" if enable_joint_state_publisher else "false"
                ),
                "urdf_path": urdf_path,
                "resource_root_dir": resource_root,
                "mesh_root_dir": mesh_root,
                "joint_state_topic": viewer_joint_state_topic,
            }.items()
        ),

        # 関節単位の指令統合と選択した出力先への接続
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "joint_control.launch.py")),
            condition=IfCondition(str(joint_control_backend != "external" and not enable_dynamixel_current_pose).lower()),
            launch_arguments={
                "params_file": params_file,
                "urdf_path": urdf_path,
                "robot_name": robot_name,
                "backend": joint_control_backend,
                "state_topic": (viewer_joint_state_topic if joint_control_backend == "viewer"
                                else f"/{robot_name}/joint_states"),
                "mapping_file": LaunchConfiguration("dynamixel_mapping_file"),
                "enable_dynamixel_input": LaunchConfiguration("enable_dynamixel_input"),
                "dynamixel_input_topic": LaunchConfiguration("dynamixel_input_topic"),
                "viewer_topic": viewer_joint_state_topic,
                "enable_direct_tracking": direct_joint_tracking,
            }.items()
        ),

        # 学習済みデータが揃った場合のGNG配信
        Node(
            package="gng_vlut_system",
            executable="topofuzzy_bridge_node",
            condition=IfCondition(str(has_learning_data and not enable_independent_arms).lower()),
            name="topofuzzy_bridge_node",
            namespace=robot_name,
            parameters=[
                params_file,
                {
                    "gng_model_path": gng_file,
                    "enable_viewer_status": True,
                    "vlut_path": vlut_file,
                    "gng.data_directory": data_dir,
                    "gng.experiment_id": exp_id,
                    "publish_hz": publish_hz,
                    "topic_name": topic_name,
                    "node_feature_topic": node_feature_topic,
                    "stamped_node_state_topic": "gng_node_states_stamped",
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
            output_format="{line}",
            additional_env={"RCUTILS_CONSOLE_OUTPUT_FORMAT": "[{severity}] [{name}] {message}"},
        )
    ]

    if enable_independent_arms and has_learning_data:
        for profile in independent_profiles:
            name = profile["profile"]
            actions.append(Node(
                package="gng_vlut_system", executable="topofuzzy_bridge_node",
                name="topofuzzy_" + name, namespace=robot_name,
                parameters=[params_file, {
                    "gng_model_path": profile["gng_path"], "vlut_path": profile["vlut_path"],
                    "topic_name": "Tmap_" + name,
                    "node_feature_topic": name + "/topological_node_features",
                    "node_state_topic": name + "/gng_node_states",
                    "stamped_node_state_topic": name + "/gng_node_states_stamped",
                    "frame_id": gng_frame_id or "base_link",
                    "source_frame_id": gng_source_frame_id or "base_link",
                    "edge_mode": safe_int(edge_mode, 1), "publish_hz": publish_hz,
                    "enable_viewer_status": True, "urdf_path": urdf_path,
                    "occupied_voxels_topic": "occupied_voxels", "danger_voxels_topic": "danger_voxels",
                    "grasp.state_topic": name + "/grasp_state",
                    "grasp.applied_state_topic": name + "/grasp_state_applied",
                    "enable_dynamic_manipulability": False,
                    "visualization_gng.enabled": False,
                }]))
        profiles_by_name = {item["profile"]: item for item in independent_profiles}
        actions.append(Node(
            package="gng_vlut_system", executable="independent_arm_pair_node",
            name="independent_arm_pair_node", namespace=robot_name,
            parameters=[params_file, {
                "left_model_path": profiles_by_name["left_arm"]["metadata_path"],
                "right_model_path": profiles_by_name["right_arm"]["metadata_path"],
                "urdf_path": urdf_path, "resource_root_dir": resource_root, "mesh_root_dir": mesh_root,
            }]))

    if not has_learning_data:
        actions.insert(0, LogInfo(msg=(
            "GNG・VLUTデータ不足のためGNG配信を省略し、ロボット本体を表示します。"
            "学習後にこのlaunchを再起動してください。不足ファイル: "
            + ", ".join(missing_result_files)
        )))

    if enable_joint_state_publisher:
        actions.insert(
            0,
            LogInfo(
                msg=(
                    "初回姿勢publisherを起動します。"
                    "継続する joint_state_publisher は起動しません。"
                )
            ),
        )

    if enable_self_recognition_viz:
        # 起動時の自己認識ボクセル生成ノード
        self_recognition_params = []
        if params_file and os.path.exists(params_file):
            self_recognition_params.append(params_file)
        node_params = {
            "urdf_path": urdf_path,
            "joint_topic": viewer_joint_state_topic,
        }
        if resource_root:
            node_params["resource_root_dir"] = resource_root
        if mesh_root:
            node_params["mesh_root_dir"] = mesh_root
        if self_recognition_resolution and self_recognition_resolution > 0.0:
            # VLUT のセルサイズへ自己認識側を追従
            node_params["self_recognition.resolution"] = self_recognition_resolution
        self_recognition_params.append(node_params)
        actions.append(
            Node(
                package="gng_vlut_system",
                executable="self_recognition_viz_node",
                name="self_recognition_viz_node",
                namespace=robot_name,
                output="screen",
                parameters=self_recognition_params,
            )
        )

    if yaml_enable_environment_self_filter:
        filter_params = []
        if params_file and os.path.exists(params_file):
            filter_params.append(params_file)
        actions.append(
            Node(
                package="gng_vlut_system",
                executable="self_voxel_filter_node",
                name="self_voxel_filter_node",
                namespace=robot_name,
                output="screen",
                parameters=filter_params + [{"enable_viewer_status": True}],
            )
        )

    enable_gripper_volume_arg = LaunchConfiguration(
        "enable_gripper_volume_graph"
    ).perform(context)
    gripper_volume_enabled = safe_bool(
        enable_gripper_volume_arg, yaml_gripper_volume_enabled
    )
    gripper_volume_config_file = LaunchConfiguration(
        "gripper_volume_config_file"
    ).perform(context).strip() or yaml_gripper_volume_config_file
    gripper_volume_grippers = LaunchConfiguration(
        "gripper_volume_grippers"
    ).perform(context).strip()
    gripper_volume_cache_directory = LaunchConfiguration(
        "gripper_volume_cache_directory"
    ).perform(context).strip() or yaml_gripper_volume_cache_directory
    gripper_volume_cache_mode = LaunchConfiguration(
        "gripper_volume_cache_mode"
    ).perform(context).strip() or yaml_gripper_volume_cache_mode

    if gripper_volume_enabled:
        if gripper_volume_config_file:
            gripper_volume_config_file = resolve_package_uri(gripper_volume_config_file)
            if not os.path.exists(gripper_volume_config_file):
                raise FileNotFoundError(
                    "Gripper volume definition file does not exist: "
                    f"{gripper_volume_config_file}"
                )
        elif not gripper_volume_grippers:
            raise ValueError(
                "Gripper volume graph is enabled but no definitions were provided. "
                "Set gripper_volume_graph.definitions_file in params_file, "
                "gripper_volume_config_file, or gripper_volume_grippers."
            )

        grasping_share = get_package_share_directory("grasping_system")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        grasping_share, "launch", "gripper_volume_graph.launch.py"
                    )
                ),
                launch_arguments={
                    "grippers_file": gripper_volume_config_file,
                    "grippers": gripper_volume_grippers,
                    "tf_prefix": robot_name,
                    "cache_directory": gripper_volume_cache_directory,
                    "cache_mode": gripper_volume_cache_mode,
                }.items(),
            )
        )

    enable_environment_voxelization = safe_bool(
        LaunchConfiguration('enable_environment_voxelization').perform(context),
        yaml_enable_environment_voxelization,
    )
    if enable_environment_voxelization:
        if not enable_self_recognition_viz or not yaml_enable_environment_self_filter:
            raise ValueError('Viewerの環境ROIには自己認識と自己除去の有効化が必要です')
        environment = yaml_environment_voxelization
        base_frame = str(environment.get('base_frame', 'base_link')).lstrip('/')
        target_frame = base_frame if base_frame.startswith(robot_name + '/') else robot_name + '/' + base_frame
        # world座標の仮定なし。ロボット基準へのTF変換後の直接ROI生成
        roi_params = {
            'enable_viewer_status': True,
            'input_topic': environment.get('input_topic', '/camera/camera/depth/color/points'),
            'output_topic': self_recognition_ns.get('raw_environment_voxel_topic', 'roi_voxels'),
            'source_frame_id': environment.get('source_frame_id', ''),
            'target_frame_id': target_frame, 'world_frame_id': target_frame,
            'enable_world_index': False, 'enable_roi_query': False,
            'enable_world_bucket_publish': False, 'allow_unconnected_source_as_world': False,
            'voxel_size': self_recognition_resolution,
            'enable_reachability_filter': True,
            'reachability_map_topic': '' if enable_independent_arms else environment.get('reachability_map_topic', 'Tmap_static'),
        }
        for key, default in (('x_shift', 42), ('y_shift', 21), ('z_shift', 0), ('offset', 1000000)):
            roi_params[key] = int(yaml_voxel_idx.get(key, default))
        for idx, axis in enumerate('xyz'):
            for direction, default in (('min', -0.1 if axis == 'x' else -1.0), ('max', 0.5 if axis == 'x' else 1.0)):
                key = direction + '_reachability_' + axis
                fallback = gng_ns.get(direction + '_' + axis,
                    yaml_gng_sampling.get(direction + '_' + axis, default))
                if enable_independent_arms:
                    fallback = (independent_bounds[direction][idx] if independent_bounds else
                                yaml_gng_sampling.get(direction + '_' + axis, default))
                roi_params[key] = float(environment.get(key, fallback))
            key = 'reachability_margin_' + axis
            roi_params[key] = float(environment.get(key, 0.2))
        actions.append(Node(package='gng_vlut_system', executable='world_index_to_voxel_node',
                            name='viewer_environment_voxelization', namespace=robot_name,
                            parameters=[roi_params], output='screen'))
        danger_source = str(environment.get('danger_source', 'environment_inflation'))
        if danger_source not in ('environment_inflation', 'vlut_distance'):
            raise ValueError('danger_sourceにはenvironment_inflationまたはvlut_distanceが必要です')
        # 自己除去後のROIをGNG状態更新の占有・危険入力へ接続
        actions.append(Node(package='gng_vlut_system', executable='voxel_to_vlut_node',
                            name='viewer_voxel_to_vlut', namespace=robot_name, output='screen',
                            parameters=[{
                                'input_topic': self_recognition_ns.get('filtered_environment_voxel_topic', 'self_filter_roi_voxels'),
                                'occupied_voxels_topic': 'occupied_voxels', 'danger_voxels_topic': 'danger_voxels',
                                'target_frame_id': target_frame, 'output_voxel_size': self_recognition_resolution,
                                'danger_inflation': (float(environment.get('danger_inflation', 0.05))
                                                     if danger_source == 'environment_inflation' else 0.0),
                                'publish_hz': float(environment.get('publish_hz', 30.0)),
                            }]))
    if enable_dynamixel_current_pose:
        mapping_file = LaunchConfiguration('dynamixel_mapping_file').perform(context).strip()
        mapping_file = resolve_package_uri(mapping_file or yaml_dynamixel_mapping_file)
        if not mapping_file or not os.path.isfile(mapping_file):
            raise FileNotFoundError('実測表示用のDynamixel関節対応設定が必要です: ' + mapping_file)
        # 実測表示専用の既存launchを利用。Viewerの再帰起動・USBの二重オープンの回避
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'dynamixel_current_pose.launch.py')),
            launch_arguments={
                'params_file': params_file, 'mapping_file': mapping_file,
                'input_topic': LaunchConfiguration('dynamixel_input_topic').perform(context),
                'enable_reader': 'false', 'enable_viewer': 'false',
            }.items(),
        ))
    enable_realsense_mount_tf = safe_bool(
        LaunchConfiguration('enable_realsense_mount_tf').perform(context),
        yaml_enable_realsense_mount_tf,
    )
    if enable_realsense_mount_tf:
        mount_config = LaunchConfiguration('realsense_mount_config').perform(context).strip()
        mount_config = resolve_package_uri(mount_config or yaml_realsense_mount_config)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'realsense_mount_tf.launch.py')),
            launch_arguments={'mount_config': mount_config}.items(),
        ))
    return actions

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("enable_independent_arms", default_value=""),
        DeclareLaunchArgument("state_topic", default_value="",
                              description="外部実測JointStateの購読先。未指定時は従来のViewer入力"),
        DeclareLaunchArgument("enable_robot_state_publisher", default_value="true",
                              description="ロボットTF配信の起動。外部配信済みの場合はfalse"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("dir", default_value="gng_results"),
        DeclareLaunchArgument("id", default_value=""),
        DeclareLaunchArgument("gng_model_path", default_value=""),
        DeclareLaunchArgument("vlut_path", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "topo_dual_arm_max_long.yaml")),
        DeclareLaunchArgument(
            "enable_joint_state_publisher",
            default_value="",
            description="初回姿勢配信の上書き。未指定時はparams_fileを使用。",
        ),
        DeclareLaunchArgument("joint_control_backend", default_value="",
                              description="関節出力先。未指定時は機体YAML、設定なしはviewer"),
        DeclareLaunchArgument("dynamixel_mapping_file", default_value=""),
        DeclareLaunchArgument('enable_dynamixel_current_pose', default_value='',
                              description='Dynamixel実測姿勢の表示。未指定時は機体YAMLを使用'),
        DeclareLaunchArgument('enable_environment_voxelization', default_value='',
                              description='環境点群のROI生成。未指定時は機体YAMLを使用'),
        DeclareLaunchArgument("enable_dynamixel_input", default_value=""),
        DeclareLaunchArgument("dynamixel_input_topic", default_value="/dynamixel/state/present"),
        DeclareLaunchArgument('enable_realsense_mount_tf', default_value='',
                              description='取付TFの同時起動。未指定時は機体YAMLを使用'),
        DeclareLaunchArgument('realsense_mount_config', default_value='',
                              description='取付TF設定YAMLの上書き'),
        DeclareLaunchArgument(
            "direct_joint_tracking",
            default_value="true",
            description="Reflect mux target joint values directly without velocity interpolation",
        ),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("robot_base_frame", default_value=""),
        DeclareLaunchArgument("arm_leaf_link_names", default_value=""),
        DeclareLaunchArgument("gng_frame_id", default_value=""),
        DeclareLaunchArgument("gng_source_frame_id", default_value=""),
        DeclareLaunchArgument("publish_hz", default_value=""),
        DeclareLaunchArgument("topic_name", default_value="Tmap_static"),
        DeclareLaunchArgument("node_feature_topic", default_value="topological_node_features"),
        DeclareLaunchArgument("grasp_state_topic", default_value="grasp_state"),
        DeclareLaunchArgument("grasp_applied_state_topic", default_value="grasp_state_applied"),
        DeclareLaunchArgument(
            "enable_self_recognition_viz",
            default_value="",
            description="自己認識ボクセル生成ノードの起動切替。空の場合は params_file を優先",
        ),
        DeclareLaunchArgument(
            "enable_gripper_volume_graph",
            default_value="",
            description="グリッパ体積の確認用トピック発行ノードの起動切替。未指定時はparams_fileを使用。",
        ),
        DeclareLaunchArgument(
            "gripper_volume_config_file",
            default_value="",
            description="Optional gripper definition YAML; overrides params_file",
        ),
        DeclareLaunchArgument(
            "gripper_volume_grippers",
            default_value="",
            description="Optional inline YAML gripper list when no definition file is used",
        ),
        DeclareLaunchArgument(
            "gripper_volume_cache_directory",
            default_value="",
            description="Optional gripper-volume cache directory; empty uses params_file",
        ),
        DeclareLaunchArgument(
            "gripper_volume_cache_mode",
            default_value="",
            description="Gripper-volume cache policy; empty uses params_file",
        ),
        DeclareLaunchArgument("edge_mode", default_value=""),
        OpaqueFunction(function=launch_setup)
    ])
