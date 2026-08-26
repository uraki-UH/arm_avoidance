import os
import math
import struct
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


def _root_parameters(params_yaml):
    for root_key in ("/**", "ros__parameters"):
        candidate = params_yaml.get(root_key, {})
        if isinstance(candidate, dict) and "ros__parameters" in candidate:
            candidate = candidate["ros__parameters"]
        if isinstance(candidate, dict):
            return candidate
    return {}


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    
    # --- YAMLから設定を自動抽出するロジック ---
    robot_name_default = LaunchConfiguration("robot_name").perform(context)
    robot_name = robot_name_default
    yaml_urdf_path = ""
    yaml_vlut_resolution = 0.0
    
    if params_file and os.path.exists(params_file):
        try:
            with open(params_file, 'r') as f:
                config = yaml.safe_load(f)
                # YAMLの全階層から robot_name を探す（/**: やノード別設定に対応）
                def find_robot_name(d):
                    if not isinstance(d, dict): return None
                    if 'ros__parameters' in d.get('ros__parameters', {}):
                        return d['ros__parameters']['robot_name']
                    if 'ros__parameters' in d:
                        return d['ros__parameters'].get('robot_name')
                    for v in d.values():
                        res = find_robot_name(v)
                        if res: return res
                    return None
                
                extracted_name = find_robot_name(config)
                if extracted_name:
                    robot_name = extracted_name

                root_params = _root_parameters(config or {})

                if isinstance(root_params, dict):
                    candidate_robot_description = root_params.get('urdf_path', '')
                    if candidate_robot_description:
                        yaml_urdf_path = str(candidate_robot_description).strip()
                    gng_params = root_params.get("gng", {})
                    if isinstance(gng_params, dict):
                        try:
                            yaml_vlut_resolution = float(gng_params.get("vlut_resolution", 0.0))
                        except Exception:
                            yaml_vlut_resolution = 0.0
        except Exception as e:
            print(f"Warning: Failed to parse YAML for robot_name: {e}")

    # コマンドラインで明示的に指定された場合はそちらを優先
    # LaunchConfigurationは一度performしないと値が取れないため注意
    user_robot_name = LaunchConfiguration("robot_name").perform(context)
    # デフォルト値以外が指定されていれば、それを優先
    if user_robot_name and user_robot_name != robot_name_default:
        robot_name = user_robot_name

    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    if not urdf_path and yaml_urdf_path:
        urdf_path = yaml_urdf_path
    if not urdf_path:
        raise FileNotFoundError(
            "No robot description path was provided. "
            "Set urdf_path in the params file or pass urdf_path explicitly."
        )
    robot_urdf = resolve_package_uri(urdf_path)
    if not os.path.exists(robot_urdf):
        raise FileNotFoundError(f"Robot description file does not exist: {robot_urdf}")

    vlut_file = ""
    if params_file and os.path.exists(params_file):
        root_params = _root_parameters(config if 'config' in locals() else {})
        gng_params = root_params.get("gng", {}) if isinstance(root_params, dict) else {}
        if isinstance(gng_params, dict):
            data_directory = str(gng_params.get("data_directory", "")).strip()
            experiment_id = str(gng_params.get("experiment_id", "")).strip()
            vlut_filename = str(gng_params.get("vlut_filename", "vlut.bin")).strip()
            if data_directory and experiment_id and vlut_filename:
                vlut_file = os.path.join(data_directory, experiment_id, vlut_filename)

    self_recognition_resolution = read_vlut_voxel_size(vlut_file)
    if self_recognition_resolution is None or self_recognition_resolution <= 0.0:
        self_recognition_resolution = yaml_vlut_resolution

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    node_params = {}
    if robot_urdf:
        node_params["urdf_path"] = robot_urdf
    if self_recognition_resolution and self_recognition_resolution > 0.0:
        # VLUT のセルサイズへ自己認識側を追従
        node_params["self_recognition.resolution"] = self_recognition_resolution
    
    # コマンドライン引数を辞書に追加（明示的に指定された場合のみ、適切な型でYAMLを上書きするようにする）
    def add_if_not_empty(name, config_name, type_func=None):
        val = LaunchConfiguration(config_name).perform(context)
        if val:
            try:
                node_params[name] = type_func(val) if type_func else val
            except ValueError:
                node_params[name] = val

    def add_override_param(name, config_name, type_func=None, aliases=None):
        val = LaunchConfiguration(config_name).perform(context)
        if not val:
            return
        try:
            parsed = type_func(val) if type_func else val
        except ValueError:
            parsed = val
        node_params[name] = parsed
        if aliases:
            for alias in aliases:
                node_params[alias] = parsed

    add_if_not_empty("marker_frame_id", "marker_frame_id")
    add_if_not_empty("joint_topic", "joint_topic")
    add_if_not_empty("robot.voxel_size", "voxel_size", float)
    add_if_not_empty("robot.inflation", "inflation", float)
    add_if_not_empty("update_hz", "update_hz", float)
    add_if_not_empty("publish_self_mask", "publish_self_mask") # boolは文字列でも解釈されることが多いが
    add_if_not_empty("publish_link_voxels", "publish_link_voxels")
    add_if_not_empty("publish_link_aabb", "publish_link_aabb")
    add_if_not_empty("display_mode", "display_mode", int)
    add_override_param("root_link", "root_link", aliases=["self_recognition.root_link"])
    add_override_param("leaf_link", "leaf_link", aliases=["self_recognition.leaf_link"])
    add_if_not_empty("target_frame_id", "target_frame_id")
    add_override_param("mask_topic", "mask_topic", aliases=["self_recognition.mask_topic", "self_output_topic", "self_recognition.self_output_topic"])

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(params_file)
    final_params_list.append(node_params)

    # 名前空間の決定 (既に上でYAML等から決定済み)

    return [
        # ロボットモデルの展開 (名前空間付き)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": robot_name,
                "urdf_path": robot_urdf,
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
            }.items()
        ),
        # 自己認識可視化ノード (名前空間付き)
        Node(
            package="gng_vlut_system",
            executable="self_recognition_viz_node",
            name="self_recognition_viz_node",
            namespace=robot_name, # 名前空間を適用
            output="screen",
            parameters=final_params_list,
            # トピックのリマップ（名前空間外の/joint_statesを参照したい場合などに対応）
            remappings=[
                ("/joint_states", f"/{robot_name}/joint_states"),
                ("tf", "/tf"),
                ("tf_static", "/tf_static"),
            ]
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ToPoDualArm"),
        DeclareLaunchArgument("urdf_path", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "ToPoDualArm.yaml")),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false"),
        DeclareLaunchArgument("marker_frame_id", default_value="world"),
        DeclareLaunchArgument("joint_topic", default_value="joint_states"),
        DeclareLaunchArgument("voxel_size", default_value="0.02"),
        DeclareLaunchArgument("inflation", default_value="0.0"),
        DeclareLaunchArgument("update_hz", default_value="10.0"),
        DeclareLaunchArgument("publish_self_mask", default_value="true"),
        DeclareLaunchArgument("publish_link_voxels", default_value="true"),
        DeclareLaunchArgument("publish_link_aabb", default_value="true"),
        DeclareLaunchArgument("display_mode", default_value="link_local"),
        DeclareLaunchArgument("root_link", default_value=""),
        DeclareLaunchArgument("leaf_link", default_value=""),
        DeclareLaunchArgument("target_frame_id", default_value="world"),
        DeclareLaunchArgument("mask_topic", default_value="/self_recognition/voxel_mask"),
        OpaqueFunction(function=launch_setup),
    ])
