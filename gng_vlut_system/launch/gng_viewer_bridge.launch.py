import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    data_dir = LaunchConfiguration("dir").perform(context)
    exp_id = LaunchConfiguration("id").perform(context)
    gng_model_path = LaunchConfiguration("gng_model_path").perform(context)
    vlut_path = LaunchConfiguration("vlut_path").perform(context)
    robot_description_file = LaunchConfiguration("robot_description_file").perform(context)
    robot_base_frame = LaunchConfiguration("robot_base_frame").perform(context)
    arm_leaf_link_names = LaunchConfiguration("arm_leaf_link_names").perform(context)
    gng_frame_id = LaunchConfiguration("gng_frame_id").perform(context)
    gng_source_frame_id = LaunchConfiguration("gng_source_frame_id").perform(context)
    publish_hz = float(LaunchConfiguration("publish_hz").perform(context))
    topic_name = LaunchConfiguration("topic_name").perform(context)

    yaml_data_dir = data_dir
    yaml_exp_id = exp_id
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
    if not exp_id:
        exp_id = yaml_exp_id
    
    # Auto-detect robot description package
    try:
        robot_desc_pkg = get_package_share_directory(f"{robot_name}_description")
        potential_urdf = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}.urdf.xacro")
        if not os.path.exists(potential_urdf):
            potential_urdf = os.path.join(robot_desc_pkg, "urdf", f"{robot_name}_pro_normal.urdf.xacro")

        robot_desc_default = potential_urdf if os.path.exists(potential_urdf) else ""
        resource_root = robot_desc_pkg
        mesh_root = os.path.join(robot_desc_pkg, "meshes")
    except Exception:
        robot_desc_default = os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")
        resource_root = os.path.join(pkg_share, "urdf")
        mesh_root = os.path.join(resource_root, "meshes", "topoarm")

    if not robot_description_file:
        robot_description_file = robot_desc_default

    def resolve_result_path(path: str, default_filename: str) -> str:
        if path:
            if os.path.isabs(path):
                return path
            if path.startswith("gng_results/") or "/" in path:
                return os.path.join(pkg_share, path)
        filename = path or default_filename
        return os.path.join(pkg_share, data_dir, exp_id, filename)

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    # YAMLの値を上書き（消去）しないよう、明示的に指定された（空でない）パラメータのみを抽出
    common_params = {}
    if robot_name:
        common_params["robot_name"] = robot_name
    if robot_description_file:
        common_params["robot_description_file"] = robot_description_file
    if arm_leaf_link_names:
        common_params["arm_leaf_link_names"] = arm_leaf_link_names
    
    # 座標系(frame_id)はデフォルトの"world"以外、または明示的に指定された場合のみ上書き
    if robot_base_frame != "world":
        common_params["frame_id"] = robot_base_frame
    
    if publish_hz != 30.0:
        common_params["publish_hz"] = publish_hz
    
    if resource_root:
        common_params["resource_root_dir"] = resource_root
    if mesh_root:
        common_params["mesh_root_dir"] = mesh_root

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
        # 1. GNGブリッジ (Topofuzzy)
        # GNGグラフを購読し、ビューアが解釈可能な形式に変換して配信します。
        Node(
            package="gng_vlut_system",
            executable="topofuzzy_bridge_node",
            name="topofuzzy_bridge_node",
            parameters=[{
                "gng_model_path": resolve_result_path(gng_model_path, gng_model_filename),
                "vlut_path": resolve_result_path(vlut_path, vlut_filename),
                "data_directory": data_dir,
                "experiment_id": exp_id,
                "frame_id": gng_frame_id,
                "source_frame_id": gng_source_frame_id,
                "publish_hz": publish_hz,
                "topic_name": topic_name,
                "robot_description_file": robot_description_file,
                "arm_leaf_link_names": arm_leaf_link_names,
            }]
        ),

        # 2. ロボットビューアブリッジ
        # ロボットのURDFモデルと現在の関節状態をビューアに送信し、3D表示させます。
        Node(
            package="gng_vlut_system",
            executable="robot_viewer_bridge_node",
            name="robot_viewer_bridge_node",
            parameters=viewer_bridge_params,
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm", description="ロボットの名前"),
        DeclareLaunchArgument("dir", default_value="gng_results", description="GNGデータのディレクトリ"),
        DeclareLaunchArgument("id", default_value="topoarm", description="実験ID"),
        DeclareLaunchArgument("gng_model_path", default_value="", description="GNGモデルバイナリへのパス"),
        DeclareLaunchArgument("vlut_path", default_value="", description="VLUTバイナリへのパス"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "topodual.yaml"), description="設定YAMLファイル"),
        DeclareLaunchArgument("robot_base_frame", default_value="world", description="ロボットのベース座標系"),
        DeclareLaunchArgument("robot_description_file", default_value="", description="URDF/Xacroファイルへのパス"),
        DeclareLaunchArgument("arm_leaf_link_names", default_value="", description="手先リンク名のリスト（カンマ区切り）"),
        DeclareLaunchArgument("gng_frame_id", default_value="world", description="グラフを表示する座標系"),
        DeclareLaunchArgument("gng_source_frame_id", default_value="world", description="グラフデータの元の座標系"),
        DeclareLaunchArgument("publish_hz", default_value="30.0", description="配信周波数"),
        DeclareLaunchArgument("topic_name", default_value="/topological_map_static", description="配信トピック名"),
        OpaqueFunction(function=launch_setup)
    ])
