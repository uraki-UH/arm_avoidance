import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_robot_description_path(pkg_share: str, raw_path: str) -> str:
    if (not raw_path):
        return os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm_dual.urdf.xacro")

    if raw_path.startswith("package://gng_vlut_system/"):
        return os.path.join(pkg_share, raw_path[len("package://gng_vlut_system/"):])

    if raw_path.startswith("package://"):
        pkg_and_path = raw_path[len("package://"):]
        _, _, rel_path = pkg_and_path.partition("/")
        if rel_path:
            return os.path.join(pkg_share, rel_path)

    return raw_path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("gng_vlut_system")
    params_file = LaunchConfiguration("params_file").perform(context)
    
    # --- YAMLから設定を自動抽出するロジック ---
    robot_name_default = LaunchConfiguration("robot_name").perform(context)
    robot_name = robot_name_default
    
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
        except Exception as e:
            print(f"Warning: Failed to parse YAML for robot_name: {e}")

    # コマンドラインで明示的に指定された場合はそちらを優先
    # LaunchConfigurationは一度performしないと値が取れないため注意
    user_robot_name = LaunchConfiguration("robot_name").perform(context)
    # デフォルト値以外が指定されていれば、それを優先
    if user_robot_name and user_robot_name != robot_name_default:
        robot_name = user_robot_name

    robot_description_raw = LaunchConfiguration("robot_description_file").perform(context)
    robot_urdf = resolve_robot_description_path(pkg_share, robot_description_raw)

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    node_params = {}
    if robot_urdf:
        node_params["robot_urdf_path"] = robot_urdf
    
    # コマンドライン引数を辞書に追加（明示的に指定された場合のみ、適切な型でYAMLを上書きするようにする）
    def add_if_not_empty(name, config_name, type_func=None):
        val = LaunchConfiguration(config_name).perform(context)
        if val:
            try:
                node_params[name] = type_func(val) if type_func else val
            except ValueError:
                node_params[name] = val

    add_if_not_empty("marker_frame_id", "marker_frame_id")
    add_if_not_empty("joint_topic", "joint_topic")
    add_if_not_empty("voxel_size", "voxel_size", float)
    add_if_not_empty("update_hz", "update_hz", float)
    add_if_not_empty("publish_self_mask", "publish_self_mask") # boolは文字列でも解釈されることが多いが
    add_if_not_empty("publish_link_voxels", "publish_link_voxels")
    add_if_not_empty("publish_link_aabb", "publish_link_aabb")
    add_if_not_empty("display_mode", "display_mode", int)
    add_if_not_empty("target_frame_id", "target_frame_id")

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
                "robot_description_file": robot_urdf,
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
        DeclareLaunchArgument("robot_name", default_value="topoarm_dual", description="ロボットの名前"),
        DeclareLaunchArgument("robot_description_file", default_value="", description="URDF/Xacroファイルへのパス"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "topoarm_dual.yaml"), description="設定YAMLファイル"),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="false", description="JointStatePublisherを起動するか"),
        DeclareLaunchArgument("marker_frame_id", default_value="world", description="マーカーを表示する座標系"),
        DeclareLaunchArgument("joint_topic", default_value="/joint_states", description="関節状態の購読トピック"),
        DeclareLaunchArgument("voxel_size", default_value="0.02", description="ボクセル解像度 [m]"),
        DeclareLaunchArgument("update_hz", default_value="10.0", description="更新周波数 [Hz]"),
        DeclareLaunchArgument("publish_self_mask", default_value="true", description="自己認識マスクを配信するか"),
        DeclareLaunchArgument("publish_link_voxels", default_value="true", description="リンク毎のボクセルを配信するか"),
        DeclareLaunchArgument("publish_link_aabb", default_value="true", description="リンク毎のAABBを配信するか"),
        DeclareLaunchArgument("display_mode", default_value="link_local", description="表示モード (link_local / world)"),
        DeclareLaunchArgument("target_frame_id", default_value="", description="ボクセル計算の基準座標系 (空ならベースリンク基準)"),
        OpaqueFunction(function=launch_setup),
    ])
