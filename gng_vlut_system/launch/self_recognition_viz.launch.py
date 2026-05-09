import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_robot_description_path(pkg_share: str, raw_path: str) -> str:
    if not raw_path:
        return os.path.join(pkg_share, "urdf", "topoarm_description", "urdf", "topoarm.urdf.xacro")

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
    
    robot_description_raw = LaunchConfiguration("robot_description_file").perform(context)
    robot_urdf = resolve_robot_description_path(pkg_share, robot_description_raw)

    # 最終的なパラメータを準備（YAMLとコマンドライン引数のマージ）
    node_params = {}
    if robot_urdf:
        node_params["robot_urdf_path"] = robot_urdf
    
    # コマンドライン引数を辞書に追加（デフォルト値でない場合や明示的な指定を想定）
    # ※ LaunchConfigurationは常に値を持つため、YAMLがある場合はそれらを含める
    node_params.update({
        "marker_frame_id": LaunchConfiguration("marker_frame_id"),
        "joint_topic": LaunchConfiguration("joint_topic"),
        "voxel_size": LaunchConfiguration("voxel_size"),
        "update_hz": LaunchConfiguration("update_hz"),
        "publish_self_mask": LaunchConfiguration("publish_self_mask"),
        "publish_link_voxels": LaunchConfiguration("publish_link_voxels"),
        "publish_link_aabb": LaunchConfiguration("publish_link_aabb"),
        "display_mode": LaunchConfiguration("display_mode"),
    })

    final_params_list = []
    if params_file and os.path.exists(params_file):
        final_params_list.append(params_file)
    final_params_list.append(node_params)

    return [
        # ロボットモデルの展開 (Digital Twin)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "robot_spawn.launch.py")),
            launch_arguments={
                "robot_name": LaunchConfiguration("robot_name"),
                "robot_description_file": robot_urdf,
                "enable_joint_state_publisher": LaunchConfiguration("enable_joint_state_publisher"),
            }.items()
        ),
        # 自己認識可視化ノード
        Node(
            package="gng_vlut_system",
            executable="self_recognition_viz_node",
            name="self_recognition_viz_node",
            output="screen",
            parameters=final_params_list,
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="topoarm", description="ロボットの名前"),
        DeclareLaunchArgument("robot_description_file", default_value="", description="URDF/Xacroファイルへのパス"),
        DeclareLaunchArgument("params_file", default_value="", description="設定YAMLファイル（任意）"),
        DeclareLaunchArgument("enable_joint_state_publisher", default_value="true", description="JointStatePublisherを起動するか"),
        DeclareLaunchArgument("marker_frame_id", default_value="world", description="マーカーを表示する座標系"),
        DeclareLaunchArgument("joint_topic", default_value="/joint_states", description="関節状態の購読トピック"),
        DeclareLaunchArgument("voxel_size", default_value="0.02", description="ボクセル解像度 [m]"),
        DeclareLaunchArgument("update_hz", default_value="10.0", description="更新周波数 [Hz]"),
        DeclareLaunchArgument("publish_self_mask", default_value="true", description="自己認識マスクを配信するか"),
        DeclareLaunchArgument("publish_link_voxels", default_value="true", description="リンク毎のボクセルを配信するか"),
        DeclareLaunchArgument("publish_link_aabb", default_value="true", description="リンク毎のAABBを配信するか"),
        DeclareLaunchArgument("display_mode", default_value="link_local", description="表示モード (link_local / world)"),
        OpaqueFunction(function=launch_setup),
    ])
