import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    robot_urdf_path = LaunchConfiguration("robot_urdf_path").perform(context)
    experiment_id = LaunchConfiguration("experiment_id").perform(context)
    vlut_only = LaunchConfiguration("vlut_only").perform(context)

    # 上書き用パラメータの準備
    overrides = {}
    if robot_urdf_path:
        overrides["robot_urdf_path"] = robot_urdf_path
    if experiment_id:
        overrides["experiment_id"] = experiment_id
    if vlut_only != "false":
        overrides["vlut_only"] = (vlut_only.lower() == "true")

    return [
        # オフラインURDFトレーナーノード
        # 指定されたURDFから、自己認識用のGNGグラフと衝突判定用のVLUTを生成します。
        Node(
            package="gng_vlut_system",
            executable="offline_urdf_trainer",
            name="offline_urdf_trainer",
            output="screen",
            parameters=[
                params_file,
                overrides
            ],
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "gng_safety_params.yaml"),
            description="設定YAMLファイルへのパス",
        ),
        DeclareLaunchArgument(
            "robot_urdf_path",
            default_value="",
            description="ロボットのURDFパスを上書き",
        ),
        DeclareLaunchArgument(
            "experiment_id",
            default_value="",
            description="実験IDを上書き",
        ),
        DeclareLaunchArgument(
            "vlut_only",
            default_value="false",
            description="'true'にするとGNG学習をスキップし、VLUT生成のみ行います",
        ),
        OpaqueFunction(function=launch_setup)
    ])
