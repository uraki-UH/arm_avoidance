import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#legacy実装かも

from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    robot_urdf_path = LaunchConfiguration("robot_urdf_path").perform(context)
    experiment_id = LaunchConfiguration("experiment_id").perform(context)
    vlut_only = LaunchConfiguration("vlut_only").perform(context)
    use_voxel_collision = LaunchConfiguration("use_voxel_collision").perform(context)
    voxel_padding = LaunchConfiguration("voxel_padding").perform(context)
    initial_collision_only = LaunchConfiguration("initial_collision_only").perform(context)
    validate_voxel_link_masks = LaunchConfiguration("validate_voxel_link_masks").perform(context)
    validation_focus_links = LaunchConfiguration("validation_focus_links").perform(context)
    validation_max_print_voxels = LaunchConfiguration("validation_max_print_voxels").perform(context)
    validation_dump_path = LaunchConfiguration("validation_dump_path").perform(context)
    gng_profile_names = LaunchConfiguration("gng_profile_names").perform(context)

    # 上書き用パラメータの準備
    overrides = {}
    if robot_urdf_path:
        overrides["robot_urdf_path"] = robot_urdf_path
    if experiment_id:
        overrides["gng.experiment_id"] = experiment_id
    if vlut_only != "false":
        overrides["vlut_only"] = (vlut_only.lower() == "true")
    if use_voxel_collision != "false":
        overrides["use_voxel_collision"] = (use_voxel_collision.lower() == "true")
    if voxel_padding:
        overrides["voxel_padding"] = float(voxel_padding)
    if initial_collision_only != "false":
        value = (initial_collision_only.lower() == "true")
        overrides["initial_collision_only"] = value
        overrides["collision.initial_collision_only"] = (
            value
        )
    if validate_voxel_link_masks != "false":
        value = (validate_voxel_link_masks.lower() == "true")
        overrides["collision.validate_voxel_link_masks"] = value
    if validation_focus_links:
        overrides["collision.validation_focus_links"] = validation_focus_links
    if validation_max_print_voxels:
        overrides["collision.validation_max_print_voxels"] = int(validation_max_print_voxels)
    if validation_dump_path:
        overrides["collision.validation_dump_path"] = validation_dump_path
    if gng_profile_names:
        overrides["gng.profile_names"] = gng_profile_names

    return [

        # 指定されたURDFから、エフェクティビティマップのGNGグラフと姿勢に対する衝突を一括検索するVoxel-LUTを生成
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
            description="'true'にするとGNG学習をスキップし、VLUT生成のみ行う",
        ),
        DeclareLaunchArgument(
            "use_voxel_collision",
            default_value="false",
            description="'true'にするとボクセルベースの衝突判定を使用",
        ),
        DeclareLaunchArgument(
            "voxel_padding",
            default_value="0.0",
            description="ボクセル衝突判定時のパディング量 (学習時は0.0推奨)",
        ),
        DeclareLaunchArgument(
            "initial_collision_only",
            default_value="false",
            description="trueにすると初期姿勢の承認用YAMLだけ生成して終了",
        ),
        DeclareLaunchArgument(
            "validate_voxel_link_masks",
            default_value="false",
            description="trueにすると初期姿勢のlink別voxel mask検証レポートを出す",
        ),
        DeclareLaunchArgument(
            "validation_focus_links",
            default_value="",
            description="検証レポートを出すリンク名のカンマ区切り。空なら全件",
        ),
        DeclareLaunchArgument(
            "validation_max_print_voxels",
            default_value="8",
            description="各リンクで詳細表示する voxel 数の上限",
        ),
        DeclareLaunchArgument(
            "validation_dump_path",
            default_value="",
            description="検証レポートを書き出すファイルパス",
        ),
        DeclareLaunchArgument(
            "gng_profile_names",
            default_value="",
            description="gng_profiles から使う profile 名のカンマ区切り (例: left_arm,right_arm)",
        ),
        OpaqueFunction(function=launch_setup)
    ])
