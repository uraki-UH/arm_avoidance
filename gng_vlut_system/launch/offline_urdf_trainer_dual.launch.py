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
    gng_data_directory = LaunchConfiguration("gng_data_directory").perform(context)
    vlut_only = LaunchConfiguration("vlut_only").perform(context)
    use_voxel_collision = LaunchConfiguration("use_voxel_collision").perform(context)
    skip_collision_checks = LaunchConfiguration("skip_collision_checks").perform(context)
    voxel_padding = LaunchConfiguration("voxel_padding").perform(context)
    initial_collision_only = LaunchConfiguration("initial_collision_only").perform(context)
    validate_voxel_link_masks = LaunchConfiguration("validate_voxel_link_masks").perform(context)
    validation_focus_links = LaunchConfiguration("validation_focus_links").perform(context)
    validation_max_print_voxels = LaunchConfiguration("validation_max_print_voxels").perform(context)
    validation_dump_path = LaunchConfiguration("validation_dump_path").perform(context)
    gng_profile_names = LaunchConfiguration("gng_profile_names").perform(context)
    use_task_density_bias = LaunchConfiguration("use_task_density_bias").perform(context)

    # 上書き用パラメータの準備
    overrides = {}
    if robot_urdf_path:
        overrides["robot_urdf_path"] = robot_urdf_path
    if experiment_id:
        overrides["gng.experiment_id"] = experiment_id
    if gng_data_directory:
        overrides["gng.data_directory"] = gng_data_directory
    vlut_only_value = (vlut_only.lower() == "true")
    overrides["vlut_only"] = vlut_only_value
    overrides["gng.vlut_only"] = vlut_only_value
    use_voxel_collision_value = (use_voxel_collision.lower() == "true")
    overrides["use_voxel_collision"] = use_voxel_collision_value
    overrides["gng.use_voxel_collision"] = use_voxel_collision_value
    skip_collision_checks_value = (skip_collision_checks.lower() == "true")
    overrides["collision.skip_checks"] = skip_collision_checks_value
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
    use_task_density_bias_value = (use_task_density_bias.lower() == "true")
    overrides["gng_params.use_task_density_bias"] = use_task_density_bias_value

    return [
        # オフラインURDFトレーナーノード (デュアルアーム用)
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
    dual_cfg = os.path.join(pkg_share, "config", "topoarm_dual.yaml")
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=dual_cfg,
        ),
        DeclareLaunchArgument(
            "robot_urdf_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "experiment_id",
            default_value="",
        ),
        DeclareLaunchArgument(
            "gng_data_directory",
            default_value="",
        ),
        DeclareLaunchArgument(
            "vlut_only",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_voxel_collision",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "skip_collision_checks",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "voxel_padding",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "initial_collision_only",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "validate_voxel_link_masks",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "validation_focus_links",
            default_value="",
        ),
        DeclareLaunchArgument(
            "validation_max_print_voxels",
            default_value="4",
        ),
        DeclareLaunchArgument(
            "validation_dump_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "gng_profile_names",
            default_value="left_arm,right_arm",
        ),
        DeclareLaunchArgument(
            "use_task_density_bias",
            default_value="false",
        ),
        OpaqueFunction(function=launch_setup)
    ])
