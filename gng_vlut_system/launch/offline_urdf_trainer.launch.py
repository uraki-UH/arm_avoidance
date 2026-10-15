import os
from pathlib import Path
import sys
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import OpaqueFunction


def require_success(event, _context):
    """追加索引の構築失敗の伝播。"""
    if event.returncode != 0:
        raise RuntimeError('追加VLUTの構築失敗')
    return []


def hierarchy_builder(context, params_file, robot_urdf_path):
    """GNG再学習を伴わないURDF階層索引の独立構築。"""
    share = Path(get_package_share_directory('gng_vlut_system'))
    builder_path = Path(get_package_prefix('gng_vlut_system'))/'lib/gng_vlut_system/urdf_vrm.py'
    avoidance_config = LaunchConfiguration('avoidance_config').perform(context) or str(share/'config/dual_arm_gng_lidar_demo.yaml')
    command = [sys.executable, str(builder_path), '--params-file', params_file,
               '--avoidance-config', avoidance_config, '--construction-method', 'hierarchical_cover']
    for name, option in (('hierarchy_group', '--group'), ('hierarchy_vlut_file', '--output'),
                         ('hierarchy_max_num_regions', '--max-num-regions'),
                         ('hierarchy_max_num_leaves_per_parent', '--max-num-leaves-per-parent'),
                         ('hierarchy_max_build_sec', '--max-build-sec')):
        value = LaunchConfiguration(name).perform(context)
        if value:
            command += [option, value]
    if robot_urdf_path:
        command += ['--urdf', robot_urdf_path]
    builder = ExecuteProcess(cmd=command, output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=builder, on_exit=require_success)), builder]


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
    vlut_backend = LaunchConfiguration("vlut_backend").perform(context)
    if vlut_backend == 'hierarchy_vlut':
        return hierarchy_builder(context, params_file, robot_urdf_path)

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
    if use_voxel_collision != "":
        use_voxel_collision_value = (use_voxel_collision.lower() == "true")
        overrides["use_voxel_collision"] = use_voxel_collision_value
        overrides["gng.use_voxel_collision"] = use_voxel_collision_value
    if skip_collision_checks != "":
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
    if vlut_backend == 'edge_vlut':
        overrides['gng.enable_edge_vlut'] = True
    elif vlut_backend == 'region_vlut':
        overrides['gng.enable_roadmap_recipe'] = True

    # 従来のGNG・ノードVLUT構築器。追加バックエンド指定時も同じ生成処理。
    trainer = Node(
            package="gng_vlut_system",
            executable="offline_urdf_trainer",
            name="offline_urdf_trainer",
            output="screen",
            parameters=[
                params_file,
                overrides
            ],
        )
    if vlut_backend == 'node_vlut':
        return [trainer]
    parameters = yaml.safe_load(Path(params_file).read_text())['/**']['ros__parameters']['gng']
    directory = Path(gng_data_directory or parameters.get('data_directory', 'gng_results'))
    directory /= experiment_id or parameters.get('experiment_id', '')
    builder_name = 'gng_region_vlut.py' if vlut_backend == 'region_vlut' else 'gng_edge_vlut.py'
    builder_path = Path(get_package_prefix('gng_vlut_system'))/'lib/gng_vlut_system'/builder_name
    command = [sys.executable, str(builder_path), '--input', str(directory), '--allow-replace']
    if vlut_backend == 'region_vlut':
        command += ['--construction-method', LaunchConfiguration('region_construction_method').perform(context)]
        command += ['--internal-policy', LaunchConfiguration('region_internal_policy').perform(context)]
        required_poses = LaunchConfiguration('required_poses').perform(context)
        if required_poses:
            command += ['--required-poses', required_poses]
    avoidance_config = LaunchConfiguration('avoidance_config').perform(context)
    if avoidance_config:
        command += ['--avoidance-config', avoidance_config]
    builder = ExecuteProcess(cmd=command, output='screen')
    def after_training(event, _context):
        require_success(event, _context)
        return [builder]
    return [RegisterEventHandler(OnProcessExit(target_action=builder, on_exit=require_success)),
            RegisterEventHandler(OnProcessExit(target_action=trainer, on_exit=after_training)), trainer]

def generate_launch_description():
    pkg_share = get_package_share_directory("gng_vlut_system")
    
    return LaunchDescription([
        DeclareLaunchArgument('vlut_backend', default_value='node_vlut', choices=['node_vlut', 'edge_vlut', 'region_vlut', 'hierarchy_vlut']),
        DeclareLaunchArgument('hierarchy_group', default_value='left_arm'),
        DeclareLaunchArgument('hierarchy_vlut_file', default_value=''),
        DeclareLaunchArgument('hierarchy_max_num_regions', default_value='128'),
        DeclareLaunchArgument('hierarchy_max_num_leaves_per_parent', default_value='16'),
        DeclareLaunchArgument('hierarchy_max_build_sec', default_value='30'),
        DeclareLaunchArgument('region_construction_method', default_value='legacy', choices=['legacy', 'quadratic_cover', 'occupancy_cover', 'efficient']),
        DeclareLaunchArgument('region_internal_policy', default_value='coupled', choices=['coupled', 'separate']),
        DeclareLaunchArgument('required_poses', default_value=''),
        DeclareLaunchArgument('avoidance_config', default_value=''),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "topo_dual_arm_max_long.yaml"),
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
            default_value="",
        ),
        DeclareLaunchArgument(
            "skip_collision_checks",
            default_value="",
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
            default_value="8",
        ),
        DeclareLaunchArgument(
            "validation_dump_path",
            default_value="",
        ),
        DeclareLaunchArgument(
            "gng_profile_names",
            default_value="",
        ),
        DeclareLaunchArgument(
            "use_task_density_bias",
            default_value="false",
        ),
        OpaqueFunction(function=launch_setup)
    ])
