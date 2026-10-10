"""選択した追加プラグインだけを使う、実行機能なしのMoveIt計画サーバー。"""
from pathlib import Path
import sys
import xml.etree.ElementTree as et

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

package_dir = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(package_dir / 'launch'))
from dual_arm_effort_config import default_urdf, load_model, validate_namespace

script_dir = package_dir / 'scripts'
if not script_dir.is_dir():
    from ament_index_python.packages import get_package_prefix
    script_dir = Path(get_package_prefix('gng_vlut_system')) / 'lib/gng_vlut_system'
sys.path.insert(0, str(script_dir))
from optional_motion_planners import motion_planner_kind, planner_specs


def semantic_model(root):
    """既存機体の左右7関節グループ。同一剛体・隣接剛体だけの干渉除外。"""
    names = {joint.get('name') for joint in root.findall('joint')}
    result = et.Element('robot', name=root.get('name'))
    for group_name, prefixes in (('left_arm', ('L',)), ('right_arm', ('R',)), ('dual_arm', ('L', 'R'))):
        group = et.SubElement(result, 'group', name=group_name)
        for prefix in prefixes:
            for idx in range(1, 8):
                name = f'{prefix}_joint{idx}'
                if name not in names:
                    raise ValueError('計画用の腕関節がURDFにありません: ' + name)
                et.SubElement(group, 'joint', name=name)
    parents = {link.get('name'): link.get('name') for link in root.findall('link')}
    def body(name):
        while parents[name] != name:
            name = parents[name]
        return name
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            parents[body(joint.find('child').get('link'))] = body(joint.find('parent').get('link'))
    adjacent = {frozenset((body(joint.find('parent').get('link')), body(joint.find('child').get('link'))))
                for joint in root.findall('joint') if joint.get('type') != 'fixed'}
    links = list(parents)
    for idx, left in enumerate(links):
        for right in links[idx+1:]:
            left_body, right_body = body(left), body(right)
            if left_body == right_body or frozenset((left_body, right_body)) in adjacent:
                et.SubElement(result, 'disable_collisions', link1=left, link2=right,
                              reason='Fixed' if left_body == right_body else 'Adjacent')
    return et.tostring(result, encoding='unicode')


def planner_parameters(root, kinds, defaults_dir):
    """MoveIt同梱の版対応設定と選択方式の組合せ。既定方式への救済切替なし。"""
    pipelines = list(dict.fromkeys(planner_specs[kind].pipeline_id for kind in kinds))
    result = {
        'robot_description': et.tostring(root, encoding='unicode'),
        'robot_description_semantic': semantic_model(root),
        'planning_pipelines': pipelines, 'default_planning_pipeline': pipelines[0],
        'allow_trajectory_execution': False, 'use_sim_time': True,
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'planning_scene_monitor_options': {'joint_state_topic': 'joint_states'},
        'capabilities': 'move_group/MoveGroupPlanService move_group/MoveGroupQueryPlannersService',
        'disable_capabilities': 'move_group/MoveGroupMoveAction move_group/MoveGroupExecuteTrajectoryAction',
    }
    for pipeline in pipelines:
        config_file = Path(defaults_dir) / (pipeline + '_planning.yaml')
        if not config_file.is_file():
            raise RuntimeError('MoveIt同梱の計画設定がありません: ' + str(config_file))
        config = yaml.safe_load(config_file.read_text())
        spec = next(planner_specs[kind] for kind in kinds if planner_specs[kind].pipeline_id == pipeline)
        plugin = spec.plugin
        if pipeline == 'ompl' and any(kind in (motion_planner_kind.bit_star, motion_planner_kind.informed_rrt_star) for kind in kinds):
            plugin = 'gng_vlut_system/OptionalOMPLPlanners'
        if 'planning_plugins' in config:
            config['planning_plugins'] = [plugin]
        else:
            config['planning_plugin'] = plugin
        # 端点変更アダプタと別計画器による救済の除外。始点の問題は呼出し元へ返却。
        if isinstance(config.get('request_adapters'), str):
            config['request_adapters'] = ' '.join(item for item in config['request_adapters'].split()
                if 'FixStartState' not in item and 'CHOMPOptimizerAdapter' not in item
                and 'AddTimeOptimalParameterization' not in item)
        if isinstance(config.get('response_adapters'), list):
            config['response_adapters'] = [item for item in config['response_adapters']
                if 'AddTimeOptimalParameterization' not in item and 'DisplayMotionPath' not in item]
        if pipeline == 'chomp':
            config['enable_failure_recovery'] = False
        if pipeline == 'ompl':
            configs = {planner_specs[kind].planner_id: {'type': planner_specs[kind].ompl_type}
                       for kind in kinds if planner_specs[kind].pipeline_id == 'ompl'}
            config['planner_configs'] = configs
            for group in ('left_arm', 'right_arm', 'dual_arm'):
                config[group] = {'planner_configs': list(configs),
                                 'default_planner_config': next(iter(configs)),
                                 'enforce_joint_model_state_space': True,
                                 'longest_valid_segment_fraction': 0.005}
        result[pipeline] = config
    # 軌道化側でも適用される速度・加速度制限。URDFに加速度の規定なし。
    joint_limits = {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed' or joint.find('mimic') is not None:
            continue
        joint_limits[joint.get('name')] = {
            'has_velocity_limits': True, 'max_velocity': float(joint.find('limit').get('velocity')),
            'has_acceleration_limits': True, 'max_acceleration': 0.5}
    result['robot_description_planning'] = {'joint_limits': joint_limits}
    return result


def launch_setup(context):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    value = lambda name: LaunchConfiguration(name).perform(context)
    namespace = validate_namespace(value('namespace'))
    try:
        kinds = tuple(motion_planner_kind(name.strip()) for name in value('planners').split(','))
    except ValueError as error:
        raise ValueError('plannersには登録済みの方式名のカンマ区切りが必要です') from error
    if len(set(kinds)) != len(kinds):
        raise ValueError('plannersの方式名が重複しています')
    required = ('moveit_ros_move_group', 'moveit_msgs', 'moveit_configs_utils',
                *dict.fromkeys(planner_specs[kind].package for kind in kinds))
    for package in required:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError as error:
            raise RuntimeError('選択方式の実行に必要な追加パッケージがありません: ' + package) from error
    defaults_dir = Path(get_package_share_directory('moveit_configs_utils')) / 'default_configs'
    if any(kind in (motion_planner_kind.bit_star, motion_planner_kind.informed_rrt_star) for kind in kinds):
        from ament_index_python.resources import get_resource
        try:
            get_resource('moveit_core__pluginlib__plugin', 'gng_vlut_system')
        except LookupError as error:
            raise RuntimeError('BIT*登録拡張が未ビルドです。-Denable_optional_motion_planners=ONでビルドしてください') from error
    settings = planner_parameters(load_model(value('urdf')), kinds, defaults_dir)
    # XMLの数値・真偽値への誤変換防止。
    for name in ('robot_description', 'robot_description_semantic'):
        settings[name] = ParameterValue(settings[name], value_type=str)
    return [Node(package='moveit_ros_move_group', executable='move_group', namespace=namespace,
                 parameters=[settings], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=str(default_urdf())),
        DeclareLaunchArgument('namespace', default_value='sim_topo_dual_arm_max_long'),
        DeclareLaunchArgument('planners', default_value='rrt_connect'),
        OpaqueFunction(function=launch_setup),
    ])
