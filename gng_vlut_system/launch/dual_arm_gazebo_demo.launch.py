import importlib.util
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_module(path):
    spec = importlib.util.spec_from_file_location('gazebo_urdf_helper', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def initial_joint_positions(root, configured):
    """初期関節位置の検査と従属関節の展開。単位はrad・m。"""
    joints = {item.get('name'): item for item in root.findall('joint') if item.get('type') != 'fixed'}
    if not isinstance(configured, dict) or any(name not in joints for name in configured):
        raise ValueError('初期姿勢には可動関節名と位置の辞書が必要です')
    positions, visiting = {}, set()

    def resolve(name):
        if name in positions:
            return positions[name]
        if name in visiting or name not in joints:
            raise ValueError('初期姿勢のmimic参照が不正です')
        visiting.add(name)
        joint = joints[name]
        mimic = joint.find('mimic')
        if mimic is not None:
            if name in configured:
                raise ValueError('初期姿勢のmimic関節は親関節で指定してください')
            value = resolve(mimic.get('joint')) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
        else:
            value = float(configured.get(name, 0.0))
        limit = joint.find('limit')
        if not math.isfinite(value) or (joint.get('type') != 'continuous' and
                (limit is None or not float(limit.get('lower')) <= value <= float(limit.get('upper')))):
            raise ValueError(f'初期関節位置がURDF範囲外です: {name}')
        positions[name] = value
        visiting.remove(name)
        return value

    for name in joints:
        resolve(name)
    return positions


def launch_setup(context):
    package_share = Path(get_package_share_directory('gng_vlut_system'))
    params_path = Path(LaunchConfiguration('params_file').perform(context))
    demo_path = Path(LaunchConfiguration('demo_config').perform(context))
    params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']
    config = yaml.safe_load(demo_path.read_text())['dual_arm_gazebo_demo']
    avoidance_path = LaunchConfiguration('avoidance_config').perform(context)
    avoidance_config = (yaml.safe_load(Path(avoidance_path).read_text())['dual_arm_avoidance_demo']
                        if avoidance_path else None)
    enable_external_control = LaunchConfiguration('enable_external_control').perform(context).lower() == 'true'
    enable_integrated_control = LaunchConfiguration('enable_integrated_control').perform(context).lower() == 'true'
    if enable_integrated_control and (enable_external_control or avoidance_config is None):
        raise ValueError('統合操作には回避設定と単一の指令出力経路が必要です')
    namespace = config.get('namespace') or 'sim_' + params['robot_name']
    if not namespace.startswith('sim_') or '/' in namespace:
        raise ValueError('デモの名前空間はsim_で始まる単一名が必要です')
    point_cloud_source = LaunchConfiguration('point_cloud_source').perform(context)
    sensor_helper = load_module(package_share/'launch/dual_arm_lidar_setup.py')
    point_cloud_topic = sensor_helper.point_cloud_topic(point_cloud_source)
    depth_helper = depth_config = None
    if point_cloud_source == 'head_depth':
        depth_helper = load_module(package_share/'launch/dual_arm_depth_camera_setup.py')
        depth_config = depth_helper.load_config(LaunchConfiguration('depth_camera_config').perform(context))
    urdf_path = Path(params['urdf_path'])
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)
    gui_arg = LaunchConfiguration('gui').perform(context)
    gui = gui_arg or str(config['enable_gui']).lower()
    auto_arg = LaunchConfiguration('enable_auto_start').perform(context)
    enable_auto_start = (auto_arg or str(config['enable_auto_start'])).lower() == 'true'
    if avoidance_config is not None:
        enable_auto_start = (auto_arg or str(avoidance_config['enable_auto_start'])).lower() == 'true'
    run_dir = Path(tempfile.mkdtemp(prefix='dual_arm_gazebo_demo_'))
    helper = load_module(package_share/'launch/robot_gazebo_spawn.launch.py')
    root_link = avoidance_config.get('root_link', 'base_footprint') if avoidance_config else 'base_footprint'
    temporary_urdf = Path(helper.write_gazebo_urdf(str(urdf_path), params['mesh_root_dir'], False, root_link))
    root = ET.parse(temporary_urdf).getroot()
    temporary_urdf.unlink()
    initial_positions = initial_joint_positions(root, config.get('initial_joint_positions', {}))
    if depth_helper is not None:
        depth_helper.add_depth_camera(root, namespace, depth_config)
    control = ET.SubElement(root, 'ros2_control', name='GazeboSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gng_vlut_system/bounded_gazebo_system'
    joint_names = []
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        mimic = joint.find('mimic')
        if mimic is None:
            joint_names.append(name)
        item = ET.SubElement(control, 'joint', name=name)
        # 物理開始後の有限トルクモータへの位置目標
        ET.SubElement(item, 'command_interface', name='position')
        ET.SubElement(item, 'param', name='position_gain').text = str(config.get('motor_position_gain', 60.0))
        ET.SubElement(item, 'param', name='motor_limit_scale').text = str(config.get('motor_limit_scale', 0.95))
        state = ET.SubElement(item, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(initial_positions[name])
        ET.SubElement(item, 'state_interface', name='velocity')
        ET.SubElement(item, 'state_interface', name='effort')
        if mimic is not None:
            ET.SubElement(item, 'param', name='mimic').text = mimic.get('joint')
            ET.SubElement(item, 'param', name='multiplier').text = mimic.get('multiplier', '1')
    controllers = {
        f'/{namespace}/controller_manager': {'ros__parameters': {
            'update_rate': 1000, 'use_sim_time': True,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'dual_arm_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'},
        }},
        f'/{namespace}/dual_arm_controller': {'ros__parameters': {
            'joints': joint_names, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'state_publish_rate': 100.0 if enable_integrated_control else 50.0,
            'action_monitor_rate': 20.0,
            'allow_partial_joints_goal': False, 'open_loop_control': False,
            'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': 0.05},
        }},
    }
    controllers_path = run_dir/'controllers.yaml'
    controllers_path.write_text(yaml.safe_dump(controllers, sort_keys=False))
    gazebo_clock_path = run_dir/'gazebo_clock.yaml'
    # 低速シミュレーション時の重複時刻・UDP目標失効の抑制。壁時計の失効期限は変更なし
    gazebo_clock_path.write_text(yaml.safe_dump({'gazebo': {'ros__parameters': {'publish_rate': 100.0}}}))
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', name='gazebo_ros2_control', filename='libgazebo_ros2_control.so')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/' + namespace
    ET.SubElement(ros, 'remapping').text = '/joint_states:=/' + namespace + '/joint_states'
    ET.SubElement(plugin, 'robot_param_node').text = '/' + namespace + '/robot_state_publisher'
    ET.SubElement(plugin, 'robot_param').text = 'robot_description'
    ET.SubElement(plugin, 'parameters').text = str(controllers_path)
    robot_description = ET.tostring(root, encoding='unicode')
    gazebo_urdf = run_dir/'robot.urdf'
    gazebo_urdf.write_text(robot_description)
    world_path = package_share/'worlds/dual_arm_demo.world'
    if avoidance_config is not None:
        world_root = ET.parse(world_path).getroot()
        world = world_root.find('world')
        physics_solver = avoidance_config.get('physics_solver', 'quick')
        if physics_solver not in ('quick', 'world'):
            raise ValueError('physics_solverはquickまたはworldが必要です')
        world.find('physics/ode/solver/type').text = physics_solver
        state_plugin = ET.SubElement(world, 'plugin', name='avoidance_state', filename='libgazebo_ros_state.so')
        state_ros = ET.SubElement(state_plugin, 'ros')
        ET.SubElement(state_ros, 'namespace').text = '/avoidance_demo'
        ET.SubElement(state_plugin, 'update_rate').text = '30.0'
        if not avoidance_config.get('enable_live_obstacles', False):
            human = ET.SubElement(world, 'model', name='human_forearm')
            ET.SubElement(human, 'static').text = 'true'
            sign = 1 if avoidance_config['sides'][0] == 'left' else -1
            ET.SubElement(human, 'pose').text = '{} {} {} 0 0 0'.format(
                avoidance_config['hand_far_x'], sign*avoidance_config['hand_y'], avoidance_config['hand_z'])
            human_link = ET.SubElement(human, 'link', name='forearm')
            length, radius = avoidance_config['arm_length'], avoidance_config['arm_radius']
            for name, shape, pose in [('hand', 'sphere', '0 0 0 0 0 0'),
                                      ('elbow', 'sphere', f'{length} 0 0 0 0 0'),
                                      ('arm', 'cylinder', f'{length/2} 0 0 0 1.5707963267948966 0')]:
                for kind in ('collision', 'visual'):
                    item = ET.SubElement(human_link, kind, name=name)
                    ET.SubElement(item, 'pose').text = pose
                    geometry = ET.SubElement(ET.SubElement(item, 'geometry'), shape)
                    ET.SubElement(geometry, 'radius').text = str(radius)
                    if shape == 'cylinder':
                        ET.SubElement(geometry, 'length').text = str(length)
                    if kind == 'visual':
                        material = ET.SubElement(item, 'material')
                        ET.SubElement(material, 'ambient').text = '1 0.5 0.1 1'
                        ET.SubElement(material, 'diffuse').text = '1 0.5 0.1 1'
        if avoidance_config.get('enable_gng_vlut', False):
            if point_cloud_source == 'external_lidar':
                sensor_helper.add_lidar(world, namespace, avoidance_config)
        world_path = run_dir/'avoidance.world'
        ET.ElementTree(world_root).write(world_path, encoding='unicode')

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-file', str(gazebo_urdf), '-entity', namespace, '-timeout', '90'], output='screen')
    controller = Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster', 'dual_arm_controller',
                                 '-c', f'/{namespace}/controller_manager',
                                 '--controller-manager-timeout', '90', '--switch-timeout', '30'], output='screen')
    demo = Node(package='gng_vlut_system', executable='dual_arm_gazebo_demo.py',
                namespace=namespace, output='screen', parameters=[{
                    'use_sim_time': True, 'demo_config': str(demo_path),
                    'urdf_path': str(urdf_path), 'enable_auto_start': enable_auto_start}])

    if avoidance_config is not None:
        demo = Node(package='gng_vlut_system', executable=('dual_arm_gng_lidar_demo.py' if avoidance_config.get('enable_gng_vlut', False)
                                               else 'dual_arm_avoidance_demo.py'),
                    namespace=namespace, output='screen', parameters=[{
                        'use_sim_time': True, 'avoidance_config': avoidance_path,
                        'urdf_path': str(urdf_path),
                        'enable_auto_start': False if enable_integrated_control else enable_auto_start,
                        'enable_stamped_commands': enable_integrated_control}],
                    remappings=[('lidar_points', point_cloud_topic)] +
                               ([('dual_arm_controller/joint_trajectory', 'control/avoidance_trajectory')]
                                if enable_integrated_control else []))

    physics_start = Node(package='gng_vlut_system', executable='start_gazebo_physics.py',
                         parameters=[{'controller_manager': f'/{namespace}/controller_manager'}],
                         output='screen')

    def after_physics_start(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazebo物理開始失敗'))]
        return []

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazeboモデル生成失敗'))]
        return [controller, physics_start]

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='関節コントローラ起動失敗'))]
        return [] if enable_external_control else [demo]

    def cleanup(_context):
        shutil.rmtree(run_dir, ignore_errors=True)
        return []

    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    gazebo_arguments = {'world': str(world_path), 'gui': gui, 'pause': 'true'}
    if enable_integrated_control:
        gazebo_arguments['params_file'] = str(gazebo_clock_path)
    actions = [
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MASTER_URI', LaunchConfiguration('gazebo_master_uri')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo_share/'launch/gazebo.launch.py')),
            launch_arguments=gazebo_arguments.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
             parameters=[{'robot_description': robot_description, 'frame_prefix': namespace+'/', 'use_sim_time': True}],
             output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', namespace+'/world'],
             parameters=[{'use_sim_time': True}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=physics_start, on_exit=after_physics_start)),
        RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=after_controllers)),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        spawn,
    ]
    if enable_external_control:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share/'launch/joint_control.launch.py')),
            launch_arguments={
                'params_file': str(params_path), 'urdf_path': str(urdf_path),
                'robot_name': namespace, 'backend': 'gazebo', 'use_sim_time': 'true',
                'enable_dynamixel_input': LaunchConfiguration('enable_dynamixel_leader'),
                'dynamixel_input_topic': LaunchConfiguration('dynamixel_input_topic'),
            }.items()))
    if enable_integrated_control:
        control = Node(package='gng_vlut_system', executable='dual_arm_control.py',
                       namespace=namespace, output='log', parameters=[{
                           'use_sim_time': True, 'urdf_path': str(urdf_path),
                           'udp_config': ParameterValue(LaunchConfiguration('udp_config'), value_type=str),
                           'allow_remote_udp': ParameterValue(LaunchConfiguration('allow_remote_udp'), value_type=bool)}],
                       remappings=[('leader_joint_states', LaunchConfiguration('leader_joint_state_topic'))])
        actions.extend([control, RegisterEventHandler(OnProcessExit(
            target_action=control, on_exit=[EmitEvent(event=Shutdown(reason='統合制御の終了'))]))])
    if config.get('enable_viewer', True):
        actions.append(Node(package='gng_vlut_system', executable='robot_viewer_bridge_node',
            name='robot_viewer_bridge_node', namespace=namespace, parameters=[str(params_path), {
                'use_sim_time': True, 'robot_name': namespace,
                'joint_state_topic': f'/{namespace}/joint_states', 'frame_id': root_link,
                'stream_topic': '/viewer/internal/stream/robot'}]))
    if avoidance_config is not None and avoidance_config.get('enable_gng_vlut', False):
        actions.extend(sensor_helper.pipeline_nodes(params_path, params, namespace, avoidance_config, point_cloud_source))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('gng_vlut_system'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(package_share/'config/topo_dual_arm_max.yaml')),
        DeclareLaunchArgument('demo_config', default_value=str(package_share/'config/dual_arm_gazebo_demo.yaml')),
        DeclareLaunchArgument('avoidance_config', default_value=''),
        DeclareLaunchArgument('point_cloud_source', default_value='external_lidar'),
        DeclareLaunchArgument('depth_camera_config', default_value=str(package_share/'config/dual_arm_depth_camera.yaml')),
        DeclareLaunchArgument('enable_external_control', default_value='false'),
        DeclareLaunchArgument('enable_integrated_control', default_value='false'),
        DeclareLaunchArgument('leader_joint_state_topic', default_value='/leader/joint_states'),
        DeclareLaunchArgument('udp_config', default_value=''),
        DeclareLaunchArgument('allow_remote_udp', default_value='false'),
        DeclareLaunchArgument('enable_dynamixel_leader', default_value='false'),
        DeclareLaunchArgument('dynamixel_input_topic', default_value='/dynamixel/state/present'),
        DeclareLaunchArgument('gui', default_value=''),
        DeclareLaunchArgument('enable_auto_start', default_value=''),
        DeclareLaunchArgument('gazebo_master_uri', default_value='http://127.0.0.1:11355'),
        OpaqueFunction(function=launch_setup),
    ])
