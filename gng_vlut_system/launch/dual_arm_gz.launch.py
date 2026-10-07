"""Gazebo Harmonicと標準effort軌道コントローラによる物理シミュレーション。"""
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as et

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_effort_config import load_model, effort_joints, controller_parameters, validate_namespace, default_urdf
from simulation_scenario import load_scenario, save_scenario, gazebo_world

package_dir = Path(__file__).resolve().parents[1]


def prepare_model(urdf_path, output_dir, namespace, tuning, topics=None):
    """元URDFを保持した資産解決・固定基台・effort制御設定の生成。"""
    topics = topics or {}
    root = load_model(urdf_path)
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    roots = links-children
    if len(roots) != 1:
        raise ValueError('単一の基台リンクが必要です')
    if 'world' not in links:
        et.SubElement(root, 'link', name='world')
        fixed = et.SubElement(root, 'joint', name='world_fixed', type='fixed')
        et.SubElement(fixed, 'parent', link='world')
        et.SubElement(fixed, 'child', link=next(iter(roots)))
    control = et.SubElement(root, 'ros2_control', name='GazeboSimSystem', type='system')
    et.SubElement(et.SubElement(control, 'hardware'), 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    gains = effort_joints(root, tuning)
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        item = et.SubElement(control, 'joint', name=name)
        if joint.find('mimic') is None:
            effort = gains[name]['u_clamp_max']
            interface = et.SubElement(item, 'command_interface', name='effort')
            et.SubElement(interface, 'param', name='min').text = str(-effort)
            et.SubElement(interface, 'param', name='max').text = str(effort)
        for interface_name in ('position', 'velocity', 'effort'):
            state = et.SubElement(item, 'state_interface', name=interface_name)
            et.SubElement(state, 'param', name='initial_value').text = '0.0'
    parameters = controller_parameters(gains, namespace)
    params_path = output_dir/'controllers.yaml'
    params_path.write_text(yaml.safe_dump(parameters))
    plugin = et.SubElement(et.SubElement(root, 'gazebo'), 'plugin',
                           filename='gz_ros2_control-system', name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    et.SubElement(plugin, 'parameters').text = str(params_path)
    # 非active時の速度直接拘束を無効化。保持と停止も制限付きeffort制御が対象
    et.SubElement(plugin, 'hold_joints').text = 'false'
    ros = et.SubElement(plugin, 'ros')
    et.SubElement(ros, 'namespace').text = '/'+namespace
    for source, target in topics.items():
        et.SubElement(ros, 'remapping').text = source+':='+target
    path = output_dir/'robot.urdf'
    path.write_text(et.tostring(root, encoding='unicode'))
    return path


def launch_setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    scenario = load_scenario(value('scenario'))
    namespace = value('namespace')
    validate_namespace(namespace)
    output = Path(value('output_dir') or tempfile.mkdtemp(prefix='dual_arm_gz_'))
    output.mkdir(parents=True, exist_ok=True)
    tuning = yaml.safe_load(Path(value('control_config')).read_text())
    model = prepare_model(Path(value('urdf')), output, namespace, tuning, {
        'joint_states': value('state_topic'),
        'robot_description': value('description_topic'),
        'dual_arm_controller/joint_trajectory': value('trajectory_topic')})
    world = output/'world.sdf'
    save_scenario(scenario, output)
    world.write_text(et.tostring(gazebo_world(scenario), encoding='unicode'))
    command = ['gz', 'sim', '-r', str(world)]
    if value('gui').lower() != 'true':
        command.insert(2, '-s')
    simulator = ExecuteProcess(cmd=command, output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=simulator,
                on_exit=[EmitEvent(event=Shutdown(reason='Gazeboの終了'))])), simulator,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
             parameters=[{'use_sim_time': True, 'robot_description': model.read_text()}],
             remappings=[('joint_states', value('state_topic')),
                         ('robot_description', value('description_topic'))]),
        Node(package='ros_gz_sim', executable='create', arguments=['-world', 'motor_test', '-file', str(model), '-name', namespace], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'dual_arm_controller', '-c', '/'+namespace+'/controller_manager',
                        '--controller-manager-timeout', '90'], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=str(default_urdf('topo_dual_arm_max'))),
        DeclareLaunchArgument('control_config', default_value=str(package_dir/'config/dual_arm_effort.yaml')),
        DeclareLaunchArgument('scenario', default_value='empty', description='環境シナリオ名またはYAMLパス'),
        DeclareLaunchArgument('namespace', default_value='sim_topo_dual_arm_max'),
        DeclareLaunchArgument('state_topic', default_value='joint_states'),
        DeclareLaunchArgument('trajectory_topic', default_value='dual_arm_controller/joint_trajectory'),
        DeclareLaunchArgument('description_topic', default_value='robot_description'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('gui', default_value='false'),
        OpaqueFunction(function=launch_setup)])
