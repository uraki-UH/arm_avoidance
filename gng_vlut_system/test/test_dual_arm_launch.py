"""機体選択・Gazebo起動設定・頭部深度センサ・光学座標の検証。"""
from pathlib import Path
from unittest.mock import patch
import importlib.util
import math
import os
import sys
import xml.etree.ElementTree as et

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from scipy.spatial.transform import Rotation
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))

from dual_arm_avoidance_geometry import robot_geometry
from dual_arm_gazebo_demo import load_motion


workspace = Path(__file__).resolve().parents[2]


share = workspace / 'gng_vlut_system'


@pytest.fixture
def launch_module(monkeypatch):
    module = load('dual_arm_control.launch')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(share))
    return module


def make_context(robot, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot': robot, 'params_file': '', 'demo_config': '', 'avoidance_config': '',
        'leader_topic': '/leader/joint_states', 'enable_keyboard': 'false',
        'udp_config': '', 'gui': 'false', 'allow_remote_udp': 'false',
        'point_cloud_source': 'external_lidar',
        'depth_camera_config': str(share/'config/dual_arm_depth_camera.yaml'),
        'gazebo_master_uri': 'http://127.0.0.1:11355', 'leader_mapping_file': '',
        **overrides})
    return context


@pytest.mark.parametrize('robot,robot_name,enable_gng', [
    ('topodualarm', 'ToPoDualArm', False),
    ('max', 'topo_dual_arm_max', True),
    ('max_long', 'topo_dual_arm_max_long', True)])
def test_selected_model_and_avoidance(launch_module, robot, robot_name, enable_gng):
    actions = launch_module.launch_setup(make_context(robot))
    include = next(action for action in actions if isinstance(action, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    params = yaml.safe_load(Path(args['params_file']).read_text())['/**']['ros__parameters']
    config = yaml.safe_load(Path(args['avoidance_config']).read_text())['dual_arm_avoidance_demo']
    assert params['robot_name'] == robot_name
    assert config['enable_gng_vlut'] is enable_gng
    assert args['enable_auto_start'] == 'false'
    assert args['udp_config'] == ''


def test_wrong_model_and_unvalidated_udp_rejected(launch_module):
    with pytest.raises(ValueError, match='機種名が不一致'):
        launch_module.launch_setup(make_context('topodualarm', params_file=str(share / 'config/topo_dual_arm_max.yaml')))
    with pytest.raises(ValueError, match='直動関節'):
        launch_module.launch_setup(make_context('topodualarm', udp_config='unused.yaml'))


def test_prismatic_gripper_poses():
    config = yaml.safe_load((share / 'config/topodualarm_gazebo_demo.yaml').read_text())['dual_arm_gazebo_demo']
    names, poses = load_motion(workspace / 'urdf/dual_arm_urdf/dual_arm_robot.urdf', config)
    assert len(names) == 19
    values = dict(poses)['grippers_open']
    assert values[names.index('L_gripper_joint')] == 0.02
    assert values[names.index('R_gripper_joint')] == 0.02


def test_left_forward_initial_pose_and_invalid_positions():
    module = load('dual_arm_gazebo_demo.launch')
    urdf_path = workspace / 'urdf/dual_arm_urdf/dual_arm_robot.urdf'
    root = et.parse(urdf_path).getroot()
    config = yaml.safe_load((share / 'config/pointcloud_avoidance_topodualarm_left_forward.yaml').read_text())['overrides']
    positions = module.initial_joint_positions(root, config['initial_joint_positions'])
    geometry = robot_geometry(urdf_path)
    values = np.array([positions[name] for name in geometry.joint_names])
    transforms = geometry.link_transforms(values)
    elbow = transforms[geometry.link_indices['L_link4'], :3, 3]
    hand = transforms[geometry.link_indices['L_gripper_base'], :3, 3]
    assert positions['L_joint1'] == pytest.approx(-np.pi/4)
    assert hand[0] > .2
    assert hand[1] == pytest.approx(elbow[1])
    assert np.arctan2(hand[2]-elbow[2], hand[0]-elbow[0]) == pytest.approx(-np.pi/4)
    assert geometry.has_internal_clearance(geometry.centers(values))
    assert all(value == 0 for name, value in positions.items() if name != 'L_joint1')
    for invalid in [{'missing': 0}, {'L_joint1': float('nan')}, {'L_joint1': 3}, {'camera_fixed': 0}]:
        with pytest.raises(ValueError):
            module.initial_joint_positions(root, invalid)
    assert all(value == 0 for value in module.initial_joint_positions(root, {}).values())


@pytest.mark.parametrize('sign', [1, -1])
def test_topodualarm_retreat_geometry(sign):
    config = yaml.safe_load((share / 'config/topodualarm_avoidance_demo.yaml').read_text())['dual_arm_avoidance_demo']
    geometry = robot_geometry(workspace / 'urdf/dual_arm_urdf/dual_arm_robot.urdf')
    home = np.zeros(len(geometry.joint_names))
    positions = home.copy()
    for x in np.linspace(config['hand_far_x'], config['hand_near_x'], 161):
        hand = np.array([x, sign * config['hand_y'], config['hand_z']])
        elbow = hand + [config['arm_length'], 0, 0]
        positions, has_candidate = geometry.choose_step(
            positions, home, hand, elbow, config['arm_radius'], config['target_clearance'], 0.035,
            config['min_planning_clearance_th'])
        assert has_candidate
        gap, centers, _, _ = geometry.clearance(positions, hand, elbow, config['arm_radius'])
        assert geometry.has_internal_clearance(centers, config['min_planning_clearance_th'])
        assert gap > config['min_clearance_th']
    assert geometry.clearance(home, hand, elbow, config['arm_radius'])[0] < 0
    assert max(abs(positions)) > 0.2


def load(name):
    spec = importlib.util.spec_from_file_location(name, share/'launch'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


depth = load('dual_arm_depth_camera_setup')


@pytest.mark.parametrize('name', ['dual_arm_control.launch', 'dual_arm_gazebo_demo.launch'])
def test_launch_description_without_process_start(name):
    """ノード未起動での引数宣言と深度設定パスの検査。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    launch = load(name)
    with patch.object(launch, 'get_package_share_directory', return_value=str(share)):
        description = launch.generate_launch_description()
    context = LaunchContext()
    declarations = {item.name: item for item in description.entities if isinstance(item, DeclareLaunchArgument)}
    assert context.perform_substitution(declarations['point_cloud_source'].default_value[0]) == 'external_lidar'
    assert context.perform_substitution(declarations['depth_camera_config'].default_value[0]) == str(share/'config/dual_arm_depth_camera.yaml')


@pytest.fixture
def config():
    return depth.load_config(share/'config/dual_arm_depth_camera.yaml')


@pytest.mark.parametrize('model', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_mount_and_optical_axes(model, config):
    root = et.parse(share.parent/'urdf'/model/'topo_dual_arm_max.urdf').getroot()
    depth.add_depth_camera(root, 'sim_'+model, config)
    joint = root.find("joint[@name='head_depth_link_fixed']")
    assert joint.find('parent').get('link') == 'camera_link'
    assert joint.find('origin').get('xyz') == '0.0 0.0 0.0'
    optical = root.find("joint[@name='head_depth_optical_frame_fixed']/origin")
    rotation = Rotation.from_euler('xyz', list(map(float, optical.get('rpy').split()))).as_matrix()
    np.testing.assert_allclose(rotation@[0, 0, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(rotation@[1, 0, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(rotation@[0, 1, 0], [0, 0, -1], atol=1e-12)
    sensor = root.find("gazebo[@reference='head_depth_link']/sensor")
    assert sensor.get('type') == 'depth'
    assert sensor.findtext('plugin/frame_name') == 'sim_'+model+'/head_depth_optical_frame'
    assert sensor.findtext('plugin/min_depth') == str(config['min_range'])
    assert sensor.findtext('plugin/max_depth') == str(config['max_range'])
    assert sensor.findtext('plugin/ros/namespace') == '/sim_'+model
    assert 'camera/points:=camera/depth/points' in [x.text for x in sensor.findall('plugin/ros/remapping')]


def test_calibration_and_duplicate_rejection(config):
    root = et.fromstring('<robot><link name="camera_link"/></robot>')
    config.update(xyz=[0.01, 0.02, 0.03], rpy=[0.1, 0.2, 0.3])
    depth.add_depth_camera(root, 'sim_test', config)
    origin = root.find("joint[@name='head_depth_link_fixed']/origin")
    assert origin.get('xyz') == '0.01 0.02 0.03'
    assert origin.get('rpy') == '0.1 0.2 0.3'
    with pytest.raises(ValueError):
        depth.add_depth_camera(root, 'sim_test', config)
    with pytest.raises(ValueError):
        depth.add_depth_camera(et.fromstring('<robot/>'), 'sim_test', config)


@pytest.mark.parametrize('name,value', [
    ('width', True), ('height', 1), ('width', 4097), ('height', 12.5),
    ('update_hz', 0), ('update_hz', math.nan), ('min_range', -1), ('max_range', 0.05),
    ('horizontal_fov', math.pi), ('horizontal_fov', math.inf), ('xyz', [0, 0]),
    ('rpy', [0, math.nan, 0]), ('xyz', [False, 0, 0]), ('unknown', 1)])
def test_invalid_config(tmp_path, config, name, value):
    config[name] = value
    path = tmp_path/'camera.yaml'
    path.write_text(yaml.safe_dump({'head_depth_camera': config}))
    with pytest.raises(ValueError):
        depth.load_config(path)


@pytest.mark.parametrize('content', ['', '[]', 'head_depth_camera: []', 'head_depth_camera: {}'])
def test_missing_config(tmp_path, content):
    path = tmp_path/'camera.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        depth.load_config(path)


@pytest.mark.parametrize('source', ['external_lidar', 'head_depth'])
def test_pipeline_selection(tmp_path, source):
    pipeline = load('dual_arm_lidar_setup')
    model_dir = tmp_path/'model'
    model_dir.mkdir()
    for name in ('gng.bin', 'vlut.bin'):
        (model_dir/name).touch()
    params = {'gng': {'data_directory': str(tmp_path), 'experiment_id': 'model'}}
    with patch.object(pipeline, 'Node', side_effect=lambda **kwargs: kwargs):
        nodes = pipeline.pipeline_nodes('params.yaml', params, 'sim_test', {'danger_inflation': 0.12}, source)
    assert sum(n['executable'] == 'static_transform_publisher' for n in nodes) == int(source == 'external_lidar')
    voxel = next(n for n in nodes if n['executable'] == 'world_index_to_voxel_node')['parameters'][-1]
    assert voxel['input_topic'] == ('camera/depth/points' if source == 'head_depth' else 'lidar_points')
    assert voxel['allow_latest_transform'] == (source == 'external_lidar')
    with pytest.raises(ValueError):
        pipeline.point_cloud_topic('unknown')


@pytest.mark.parametrize('model', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_harmonic_effort_limits_and_passive_mimics(tmp_path, model):
    """実URDF上限と標準PID飽和の一致、元モデルの保持、mimic指令の禁止。"""
    launch = load('dual_arm_gz.launch')
    urdf = workspace/'urdf'/model/'topo_dual_arm_max.urdf'
    before = urdf.read_bytes()
    tuning = yaml.safe_load((share/'config/dual_arm_effort.yaml').read_text())
    generated = launch.prepare_model(urdf, tmp_path, 'sim_test', tuning)
    assert urdf.read_bytes() == before
    root = et.parse(generated).getroot()
    params = yaml.safe_load((tmp_path/'controllers.yaml').read_text())
    controller = params['/sim_test/dual_arm_controller']['ros__parameters']
    assert controller['command_interfaces'] == ['effort']
    assert controller['state_interfaces'] == ['position', 'velocity']
    assert 'gazebo_ros2_control' not in generated.read_text()
    assert root.findtext('ros2_control/hardware/plugin') == 'gz_ros2_control/GazeboSimSystem'
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        control = root.find(f"ros2_control/joint[@name='{name}']")
        if joint.find('mimic') is not None:
            assert control.find('command_interface') is None and name not in controller['joints']
            continue
        effort = float(joint.find('limit').get('effort'))
        gains = controller['gains'][name]
        assert gains['u_clamp_min'] == -effort and gains['u_clamp_max'] == effort
        assert float(control.findtext("command_interface/param[@name='max']")) == effort
        assert float(control.findtext("command_interface/param[@name='min']")) == -effort
    for mesh in root.findall('.//mesh'):
        assert Path(mesh.get('filename').removeprefix('file://')).is_file()


def test_harmonic_standard_topic_remapping(tmp_path):
    """標準トピックの変更先をGazeboとcontroller_managerへ共有。"""
    module = load('dual_arm_gz.launch')
    tuning = yaml.safe_load((share/'config/dual_arm_effort.yaml').read_text())
    topics = {'joint_states': '/robot_io/states', 'robot_description': '/robot_io/description',
              'dual_arm_controller/joint_trajectory': '/robot_io/trajectory'}
    generated = module.prepare_model(workspace/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf',
                                     tmp_path, 'sim_test', tuning, topics)
    root = et.parse(generated).getroot()
    assert {item.text for item in root.findall('gazebo/plugin/ros/remapping')} == {
        name+':='+target for name, target in topics.items()}


@pytest.mark.parametrize('enable_shared', [False, True])
@pytest.mark.parametrize('enable_world_index', [False, True])
def test_viewer_shared_roi_replaces_duplicate_filter(tmp_path, enable_shared, enable_world_index):
    """直接登録・共有world索引とCPU GNGの同居、入力とマスクの共有。"""
    from launch.actions import DeclareLaunchArgument
    module = load('gng_viewer_bridge.launch')
    params = tmp_path/'params.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'robot_name': 'sim_test',
        'urdf_path': str(workspace/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf'),
        'self_recognition': {'enable': True, 'enable_environment_self_filter': True},
        'environment_voxelization': {'enable': True, 'world_index': {
            'enable_build': enable_world_index, 'frame_id': 'world', 'bucket_size': .3}},
    }}}))
    context = LaunchContext()
    def package_path(name):
        return str(workspace/'ais_gng_cpu/src/ais_gng') if name == 'ais_gng' else str(share)
    with patch.object(module, 'get_package_share_directory', side_effect=package_path):
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        assert context.launch_configurations['enable_environment_world_index'] == 'false'
        context.launch_configurations.update({
            'params_file': str(params), 'robot_name': 'sim_test',
            'enable_environment_voxelization': 'true', 'enable_self_recognition_viz': 'true',
            'enable_shared_roi_gng': str(enable_shared).lower(),
            'enable_environment_world_index': '',
            'environment_input_topic': '/test/points',
        })
        with patch.object(module, 'Node', side_effect=lambda **kw: kw), \
                patch.object(module, 'ComposableNode', side_effect=lambda **kw: kw), \
                patch.object(module, 'ComposableNodeContainer', side_effect=lambda **kw: kw):
            actions = module.launch_setup(context)
    nodes = [action for action in actions if isinstance(action, dict)]
    filters = [node for node in nodes if node.get('executable') == 'self_voxel_filter_node']
    containers = [node for node in nodes if 'composable_node_descriptions' in node]
    assert len(filters) == int(not enable_shared)
    assert len(containers) == int(enable_shared)
    if enable_shared:
        roi, gng = containers[0]['composable_node_descriptions']
        roi_params, gng_params = roi['parameters'][0], gng['parameters'][0]
        assert roi_params['shared_point_store'] == gng_params['input.shared_point_store']
        assert roi_params['input_topic'] == '/test/points'
        assert roi_params['enable_world_index'] == enable_world_index
        assert roi_params['enable_roi_query'] == enable_world_index
        assert roi_params['world_frame_id'] == ('world' if enable_world_index else 'sim_test/base_link')
        assert roi_params['bucket_size'] == .3
        assert not roi_params['allow_latest_transform']
        assert gng_params['self_filter.mask_topic'] == ''


def test_viewer_external_measured_state(tmp_path):
    """外部実測の直接購読とTF・初回姿勢・制御出力の二重起動防止。"""
    from launch.actions import DeclareLaunchArgument
    module = load('gng_viewer_bridge.launch')
    params = tmp_path/'params.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'robot_name': 'sim_test', 'urdf_path': str(workspace/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf')}}}))
    context = LaunchContext()
    with patch.object(module, 'get_package_share_directory', return_value=str(share)):
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update({
            'params_file': str(params), 'robot_name': 'sim_test',
            'joint_control_backend': 'external', 'state_topic': '/robot_io/states',
            'enable_robot_state_publisher': 'false', 'use_sim_time': 'true'})
        with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
            actions = module.launch_setup(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    spawn = next(action for action in includes if 'joint_state_topic' in dict(action.launch_arguments))
    assert not spawn.condition.evaluate(context)
    assert dict(spawn.launch_arguments)['publish_initial_joint_state'] == 'false'
    control = next(action for action in includes if 'backend' in dict(action.launch_arguments))
    assert not control.condition.evaluate(context)
    viewer = next(action for action in actions if isinstance(action, dict) and action.get('executable') == 'robot_viewer_bridge_node')
    assert any(isinstance(params, dict) and params.get('joint_state_topic') == '/robot_io/states'
               for params in viewer['parameters'])


@pytest.mark.parametrize('model', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_isaac_effort_contract_matches_harmonic(tmp_path, model):
    """両バックエンドの関節・制御方式・トルク制限の同一契約。"""
    isaac = load('dual_arm_isaac')
    harmonic = load('dual_arm_gz.launch')
    urdf = workspace/'urdf'/model/'topo_dual_arm_max.urdf'
    before = urdf.read_bytes()
    tuning = yaml.safe_load((share/'config/dual_arm_effort.yaml').read_text())
    root, gains, path, params_path = isaac.prepare_config(urdf, tmp_path/'isaac', tuning)
    harmonic.prepare_model(urdf, tmp_path, 'sim_test', tuning)
    expected = yaml.safe_load((tmp_path/'controllers.yaml').read_text())
    actual = yaml.safe_load(params_path.read_text())
    assert actual == {name.removeprefix('/sim_test/'): value for name, value in expected.items()}
    assert all(j.get('name') not in gains for j in root.findall('joint') if j.find('mimic') is not None)
    assert not et.parse(path).getroot().findall('gazebo/plugin')
    assert urdf.read_bytes() == before


@pytest.mark.parametrize('effort', ['0', '-1', 'nan', 'inf'])
def test_sim_effort_rejects_invalid_limit(effort):
    from dual_arm_effort_config import effort_joints
    root = et.fromstring(f'<robot><joint name="motor" type="revolute"><limit effort="{effort}"/></joint></robot>')
    with pytest.raises(ValueError, match='トルク上限'):
        effort_joints(root, {'default_gains': {'p': 1.0}})


def test_isaac_connection_uses_external_description():
    """Isaac合成URDFの購読と標準controller起動。二重状態配信なし。"""
    module = load('dual_arm_isaac.launch')
    context = LaunchContext()
    context.launch_configurations['namespace'] = 'sim_test'
    with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
        nodes = module.launch_setup(context)
    state = next(node for node in nodes if node['package'] == 'robot_state_publisher')
    assert state['parameters'] == [{'use_sim_time': True, 'use_robot_description_topic': True}]
    assert state['namespace'] == 'sim_test'
    spawner = next(node for node in nodes if node['package'] == 'controller_manager')
    assert '/sim_test/controller_manager' in spawner['arguments']
    assert all(node['package'] not in ('ros_gz_sim', 'ros_gz_bridge') for node in nodes)


def test_isaac_usd_force_limits_and_mimic():
    """実USD APIでの力制御・上限設定とmimic二重駆動の排除。"""
    pytest.importorskip('pxr.Usd')
    from pxr import Usd, UsdGeom, UsdPhysics
    module = load('dual_arm_isaac')
    stage = Usd.Stage.CreateInMemory()
    prim = UsdGeom.Xform.Define(stage, '/Robot').GetPrim()
    base = UsdGeom.Xform.Define(stage, '/Robot/base').GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(base)
    motor = UsdPhysics.RevoluteJoint.Define(stage, '/Robot/motor').GetPrim()
    follower = UsdPhysics.RevoluteJoint.Define(stage, '/Robot/follower').GetPrim()
    for item in (motor, follower):
        drive = UsdPhysics.DriveAPI.Apply(item, 'angular')
        drive.CreateStiffnessAttr(100.0)
        drive.CreateDampingAttr(10.0)
    model = et.fromstring('<robot><joint name="motor" type="revolute"><limit effort="0.5"/></joint>'
                          '<joint name="follower" type="revolute"><mimic joint="motor"/></joint></robot>')
    from dual_arm_effort_config import effort_joints
    gains = effort_joints(model, {'default_gains': {'p': 20.0, 'd': 2.0}})
    assert module.configure_drives(stage, '/Robot', model, gains) == '/Robot'
    drive = UsdPhysics.DriveAPI(motor, 'angular')
    assert drive.GetTypeAttr().Get() == 'force'
    assert drive.GetMaxForceAttr().Get() == .5
    assert drive.GetStiffnessAttr().Get() == 0.0
    assert drive.GetDampingAttr().Get() == 0.0
    assert not follower.HasAPI(UsdPhysics.DriveAPI, 'angular')
    UsdPhysics.RevoluteJoint.Define(stage, '/Robot/unknown')
    with pytest.raises(ValueError, match='不一致'):
        module.configure_drives(stage, '/Robot', model, gains)


@pytest.mark.parametrize('enable_observer', [False, True])
def test_joint_motion_observer_is_opt_in(enable_observer):
    """無効時の空起動と、有効時の独立した観測ノード。"""
    from launch.actions import DeclareLaunchArgument

    module = load('joint_motion_state.launch')
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['enable_joint_motion_state'] == 'false'
    context.launch_configurations.update(enable_joint_motion_state=str(enable_observer).lower(),
                                         namespace='sim_observer', use_sim_time='true')
    with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
        actions = module.launch_setup(context)
    if not enable_observer:
        assert actions == []
        return
    assert len(actions) == 1
    assert actions[0]['executable'] == 'joint_motion_observer.py'
    assert actions[0]['namespace'] == 'sim_observer'
    assert actions[0]['parameters'][0]['use_sim_time'] is True
    assert actions[0]['parameters'][0]['max_derivative_order'] == 3


@pytest.mark.parametrize('scenario_name', [path.stem for path in sorted((share/'config/simulation/scenarios').glob('*.yaml'))])
def test_shared_environment_shapes_physics_and_replay(tmp_path, scenario_name):
    """共通物体のSDF・USD形状、物理属性、姿勢と保存設定の再読込。"""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    from simulation_scenario import load_scenario, save_scenario, gazebo_world, add_isaac_environment, diagonal_inertia

    scenario = load_scenario(scenario_name)
    save_scenario(scenario, tmp_path)
    assert load_scenario(tmp_path/'scenario.yaml') == scenario
    world = gazebo_world(scenario).find('world')
    assert world.findtext('gravity') == '0 0 -9.81'
    assert float(world.findtext('physics/max_step_size')) == 0.001
    assert len(world.findall('model')) == len(scenario['objects'])
    stage = Usd.Stage.CreateInMemory()
    add_isaac_environment(stage, scenario)
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    for item in scenario['objects']:
        model = world.find("model[@name='environment_" + item['name'] + "']")
        assert model.findtext('static') == str(item['is_static']).lower()
        np.testing.assert_allclose(list(map(float, model.findtext('pose').split())), item['position'] + item['rpy'])
        body = stage.GetPrimAtPath('/World/Environment/' + item['name'])
        shape = stage.GetPrimAtPath(str(body.GetPath()) + '/shape')
        transform = np.array(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
        np.testing.assert_allclose(transform[:3, 3], item['position'])
        np.testing.assert_allclose(transform[:3, :3], Rotation.from_euler('xyz', item['rpy']).as_matrix(), atol=1e-7)
        assert shape.HasAPI(UsdPhysics.CollisionAPI)
        assert body.HasAPI(UsdPhysics.RigidBodyAPI) is (not item['is_static'])
        geometry = model.find('link/collision/geometry')
        assert geometry.find(item['shape']) is not None
        if item['shape'] == 'box':
            assert UsdGeom.Cube(shape).GetSizeAttr().Get() == 1.0
            np.testing.assert_allclose(shape.GetAttribute('xformOp:scale').Get(), item['size'])
        elif item['shape'] == 'sphere':
            assert UsdGeom.Sphere(shape).GetRadiusAttr().Get() == item['radius']
        elif item['shape'] == 'cylinder':
            assert UsdGeom.Cylinder(shape).GetHeightAttr().Get() == item['length']
            assert UsdGeom.Cylinder(shape).GetRadiusAttr().Get() == item['radius']
        material, _ = UsdShade.MaterialBindingAPI(shape).ComputeBoundMaterial('physics')
        assert UsdPhysics.MaterialAPI(material).GetStaticFrictionAttr().Get() == pytest.approx(item['friction'])
        if not item['is_static']:
            assert UsdPhysics.MassAPI(body).GetMassAttr().Get() == pytest.approx(item['mass'])
            np.testing.assert_allclose(UsdPhysics.MassAPI(body).GetDiagonalInertiaAttr().Get(), diagonal_inertia(item))
            assert float(model.findtext('link/inertial/mass')) == item['mass']
    with pytest.raises(ValueError, match='既に存在'):
        add_isaac_environment(stage, scenario)


def test_environment_external_library_and_rotated_sphere(tmp_path):
    """外部シナリオの相対参照と球形物体の物理属性。"""
    from pxr import Usd, UsdGeom, UsdPhysics
    from simulation_scenario import load_scenario, gazebo_world, add_isaac_environment

    (tmp_path/'objects.yaml').write_text(yaml.safe_dump({'ball': {
        'shape': 'sphere', 'radius': 0.1, 'is_static': False, 'mass': 2.0}}))
    path = tmp_path/'case.yaml'
    path.write_text(yaml.safe_dump({'description': '球の確認', 'objects_file': 'objects.yaml', 'objects': [
        {'name': 'ball', 'asset': 'ball', 'position': [0.1, 0.2, 0.3], 'rpy': [0.2, -0.3, 0.4]}]}))
    scenario = load_scenario(path)
    stage = Usd.Stage.CreateInMemory()
    add_isaac_environment(stage, scenario)
    body = stage.GetPrimAtPath('/World/Environment/ball')
    np.testing.assert_allclose(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
        np.block([[Rotation.from_euler('xyz', [0.2, -0.3, 0.4]).as_matrix(), np.array([[0.1], [0.2], [0.3]])],
                  [np.array([[0.0, 0.0, 0.0, 1.0]])]]).T, atol=1e-7)
    assert UsdGeom.Sphere(stage.GetPrimAtPath('/World/Environment/ball/shape')).GetRadiusAttr().Get() == 0.1
    np.testing.assert_allclose(UsdPhysics.MassAPI(body).GetDiagonalInertiaAttr().Get(), [0.008] * 3)
    assert float(gazebo_world(scenario).findtext('world/model/link/inertial/inertia/ixx')) == pytest.approx(0.008)


@pytest.mark.parametrize('change', [
    {'shape': 'mesh'}, {'shape': ['box']}, {'size': [0.1, 0.2]}, {'size': [0.1, -0.2, 0.3]},
    {'size': [0.1, float('nan'), 0.3]}, {'mass': 0.0}, {'mass': float('inf')},
    {'is_static': 'false'}, {'color': [2.0, 0.0, 0.0]}, {'friction': -1.0}, {'typo': 1}])
def test_environment_invalid_asset_rejected(tmp_path, change):
    """寸法・質量・型・未知設定の起動前拒否。"""
    from simulation_scenario import load_scenario

    (tmp_path/'objects.yaml').write_text(yaml.safe_dump({'object': {
        'shape': 'box', 'size': [0.1, 0.2, 0.3], 'is_static': False, 'mass': 1.0, **change}}))
    (tmp_path/'case.yaml').write_text(yaml.safe_dump({
        'description': '入力検査', 'objects_file': 'objects.yaml', 'objects': []}))
    with pytest.raises(ValueError):
        load_scenario(tmp_path/'case.yaml')


@pytest.mark.parametrize('objects', [
    [{'name': 'item', 'asset': 'missing', 'position': [0, 0, 0]}],
    [{'name': 'bad/name', 'asset': 'floor', 'position': [0, 0, 0]}],
    [{'name': 'item', 'asset': 'floor', 'position': [0, 0, float('inf')]}],
    [{'name': 'item', 'asset': 'floor', 'position': [0, 0, 0], 'velocity': [1, 0, 0]}],
    [{'name': 'item', 'asset': 'floor', 'position': [0, 0, 0]}] * 2])
def test_environment_invalid_placement_rejected(tmp_path, objects):
    """物体参照・名前・重複配置・非有限姿勢・未対応動作の拒否。"""
    from simulation_scenario import load_scenario

    (tmp_path/'case.yaml').write_text(yaml.safe_dump({'description': '配置検査',
        'objects_file': str(share/'config/simulation/objects.yaml'), 'objects': objects}))
    with pytest.raises(ValueError):
        load_scenario(tmp_path/'case.yaml')


@pytest.mark.parametrize('model', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_environment_clear_of_initial_robot_pose(model):
    """全シナリオの初期姿勢に対する外接球と保守的な障害物箱の非干渉。"""
    from simulation_scenario import load_scenario, scenario_dir

    geometry = robot_geometry(workspace/'urdf'/model/'topo_dual_arm_max.urdf')
    centers = geometry.centers(np.zeros(len(geometry.joint_names)))
    for path in scenario_dir.glob('*.yaml'):
        for item in load_scenario(path)['objects']:
            local = (centers - item['position']) @ Rotation.from_euler('xyz', item['rpy']).as_matrix()
            size = (item['size'] if item['shape'] == 'box' else
                    [2 * item['radius'], 2 * item['radius'], item.get('length', 2 * item['radius'])])
            nearest = np.maximum(np.abs(local) - np.array(size) / 2, 0)
            assert np.all(np.linalg.norm(nearest, axis=1) > geometry.radii), (model, path.name, item['name'])


def test_harmonic_environment_launch_selection(tmp_path):
    """環境選択の起動経路と既定の空環境。制御設定との分離。"""
    from launch.actions import DeclareLaunchArgument
    from simulation_scenario import load_scenario

    module = load('dual_arm_gz.launch')
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['scenario'] == 'empty'
    context.launch_configurations.update(scenario='tabletop', output_dir=str(tmp_path))
    with patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs), patch.object(module, 'ExecuteProcess', side_effect=lambda **kwargs: kwargs):
        actions = module.launch_setup(context)
    assert actions[0]['cmd'][0:3] == ['gz', 'sim', '-s']
    assert len(et.parse(tmp_path/'world.sdf').findall('world/model')) == 4
    assert load_scenario(tmp_path/'scenario.yaml') == load_scenario('tabletop')


@pytest.mark.skipif(os.environ.get('GZ_PARTITION') != 'uraki_rolling_check',
                    reason='実物理試験は専用GZ_PARTITION=uraki_rolling_checkのみ')
@pytest.mark.parametrize('scenario_name', ['rolling_ball', 'rolling_balls'])
def test_rolling_environment_physics(tmp_path, scenario_name):
    """重力による往復・回転・接触点の滑りと、試験所有プロセスの終了確認。"""
    import json
    import subprocess
    import time
    from check_gazebo_software_stop import owned_launch, process_snapshot, save_json
    from simulation_scenario import load_scenario, gazebo_world

    scenario = load_scenario(scenario_name)
    world = tmp_path/'world.sdf'
    world.write_text(et.tostring(gazebo_world(scenario), encoding='unicode'))
    checked = subprocess.run(['gz', 'sdf', '-k', str(world)], capture_output=True, text=True, timeout=15)
    assert checked.returncode == 0, checked.stderr
    launch = owned_launch(tmp_path, process_snapshot())
    collector = None
    report = {'scenario': scenario_name, 'result': 'failed', 'balls': {}}
    try:
        launch.start(['gz', 'sim', '-s', str(world)])
        deadline = time.monotonic() + 30
        while True:
            result = subprocess.run(['gz', 'service', '-s', '/world/motor_test/scene/info',
                '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '1000', '--req', ''],
                capture_output=True, text=True, timeout=5)
            if result.returncode == 0 and 'environment_' in result.stdout:
                break
            assert time.monotonic() < deadline, '環境起動の時間切れ'
        with (tmp_path/'poses.jsonl').open('w') as stream:
            collector = subprocess.Popen(['gz', 'topic', '-e', '-d', '12', '--json-output',
                '-t', '/world/motor_test/pose/info'], stdout=stream, stderr=subprocess.PIPE, text=True)
            time.sleep(0.5)
            result = subprocess.run(['gz', 'service', '-s', '/world/motor_test/control',
                '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000', '--req', 'pause: false'], capture_output=True, text=True, timeout=5)
            assert result.returncode == 0 and 'true' in result.stdout
            _, error = collector.communicate(timeout=20)
            assert collector.returncode == 0, error
        messages = [json.loads(line) for line in (tmp_path/'poses.jsonl').read_text().splitlines() if line.strip()]
        for item in scenario['objects']:
            if item['is_static']:
                continue
            samples = []
            for message in messages:
                stamp = message.get('header', {}).get('stamp', {})
                stamp_sec = int(stamp.get('sec', 0)) + stamp.get('nsec', 0)*1e-9
                pose = next((pose for pose in message.get('pose', []) if pose.get('name') == 'environment_' + item['name']), None)
                if pose is None or (samples and stamp_sec <= samples[-1][0]):
                    continue
                position, orientation = pose.get('position', {}), pose.get('orientation', {})
                samples.append([stamp_sec, *[position.get(axis, 0) for axis in ('x', 'y', 'z')],
                                *[orientation.get(axis, 0) for axis in ('x', 'y', 'z')], orientation.get('w', 0)])
            values = np.array(samples)
            assert len(values) > 100
            period = np.diff(values[:, 0])
            velocity = np.diff(values[:, 1:4], axis=0) / period[:, None]
            rotation = Rotation.from_quat(values[:, 4:8])
            angular_velocity = (rotation[1:] * rotation[:-1].inv()).as_rotvec() / period[:, None]
            middle = (values[:-1, :4] + values[1:, :4]) / 2
            side = np.sign(middle[:, 2])
            normal = np.column_stack((np.zeros(len(side)), -side*math.sin(0.1), np.full(len(side), math.cos(0.1))))
            slip = np.linalg.norm(velocity - item['radius'] * np.cross(angular_velocity, normal), axis=1)
            surface_z = 0.2 + abs(middle[:, 2])*math.tan(0.1) + item['radius']/math.cos(0.1)
            is_rolling = ((middle[:, 0] > 0.2) & (abs(middle[:, 2]) > 0.15) &
                          (abs(middle[:, 2]) < 0.8) & (abs(middle[:, 3]-surface_z) < 0.005) &
                          (abs(velocity[:, 1]) > 0.06))
            assert np.count_nonzero(is_rolling) > 20
            metrics = {'min_y_m': float(values[:, 2].min()), 'max_y_m': float(values[:, 2].max()),
                       'min_velocity_y_m_sec': float(velocity[:, 1].min()),
                       'max_velocity_y_m_sec': float(velocity[:, 1].max()),
                       'max_angular_velocity_rad_sec': float(np.linalg.norm(angular_velocity, axis=1).max()),
                       'contact_slip_percentile_90_m_sec': float(np.percentile(slip[is_rolling], 90))}
            report['balls'][item['name']] = metrics
            assert metrics['min_y_m'] < -0.1 and metrics['max_y_m'] > 0.1
            assert metrics['min_velocity_y_m_sec'] < -0.05 and metrics['max_velocity_y_m_sec'] > 0.05
            assert metrics['max_angular_velocity_rad_sec'] > 1.0
            assert metrics['contact_slip_percentile_90_m_sec'] < 0.05
        report['result'] = 'passed'
    finally:
        if collector is not None and collector.poll() is None:
            collector.terminate()
            try:
                collector.wait(timeout=3)
            except subprocess.TimeoutExpired:
                collector.kill()
                collector.wait(timeout=3)
        report['cleanup'] = launch.cleanup()
        save_json(tmp_path/'rolling_report.json', report)
        assert report['cleanup']['is_success']
