"""機体選択・Gazebo起動設定・頭部深度センサー・光学座標の検証。"""
from pathlib import Path
from unittest.mock import patch
import importlib.util
import math
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
