"""任意の関節名・異なるグループ関節数・入力次元とセンサー構成の検証。"""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as et

import numpy as np
import pytest
from scipy.spatial import cKDTree
import yaml

share = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(share / 'scripts'))
sys.path.insert(0, str(share / 'launch'))
from pointcloud_avoidance_config import load_config, gng_angle_num, resolve_clearance_margins
from dual_arm_avoidance_geometry import robot_geometry
from dual_arm_gng_lidar_demo import gng_lidar_demo
from gng_avoidance_planner import gng_path_search
import dual_arm_lidar_setup as lidar_setup


def write_gng(path, num):
    path.write_bytes(struct.pack('<IIiiffqq', 9, 1, 1, 0, 0., 0., num, 1))


@pytest.fixture
def robot_config(tmp_path):
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('''<robot name="fixture"><link name="mount"/>
      <link name="upper"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>
      <link name="tool"><collision><geometry><cylinder radius="0.02" length="0.1"/></geometry></collision></link>
      <link name="cover"><collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision></link>
      <link name="slider"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>
      <joint name="shoulder" type="revolute"><parent link="mount"/><child link="upper"/>
        <origin xyz="0 0.3 0.4"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
      <joint name="elbow" type="revolute"><parent link="upper"/><child link="tool"/>
        <origin xyz="0.15 0 0"/><limit lower="-1" upper="1"/></joint>
      <joint name="cover_fixed" type="fixed"><parent link="tool"/><child link="cover"/></joint>
      <joint name="slide" type="prismatic"><parent link="mount"/><child link="slider"/>
        <origin xyz="0 -0.3 0.4"/><axis xyz="1 0 0"/><limit lower="0" upper="0.2"/></joint></robot>''')
    common = yaml.safe_load((share / 'config/pointcloud_avoidance_common.yaml').read_text())
    # 手動調整中の実機設定に依存しない、試験用の有効な距離余裕
    common['clearance_margins'] = {'min_clearance_th': .01, 'min_internal_clearance_th': .005,
                                 'min_planning_clearance_th': .01}
    (tmp_path / 'pointcloud_avoidance_common.yaml').write_text(yaml.safe_dump(common))
    write_gng(tmp_path / 'gng.bin', 3)
    (tmp_path / 'vlut.bin').write_bytes(b'fixture')
    params = {'robot_name': 'example_robot', 'urdf_path': str(urdf),
              'gng': {'data_directory': str(tmp_path), 'experiment_id': '.'}}
    (tmp_path / 'params.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    value = {'params_file': 'params.yaml', 'planning_groups': [
        {'name': 'manipulator', 'joint_names': ['elbow', 'shoulder'], 'link_names': ['upper']},
        {'name': 'carriage', 'joint_names': ['slide'], 'link_names': ['slider']}]}
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(value))
    return path, value


def test_arbitrary_root_order_and_descendants(robot_config):
    path, _ = robot_config
    _, params, config = load_config(path)
    assert config['root_link'] == config['pipeline']['base_frame'] == 'mount'
    assert config['planning_groups'][0]['joint_names'] == ['elbow', 'shoulder']
    assert config['planning_groups'][0]['link_names'] == ['cover', 'tool', 'upper']
    geometry = robot_geometry(params['urdf_path'], config['planning_groups'])
    assert geometry.root_link == 'mount'
    assert geometry.arm_indices == [1, 0, 2]
    assert geometry.is_arm.all()
    assert np.isfinite(geometry.centers(np.zeros(3))).all()


def test_single_cloud_margin_after_robot_and_input_overrides(robot_config, tmp_path):
    path, value = robot_config
    value['overrides'] = {'clearance_margins': {'min_clearance_th': .025}}
    path.write_text(yaml.safe_dump(value))
    overlay = tmp_path/'input.yaml'
    overlay.write_text(yaml.safe_dump({'clearance_margins': {'min_clearance_th': .03}}))
    _, _, config = load_config(path, overlay)
    assert config['min_clearance_th'] == config['min_cloud_clearance_th'] == .03
    assert config['min_internal_clearance_th'] == .005
    assert config['min_planning_clearance_th'] == .01


@pytest.mark.parametrize('value', [0, -.01, float('nan'), float('inf'), True, '0.01'])
def test_invalid_margin_rejected(value):
    config = {'target_clearance': .05, 'clearance_margins': {
        'min_clearance_th': value, 'min_internal_clearance_th': .005, 'min_planning_clearance_th': .01}}
    with pytest.raises(ValueError, match='有限の正数'):
        resolve_clearance_margins(config)


@pytest.mark.parametrize('override', [
    {'min_clearance_th': .06}, {'min_internal_clearance_th': .02}, {'unknown_margin': .01},
])
def test_inconsistent_margins_rejected(override):
    config = {'target_clearance': .05, 'clearance_margins': {
        'min_clearance_th': .01, 'min_internal_clearance_th': .005, 'min_planning_clearance_th': .01}}
    config['clearance_margins'].update(override)
    with pytest.raises(ValueError):
        resolve_clearance_margins(config)


def test_legacy_duplicate_margin_in_overlay_rejected(robot_config, tmp_path):
    path, _ = robot_config
    overlay = tmp_path/'input.yaml'
    overlay.write_text('min_cloud_clearance_th: 0.04\n')
    with pytest.raises(ValueError, match='重複設定'):
        load_config(path, overlay)


@pytest.mark.parametrize('kind', ['unknown', 'duplicate', 'fixed', 'overlap', 'dimension', 'missing', 'roi', 'pose', 'samples'])
def test_invalid_robot_rejected(robot_config, kind):
    path, value = robot_config
    if kind in ('unknown', 'duplicate', 'fixed'):
        value['planning_groups'][0]['joint_names'][0] = {
            'unknown': 'absent', 'duplicate': 'shoulder', 'fixed': 'cover_fixed'}[kind]
    elif kind == 'overlap':
        value['planning_groups'][1]['link_names'].append('cover')
    elif kind == 'dimension':
        write_gng(path.parent / 'gng.bin', 14)
    elif kind == 'missing':
        (path.parent / 'vlut.bin').unlink()
    else:
        value['overrides'] = {'pipeline': {
            'roi': {'roi_min': [1, 2, 3]}, 'pose': {'lidar': {'pose': [0, 0, float('nan'), 0, 0, 0]}},
            'samples': {'lidar': {'horizontal_samples': 1}}}[kind]}
    path.write_text(yaml.safe_dump(value))
    with pytest.raises((ValueError, FileNotFoundError)):
        load_config(path)


def test_truncated_gng_rejected(tmp_path):
    path = tmp_path / 'gng.bin'
    path.write_bytes(struct.pack('<I', 9) + bytes(28))
    with pytest.raises(ValueError, match='ヘッダー'):
        gng_angle_num(path)


@pytest.mark.parametrize('shape', ['<sphere radius="nan"/>', '<sphere radius="-1"/>',
                                 '<cylinder radius="0.1" length="0"/>', '<cylinder radius="inf" length="1"/>'])
def test_invalid_primitive_rejected(tmp_path, shape):
    path = tmp_path / 'invalid.urdf'
    path.write_text('<robot name="invalid"><link name="mount"><collision><geometry>'
                    + shape + '</geometry></collision></link></robot>')
    with pytest.raises(ValueError, match='寸法'):
        robot_geometry(path)


@pytest.mark.parametrize('num', [3, 7, 14])
def test_feature_dimension_and_finiteness(num):
    errors = []
    target = SimpleNamespace(arm_names=list(range(num)), angles={}, feature_sub=None, fail=errors.append)
    feature = SimpleNamespace(node_id=9, weight_angle=[0.] * num)
    gng_lidar_demo.on_features(target, SimpleNamespace(features=[feature]))
    assert len(target.angles[9]) == num
    feature.weight_angle = [0.] * (num + 1)
    gng_lidar_demo.on_features(target, SimpleNamespace(features=[feature]))
    feature.weight_angle = [float('nan')] * num
    gng_lidar_demo.on_features(target, SimpleNamespace(features=[feature]))
    assert len(errors) == 2


def test_unequal_groups_select_only_threatened_joint_names():
    search = gng_path_search()
    search.arm_names = ['elbow', 'shoulder', 'slide']
    search.arm_indices = [1, 0, 2]
    search.positions = search.home = np.zeros(3)
    search.path = []
    search.planning_groups = [
        {'name': 'arm', 'joint_names': ['elbow', 'shoulder'], 'link_names': ['tool']},
        {'name': 'carriage', 'joint_names': ['slide'], 'link_names': ['slider']}]
    search.geometry = SimpleNamespace(is_arm=np.array([True, True]), radii=np.array([.02, .02]),
        spheres=[('tool', None, .02), ('slider', None, .02)],
        centers=lambda _: np.array([[0., 0., 0.], [1., 0., 0.]]))
    search.cell_radius = .01
    search.cloud_tree = cKDTree([[1.1, 0., 0.]])
    search.config = {'min_retreat_dist_th': .3}
    assert search.select_active_arms().tolist() == [2]
    search.cloud_tree = cKDTree([[.1, 0., 0.]])
    assert search.select_active_arms().tolist() == [0, 1]


def test_sensor_pose_tf_roi_and_external_input(robot_config, monkeypatch):
    path, _ = robot_config
    params_path, params, config = load_config(path)
    monkeypatch.setattr(lidar_setup, 'Node', lambda **values: values)
    pose = [1., 2., 3., .1, .2, .3]
    config['pipeline']['lidar']['pose'] = pose
    config['pipeline']['points_topic'] = '/sensor/points'
    world = et.Element('world')
    lidar_setup.add_lidar(world, 'sim_fixture', config)
    assert list(map(float, world.find('model/pose').text.split())) == pose
    nodes = lidar_setup.pipeline_nodes(params_path, params, 'sim_fixture', config)
    tf = next(node for node in nodes if node['executable'] == 'static_transform_publisher')
    assert [float(tf['arguments'][idx]) for idx in range(1, 12, 2)] == pose
    voxel = next(node for node in nodes if node['executable'] == 'world_index_to_voxel_node')['parameters'][-1]
    assert voxel['reachability_margin_z'] == 0.
    assert voxel['input_topic'] == '/sensor/points'
    assert voxel['target_frame_id'] == 'sim_fixture/mount'
    assert voxel['output_topic'] == 'roi_voxels'
    vlut = next(node for node in nodes if node['executable'] == 'voxel_to_vlut_node')['parameters'][-1]
    assert vlut['input_topic'] == 'self_filter_roi_voxels'
    config['pipeline']['enable_lidar'] = False
    world = et.Element('world')
    lidar_setup.add_lidar(world, 'sim_fixture', config)
    assert not list(world)
    assert all(node['executable'] != 'static_transform_publisher' for node in
               lidar_setup.pipeline_nodes(params_path, params, 'sim_fixture', config))


def test_real_cloud_config_requires_pose_and_real_self_filter(robot_config, monkeypatch):
    path, _ = robot_config
    input_path = path.parent / 'input.yaml'
    input_value = yaml.safe_load((share / 'config/realsense_gazebo_input.yaml').read_text())
    input_value['pipeline']['external_cloud']['robot_camera_link'] = 'tool'
    input_path.write_text(yaml.safe_dump(input_value))
    with pytest.raises(ValueError, match='camera_pose'):
        load_config(path, input_path)
    params_path, params, config = load_config(path, input_path, [0, 0, 1, 0, 0, 0])
    assert config['enable_live_obstacles']
    assert all(type(value) is float for value in config['pipeline']['external_cloud']['camera_pose'])
    monkeypatch.setattr(lidar_setup, 'Node', lambda **values: values)
    nodes = lidar_setup.pipeline_nodes(params_path, params, 'sim_fixture', config)
    names = [node['executable'] for node in nodes]
    assert 'external_pointcloud_bridge.py' in names
    assert 'self_voxel_filter_node' in names
    assert 'self_recognition_viz_node' in names
    recognition = next(node for node in nodes if node['executable'] == 'self_recognition_viz_node')
    assert recognition['namespace'] == 'sim_fixture/real'
    settings = recognition['parameters'][-1]
    assert settings['joint_topic'] == '/sim_fixture/real_joint_states'
    assert settings['self_recognition.target_frame_id'] == '/sim_fixture/mount'
    assert settings['max_joint_state_age_sec'] == .5
    assert settings['self_recognition.mask_topic'] == '/sim_fixture/real/self_voxel'
    simulated = next(node for node in nodes if node['executable'] == 'self_recognition_viz_node' and node['namespace'] == 'sim_fixture')
    assert simulated['parameters'][-1]['joint_topic'] == '/sim_fixture/joint_states'
    assert simulated['parameters'][-1]['self_recognition.mask_topic'] == '/sim_fixture/self_voxel'
    filtering = next(node for node in nodes if node['executable'] == 'self_voxel_filter_node')
    assert filtering['parameters'][-1]['self_recognition.mask_topic'] == '/sim_fixture/real/self_voxel'
    bridge = next(node for node in nodes if node['executable'] == 'external_pointcloud_bridge.py')
    assert bridge['parameters'][-1]['use_sim_time'] is False
    voxel = next(node for node in nodes if node['executable'] == 'world_index_to_voxel_node')['parameters'][-1]
    vlut = next(node for node in nodes if node['executable'] == 'voxel_to_vlut_node')['parameters'][-1]
    assert voxel['output_topic'] == 'roi_voxels'
    assert vlut['input_topic'] == 'self_filter_roi_voxels'
    input_value['pipeline']['enable_self_filter'] = False
    input_path.write_text(yaml.safe_dump(input_value))
    with pytest.raises(ValueError, match='自己除去の省略'):
        load_config(path, input_path, [0, 0, 1, 0, 0, 0])
