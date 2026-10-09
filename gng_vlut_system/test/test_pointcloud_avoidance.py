"""点群回避の機体設定・環境入力・座標変換・鮮度・欠測拒否の検証。"""
from pathlib import Path
from types import SimpleNamespace
import struct
import sys
import time
import xml.etree.ElementTree as et

from ais_gng_msgs.msg import TopologicalMap, TopologicalNode, TopologicalNodeStates
from scipy.spatial import cKDTree
from sensor_msgs.msg import JointState, PointCloud2, PointField
from voxel_msgs.msg import Voxel
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))

from dual_arm_avoidance_geometry import robot_geometry
from dual_arm_gng_lidar_demo import gng_lidar_demo, graph_topology_hash
from external_pointcloud_bridge import cloud_xyz, transform_xyz, has_fresh_input, external_pointcloud_bridge, pose_matrix, real_root_transform
from gng_avoidance_planner import gng_path_search
from pointcloud_avoidance_config import load_config, gng_angle_num, resolve_clearance_margins
import dual_arm_lidar_setup as lidar_setup


share = Path(__file__).resolve().parents[1]


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


def stamp(message, age=0.0):
    value = time.time_ns()-int(age*1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(value, 1_000_000_000)
    return message


def test_fixed_source_frame_and_existing_self_filter(robot_config, monkeypatch):
    path, _ = robot_config
    params_path = path.parent/'params.yaml'
    params = yaml.safe_load(params_path.read_text())
    params['/**']['ros__parameters'].update(frame_id='mount', environment_voxelization={'input_topic': '/camera/points'})
    params_path.write_text(yaml.safe_dump(params))
    urdf_path = path.parent/'robot.urdf'
    urdf_path.write_text(urdf_path.read_text().replace('<link name="mount"/>', '''<link name="foundation"/>
      <link name="mount"/><joint name="mount_fixed" type="fixed"><parent link="foundation"/>
      <child link="mount"/><origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>'''))
    overlay = path.parent/'input.yaml'
    overlay.write_text(yaml.safe_dump({'enable_live_obstacles': True,
        'pipeline': {'enable_lidar': False, 'external_environment': {'max_input_age_sec': 1.}}}))
    params_path, params, config = load_config(path, overlay)
    source = config['pipeline']['external_environment']
    np.testing.assert_allclose(np.asarray(source['root_from_source']) @ [1, 0, 0, 1], [1, 3, 3, 1])
    assert source['voxel_topic'] == '/example_robot/self_filter_roi_voxels'
    assert source['graph_topic'] == '/example_robot/Tmap_static'
    assert source['state_topic'] == '/example_robot/gng_node_states_stamped'
    monkeypatch.setattr(lidar_setup, 'Node', lambda **values: values)
    nodes = lidar_setup.pipeline_nodes(params_path, params, 'sim_example_robot', config)
    assert len(nodes) == 1 and nodes[0]['executable'] == 'self_recognition_viz_node'
    settings = nodes[0]['parameters'][-1]
    assert settings['joint_topic'] == '/sim_example_robot/joint_states'
    assert settings['self_recognition.mask_topic'] == '/sim_example_robot/self_voxel'
    assert settings['self_recognition.target_frame_id'] == 'foundation'
    assert settings['max_joint_state_age_sec'] == .5
    params['frame_id'] = 'upper'
    params_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    with pytest.raises(ValueError, match='固定リンク'):
        load_config(path, overlay)


@pytest.fixture
def receiver():
    target = gng_lidar_demo.__new__(gng_lidar_demo)
    target.external_environment = {'source_frame': 'actual/base', 'max_input_age_sec': 1.,
        'root_from_source': [[0., -1., 0., 1.], [1., 0., 0., 2.], [0., 0., 1., 3.], [0., 0., 0., 1.]]}
    target.source_stamps = {}
    target.voxel_frame = 'sim/base'
    target.last_voxel_stamp = -1
    target.voxel_time = target.real_joint_time = 0.
    target.graph_ids, target.graph_edges, target.graph_sub = (), None, object()
    target.geometry = SimpleNamespace(joint_names=['waist', 'neck'])
    return target


def test_voxel_transform_and_stale_or_replayed_input(receiver):
    message = stamp(Voxel(voxel_size=.02, x_shift=42, y_shift=21, z_shift=0, offset=0, data=[0]))
    message.header.frame_id = 'actual/base'
    receiver.on_voxels(message)
    np.testing.assert_allclose(receiver.cloud_tree.data, [[.99, 2.01, 3.01]])
    received = receiver.voxel_time
    receiver.on_voxels(message)
    receiver.on_voxels(stamp(message, 3.))
    receiver.on_voxels(stamp(message, -3.))
    assert receiver.voxel_time == received
    stamp(message)
    message.header.frame_id = 'camera'
    receiver.on_voxels(message)
    assert receiver.voxel_time == received


def test_external_graph_keeps_receiving_safety_changes(receiver):
    def graph(label):
        message = stamp(TopologicalMap(nodes=[TopologicalNode(id=9, label=label)]))
        message.header.frame_id = 'actual/base'
        return message
    receiver.on_graph(graph(1))
    assert receiver.graph_sub is not None
    assert receiver.graph_message.header.frame_id == 'sim/base'
    point = receiver.graph_message.nodes[0].pos
    np.testing.assert_allclose([point.x, point.y, point.z], [1, 2, 3])
    receiver.on_graph(graph(2))
    assert receiver.labels == {9: 2}
    receiver.on_graph(graph(1))
    assert receiver.labels == {9: 1}


def test_stamped_states_preserve_freshness_and_topology(receiver):
    receiver.external_environment['state_topic'] = '/actual/states'
    destroyed = []
    receiver.destroy_subscription = destroyed.append
    graph = stamp(TopologicalMap(nodes=[TopologicalNode(id=9, label=1), TopologicalNode(id=12, label=1)], edges=[0, 1]))
    graph.header.frame_id = 'actual/base'
    receiver.on_graph(graph)
    assert receiver.graph_sub is None and len(destroyed) == 1
    message = stamp(TopologicalNodeStates(topology_hash=graph_topology_hash([9, 12], [0, 1]),
                                        node_ids=[9, 12], labels=[2, 3]))
    message.header.frame_id = 'actual/base'
    receiver.on_stamped_states(message)
    assert receiver.labels == {9: 2, 12: 3}
    received = receiver.graph_time
    receiver.on_stamped_states(message)
    receiver.on_stamped_states(stamp(message, 2.))
    receiver.on_stamped_states(stamp(message, -2.))
    assert receiver.graph_time == received
    for kind in ('topology', 'order', 'length', 'label', 'frame'):
        invalid = stamp(TopologicalNodeStates(topology_hash=receiver.graph_topology_hash,
                                             node_ids=[9, 12], labels=[1, 1]))
        invalid.header.frame_id = 'actual/base'
        if kind == 'topology':
            invalid.topology_hash = graph_topology_hash([9, 12], [1, 0])
        elif kind == 'order':
            invalid.node_ids = [12, 9]
        elif kind == 'length':
            invalid.labels = [1]
        elif kind == 'label':
            invalid.labels = [0, 1]
        else:
            invalid.header.frame_id = 'camera'
        receiver.on_stamped_states(invalid)
        assert receiver.graph_time == received and receiver.labels == {9: 2, 12: 3}


def test_first_neighbors_use_measured_angles_and_reject_unknown_labels():
    from scipy.spatial import cKDTree
    target = SimpleNamespace(positions=np.array([.01]), arm_indices=[0],
        angle_tree=cKDTree([[0.], [1.], [2.]]), angle_node_ids=(10, 20, 30),
        adjacency={10: [20], 20: [10, 30], 30: [20]}, labels={10: 1, 20: 1, 30: 3})
    assert gng_lidar_demo.has_safe_measured_neighbors(target)
    assert target.current_node_id == 10
    target.positions[:] = .95
    assert not gng_lidar_demo.has_safe_measured_neighbors(target)
    target.labels[30] = 1
    assert gng_lidar_demo.has_safe_measured_neighbors(target)
    for label in (2, 3, 0):
        target.labels[10] = label
        assert not gng_lidar_demo.has_safe_measured_neighbors(target)
    del target.labels[10]
    assert not gng_lidar_demo.has_safe_measured_neighbors(target)
    target.labels.update({10: 1, 20: 2})
    assert not gng_lidar_demo.has_safe_measured_neighbors(target)


def test_real_pose_requires_all_joints_and_fresh_measurements(receiver):
    receiver.on_real_joints(stamp(JointState(name=['neck'], position=[1.24])))
    assert receiver.real_joint_time == 0.
    message = stamp(JointState(name=['neck', 'waist'], position=[1.24, 0.]))
    receiver.on_real_joints(message)
    received = receiver.real_joint_time
    assert received > 0.
    receiver.on_real_joints(message)
    receiver.on_real_joints(stamp(message, 2.))
    message.position[0] = float('nan')
    receiver.on_real_joints(stamp(message))
    assert receiver.real_joint_time == received


def test_delayed_arrival_does_not_extend_source_lifetime(receiver, monkeypatch):
    now = time.time_ns()
    receiver.source_stamps = {kind: now-900_000_000 for kind in ('cloud', 'voxels', 'graph', 'joints')}
    monkeypatch.setattr(time, 'time_ns', lambda: now)
    assert receiver.has_fresh_environment()
    monkeypatch.setattr(time, 'time_ns', lambda: now+200_000_000)
    assert not receiver.has_fresh_environment()


@pytest.mark.parametrize('is_bigendian', [True, False])
def test_organized_cloud_with_padding_and_invalid_point(is_bigendian):
    message = PointCloud2(height=2, width=1, point_step=16, row_step=20, is_bigendian=is_bigendian)
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x', 4), ('y', 8), ('z', 12)]]
    order = '>' if is_bigendian else '<'
    message.data = struct.pack(order+'ffffIffffI', 8, 1, 2, 3, 0, 9, float('nan'), 0, 0, 0)
    np.testing.assert_equal(cloud_xyz(message), [[1, 2, 3]])
    message.row_step = 12
    with pytest.raises(ValueError, match='欠損'):
        cloud_xyz(message)


def test_optical_forward_and_mount_rotation():
    # 光学Z前方→本体X前方→取付yaw 90度による仮想Y前方
    result = transform_xyz(np.array([[0., 0., 2.]]), [1., 2., 3., 0., 0., np.pi/2],
                           [0., 0., 0.], [-.5, .5, -.5, .5])
    np.testing.assert_allclose(result, [[1, 4, 3]], atol=1e-12)


@pytest.mark.parametrize('stamp,last,now,expected', [
    (1_000_000_000, 0, 1.1, True), (1_000_000_000, 0, 2.1, False),
    (1_000_000_000, 1_000_000_000, 1.1, False), (2_000_000_000, 0, 1., False)])
def test_stale_duplicate_future_input_rejected(stamp, last, now, expected):
    assert has_fresh_input(stamp, last, now, 1.) is expected


def test_missing_clock_never_relabels_old_data():
    target = SimpleNamespace(settings={'source_frame': 'optical', 'max_input_age_sec': 1.},
                             last_input_stamp=-1, sim_stamp=None, num_rejected=0)
    import time
    stamp = time.time_ns()
    message = PointCloud2()
    message.header.frame_id = 'optical'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    external_pointcloud_bridge.on_cloud(target, message)
    assert target.num_rejected == 1
    assert 'clock' in target.reason


def test_real_robot_mask_and_camera_points_share_transform():
    camera_pose = [.2, -.1, .6, .1, .4, -.3]
    mount = [.02, 0., .01, 0., .1, 0.]
    real_camera = pose_matrix([.1, .05, .4, -.2, .3, .5])
    camera_point = np.array([.2, .1, .4, 1.])
    real_point = real_camera @ pose_matrix(mount) @ camera_point
    placement = real_root_transform(camera_pose, mount, real_camera)
    np.testing.assert_allclose(placement @ real_point, pose_matrix(camera_pose) @ camera_point, atol=1e-12)


def test_incomplete_real_joints_never_default_to_zero():
    import time
    target = SimpleNamespace(geometry=SimpleNamespace(joint_names=['waist_joint', 'neck_joint'],
        limits=np.array([[-1., 1.], [-1., 1.]])), real_positions=None, last_real_stamp=-1,
        settings={'max_input_age_sec': 1.}, real_joint_time=0.)
    message = JointState(name=['neck_joint'], position=[.2])
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
    external_pointcloud_bridge.on_real_joints(target, message)
    assert target.real_positions is None
    assert 'waist_joint' in target.real_state_detail
    message.name, message.position = ['waist_joint', 'neck_joint'], [.1, .2]
    external_pointcloud_bridge.on_real_joints(target, message)
    np.testing.assert_equal(target.real_positions, [.1, .2])
    received = target.real_joint_time
    external_pointcloud_bridge.on_real_joints(target, message)
    assert target.real_joint_time == received


@pytest.mark.parametrize('value', [-.1, float('nan'), float('inf'), True, '0.5'])
def test_invalid_return_delay_rejected_before_launch(robot_config, value):
    path, _ = robot_config
    overlay = path.parent/'delay.yaml'
    overlay.write_text(yaml.safe_dump({'return_clear_sec': value}))
    with pytest.raises(ValueError, match='return_clear_sec'):
        load_config(path, overlay)
