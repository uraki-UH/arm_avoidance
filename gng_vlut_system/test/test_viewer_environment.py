"""実時間の既存環境入力・固定座標変換・更新失効の検証。"""
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode, TopologicalNodeStates
from sensor_msgs.msg import JointState
from voxel_msgs.msg import Voxel

from test_pointcloud_avoidance import robot_config, load_config, lidar_setup, gng_lidar_demo
from dual_arm_gng_lidar_demo import graph_topology_hash


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
