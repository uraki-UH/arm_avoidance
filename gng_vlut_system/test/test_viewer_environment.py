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


def test_native_target_validation_and_joint_order(receiver):
    receiver.arm_names, receiver.arm_indices = ['neck', 'waist'], [1, 0]
    receiver.geometry.limits = np.array([[-1., 1.], [-2., 2.]])
    receiver.native_target_stamp, receiver.native_target_time = -1, 0.
    receiver.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns()))
    message = stamp(JointState(name=['waist', 'neck'], position=[.2, .7]))
    receiver.on_native_target(message)
    np.testing.assert_allclose(receiver.native_target, [.7, .2])
    received = receiver.native_target_time
    receiver.on_native_target(stamp(message, 2.))
    assert receiver.native_target_time == received
    for names, values in [(['neck'], [.7]), (['neck', 'waist'], [float('nan'), .2]),
                          (['neck', 'waist'], [.7, 1.1]), (['neck', 'neck'], [.7, .8])]:
        receiver.on_native_target(stamp(JointState(name=names, position=values)))
        assert receiver.native_target is None


def test_native_target_step_limit_and_collision_fallback():
    target = SimpleNamespace(positions=np.zeros(3), home=np.zeros(3), native_target=np.array([.8]),
        arm_indices=[1], config={'min_cloud_clearance_th': .015, 'min_retreat_dist_th': .3, 'target_clearance': .1},
        select_active_arms=lambda: np.array([0]), cloud_clearance=lambda value: (.05+value[1], None),
        can_bridge=lambda *args: True, num_selected_gng=0, native_node_path=[1, 2],
        has_safe_first_neighbors=lambda: False,
        refine_target=lambda step: (np.array([0., -step, 0.]), True))
    value, has_target = gng_lidar_demo.select_native_target(target, .032)
    assert has_target and target.num_selected_gng == 1
    np.testing.assert_allclose(value, [0., .032, 0.])
    target.can_bridge = lambda *args: False
    value, has_target = gng_lidar_demo.select_native_target(target, .032)
    assert has_target
    np.testing.assert_allclose(value, [0., -.032, 0.])
    target.can_bridge = lambda *args: True
    target.native_target = np.array([.0001])
    value, has_target = gng_lidar_demo.select_native_target(target, .032)
    assert has_target
    np.testing.assert_allclose(value, [0., .0001, 0.])
    target.has_safe_first_neighbors = lambda: True
    value, has_target = gng_lidar_demo.select_native_target(target, .032)
    assert has_target
    np.testing.assert_allclose(value, [0., -.032, 0.])


@pytest.mark.parametrize('label', [2, 3])
@pytest.mark.parametrize('gap', [.07, .4])
def test_neighbor_risk_drives_motion_without_cloud_clearance_increase(label, gap):
    labels = {1: 1, 2: label}
    target = SimpleNamespace(positions=np.array([.3, .2]), home=np.zeros(2),
        native_target=np.array([.8, .7]), arm_indices=[0, 1], native_node_path=[2, 3], num_selected_gng=0,
        max_home_error_th=.015,
        config={'min_cloud_clearance_th': .015, 'min_clearance_th': .035, 'target_clearance': .1},
        select_active_arms=lambda: np.array([0]), can_bridge=lambda *args: True,
        cloud_clearance=lambda _: (gap, None),
        has_safe_first_neighbors=lambda: all(value == 1 for value in labels.values()),
        refine_target=lambda step: (np.array([.3, .2]), True))
    value, is_valid = gng_lidar_demo.select_native_target(target, .032)
    assert is_valid and target.phase == 'avoiding'
    np.testing.assert_allclose(value, [.332, .232])
    # 隣接危険であっても、経路の停止余裕不足は許可対象外
    target.can_bridge = lambda *args: False
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    np.testing.assert_allclose(value, [.3, .2])


def test_native_avoid_wait_return_and_monitor_cycle():
    gaps = {'current': .07, 'home': .03}
    labels = {1: 1, 2: 3}
    target = SimpleNamespace(positions=np.array([.3]), home=np.array([0.]),
        native_target=np.array([.8]), arm_indices=[0], native_node_path=[2, 3], num_selected_gng=0,
        max_home_error_th=.015,
        config={'min_cloud_clearance_th': .015, 'min_clearance_th': .035, 'target_clearance': .1,
                'return_clear_sec': 0.},
        select_active_arms=lambda: np.array([0]), can_bridge=lambda *args: gaps['home'] > .035,
        has_safe_first_neighbors=lambda: all(label == 1 for label in labels.values()),
        refine_target=lambda step: (np.array([.3+step]), True))
    target.cloud_clearance = lambda value: (
        gaps['home'] if np.allclose(value, target.home) else gaps['current']+.2*(value[0]-target.positions[0]), None)
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'avoiding' and value[0] > .3
    gaps.update(current=.15, home=.06)
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'avoiding' and value[0] > .3
    labels[2] = 1
    gaps['home'] = .03
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'waiting_for_clearance' and value[0] == .3
    gaps['home'] = .06
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'returning' and value[0] == pytest.approx(.268)
    gaps['current'] = .07
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'returning' and value[0] == pytest.approx(.268)
    target.positions[:] = 0.
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'monitoring' and value[0] == 0.
    gaps.update(current=.07, home=.07)
    labels[2] = 2
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'avoiding' and value[0] > 0.
    # 点群距離による対象腕なしの場合も、隣接危険からの退避を継続
    target.select_active_arms = lambda: np.array([], dtype=int)
    gaps.update(current=.4, home=.4)
    value, _ = gng_lidar_demo.select_native_target(target, .032)
    assert target.phase == 'avoiding' and value[0] > 0.


def test_first_neighbors_use_measured_angles_and_reject_unknown_labels():
    from scipy.spatial import cKDTree
    target = SimpleNamespace(positions=np.array([.01]), arm_indices=[0],
        angle_tree=cKDTree([[0.], [1.], [2.]]), angle_node_ids=(10, 20, 30),
        adjacency={10: [20], 20: [10, 30], 30: [20]}, labels={10: 1, 20: 1, 30: 3})
    assert gng_lidar_demo.has_safe_first_neighbors(target)
    assert target.current_node_id == 10
    target.positions[:] = .95
    assert not gng_lidar_demo.has_safe_first_neighbors(target)
    target.labels[30] = 1
    assert gng_lidar_demo.has_safe_first_neighbors(target)
    for label in (2, 3, 0):
        target.labels[10] = label
        assert not gng_lidar_demo.has_safe_first_neighbors(target)
    del target.labels[10]
    assert not gng_lidar_demo.has_safe_first_neighbors(target)
    target.labels.update({10: 1, 20: 2})
    assert not gng_lidar_demo.has_safe_first_neighbors(target)


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
