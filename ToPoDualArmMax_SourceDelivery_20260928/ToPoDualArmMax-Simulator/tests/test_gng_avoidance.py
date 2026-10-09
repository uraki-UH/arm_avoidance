"""共通GNG方策、入力失効、座標・モデル照合、MuJoCo接続の検証。"""
from pathlib import Path
from types import SimpleNamespace as ns
import sys
import threading
import time

import numpy as np
import pytest
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'integrations/ros2'))
import gng_avoidance as adapter
from gng_inputs import gng_inputs
from gng_avoidance_planner import graph_topology_hash
from physics_scene import PhysicsScene


def header(frame='robot/base_link', stamp=1_000_000_000):
    return ns(frame_id=frame, stamp=ns(sec=stamp//1_000_000_000, nanosec=stamp % 1_000_000_000))


@pytest.fixture
def receiver():
    value = gng_inputs.__new__(gng_inputs)
    value.models = {'arm': dict(num_nodes=2, joint_names=['joint'])}
    value.graphs, value.stamps = {}, {}
    value.graph_frame = 'robot/base_link'
    value.root_frame = 'robot/base_footprint'
    value.max_age_sec = 1.
    value.node = ns(get_clock=lambda: ns(now=lambda: ns(nanoseconds=1_000_000_001)))
    value.lock = threading.Lock()
    value.cloud, value.error = None, ''
    return value


def test_input_graph_order_freshness_and_topology(receiver):
    receiver.on_features('arm', ns(features=[ns(node_id=4, weight_angle=[.2]), ns(node_id=9, weight_angle=[.3])]))
    receiver.on_graph('arm', ns(header=header(), nodes=[ns(id=9), ns(id=4)], edges=[0, 1]))
    states = ns(header=header(), topology_hash=graph_topology_hash([9, 4], [0, 1]), node_ids=[9, 4], labels=[1, 2])
    receiver.on_states('arm', states)
    receiver.cloud = (cKDTree([[3, 3, 3]]), .01, time.monotonic())
    snapshot, message = receiver.snapshot()
    assert not message and snapshot[1]['arm']['labels'] == {9: 1, 4: 2}
    assert snapshot[1]['arm']['adjacency'] == {9: [4], 4: [9]}
    received = receiver.graphs['arm']['received']
    receiver.on_states('arm', states)
    assert receiver.graphs['arm']['received'] == received
    states.topology_hash += 1
    with pytest.raises(ValueError, match='不一致'):
        receiver.on_states('arm', states)
    receiver.graphs['arm']['received'] -= 2
    assert receiver.snapshot()[0] is None


@pytest.mark.parametrize('edges', [[0], [0, 2], [-1, 1]])
def test_invalid_edges_rejected(receiver, edges):
    with pytest.raises(ValueError):
        receiver.on_graph('arm', ns(header=header(), nodes=[ns(id=9), ns(id=4)], edges=edges))


def test_voxel_empty_and_stale(receiver):
    message = ns(header=header('robot/base_footprint'), data=[0], voxel_size=.02,
                 x_shift=42, y_shift=21, z_shift=0, offset=0, origin_x=0., origin_y=0., origin_z=0.)
    receiver.on_voxels('voxels', message)
    np.testing.assert_allclose(receiver.cloud[0].data, [[.01, .01, .01]])
    message.data = []
    message.header = header('robot/base_footprint', 1_000_000_001)
    receiver.on_voxels('voxels', message)
    assert receiver.cloud is None


class simple_geometry:
    joint_names = ['a', 'b']
    joints = [('a', 'root', 'arm_a'), ('b', 'root', 'arm_b')]
    spheres = [('arm_a', None, .01), ('arm_b', None, .01)]
    radii = np.array([.01, .01])
    is_arm = np.array([True, True])
    limits = np.array([[-1, 1], [-1, 1]])
    environment_shapes = ()

    def centers(self, q):
        return np.array([[q[0], 0, 0], [q[1], 1, 0]])

    def has_internal_clearance(self, centers, *args):
        return True

    def has_inter_arm_clearance(self, centers, *args):
        return True


def test_common_policy_retreat_and_return():
    controller = adapter.independent_policy(simple_geometry(), {'first': dict(joint_names=['a']), 'second': dict(joint_names=['b'])})
    angles = {0: np.array([0.]), 1: np.array([.3]), 2: np.array([.5])}
    graph = dict(angles=angles, labels={0: 2, 1: 1, 2: 1}, adjacency={0: [], 1: [2], 2: [1]},
                 angle_node_ids=(0, 1, 2), angle_tree=cKDTree(list(angles.values())))
    graphs = {'first': graph, 'second': dict(graph, labels={0: 1, 1: 1, 2: 1})}
    q, home = np.zeros(2), np.zeros(2)
    try:
        for policy in controller.policies.values():
            policy.config['return_clear_sec'] = 0.
        deadline = time.monotonic()+5
        while q[0] < .15 and time.monotonic() < deadline:
            q, status = controller.propose(q, home, (cKDTree([[-.06, 0, 0]]), .005), graphs, ())
            time.sleep(.01)
        assert q[0] >= .15 and q[1] == 0
        assert controller.policies['first'].num_selected_gng > 0
        for _ in range(10):
            q, status = controller.propose(q, home, (cKDTree([[-.06, 0, 0]]), .005), graphs, ())
            assert q[0] >= .15
        graphs['first'] = dict(graph, labels={0: 1, 1: 1, 2: 1})
        for _ in range(50):
            q, status = controller.propose(q, home, (cKDTree([[5, 5, 5]]), .005), graphs, ())
        assert abs(q[0]) < .02 and q[1] == 0
    finally:
        controller.close()


@pytest.fixture(scope='module')
def robot_scene():
    return PhysicsScene([], dict(model='long', pose={}, position=[0, 0, 0], quaternion=[0, 0, 0, 1]))


def test_mujoco_adapter_and_stale_stop(monkeypatch, robot_scene):
    class fake_inputs:
        def __init__(self, node, params, models, root):
            self.is_stale = False
            self.is_closed = False
            self.graphs = {}
            for name, model in models.items():
                angle = np.array([robot_scene.robot.state()[joint] for joint in model['joint_names']])
                self.graphs[name] = dict(angles={0: angle}, labels={0: 1}, adjacency={0: []},
                                         angle_node_ids=(0,), angle_tree=cKDTree([angle]))

        def snapshot(self):
            return (None, '欠測') if self.is_stale else (((cKDTree([[5, 5, 5]]), .02, time.monotonic()), self.graphs), '')

        def close(self):
            self.is_closed = True
    monkeypatch.setattr(adapter, 'gng_inputs', fake_inputs)
    control = adapter.gng_filter(robot_scene, {}, object(), {'model': 'long'})
    robot_scene.avoidance = control
    try:
        desired = dict(robot_scene.robot.initial, L_joint7=.15)
        control.set_targets(desired)
        # 復帰待ち時間の短縮のみ。共通方策とMuJoCo駆動は通常経路
        for policy in control.policy.policies.values():
            policy.config['return_clear_sec'] = 0.
        for _ in range(300):
            robot_scene.step()
            time.sleep(.005)
        assert robot_scene.robot.state()['L_joint7'] > .02
        assert abs(robot_scene.robot.state()['R_joint7']) < .02
        control.inputs.is_stale = True
        before = robot_scene.data.time
        with pytest.raises(RuntimeError, match='欠測'):
            robot_scene.step()
        assert robot_scene.data.time == before
        with pytest.raises(ValueError, match='固定関節'):
            control.set_targets(dict(desired, waist_joint=.1))
    finally:
        control.close()
        robot_scene.avoidance = None
    assert control.inputs.is_closed


def test_model_mismatch():
    with pytest.raises(ValueError, match='不一致'):
        adapter.check_kinematics('/ros2_ws/src/urdf/topo_dual_arm_max_long/topo_dual_arm_max.urdf', 'standard')
