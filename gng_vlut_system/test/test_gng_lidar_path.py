"""VLUTラベルと観測点群を用いた経路入口・中間姿勢の検証。"""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_gng_lidar_demo import gng_lidar_demo, build_path_message, voxel_centers
from avoidance_motion import motion_flags
from builtin_interfaces.msg import Time
from gng_avoidance_planner import gng_avoidance_policy
from dual_arm_avoidance_geometry import robot_geometry
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from std_msgs.msg import UInt16MultiArray
from voxel_msgs.msg import Voxel


class test_geometry:
    is_arm = np.array([True])
    radii = np.array([.05])

    def centers(self, positions):
        return np.array([[positions[0], 0., 0.]])

    def has_internal_clearance(self, centers):
        return True

    def has_inter_arm_clearance(self, centers, min_clearance_th=.005):
        return True


class paired_geometry(test_geometry):
    is_arm = np.array([True, True])
    radii = np.array([.05, .05])

    def centers(self, positions):
        return np.column_stack((positions, np.zeros((2, 2))))

    def has_inter_arm_clearance(self, centers, min_clearance_th=.005):
        return np.linalg.norm(centers[0]-centers[1]) >= .2

    def has_internal_clearance(self, centers):
        return self.has_inter_arm_clearance(centers)


class test_gng_lidar_path(unittest.TestCase):
    def test_path_message_preserves_cached_graph_and_source_labels(self):
        graph = TopologicalMap(nodes=[TopologicalNode(id=4, label=1), TopologicalNode(id=9, label=1)], edges=[0, 1])
        graph.header.frame_id, graph.header.stamp.sec = 'sim/base', 2
        before = deepcopy(graph)
        message = build_path_message(graph, [9, 100, 4], {9: 3}, Time(sec=8))
        self.assertEqual([node.id for node in message.nodes], [9, 4])
        self.assertEqual([node.label for node in message.nodes], [3, 0])
        self.assertEqual(list(message.edges), [0, 1])
        self.assertEqual((message.header.frame_id, message.header.stamp.sec), ('sim/base', 8))
        self.assertEqual(graph, before)
        message.nodes[0].pos.x = 99.
        self.assertEqual(graph, before)
        empty = build_path_message(graph, [], {}, Time(sec=9))
        self.assertEqual(empty.nodes, [])
        self.assertEqual(list(empty.edges), [])

    def test_voxel_centers_preserve_source_and_coordinate_order(self):
        cells = np.array([[-2, 3, 1], [4, -1, 0]], dtype=np.int64)
        transform = np.array([[0., -1., 0., 1.], [1., 0., 0., 2.], [0., 0., 1., 3.], [0., 0., 0., 1.]])
        for shifts in ((42, 21, 0), (0, 21, 42)):
            ids = [sum(int(value+10) << shift for value, shift in zip(cell, shifts)) for cell in cells]
            message = Voxel(voxel_size=.2, x_shift=shifts[0], y_shift=shifts[1], z_shift=shifts[2],
                            offset=10, origin_x=.1, origin_y=.2, origin_z=.3, data=ids)
            before = deepcopy(message)
            expected = (cells+.5)*message.voxel_size + [.1, .2, .3]
            np.testing.assert_allclose(voxel_centers(message), expected)
            np.testing.assert_allclose(voxel_centers(message, transform), expected @ transform[:3, :3].T + transform[:3, 3])
            self.assertEqual(message, before)

    def test_invalid_and_empty_voxels_do_not_refresh_valid_input(self):
        target = gng_lidar_demo.__new__(gng_lidar_demo)
        target.external_environment, target.voxel_frame = None, 'sim/base'
        for invalid in (dict(voxel_size=0.), dict(voxel_size=float('nan')), dict(x_shift=0), dict(origin_z=float('inf'))):
            with self.subTest(invalid=invalid):
                message = Voxel(voxel_size=.02, x_shift=42, y_shift=21, z_shift=0, data=[0])
                message.header.frame_id, message.header.stamp.sec = 'sim/base', 1
                for name, value in invalid.items():
                    setattr(message, name, value)
                previous = target.cloud_tree = object()
                target.voxel_time, target.last_voxel_stamp, target.num_voxels, target.cell_radius = 5., 6, 11, .01
                target.on_voxels(message)
                self.assertIs(target.cloud_tree, previous)
                self.assertEqual((target.voxel_time, target.last_voxel_stamp, target.num_voxels, target.cell_radius), (5., 6, 11, .01))
        message = Voxel(voxel_size=.02, x_shift=42, y_shift=21, z_shift=0)
        message.header.frame_id, message.header.stamp.sec = 'sim/base', 1
        target.on_voxels(message)
        self.assertIsNone(target.cloud_tree)
        self.assertEqual((target.voxel_time, target.num_voxels, target.last_voxel_stamp), (5., 11, 1_000_000_000))

    def test_diagnostics_keep_schema_and_one_observation_time(self):
        state = SimpleNamespace(num_cloud=2, num_voxels=3, qp=None, motion_phase='monitoring',
            motion_flags=motion_flags(), current_node_id=9, has_safe_neighbors=True,
            cloud_time=17., voxel_time=18., graph_time=19., real_joint_time=16.,
            external_environment={'source_namespace': 'actual'}, labels={4: 1, 9: 3, 10: 2},
            num_local_steps=5, num_plans=6, num_selected_gng=7, path=[9], cloud_gap=.2,
            coordination_source_indices=np.array([0]), arm_names=['L_joint1', 'R_joint1'],
            active_angle_indices=np.array([1]))
        status = gng_lidar_demo.diagnostic_status(state, 20.)
        self.assertEqual(json.loads(json.dumps(status)), {
            'num_cloud': 2, 'num_voxels': 3, 'local_qp': None, 'planner_backend': 'gng_avoidance_policy',
            'motion_phase': 'monitoring', 'motion_flags': dict(is_stop_requested=False, has_valid_input=True,
                has_active_joints=False, has_safe_neighbors=False, can_finish_retreat=False, can_return=False, is_home=False),
            'current_node_id': 9, 'has_safe_first_neighbors': True,
            'cloud_age_sec': 3., 'voxel_age_sec': 2., 'graph_age_sec': 1., 'real_joint_age_sec': 4.,
            'environment_namespace': 'actual', 'num_safe': 1, 'num_danger': 1, 'num_collision': 1,
            'num_local_steps': 5, 'num_plans': 6, 'num_selected_gng': 7, 'node_path': [9],
            'cloud_clearance_m': .2, 'is_coordinated': True, 'active_arm_joints': ['R_joint1']})
        status['node_path'].clear()
        self.assertEqual(state.path, [9])
        state.external_environment = None
        status = gng_lidar_demo.diagnostic_status(state, 20.)
        self.assertIsNone(status['real_joint_age_sec'])
        self.assertIsNone(status['environment_namespace'])

    def test_tick_keeps_safety_and_plan_reset_before_diagnostics(self):
        for state, phase, has_reset in [('running', 'monitoring', False), ('running', 'obstacle_wait', True),
                                       ('stopped', 'software_stop', True), ('fault', 'fault', True)]:
            with self.subTest(state=state, phase=phase):
                target = gng_lidar_demo.__new__(gng_lidar_demo)
                target.state, target.phase, target.motion_phase = state, phase, 'avoiding'
                events = []
                target.clear_plan = lambda: events.append('clear')
                target.publish_diagnostics = lambda: events.append('diagnostics')
                with patch('dual_arm_gng_lidar_demo.avoidance_demo.tick', side_effect=lambda: events.append('control')), \
                     patch('dual_arm_gng_lidar_demo.has_stable_return_clearance', side_effect=lambda *_: events.append('confirmation')):
                    target.tick()
                expected = ['control'] + (['confirmation', 'clear'] if has_reset else []) + ['diagnostics']
                self.assertEqual(events, expected)
                self.assertEqual(target.motion_phase, phase if has_reset else 'avoiding')

    def test_reset_planning_cycle_cancels_only_pending_plan(self):
        target = gng_lidar_demo.__new__(gng_lidar_demo)
        target.plan_future = future = Mock()
        target.path, target.next_plan_sec, target.return_clear_since_sec = [4], 10., 3.
        target.reset_planning_cycle()
        future.cancel.assert_called_once_with()
        self.assertEqual(target.path, [])
        self.assertIsNone(target.plan_future)
        self.assertIsNone(target.return_clear_since_sec)
        self.assertEqual(target.next_plan_sec, 0.)

    def test_python_neighbor_risk_bypasses_distance_only_hold(self):
        search = self.make_search()
        search.arm_names = ['L_joint1']
        search.angles = {4: np.array([.2]), 9: np.array([-.8]), 20: np.array([.5])}
        search.labels = {4: 1, 9: 3, 20: 1}
        search.adjacency = {4: [9, 20], 9: [4], 20: [4]}
        search.config['min_retreat_dist_th'] = .3
        search.path = []
        search.num_plans = search.num_selected_gng = 0
        search.select_active_arms = lambda: np.array([], dtype=int)
        search.cloud_clearance = lambda _: (.4, None)
        search.can_bridge = lambda *args: True
        search.plan_future = SimpleNamespace(done=lambda: True, result=lambda: dict(
            path=[4, 20], active_angle_indices=np.array([0]), coordination_source_indices=np.array([], dtype=int)))
        target, is_valid = gng_lidar_demo.select_target(search, None, None, .1)
        assert is_valid and search.num_plans == 1
        assert search.path == [4, 20]
        np.testing.assert_allclose(target, [-.9])

    def test_retreat_skips_unsafe_neighbor_goal_but_keeps_escape_path(self):
        search = self.make_search()
        search.positions = np.zeros(1)
        search.angles = {0: np.array([.1]), 1: np.array([.2]), 2: np.array([.3]), 4: np.array([-.1])}
        search.labels = {0: 1, 1: 1, 2: 1, 4: 3}
        search.adjacency = {0: [1, 4], 1: [0, 2], 2: [1], 4: [0]}
        search.config['max_entry_candidates'] = 1
        search.cloud_clearance = lambda _: (.2, None)
        search.can_bridge = lambda *args: True
        self.assertEqual(search.plan(.2), [0, 1])
        search.labels[2] = 2
        self.assertEqual(search.plan(.2), [])

    def make_search(self):
        search = gng_avoidance_policy()
        search.geometry = test_geometry()
        search.max_home_error_th = .015
        search.home = np.zeros(1)
        search.positions = np.array([-1.])
        search.arm_indices = [0]
        search.active_angle_indices = np.array([0])
        search.coordination_source_indices = np.array([], dtype=int)
        search.cell_radius = .02
        search.cloud_tree = cKDTree([[0., 0., 0.]])
        search.config = dict(max_plan_sec=1., max_bridge_step=.05, max_entry_candidates=40,
                             min_cloud_clearance_th=.015, min_clearance_th=.035, target_clearance=.12)
        search.angles = {4: np.array([1.]), 9: np.array([-.8])}
        search.labels = {4: 1, 9: 1}
        search.adjacency = {4: [9], 9: [4]}
        return search

    def make_paired_search(self):
        search = self.make_search()
        search.geometry = paired_geometry()
        search.arm_indices = [0, 1]
        search.positions = np.array([-1., 0.])
        search.home = np.zeros(2)
        search.angles = {4: np.array([.7, 1.7])}
        search.labels = {4: 1}
        search.adjacency = {4: []}
        search.cloud_tree = cKDTree([[3., 0., 0.]])
        return search

    def test_real_geometry_distinguishes_floor_and_inter_arm_pairs(self):
        model = 'topo_dual_arm_max'
        geometry = robot_geometry(Path(__file__).resolve().parents[2]/'urdf'/model/(model+'.urdf'))
        centers = geometry.centers(np.zeros(len(geometry.joint_names)))
        self.assertGreater(len(geometry.inter_arm_pairs), 0)
        self.assertTrue(geometry.has_inter_arm_clearance(centers))
        below_floor = centers-np.array([0., 0., 10.])
        self.assertTrue(geometry.has_inter_arm_clearance(below_floor))
        self.assertFalse(geometry.has_internal_clearance(below_floor))
        first, second = geometry.inter_arm_pairs[0]
        centers[first] = centers[second]
        self.assertFalse(geometry.has_inter_arm_clearance(centers))
        self.assertFalse(geometry.has_internal_clearance(centers))

    def test_inter_arm_blockage_enables_validated_coordination(self):
        search = self.make_paired_search()
        self.assertEqual(search.plan(.9), [])
        self.assertTrue(search.has_inter_arm_rejection)
        with ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn')) as pool:
            result = pool.submit(search.plan_with_coordination, .9).result(timeout=15)
        self.assertEqual(result['path'], [4])
        np.testing.assert_array_equal(result['active_angle_indices'], [0, 1])
        np.testing.assert_array_equal(result['coordination_source_indices'], [0])

    def test_single_arm_success_does_not_enable_coordination(self):
        search = self.make_paired_search()
        search.angles[4] = np.array([-1.3, 1.7])
        result = search.plan_with_coordination(.9)
        self.assertEqual(result['path'], [4])
        np.testing.assert_array_equal(result['active_angle_indices'], [0])
        self.assertEqual(len(result['coordination_source_indices']), 0)

    def test_cloud_blockage_does_not_enable_coordination(self):
        search = self.make_paired_search()
        search.cloud_tree = cKDTree([[-.5, 0., 0.]])
        result = search.plan_with_coordination(.9)
        self.assertEqual(result['path'], [])
        self.assertFalse(search.has_inter_arm_rejection)
        np.testing.assert_array_equal(result['active_angle_indices'], [0])

    def test_floor_or_body_blockage_does_not_enable_coordination(self):
        search = self.make_paired_search()
        search.geometry.has_internal_clearance = lambda _: False
        search.geometry.has_inter_arm_clearance = lambda _, min_clearance_th=.005: True
        result = search.plan_with_coordination(.9)
        self.assertEqual(result['path'], [])
        np.testing.assert_array_equal(result['active_angle_indices'], [0])

    def test_failed_coordination_restores_single_arm_selection(self):
        search = self.make_paired_search()
        search.geometry.has_inter_arm_clearance = lambda _, min_clearance_th=.005: False
        result = search.plan_with_coordination(.9)
        self.assertEqual(result['path'], [])
        np.testing.assert_array_equal(result['active_angle_indices'], [0])
        self.assertEqual(len(result['coordination_source_indices']), 0)

    def test_timeout_does_not_enable_coordination(self):
        search = self.make_paired_search()
        search.config['max_plan_sec'] = -1.
        result = search.plan_with_coordination(.9)
        self.assertTrue(search.has_timed_out)
        np.testing.assert_array_equal(result['active_angle_indices'], [0])

    def test_timeout_during_last_collision_check_does_not_expand(self):
        search = self.make_paired_search()
        elapsed = [0.]
        def reject_after_deadline(*_):
            elapsed[0] = 2.
            search.has_inter_arm_rejection = True
            return False
        search.can_bridge = reject_after_deadline
        with patch('gng_avoidance_planner.time.monotonic', side_effect=lambda: elapsed[0]):
            result = search.plan_with_coordination(.9)
        self.assertTrue(search.has_timed_out)
        np.testing.assert_array_equal(result['active_angle_indices'], [0])

    def test_projection_holds_other_arm_and_body(self):
        search = self.make_search()
        search.positions = np.array([.3, -.5, .7])
        search.home = np.zeros(3)
        search.arm_indices = [1, 2]
        search.angles = {4: np.array([1., -1.])}
        np.testing.assert_array_equal(search.pose(4), [.3, 1., .7])
        search.active_angle_indices = np.array([1])
        np.testing.assert_array_equal(search.pose(4), [.3, -.5, -1.])
        search.active_angle_indices = np.array([0, 1])
        np.testing.assert_array_equal(search.pose(4), [.3, 1., -1.])

    def test_arm_selection_uses_observed_clearance(self):
        search = self.make_search()
        search.arm_names = [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)]
        search.arm_indices = list(range(14))
        search.positions = search.home = np.zeros(14)
        search.path = []
        search.config['min_retreat_dist_th'] = .2
        search.geometry = SimpleNamespace(
            spheres=[('L_link4', None, .05), ('R_link4', None, .05)],
            is_arm=np.array([True, True]), radii=np.array([.05, .05]),
            centers=lambda _: np.array([[.15, 0., 0.], [1., 0., 0.]]))
        np.testing.assert_array_equal(search.select_active_arms(), np.arange(7))
        search.cloud_tree = cKDTree([[1., 0., 0.]])
        np.testing.assert_array_equal(search.select_active_arms(), np.arange(7, 14))
        search.cloud_tree = cKDTree([[.15, 0., 0.], [1., 0., 0.]])
        np.testing.assert_array_equal(search.select_active_arms(), np.arange(14))
        search.cloud_tree = cKDTree([[3., 0., 0.]])
        self.assertEqual(len(search.select_active_arms()), 0)

    def test_projected_pose_still_checks_internal_collision(self):
        search = self.make_search()
        search.geometry.has_internal_clearance = lambda _: False
        self.assertEqual(search.plan(.93), [])

    def test_return_holds_other_arm_and_discards_previous_plan(self):
        search = self.make_search()
        # 復帰時の対象腕選択に限定した、継続時間なしの検証
        search.config['return_clear_sec'] = 0.
        search.arm_names = [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)]
        search.arm_indices = list(range(14))
        search.home = np.zeros(14)
        search.positions = np.full(14, .5)
        search.path = []
        search.active_angle_indices = np.arange(7, 14)
        cancelled = []
        search.plan_future = SimpleNamespace(cancel=lambda: cancelled.append(True))
        search.cloud_tree = cKDTree([[3., 0., 0.]])
        search.config['min_retreat_dist_th'] = .2
        search.geometry = SimpleNamespace(
            spheres=[('L_link4', None, .05), ('R_link4', None, .05)],
            is_arm=np.array([True, True]), radii=np.array([.05, .05]),
            centers=lambda _: np.array([[.15, 0., 0.], [1., 0., 0.]]),
            has_internal_clearance=lambda _: True)
        target, is_valid = gng_lidar_demo.select_target(search, None, None, .1)
        self.assertTrue(is_valid)
        np.testing.assert_array_equal(target[7:], search.positions[7:])
        np.testing.assert_allclose(target[:7], .4)
        self.assertEqual(cancelled, [True])
        self.assertIsNone(search.plan_future)

    def test_coordinated_result_survives_next_control_tick(self):
        search = self.make_search()
        search.arm_names = [f'{side}_joint{idx}' for side in ('L', 'R') for idx in range(1, 8)]
        search.arm_indices = list(range(14))
        search.active_angle_indices = np.arange(7)
        search.positions = search.home = np.zeros(14)
        search.path = []
        search.angles = {4: np.full(14, .5)}
        search.labels = {4: 1}
        search.adjacency = {4: []}
        search.num_plans = search.num_selected_gng = 0
        search.config['min_retreat_dist_th'] = .2
        search.geometry = SimpleNamespace(
            spheres=[('L_link4', None, .05), ('R_link4', None, .05)],
            is_arm=np.array([True, True]), radii=np.array([.05, .05]),
            centers=lambda _: np.array([[.15, 0., 0.], [1., 0., 0.]]),
            has_internal_clearance=lambda _: True)
        search.plan_future = SimpleNamespace(done=lambda: True, result=lambda: dict(
            path=[4], active_angle_indices=np.arange(14), coordination_source_indices=np.arange(7)))
        for _ in range(2):
            target, is_valid = gng_lidar_demo.select_target(search, None, None, .1)
            self.assertTrue(is_valid)
            np.testing.assert_allclose(target, .1)
            np.testing.assert_array_equal(search.active_angle_indices, np.arange(14))
        self.assertEqual(search.num_plans, 1)
        search.path = []
        np.testing.assert_array_equal(search.select_active_arms(), np.arange(7))

    def test_compact_states_require_complete_valid_ids(self):
        state = SimpleNamespace(graph_ids=(4, 9), labels={4: 1, 9: 1}, graph_time=0.)
        message = UInt16MultiArray(data=[4, 3, 9, 2])
        gng_lidar_demo.on_states(state, message)
        self.assertEqual(state.labels, {4: 3, 9: 2})
        stamp = state.graph_time
        for data in ([4, 1], [4, 1, 8, 1], [4, 1, 9, 0], [4, 1, 4, 2]):
            gng_lidar_demo.on_states(state, UInt16MultiArray(data=data))
            self.assertEqual(state.graph_time, stamp)
            self.assertEqual(state.labels, {4: 3, 9: 2})

    def test_safety_update_reuses_edges_but_updates_labels(self):
        state = SimpleNamespace(graph_ids=(), graph_edges=None)
        message = TopologicalMap()
        first, second = TopologicalNode(), TopologicalNode()
        first.id, first.label = 4, 1
        second.id, second.label = 9, 1
        message.nodes, message.edges = [first, second], [0, 1]
        gng_lidar_demo.on_graph(state, message)
        adjacency = state.adjacency
        message.nodes[0].label = 3
        gng_lidar_demo.on_graph(state, message)
        self.assertIs(state.adjacency, adjacency)
        self.assertEqual(state.labels[4], 3)
        message.edges = []
        gng_lidar_demo.on_graph(state, message)
        self.assertEqual(state.adjacency, {4: [], 9: []})

    def test_process_search_preserves_collision_checks(self):
        search = self.make_search()
        with ProcessPoolExecutor(max_workers=1, mp_context=get_context('spawn')) as pool:
            self.assertEqual(pool.submit(search.plan, .93).result(timeout=15), [9])

    def test_crossing_cloud_is_rejected(self):
        search = self.make_search()
        self.assertFalse(search.can_bridge(np.array([-1.]), np.array([1.]), .015))
        self.assertTrue(search.can_bridge(np.array([-1.]), np.array([-.8]), .015))
        self.assertEqual(search.plan(.93), [9])

    def test_bridge_rejects_stop_zone_between_safe_endpoints(self):
        search = self.make_search()
        search.cloud_tree = cKDTree([[0., .09, 0.]])
        first, second = np.array([-.2]), np.array([.2])
        self.assertGreater(search.cloud_clearance(first)[0], .035)
        self.assertGreater(search.cloud_clearance(second)[0], .035)
        # 両端は安全、区間中央の余裕は20 mm。計画側15 mm指定でも実行側35 mmで棄却
        self.assertFalse(search.can_bridge(first, second, .015))
        search.cloud_tree = cKDTree([[0., .11, 0.]])
        self.assertTrue(search.can_bridge(first, second, .015))
        self.assertFalse(search.can_bridge(first, second, .05))

    def test_unsafe_vlut_nodes_are_excluded(self):
        search = self.make_search()
        search.labels = {4: 2, 9: 3}
        self.assertEqual(search.plan(.93), [])

    def test_safe_endpoint_with_blocked_entry_is_rejected(self):
        search = self.make_search()
        search.labels[9] = 3
        self.assertEqual(search.plan(.93), [])


if __name__ == '__main__':
    unittest.main()
