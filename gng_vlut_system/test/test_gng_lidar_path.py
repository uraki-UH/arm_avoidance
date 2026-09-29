"""VLUTラベルと観測点群を用いた経路入口・中間姿勢の検証。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_gng_lidar_demo import gng_path_search, gng_lidar_demo
from dual_arm_avoidance_geometry import robot_geometry
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from std_msgs.msg import UInt16MultiArray


class test_geometry:
    is_arm = np.array([True])
    radii = np.array([.05])

    def centers(self, positions):
        return np.array([[positions[0], 0., 0.]])

    def has_internal_clearance(self, centers):
        return True

    def has_inter_arm_clearance(self, centers):
        return True


class paired_geometry(test_geometry):
    is_arm = np.array([True, True])
    radii = np.array([.05, .05])

    def centers(self, positions):
        return np.column_stack((positions, np.zeros((2, 2))))

    def has_inter_arm_clearance(self, centers):
        return np.linalg.norm(centers[0]-centers[1]) >= .2

    def has_internal_clearance(self, centers):
        return self.has_inter_arm_clearance(centers)


class test_gng_lidar_path(unittest.TestCase):
    def make_search(self):
        search = gng_path_search()
        search.geometry = test_geometry()
        search.home = np.zeros(1)
        search.positions = np.array([-1.])
        search.arm_indices = [0]
        search.active_angle_indices = np.array([0])
        search.coordination_source_indices = np.array([], dtype=int)
        search.cell_radius = .02
        search.cloud_tree = cKDTree([[0., 0., 0.]])
        search.config = dict(max_plan_sec=1., max_bridge_step=.05, max_entry_candidates=40,
                             min_cloud_clearance_th=.015, target_clearance=.12)
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
        search.geometry.has_inter_arm_clearance = lambda _: True
        result = search.plan_with_coordination(.9)
        self.assertEqual(result['path'], [])
        np.testing.assert_array_equal(result['active_angle_indices'], [0])

    def test_failed_coordination_restores_single_arm_selection(self):
        search = self.make_paired_search()
        search.geometry.has_inter_arm_clearance = lambda _: False
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
        with patch('dual_arm_gng_lidar_demo.time.monotonic', side_effect=lambda: elapsed[0]):
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
