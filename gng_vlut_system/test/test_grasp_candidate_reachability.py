import argparse
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

import rclpy
from geometry_msgs.msg import Point, Point32, Pose, PoseArray, TransformStamped
from std_msgs.msg import Float32MultiArray, Header
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode


script_path = Path(__file__).resolve().parents[1] / "launch/topological_map_goal_selector_node.py"
spec = importlib.util.spec_from_file_location("grasp_goal_selector_test_module", script_path)
selector_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = selector_module
spec.loader.exec_module(selector_module)


class capture_publisher:
    def publish(self, msg):
        self.latest = copy.deepcopy(msg)


def make_map(points, frame="base"):
    msg = TopologicalMap(header=Header(frame_id=frame))
    for node_id, point in enumerate(points):
        node = TopologicalNode()
        node.id = node_id
        node.label = 1
        node.pos = Point32(x=float(point), y=0.01, z=0.01)
        node.normal.z = 1.0
        msg.nodes.append(node)
    return msg


def make_candidates(points, frame="base"):
    msg = PoseArray(header=Header(frame_id=frame))
    for point in points:
        pose = Pose(position=Point(x=float(point), y=0.01, z=0.01))
        pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


class test_grasp_candidate_reachability(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        args = argparse.Namespace(
            topological_map_topic="/test/map", output_topic="/test/selected",
            marker_topic="/test/selected_markers", candidate_count=8,
            non_collision_only=True, orientation_weight=0.0,
            target_pose_topic="", target_point_topic="", target_pose_array_topic="/test/poses",
            target_score_topic="/test/scores", goal_candidate_ids_topic="/test/goals",
            node_feature_topic="", manipulability_weight=0.0, allow_untransformed_target=False,
            reachability_map_topic="", reachability_voxel_size=0.1,
            reachability_voxel_origin=[0.0, 0.0, 0.0], reachability_publish_hz=5.0,
            reachability_topic="/test/reachability", reachability_marker_topic="/test/reachability_markers",
        )
        self.node = selector_module.TopologicalMapGoalSelector(args)
        for name in ("reachability_pub", "reachability_marker_pub", "output_pub",
                     "marker_pub", "goal_candidate_ids_pub"):
            setattr(self.node, name, capture_publisher())

    def tearDown(self):
        self.node.destroy_node()

    def test_all_candidates_preserved_and_only_inside_goals_selected(self):
        self.node._on_map(make_map([0.01, 0.21]))
        source = make_candidates([9.0, 0.02, 0.22])
        self.node._on_pose_scores(Float32MultiArray(data=[99.0, 1.0, 2.0]))
        self.node._on_pose_array(source)
        out = self.node.reachability_pub.latest
        self.assertEqual([row.candidate_idx for row in out.candidates], [0, 1, 2])
        self.assertEqual([row.pose for row in out.candidates], source.poses)
        self.assertEqual([row.is_in_reachability for row in out.candidates], [False, True, True])
        self.assertEqual([row.can_plan for row in out.candidates], [False, True, True])
        self.assertEqual(list(self.node.goal_candidate_ids_pub.latest.data), [0, 1])
        self.assertEqual(out.candidates[0].shape_score, 99.0)

    def test_all_outside_clears_old_goals(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_pose_array(make_candidates([0.02]))
        self.assertTrue(self.node.goal_candidate_ids_pub.latest.data)
        self.node._on_pose_array(make_candidates([8.0]))
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
        self.assertFalse(self.node.output_pub.latest.nodes)
        self.assertEqual(len(self.node.reachability_pub.latest.candidates), 1)

    def test_empty_input_clears_results_and_markers(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_pose_array(make_candidates([0.02]))
        self.node._on_pose_array(make_candidates([]))
        self.assertFalse(self.node.reachability_pub.latest.candidates)
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
        self.assertEqual(len(self.node.reachability_marker_pub.latest.markers), 1)
        self.assertEqual(self.node.marker_pub.latest.markers[0].action, 3)

    def test_missing_map_preserves_unknown_candidates(self):
        self.node._on_pose_array(make_candidates([0.02]))
        row = self.node.reachability_pub.latest.candidates[0]
        self.assertFalse(row.has_evaluation)
        self.assertFalse(row.can_plan)
        self.assertEqual(row.reason, "unknown_map")

    def test_missing_transform_clears_old_goals(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_pose_array(make_candidates([0.02]))
        self.node._on_pose_array(make_candidates([0.02], "missing_frame"))
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
        self.assertEqual(self.node.reachability_pub.latest.candidates[0].reason, "unknown_transform")

    def test_motion_rechecks_cached_candidates_without_new_input(self):
        self.node._on_map(make_map([0.01]))
        transform = TransformStamped()
        transform.header.frame_id = "base"
        transform.child_frame_id = "world"
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = -1.0
        self.node.tf_buffer.set_transform_static(transform, "test")
        self.node._on_pose_array(make_candidates([1.02], "world"))
        self.assertTrue(self.node.reachability_pub.latest.candidates[0].can_plan)
        transform.transform.translation.x = 0.0
        self.node.tf_buffer.set_transform_static(transform, "test")
        self.node._maybe_publish()
        self.assertFalse(self.node.reachability_pub.latest.candidates[0].is_in_reachability)
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
        transform.transform.translation.x = -1.0
        self.node.tf_buffer.set_transform_static(transform, "test")
        self.node._maybe_publish()
        self.assertTrue(self.node.reachability_pub.latest.candidates[0].can_plan)

    def test_negative_cells_and_boundary(self):
        self.node._on_map(make_map([-0.05, 0.01]))
        self.node._on_pose_array(make_candidates([-0.1, -0.10001, 0.09999, 0.1]))
        self.assertEqual([row.is_in_reachability for row in self.node.reachability_pub.latest.candidates],
                         [True, False, True, False])

    def test_collision_keeps_inside_status_but_no_goal(self):
        msg = make_map([0.01])
        msg.nodes[0].label = 2
        self.node._on_map(msg)
        self.node._on_pose_array(make_candidates([0.02]))
        row = self.node.reachability_pub.latest.candidates[0]
        self.assertTrue(row.is_in_reachability)
        self.assertFalse(row.can_plan)
        self.assertEqual(row.reason, "inside_no_goal_nodes")

    def test_independent_map_does_not_pass_visualization_ids_to_planner(self):
        self.node.reachability_map_topic = "/independent_map"
        self.node._on_map(make_map([0.01]))
        msg = make_map([0.01, 0.21])
        msg.nodes[0].id = 900
        msg.nodes[1].id = 901
        self.node._on_reachability_map(msg)
        self.node._on_pose_array(make_candidates([0.02, 0.22]))
        rows = self.node.reachability_pub.latest.candidates
        self.assertEqual([row.is_in_reachability for row in rows], [True, True])
        self.assertEqual(list(rows[0].goal_node_ids), [0])
        self.assertFalse(rows[1].goal_node_ids)
        self.assertEqual(list(self.node.goal_candidate_ids_pub.latest.data), [0])

    def test_map_replacement_clears_registered_cells(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_pose_array(make_candidates([0.02]))
        self.node._on_map(make_map([]))
        self.assertFalse(self.node.reachability_pub.latest.candidates[0].is_in_reachability)
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)

    def test_invalid_pose_preserved_without_planning(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_pose_array(make_candidates([float("nan")]))
        self.assertEqual(self.node.reachability_pub.latest.candidates[0].reason, "invalid_pose")
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)


if __name__ == "__main__":
    unittest.main()
