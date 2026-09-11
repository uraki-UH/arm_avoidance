import argparse
import copy
import importlib.util
from pathlib import Path
import sys
import unittest

import rclpy
from geometry_msgs.msg import Point, Point32, Pose, TransformStamped
from std_msgs.msg import Header
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
from gng_control_msgs.msg import GraspCandidate, GraspCandidateArray

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
        node = TopologicalNode(id=node_id, label=1)
        node.pos = Point32(x=float(point), y=0.01, z=0.01)
        node.normal.z = 1.0
        msg.nodes.append(node)
    return msg


def make_candidates(points, frame="base", state=GraspCandidate.INSIDE):
    msg = GraspCandidateArray(header=Header(frame_id=frame), update_id=1,
                              evaluation_header=Header(frame_id="base"), voxel_size=0.1)
    for idx, point in enumerate(points):
        pose = Pose(position=Point(x=float(point), y=0.01, z=0.01))
        pose.orientation.w = 1.0
        msg.candidates.append(GraspCandidate(id=42 + idx, pose=pose, state=state))
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
            candidate_count=8,
            non_collision_only=True, orientation_weight=0.0,
            candidate_topic="/test/candidates", goal_candidate_ids_topic="/test/goals",
            node_feature_topic="", manipulability_weight=0.0, goal_update_hz=5.0)
        self.node = selector_module.TopologicalMapGoalSelector(args)
        for name in ("output_pub", "goal_candidate_ids_pub"):
            setattr(self.node, name, capture_publisher())

    def tearDown(self):
        self.node.destroy_node()

    def test_only_inside_candidates_select_planning_ids(self):
        self.node._on_map(make_map([0.01, 0.21]))
        source = make_candidates([0.02, 0.22, 8.0])
        source.candidates[0].state = GraspCandidate.OUTSIDE
        source.candidates[2].state = GraspCandidate.UNKNOWN
        before = copy.deepcopy(source)
        self.node._on_candidates(source)
        self.assertEqual(list(self.node.goal_candidate_ids_pub.latest.data), [1])
        self.assertEqual(source, before)
        self.assertFalse(hasattr(self.node, "reachability_pub"))
        self.assertFalse(hasattr(self.node, "reachability_marker_pub"))
        self.assertFalse(hasattr(self.node, "marker_pub"))
        self.assertEqual([node.id for node in self.node.output_pub.latest.nodes], [1])

    def test_unknown_outside_and_empty_clear_old_goals(self):
        self.node._on_map(make_map([0.01]))
        for source in (make_candidates([0.02], state=GraspCandidate.UNKNOWN),
                       make_candidates([0.02], state=GraspCandidate.OUTSIDE),
                       make_candidates([])):
            self.node._on_candidates(make_candidates([0.02]))
            self.assertTrue(self.node.goal_candidate_ids_pub.latest.data)
            self.node._on_candidates(source)
            self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
            self.assertFalse(self.node.output_pub.latest.nodes)

    def test_missing_map_and_tf_and_invalid_pose(self):
        self.node._on_candidates(make_candidates([0.02]))
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)
        self.node._on_map(make_map([0.01]))
        for source in (make_candidates([0.02], "missing_frame"), make_candidates([float("nan")])):
            self.node._on_candidates(source)
            self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)

    def test_motion_rechecks_goal_cells(self):
        self.node._on_map(make_map([0.01]))
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base", "world"
        tf.transform.rotation.w = 1.0
        for offset, expected in ((-1.0, True), (0.0, False), (-1.0, True)):
            tf.transform.translation.x = offset
            self.node.tf_buffer.set_transform_static(tf, "test")
            self.node._on_candidates(make_candidates([1.02], "world"))
            self.assertEqual(bool(self.node.goal_candidate_ids_pub.latest.data), expected)

    def test_inside_without_safe_planning_nodes(self):
        graph = make_map([0.01])
        graph.nodes[0].label = 2
        self.node._on_map(graph)
        self.node._on_candidates(make_candidates([0.02, 0.22]))
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)

    def test_independent_reachability_cells_do_not_supply_goal_ids(self):
        self.node._on_map(make_map([0.01]))
        self.node._on_candidates(make_candidates([0.02, 0.22]))
        self.assertEqual(list(self.node.goal_candidate_ids_pub.latest.data), [0])
        self.node._on_map(make_map([]))
        self.assertFalse(self.node.goal_candidate_ids_pub.latest.data)


if __name__ == "__main__":
    unittest.main()
