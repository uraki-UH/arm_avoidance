"""購読なしで実行可能な、ノードID基準の差分計測テスト。"""

import unittest
from types import SimpleNamespace as Message

from measure_plane_change import graph_delta, graph_snapshot, plane_delta, plane_snapshot


def vector(x, y=0.0, z=0.0):
    return Message(x=x, y=y, z=z)


def graph_message(ids, edges, frame=1):
    return Message(
        header=Message(frame_id="map", stamp=Message(sec=frame, nanosec=0)),
        frame_number=frame,
        nodes=[Message(id=node_id, pos=vector(float(node_id)), normal=vector(0, 0, 1),
                       rho=0.0, label=0) for node_id in ids], edges=edges)


class PlaneChangeTest(unittest.TestCase):
    def test_reordered_nodes_keep_members_and_edges(self):
        first = graph_snapshot(graph_message([10, 20, 30], [0, 1, 1, 2]))
        second = graph_snapshot(graph_message([30, 10, 20], [1, 2, 2, 0], 2))
        row, dirty = graph_delta(first, second)
        self.assertEqual(row["node_dirty_ratio"], 0.0)
        self.assertEqual(row["edge_changed_ratio"], 0.0)
        self.assertFalse(dirty)

    def test_one_hop_expansion_does_not_flood_component(self):
        first = graph_snapshot(graph_message([1, 2, 3, 4], [0, 1, 1, 2, 2, 3]))
        changed = graph_message([1, 2, 3, 4], [0, 1, 1, 2, 2, 3], 2)
        changed.nodes[0].pos.x += 0.001
        row, dirty = graph_delta(first, graph_snapshot(changed))
        self.assertEqual(dirty, {1, 2})
        self.assertEqual(row["position_changed_ratio"], 0.25)
        self.assertEqual(row["node_dirty_one_hop_ratio"], 0.5)

    def test_normal_sign_flip_is_ignored(self):
        first = graph_snapshot(graph_message([1, 2], [0, 1]))
        changed = graph_message([1, 2], [0, 1], 2)
        changed.nodes[0].normal.z = -1
        row, dirty = graph_delta(first, graph_snapshot(changed))
        self.assertEqual(row["normal_changed_ratio"], 0.0)
        self.assertEqual(row["normal_angle_gt_0.1deg_ratio"], 0.0)
        self.assertFalse(dirty)

    def test_removed_node_and_edges_dirty_neighbors(self):
        first = graph_snapshot(graph_message([1, 2, 3], [0, 1, 1, 2]))
        second = graph_snapshot(graph_message([1, 3], [], 2))
        row, dirty = graph_delta(first, second)
        self.assertEqual(row["num_removed_nodes"], 1)
        self.assertEqual(row["num_removed_edges"], 2)
        self.assertEqual(row["node_dirty_one_hop_ratio"], 1.0)
        self.assertEqual(dirty, {1, 2, 3})

    def test_plane_indices_resolve_to_matching_graph_ids(self):
        first = graph_snapshot(graph_message([10, 20], [0, 1]))
        second = graph_snapshot(graph_message([20, 10], [1, 0], 2))
        cluster = Message(id=7, node_indices=[0, 1], centroid=vector(15),
                          normal=vector(0, 0, 1), position_covariance=[0.0] * 9,
                          local_spacing=10.0, planarity=1.0, residual_ratio=0.0)
        planes = Message(clusters=[cluster])
        old = plane_snapshot(planes, first)
        new = plane_snapshot(planes, second)
        row = plane_delta(old, new, second, set())
        self.assertEqual(row["cluster_membership_changed_ratio"], 0.0)
        self.assertEqual(row["cluster_internal_edges_changed_ratio"], 0.0)
        self.assertEqual(row["cluster_affected_ratio"], 0.0)
        self.assertEqual(plane_delta(old, new, second, {10})["cluster_affected_nodes_ratio"], 1.0)

        # 入力差分がなくても、平滑化などによる出力平面の変化を更新対象へ算入。
        cluster.centroid.x += 0.001
        changed = plane_snapshot(planes, second)
        self.assertEqual(plane_delta(new, changed, second, set())["cluster_affected_ratio"], 1.0)

    def test_duplicate_ids_and_invalid_edges_are_rejected(self):
        with self.assertRaises(ValueError):
            graph_snapshot(graph_message([1, 1], [0, 1]))
        with self.assertRaises(ValueError):
            graph_snapshot(graph_message([1, 2], [0, 5]))


if __name__ == "__main__":
    unittest.main()
