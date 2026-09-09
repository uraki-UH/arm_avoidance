#!/usr/bin/env python3
"""GNG入力の差分更新適性を調べる、購読専用の有限時間計測。"""

import argparse
import json
import math
import statistics
import time
from collections import OrderedDict
from pathlib import Path


def vector_tuple(value):
    return (float(value.x), float(value.y), float(value.z))


def frame_key(message):
    return (message.header.frame_id, message.header.stamp.sec,
            message.header.stamp.nanosec, int(message.frame_number))


def graph_snapshot(message):
    ids = [int(node.id) for node in message.nodes]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate node IDs")
    nodes = {int(node.id): (vector_tuple(node.pos), vector_tuple(node.normal),
                           float(node.rho), int(node.label)) for node in message.nodes}
    if any(not math.isfinite(value) for data in nodes.values()
           for value in (*data[0], *data[1], data[2])):
        raise ValueError("nonfinite position, normal or rho")
    edges = set()
    if len(message.edges) % 2:
        raise ValueError("odd edge array length")
    for idx in range(0, len(message.edges), 2):
        first, second = int(message.edges[idx]), int(message.edges[idx + 1])
        if not (0 <= first < len(ids) and 0 <= second < len(ids)):
            raise ValueError("edge endpoint outside node array")
        if first != second:
            edges.add(tuple(sorted((ids[first], ids[second]))))
    return {"key": frame_key(message), "ids": ids, "nodes": nodes, "edges": edges}


def ratio(count, total):
    return count / total if total else 0.0


def normal_angle_deg(first, second):
    norm = math.sqrt(sum(v * v for v in first) * sum(v * v for v in second))
    if norm == 0:
        return None
    cos = abs(sum(a * b for a, b in zip(first, second))) / norm
    return math.degrees(math.acos(min(1.0, max(0.0, cos))))


def graph_delta(previous, current):
    old, new = previous["nodes"], current["nodes"]
    common = old.keys() & new.keys()
    added, removed = new.keys() - old.keys(), old.keys() - new.keys()
    moved, normal_changed, rho_changed, label_changed = set(), set(), set(), set()
    shifts, angles = [], []
    for node_id in common:
        first, second = old[node_id], new[node_id]
        if first[0] != second[0]:
            moved.add(node_id)
        shifts.append(math.dist(first[0], second[0]))
        if first[1] != second[1] and first[1] != tuple(-v for v in second[1]):
            normal_changed.add(node_id)
        angle = normal_angle_deg(first[1], second[1])
        if angle is not None:
            angles.append(angle)
        if first[2] != second[2]:
            rho_changed.add(node_id)
        if first[3] != second[3]:
            label_changed.add(node_id)
    added_edges = current["edges"] - previous["edges"]
    removed_edges = previous["edges"] - current["edges"]
    dirty = set(added | removed | moved | normal_changed | rho_changed | label_changed)
    for edge in added_edges | removed_edges:
        dirty.update(edge)
    expanded = set(dirty)
    # 局所間隔・近傍法線・代替法線推定への影響を含む1ホップ拡張。
    for first, second in previous["edges"] | current["edges"]:
        if first in dirty or second in dirty:
            expanded.update((first, second))
    row = {
        "frame_number": current["key"][3],
        "frame_gap": (current["key"][3] - previous["key"][3]) % (2 ** 32),
        "num_nodes": len(new), "num_edges": len(current["edges"]),
        "num_common_nodes": len(common), "num_added_nodes": len(added),
        "num_removed_nodes": len(removed), "num_added_edges": len(added_edges),
        "num_removed_edges": len(removed_edges),
        "position_changed_ratio": ratio(len(moved), len(common)),
        "normal_changed_ratio": ratio(len(normal_changed), len(common)),
        "rho_changed_ratio": ratio(len(rho_changed), len(common)),
        "label_changed_ratio": ratio(len(label_changed), len(common)),
        "node_dirty_ratio": ratio(len(dirty & new.keys()), len(new)),
        "node_dirty_one_hop_ratio": ratio(len(expanded & new.keys()), len(new)),
        "edge_changed_ratio": ratio(len(added_edges | removed_edges),
                                    len(previous["edges"] | current["edges"])),
        "mean_position_shift_m": statistics.fmean(shifts) if shifts else 0.0,
        "max_position_shift_m": max(shifts, default=0.0),
        "num_invalid_normal_pairs": len(common) - len(angles),
    }
    # 許容幅導入時の参考値。現在の検出条件や厳密な差分率とは別の観測量。
    for dist_th in (0.0001, 0.001, 0.005):
        row[f"position_shift_gt_{dist_th:g}m_ratio"] = ratio(
            sum(value > dist_th for value in shifts), len(shifts))
    for angle_th in (0.1, 1.0, 5.0):
        row[f"normal_angle_gt_{angle_th:g}deg_ratio"] = ratio(
            sum(value > angle_th for value in angles), len(angles))
    return row, expanded


def plane_snapshot(message, graph):
    clusters, owners = {}, {}
    for cluster in message.clusters:
        indices = [int(idx) for idx in cluster.node_indices]
        if any(idx < 0 or idx >= len(graph["ids"]) for idx in indices):
            raise ValueError("plane member outside matching graph")
        members = {graph["ids"][idx] for idx in indices}
        cluster_id = int(cluster.id)
        if cluster_id in clusters or any(node_id in owners for node_id in members):
            raise ValueError("duplicate plane IDs or overlapping memberships")
        fit = (*vector_tuple(cluster.centroid), *vector_tuple(cluster.normal),
               *cluster.position_covariance, float(cluster.local_spacing),
               float(cluster.planarity), float(cluster.residual_ratio))
        clusters[cluster_id] = {"members": members, "fit": fit, "edges": set()}
        owners.update({node_id: cluster_id for node_id in members})
    for edge in graph["edges"]:
        owner = owners.get(edge[0])
        if owner is not None and owner == owners.get(edge[1]):
            clusters[owner]["edges"].add(edge)
    return clusters


def plane_delta(previous, current, graph, dirty):
    membership_changed, edge_changed, fit_changed, affected = set(), set(), set(), set()
    for cluster_id, cluster in current.items():
        old = previous.get(cluster_id)
        if old is None or old["members"] != cluster["members"]:
            membership_changed.add(cluster_id)
        if old is None or old["edges"] != cluster["edges"]:
            edge_changed.add(cluster_id)
        if old is None or old["fit"] != cluster["fit"]:
            fit_changed.add(cluster_id)
        if (cluster_id in membership_changed or cluster_id in edge_changed or
                cluster_id in fit_changed or cluster["members"] & dirty):
            affected.add(cluster_id)
    affected_nodes = set()
    for cluster_id in affected:
        affected_nodes.update(current[cluster_id]["members"])
    return {
        "frame_number": graph["key"][3], "num_clusters": len(current),
        "num_added_clusters": len(current.keys() - previous.keys()),
        "num_removed_clusters": len(previous.keys() - current.keys()),
        "cluster_membership_changed_ratio": ratio(len(membership_changed), len(current)),
        "cluster_internal_edges_changed_ratio": ratio(len(edge_changed), len(current)),
        "cluster_fit_changed_ratio": ratio(len(fit_changed), len(current)),
        "cluster_affected_ratio": ratio(len(affected), len(current)),
        "cluster_affected_nodes_ratio": ratio(len(affected_nodes), len(graph["nodes"])),
    }


def summarize(rows):
    if not rows:
        return {}
    output = {}
    for key in rows[0]:
        if key == "frame_number":
            continue
        values = sorted(row[key] for row in rows)
        output[key] = {"mean": statistics.fmean(values), "p50": statistics.median(values),
                       "p95": values[math.ceil(0.95 * len(values)) - 1], "max": values[-1]}
    return output


def main():
    # 純粋な差分計算の単体テストではROS環境を不要とする遅延import。
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from ais_gng_msgs.msg import PlaneClusterArray, TopologicalMap

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-topic", default="/topological_map")
    parser.add_argument("--plane-topic", default="/plane_clusters")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.duration_sec <= 0 or args.max_frames < 2:
        parser.error("duration must be positive and max-frames must be at least 2")
    rclpy.init()
    node = rclpy.create_node("plane_change_probe")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    graphs, planes = OrderedDict(), OrderedDict()
    graph_rows, plane_rows, errors = [], [], []
    previous_graph = None
    previous_pair = None
    num_maps = num_planes = 0
    num_captured_maps = num_captured_planes = 0
    raw_messages = []
    started = time.monotonic()

    def match(key):
        nonlocal previous_pair
        if key not in graphs or key not in planes:
            return
        graph, message = graphs.pop(key), planes.pop(key)
        try:
            clusters = plane_snapshot(message, graph)
            if previous_pair is not None and previous_pair[0]["key"][0] == key[0]:
                row, dirty = graph_delta(previous_pair[0], graph)
                plane_row = plane_delta(previous_pair[1], clusters, graph, dirty)
                plane_row["frame_gap"] = row["frame_gap"]
                plane_rows.append(plane_row)
            previous_pair = (graph, clusters)
        except ValueError as error:
            errors.append(str(error))
            previous_pair = None

    def on_map(message):
        nonlocal previous_graph, num_maps
        if num_maps >= args.max_frames:
            return
        num_maps += 1
        try:
            graph = graph_snapshot(message)
            if previous_graph is not None and previous_graph["key"][0] == graph["key"][0]:
                graph_rows.append(graph_delta(previous_graph, graph)[0])
            previous_graph = graph
            graphs[graph["key"]] = graph
            match(graph["key"])
            while len(graphs) > 16:
                graphs.popitem(last=False)
        except ValueError as error:
            errors.append(str(error))
            previous_graph = None

    def on_plane(message):
        nonlocal num_planes
        num_planes += 1
        key = frame_key(message)
        planes[key] = message
        match(key)
        while len(planes) > 16:
            planes.popitem(last=False)

    def capture_map(data):
        nonlocal num_captured_maps
        if num_captured_maps < args.max_frames:
            raw_messages.append((True, data))
            num_captured_maps += 1

    def capture_plane(data):
        nonlocal num_captured_planes
        if num_captured_planes < 2 * args.max_frames:
            raw_messages.append((False, data))
            num_captured_planes += 1

    # 受信中はCDRバイト列の保持だけとし、Python差分計算による購読遅延を回避。
    node.create_subscription(TopologicalMap, args.map_topic, capture_map, qos, raw=True)
    node.create_subscription(PlaneClusterArray, args.plane_topic, capture_plane, qos, raw=True)
    try:
        while time.monotonic() - started < args.duration_sec and num_captured_maps < args.max_frames:
            rclpy.spin_once(node, timeout_sec=0.1)
        # 最終グラフと同一フレームの平面メッセージの受信猶予。
        end = time.monotonic() + 0.5
        while num_captured_maps and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        capture_elapsed_sec = time.monotonic() - started
    finally:
        node.destroy_node()
        rclpy.shutdown()

    processing_started = time.monotonic()
    for is_graph, data in raw_messages:
        if is_graph:
            on_map(deserialize_message(data, TopologicalMap))
        else:
            on_plane(deserialize_message(data, PlaneClusterArray))
    consecutive_graph_rows = [row for row in graph_rows if row["frame_gap"] == 1]
    consecutive_plane_rows = [row for row in plane_rows if row["frame_gap"] == 1]
    report = {
        "map_topic": args.map_topic, "plane_topic": args.plane_topic,
        "elapsed_sec": capture_elapsed_sec,
        "processing_sec": time.monotonic() - processing_started,
        "num_map_messages": num_maps, "num_plane_messages": num_planes,
        "num_graph_comparisons": len(graph_rows), "num_plane_comparisons": len(plane_rows),
        "num_graph_gap_pairs": len(graph_rows) - len(consecutive_graph_rows),
        "num_plane_gap_pairs": len(plane_rows) - len(consecutive_plane_rows),
        "errors": errors, "graph_summary": summarize(graph_rows),
        "plane_summary": summarize(plane_rows),
        "graph_consecutive_summary": summarize(consecutive_graph_rows),
        "plane_consecutive_summary": summarize(consecutive_plane_rows),
        "graph_rows": graph_rows, "plane_rows": plane_rows,
    }
    Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("graph_rows", "plane_rows")}, indent=2))
    return 0 if graph_rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
