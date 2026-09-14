"""上方把持launchの共通トピック出力と個別指定の結合確認。"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ais_gng_msgs.msg import PlaneCluster, PlaneClusterArray, TopologicalMap, TopologicalNode
from gng_control_msgs.msg import GraspCandidate, GraspCandidateArray
from geometry_msgs.msg import TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def has_graph_state(graph, semantic_label):
    return (len(graph.nodes) == 5 and len(graph.clusters) == 1 and
            graph.clusters[0].semantic_label == semantic_label and
            all(point.semantic_label == semantic_label for point in graph.nodes))


def check_internal_edges(graph):
    membership = {node_id: cluster.id for cluster in graph.clusters for node_id in cluster.nodes}
    assert len(membership) == len(graph.nodes)
    for first, second in zip(graph.edges[::2], graph.edges[1::2]):
        assert first < len(graph.nodes) and second < len(graph.nodes)
        assert membership[graph.nodes[first].id] == membership[graph.nodes[second].id]


def check_case(node, qos, root, enable_override, candidate_frame="topic_contract_candidate_frame"):
    prefix = "/topic_contract" if enable_override else ""
    topics = {
        "candidate_topic": prefix + "/grasp_pose_cands",
        "candidate_graph_topic": prefix + "/grasp_pose_cands/Tmap",
        "summary_topic": prefix + "/grasp_pose_cands/summary",
    }
    received = {}
    subscriptions = []
    for key, msg_type in (
        ("candidate_topic", GraspCandidateArray),
        ("summary_topic", String),
        ("candidate_graph_topic", TopologicalMap),
    ):
        subscriptions.append(node.create_subscription(
            msg_type, topics[key],
            lambda msg, key=key: received.__setitem__(key, msg), qos))

    map_pub = node.create_publisher(TopologicalMap, "/topological_map", qos)
    cluster_pub = node.create_publisher(PlaneClusterArray, "/plane_clusters", qos)
    reach_map_topic = prefix + "/test/reach_map"
    reach_map_pub = node.create_publisher(TopologicalMap, reach_map_topic, qos)
    graph = TopologicalMap()
    graph.header.frame_id = "topic_contract_frame"
    cluster = PlaneCluster()
    cluster.id = 1
    cluster.centroid.z = 0.1
    cluster.normal.z = 1.0
    cluster.local_spacing = 0.05
    for x, y in ((-0.015, -0.02), (-0.015, 0.02), (0.015, -0.02), (0.015, 0.02)):
        point = TopologicalNode()
        point.id = len(graph.nodes)
        point.pos.x, point.pos.y, point.pos.z = x, y, 0.1
        cluster.node_indices.append(len(graph.nodes))
        graph.nodes.append(point)
    attached = TopologicalNode()
    attached.id = 99
    attached.nonplane_component_id = 7
    attached.pos.x, attached.pos.z = 0.025, 0.08
    graph.nodes.append(attached)
    graph.edges = [3, 4]
    clusters = PlaneClusterArray()
    # 参照面を用いた通常方式の付属抽出。付属ノードの添字4は維持
    reference = PlaneCluster()
    reference.id = 90
    reference.normal.z = 1.0
    for x, y in ((-0.3, -0.3), (-0.3, 0.3), (0.3, -0.3), (0.3, 0.3)):
        point = TopologicalNode()
        point.id = len(graph.nodes)
        point.pos.x, point.pos.y, point.pos.z = x, y, 0.0
        reference.node_indices.append(len(graph.nodes))
        graph.nodes.append(point)
    graph.edges.extend([0, reference.node_indices[0]])
    clusters.clusters = [cluster, reference]
    params_path = root / "gng_vlut_system/config/ToPoDualArm.yaml"
    config_dir = tempfile.TemporaryDirectory(prefix="top_grasp_topic_contract_")
    # 名前付きYAMLとlaunch引数の出力先・候補座標系の整合
    config = yaml.safe_load(params_path.read_text())
    config_parameters = config["/top_grasp_surface_estimator"]["ros__parameters"]
    config_parameters.update(topics)
    # 通常の付属探索と、独立した進入障害物判定の回帰検証
    config_parameters.update(
        max_surface_tilt_deg=25.0,
        enable_candidate_frame_passthrough=False,
        enable_approach_check=True,
        candidate_confirm_updates=3,
        candidate_missing_update_allowance=2,
        candidate_position_ema_alpha=1.0,
        candidate_orientation_ema_alpha=1.0,
    )
    config_parameters["candidate_frame"] = candidate_frame
    config_parameters["tcp_frame"] = "test_tcp"
    config_parameters["reachability_map_topic"] = reach_map_topic
    config_parameters["reachability_voxel_size"] = 0.05
    params_path = Path(config_dir.name) / "params.yaml"
    params_path.write_text(yaml.safe_dump(config))
    command = [
        "ros2", "launch", "grasping_system", "top_grasp_pose_candidates.launch.py",
        f"params_file:={params_path}",
    ]
    if enable_override:
        command.extend(f"{key}:={value}" for key, value in topics.items())
    process = None
    try:
        with tempfile.TemporaryFile(mode="w+") as log:
            print("起動: " + " ".join(command), flush=True)
            process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 25.0
                observed_unconfirmed_candidate = False
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError("launchの予期しない終了")
                    graph.frame_number += 1
                    graph.header.stamp = node.get_clock().now().to_msg()
                    clusters.header = graph.header
                    clusters.frame_number = graph.frame_number
                    map_pub.publish(graph)
                    cluster_pub.publish(clusters)
                    # 次の入力前に3出力を受信するための待機。depth 1による未確定出力の上書き防止
                    receive_until = time.monotonic() + 0.1
                    while time.monotonic() < receive_until:
                        rclpy.spin_once(node, timeout_sec=0.01)
                    if len(received) != len(topics):
                        continue
                    summary = json.loads(received["summary_topic"].data)
                    poses = received["candidate_topic"]
                    if not poses.candidates:
                        observed_unconfirmed_candidate = True
                        continue
                    candidate_graph = received["candidate_graph_topic"]
                    if len(poses.candidates) != 1 or len(candidate_graph.nodes) != 5:
                        continue
                    if (summary["candidate_count"] != 1 or
                            summary["raw_candidate_count"] != 1):
                        continue
                    assert len(poses.candidates) == 1
                    assert summary["candidate_count"] == 1
                    assert summary["raw_candidate_count"] == 1
                    assert summary["candidates"][0]["cluster_id"] == 1
                    assert summary["candidates"][0]["attached_node_num"] == 1
                    assert summary["candidates"][0]["attached_component_num"] == 1
                    expected_frame = candidate_frame or graph.header.frame_id
                    assert summary["candidate_frame"] == expected_frame
                    assert summary["tcp_frame"] == "test_tcp"
                    assert poses.header.frame_id == expected_frame
                    assert poses.tcp_frame == "test_tcp"
                    assert 0.0 < poses.candidates[0].shape_score <= 1.0
                    assert poses.update_id > 0
                    assert observed_unconfirmed_candidate
                    assert poses.candidates[0].id == 1
                    assert poses.candidates[0].state == GraspCandidate.UNKNOWN
                    assert has_graph_state(candidate_graph, TopologicalMap.SEMANTIC_GRASP_UNKNOWN)
                    expected_offset = (0.2, -0.1, 0.3) if candidate_frame else (0.0, 0.0, 0.0)
                    assert abs(poses.candidates[0].pose.position.x - expected_offset[0]) < 1.0e-6
                    assert abs(poses.candidates[0].pose.position.y - expected_offset[1]) < 1.0e-6
                    assert abs(poses.candidates[0].pose.position.z - (0.1 + expected_offset[2])) < 1.0e-6
                    assert candidate_graph.header == poses.header
                    assert candidate_graph.clusters[0].id == poses.candidates[0].id
                    assert list(candidate_graph.clusters[0].nodes) == list(range(5))
                    assert list(candidate_graph.edges) == [3, 4]
                    assert candidate_graph.nodes[4].nonplane_component_id == 7
                    check_internal_edges(candidate_graph)
                    assert node.count_publishers(prefix + "/grasp_pose_cands/nodes") == 0
                    # 元ノード添字からの抽出と、候補評価座標系への一度だけの変換
                    for entry, original in zip(candidate_graph.nodes[:4], graph.nodes[:4]):
                        point = entry.pos
                        assert abs(point.x - (original.pos.x + expected_offset[0])) < 1.0e-6
                        assert abs(point.y - (original.pos.y + expected_offset[1])) < 1.0e-6
                        assert abs(point.z - (original.pos.z + expected_offset[2])) < 1.0e-6
                    assert abs(candidate_graph.nodes[4].pos.x - (0.025 + expected_offset[0])) < 1.0e-6
                    assert abs(candidate_graph.nodes[4].pos.z - (0.08 + expected_offset[2])) < 1.0e-6
                    assert node.count_publishers(topics["candidate_graph_topic"]) == 1
                    assert node.count_publishers(topics["candidate_topic"]) == 1
                    assert node.count_publishers("/grasp_pose_cands/reachability") == 0
                    assert node.count_publishers("/grasp_pose_cands/reachability_markers") == 0
                    assert node.count_publishers("/grasp_pose_markers") == 0
                    assert not any(
                        "grasp_pose_marker_bridge" in name
                        for name in node.get_node_names())
                    print(f"確認成功: {topics}", flush=True)
                    # 新しい入力を送らない状態での、遅延購読による最新表示取得
                    late_graphs = []
                    late_subscription = node.create_subscription(
                        TopologicalMap, topics["candidate_graph_topic"], late_graphs.append, qos)
                    try:
                        deadline = time.monotonic() + 5.0
                        while not late_graphs and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        assert late_graphs and len(late_graphs[-1].nodes) == 5
                    finally:
                        node.destroy_subscription(late_subscription)
                    # 学習入力なしでの到達map更新。位置・候補IDを維持した色だけの遷移
                    deadline = time.monotonic() + 0.2
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    held_poses = received["candidate_topic"]
                    held_candidate_graph = received["candidate_graph_topic"]
                    reach_node = TopologicalNode()
                    point = held_poses.candidates[0].pose.position
                    reach_node.pos.x = (math.floor(point.x / 0.05) + 0.5) * 0.05
                    reach_node.pos.y = (math.floor(point.y / 0.05) + 0.5) * 0.05
                    reach_node.pos.z = (math.floor(point.z / 0.05) + 0.5) * 0.05
                    for state, semantic_label in (
                        (GraspCandidate.INSIDE, TopologicalMap.SEMANTIC_GRASP_INSIDE),
                        (GraspCandidate.OUTSIDE, TopologicalMap.SEMANTIC_GRASP_OUTSIDE),
                        (GraspCandidate.UNKNOWN, TopologicalMap.SEMANTIC_GRASP_UNKNOWN),
                        (GraspCandidate.INSIDE, TopologicalMap.SEMANTIC_GRASP_INSIDE),
                    ):
                        reach_map = TopologicalMap()
                        reach_map.header.frame_id = (
                            "missing_reach_frame" if state == GraspCandidate.UNKNOWN else expected_frame)
                        reach_map.nodes = [] if state == GraspCandidate.OUTSIDE else [reach_node]
                        reach_map_pub.publish(reach_map)
                        deadline = time.monotonic() + 5.0
                        while time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                            poses = received["candidate_topic"]
                            candidate_graph = received["candidate_graph_topic"]
                            if poses.candidates[0].state == state and has_graph_state(candidate_graph, semantic_label):
                                assert poses.update_id == held_poses.update_id
                                assert poses.candidates[0].id == held_poses.candidates[0].id
                                assert poses.candidates[0].pose == held_poses.candidates[0].pose
                                assert candidate_graph.header == held_candidate_graph.header
                                assert candidate_graph.frame_number == held_candidate_graph.frame_number
                                assert candidate_graph.edges == held_candidate_graph.edges
                                for entry, held in zip(candidate_graph.nodes, held_candidate_graph.nodes):
                                    assert entry.pos == held.pos
                                    assert entry.id == held.id
                                break
                        else:
                            raise TimeoutError(f"採用ノードの到達性色待機の時間超過: {state}")
                    print("到達mapだけの更新によるグラフ状態・候補状態の切り替え確認", flush=True)
                    # 上方障害物で空候補へ遷移し、除去後に同じ平面から再生成
                    for is_blocked in (True, False):
                        graph.nodes[4].pos.z = 0.15 if is_blocked else 0.08
                        received.clear()
                        min_frame = graph.frame_number + 1
                        start_stamp = node.get_clock().now().nanoseconds
                        deadline = time.monotonic() + 10.0
                        while time.monotonic() < deadline:
                            if process.poll() is not None:
                                raise RuntimeError("launchの予期しない終了")
                            graph.frame_number += 1
                            graph.header.stamp = node.get_clock().now().to_msg()
                            clusters.header = graph.header
                            clusters.frame_number = graph.frame_number
                            map_pub.publish(graph)
                            cluster_pub.publish(clusters)
                            rclpy.spin_once(node, timeout_sec=0.1)
                            if len(received) != len(topics):
                                continue
                            summary = json.loads(received["summary_topic"].data)
                            poses = received["candidate_topic"]
                            candidate_graph = received["candidate_graph_topic"]
                            stamp = poses.header.stamp
                            if (summary["frame_number"] < min_frame or
                                    stamp.sec * 1000000000 + stamp.nanosec < start_stamp or
                                    candidate_graph.header.stamp.sec * 1000000000 +
                                    candidate_graph.header.stamp.nanosec < start_stamp):
                                continue
                            expected_candidate_num = 0 if is_blocked else 1
                            expected_node_num = 0 if is_blocked else 5
                            if (len(candidate_graph.nodes) != expected_node_num or
                                    len(poses.candidates) != expected_candidate_num):
                                continue
                            expected_raw_candidate_num = 0 if is_blocked else 1
                            if (summary["candidate_count"] != expected_candidate_num or
                                    summary["raw_candidate_count"] != expected_raw_candidate_num or
                                    summary["rejected_approach_obstacle"] != int(is_blocked)):
                                continue
                            assert len(candidate_graph.clusters) == expected_candidate_num
                            if is_blocked:
                                assert not candidate_graph.edges
                            assert summary["candidate_count"] == len(poses.candidates)
                            assert summary["raw_candidate_count"] == expected_raw_candidate_num
                            assert summary["rejected_approach_obstacle"] == int(is_blocked)
                            if not is_blocked:
                                assert summary["candidates"][0]["attached_node_num"] == 1
                            print(f"進入障害物の遷移確認: is_blocked={is_blocked}", flush=True)
                            break
                        else:
                            raise TimeoutError("障害物変更後の候補待機の時間超過")
                    # 異なる状態の候補が同居する場合のID対応。高い別平面は範囲外
                    other = PlaneCluster()
                    other.id = 2
                    other.centroid.x, other.centroid.z = 0.3, 0.12
                    other.normal.z = 1.0
                    for original in graph.nodes[:4]:
                        point = TopologicalNode()
                        point.pos.x = original.pos.x + 0.3
                        point.pos.y, point.pos.z = original.pos.y, 0.12
                        other.node_indices.append(len(graph.nodes))
                        graph.nodes.append(point)
                    clusters.clusters.append(other)
                    # 同一候補内の元エッジと、別候補への元エッジの混在
                    graph.edges.extend([other.node_indices[0], other.node_indices[1], 3, other.node_indices[0]])
                    deadline = time.monotonic() + 5.0
                    while time.monotonic() < deadline:
                        graph.frame_number += 1
                        graph.header.stamp = node.get_clock().now().to_msg()
                        clusters.header, clusters.frame_number = graph.header, graph.frame_number
                        map_pub.publish(graph)
                        cluster_pub.publish(clusters)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        poses = received["candidate_topic"]
                        candidate_graph = received["candidate_graph_topic"]
                        if len(poses.candidates) != 2 or len(candidate_graph.clusters) != 2:
                            continue
                        assert poses.candidates[0].state == GraspCandidate.OUTSIDE
                        assert poses.candidates[1].state == GraspCandidate.INSIDE
                        # 再生成時にも採番し直される候補IDと、配信状態との対応
                        state_by_id = {candidate.id: candidate.state for candidate in poses.candidates}
                        assert len(candidate_graph.nodes) == 9
                        assert len(candidate_graph.edges) == 4
                        check_internal_edges(candidate_graph)
                        for group in candidate_graph.clusters:
                            expected = (TopologicalMap.SEMANTIC_GRASP_OUTSIDE
                                        if state_by_id[group.id] == GraspCandidate.OUTSIDE
                                        else TopologicalMap.SEMANTIC_GRASP_INSIDE)
                            assert group.semantic_label == expected
                            assert all(candidate_graph.nodes[idx].semantic_label == expected for idx in group.nodes)
                        print("異なる候補IDのグラフ状態・候補状態の同時表示確認", flush=True)
                        break
                    else:
                        raise TimeoutError("複数候補の色分け待機の時間超過")
                    if not candidate_frame:
                        return
                    # TF欠落時の空候補配信による過去候補の明示的な無効化
                    graph.header.frame_id = "topic_contract_missing_frame"
                    received.clear()
                    min_frame = graph.frame_number + 1
                    start_stamp = node.get_clock().now().nanoseconds
                    deadline = time.monotonic() + 10.0
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError("launchの予期しない終了")
                        graph.frame_number += 1
                        graph.header.stamp = node.get_clock().now().to_msg()
                        clusters.header = graph.header
                        clusters.frame_number = graph.frame_number
                        map_pub.publish(graph)
                        cluster_pub.publish(clusters)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if len(received) != len(topics):
                            continue
                        summary = json.loads(received["summary_topic"].data)
                        poses = received["candidate_topic"]
                        candidate_graph = received["candidate_graph_topic"]
                        stamp = poses.header.stamp
                        if (summary["frame_number"] < min_frame or
                                stamp.sec * 1000000000 + stamp.nanosec < start_stamp or
                                candidate_graph.header.stamp.sec * 1000000000 +
                                candidate_graph.header.stamp.nanosec < start_stamp):
                            continue
                        assert summary["status"] == "tf_unavailable"
                        assert not poses.candidates
                        assert poses.header.frame_id == "topic_contract_candidate_frame"
                        assert poses.tcp_frame == "test_tcp"
                        assert not candidate_graph.nodes and not candidate_graph.edges and not candidate_graph.clusters
                        assert candidate_graph.header.frame_id == poses.header.frame_id
                        print("TF欠落時の空候補配信確認", flush=True)
                        return
                    raise TimeoutError("TF欠落後の空候補待機の時間超過")
                raise TimeoutError(f"出力待機の時間超過: {list(received)}")
            except Exception:
                log.seek(0)
                print(log.read(), flush=True)
                raise
            finally:
                # launch配下を含む、このテスト所有のプロセス群だけの停止
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                print("停止済み: " + " ".join(command), flush=True)
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_publisher(map_pub)
        node.destroy_publisher(cluster_pub)
        node.destroy_publisher(reach_map_pub)
        config_dir.cleanup()


def main():
    # 通常運用と独立したROSドメイン。CLIデーモンの起動なし
    if os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        raise RuntimeError("検証用の非ゼロROS_DOMAIN_IDの指定が必要")
    root = Path(__file__).resolve().parents[2]
    rclpy.init()
    node = rclpy.create_node("top_grasp_topic_contract_test")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = "topic_contract_candidate_frame"
    transform.child_frame_id = "topic_contract_frame"
    transform.transform.translation.x = 0.2
    transform.transform.translation.y = -0.1
    transform.transform.translation.z = 0.3
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    try:
        check_case(node, qos, root, False)
        check_case(node, qos, root, True)
        check_case(node, qos, root, False, candidate_frame="")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
