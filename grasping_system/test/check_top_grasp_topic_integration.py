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
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def has_marker_color(marker, color):
    return all(
        abs(value - expected) < 1.0e-6
        for value, expected in zip(
            (marker.color.r, marker.color.g, marker.color.b, marker.color.a), color))


def has_node_color(markers, color):
    return len(markers) == 3 and all(has_marker_color(marker, color) for marker in markers[1:])


def check_case(node, qos, root, enable_override, candidate_frame="topic_contract_candidate_frame"):
    prefix = "/topic_contract" if enable_override else ""
    topics = {
        "candidate_topic": prefix + "/grasp_pose_cands",
        "candidate_nodes_topic": prefix + "/grasp_pose_cands/nodes",
        "summary_topic": prefix + "/grasp_pose_cands/summary",
    }
    received = {}
    subscriptions = []
    for key, msg_type in (
        ("candidate_topic", GraspCandidateArray),
        ("summary_topic", String),
        ("candidate_nodes_topic", MarkerArray),
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
    clusters.clusters = [cluster]
    params_path = root / "gng_vlut_system/config/ToPoDualArm.yaml"
    config_dir = tempfile.TemporaryDirectory(prefix="top_grasp_topic_contract_")
    # 名前付きYAMLとlaunch引数の出力先・候補座標系の整合
    config = yaml.safe_load(params_path.read_text())
    config_parameters = config["/top_grasp_surface_estimator"]["ros__parameters"]
    config_parameters.update(topics)
    # 通常運用の旧方式設定とは独立した、追加判定の回帰検証
    config_parameters.update(
        max_surface_tilt_deg=25.0,
        enable_nonplane_attachment=True,
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
        "ros2", "launch", "grasping_system", "top_grasp_surface_estimator.launch.py",
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
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if len(received) != len(topics):
                        continue
                    summary = json.loads(received["summary_topic"].data)
                    poses = received["candidate_topic"]
                    if not poses.candidates:
                        observed_unconfirmed_candidate = True
                        continue
                    markers = received["candidate_nodes_topic"].markers
                    if len(poses.candidates) != 1 or len(markers) != 3:
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
                    assert has_node_color(markers, (0.1, 0.85, 1.0, 1.0))
                    expected_offset = (0.2, -0.1, 0.3) if candidate_frame else (0.0, 0.0, 0.0)
                    assert abs(poses.candidates[0].pose.position.x - expected_offset[0]) < 1.0e-6
                    assert abs(poses.candidates[0].pose.position.y - expected_offset[1]) < 1.0e-6
                    assert abs(poses.candidates[0].pose.position.z - (0.1 + expected_offset[2])) < 1.0e-6
                    assert markers[0].action == Marker.DELETEALL
                    assert markers[1].ns == "grasp_plane"
                    assert markers[2].ns == "grasp_nonplane"
                    assert len(markers[1].points) == 4
                    assert len(markers[2].points) == 1
                    for marker in markers[1:]:
                        assert marker.id == poses.candidates[0].id
                        assert marker.type == Marker.SPHERE_LIST
                        assert marker.header.frame_id == poses.header.frame_id
                        assert marker.pose.orientation.w == 1.0
                        assert abs(marker.scale.x - 0.012) < 1.0e-9
                        assert marker.color.a == 1.0
                    # 元ノード添字からの抽出と、候補評価座標系への一度だけの変換
                    for point, original in zip(markers[1].points, graph.nodes[:4]):
                        assert abs(point.x - (original.pos.x + expected_offset[0])) < 1.0e-6
                        assert abs(point.y - (original.pos.y + expected_offset[1])) < 1.0e-6
                        assert abs(point.z - (original.pos.z + expected_offset[2])) < 1.0e-6
                    assert abs(markers[2].points[0].x - (0.025 + expected_offset[0])) < 1.0e-6
                    assert abs(markers[2].points[0].z - (0.08 + expected_offset[2])) < 1.0e-6
                    assert node.count_publishers(topics["candidate_nodes_topic"]) == 1
                    assert node.count_publishers(topics["candidate_topic"]) == 1
                    assert node.count_publishers("/grasp_pose_cands/reachability") == 0
                    assert node.count_publishers("/grasp_pose_cands/reachability_markers") == 0
                    assert node.count_publishers("/grasp_pose_markers") == 0
                    assert not any(
                        "grasp_pose_marker_bridge" in name
                        for name in node.get_node_names())
                    print(f"確認成功: {topics}", flush=True)
                    # 新しい入力を送らない状態での、遅延購読による最新表示取得
                    late_markers = []
                    late_subscription = node.create_subscription(
                        MarkerArray, topics["candidate_nodes_topic"], late_markers.append, qos)
                    try:
                        deadline = time.monotonic() + 5.0
                        while not late_markers and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        assert late_markers and len(late_markers[-1].markers) == 3
                    finally:
                        node.destroy_subscription(late_subscription)
                    # 学習入力なしでの到達map更新。位置・候補IDを維持した色だけの遷移
                    deadline = time.monotonic() + 0.2
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    held_poses = received["candidate_topic"]
                    held_markers = received["candidate_nodes_topic"].markers
                    reach_node = TopologicalNode()
                    point = held_poses.candidates[0].pose.position
                    reach_node.pos.x = (math.floor(point.x / 0.05) + 0.5) * 0.05
                    reach_node.pos.y = (math.floor(point.y / 0.05) + 0.5) * 0.05
                    reach_node.pos.z = (math.floor(point.z / 0.05) + 0.5) * 0.05
                    for state, color in (
                        (GraspCandidate.INSIDE, (0.0, 0.6375969, 1.0, 1.0)),
                        (GraspCandidate.OUTSIDE, (0.1, 0.85, 1.0, 1.0)),
                        (GraspCandidate.UNKNOWN, (0.1, 0.85, 1.0, 1.0)),
                        (GraspCandidate.INSIDE, (0.0, 0.6375969, 1.0, 1.0)),
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
                            markers = received["candidate_nodes_topic"].markers
                            if poses.candidates[0].state == state and has_node_color(markers, color):
                                assert poses.update_id == held_poses.update_id
                                assert poses.candidates[0].id == held_poses.candidates[0].id
                                assert poses.candidates[0].pose == held_poses.candidates[0].pose
                                for marker, held in zip(markers, held_markers):
                                    assert marker.header == held.header
                                    assert marker.points == held.points
                                    assert marker.id == held.id
                                break
                        else:
                            raise TimeoutError(f"採用ノードの到達性色待機の時間超過: {state}")
                    print("到達mapだけの更新によるHANDLE色・候補色の切り替え確認", flush=True)
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
                            markers = received["candidate_nodes_topic"].markers
                            stamp = poses.header.stamp
                            if (summary["frame_number"] < min_frame or
                                    stamp.sec * 1000000000 + stamp.nanosec < start_stamp or
                                    not markers or
                                    markers[0].header.stamp.sec * 1000000000 +
                                    markers[0].header.stamp.nanosec < start_stamp):
                                continue
                            expected_candidate_num = 0 if is_blocked else 1
                            expected_marker_num = 1 if is_blocked else 3
                            if (len(markers) != expected_marker_num or
                                    len(poses.candidates) != expected_candidate_num):
                                continue
                            expected_raw_candidate_num = 0 if is_blocked else 1
                            if (summary["candidate_count"] != expected_candidate_num or
                                    summary["raw_candidate_count"] != expected_raw_candidate_num or
                                    summary["rejected_approach_obstacle"] != int(is_blocked)):
                                continue
                            assert markers[0].action == Marker.DELETEALL
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
                    deadline = time.monotonic() + 5.0
                    while time.monotonic() < deadline:
                        graph.frame_number += 1
                        graph.header.stamp = node.get_clock().now().to_msg()
                        clusters.header, clusters.frame_number = graph.header, graph.frame_number
                        map_pub.publish(graph)
                        cluster_pub.publish(clusters)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        poses = received["candidate_topic"]
                        markers = received["candidate_nodes_topic"].markers
                        if len(poses.candidates) != 2 or len(markers) != 4:
                            continue
                        assert poses.candidates[0].state == GraspCandidate.OUTSIDE
                        assert poses.candidates[1].state == GraspCandidate.INSIDE
                        for marker in markers[1:]:
                            expected = (0.1, 0.85, 1.0, 1.0) if marker.id == 2 else (0.0, 0.6375969, 1.0, 1.0)
                            assert has_marker_color(marker, expected)
                        print("異なる候補IDのHANDLE色・候補色の同時表示確認", flush=True)
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
                        markers = received["candidate_nodes_topic"].markers
                        stamp = poses.header.stamp
                        if (summary["frame_number"] < min_frame or
                                stamp.sec * 1000000000 + stamp.nanosec < start_stamp or
                                not markers or markers[0].header.stamp.sec * 1000000000 +
                                markers[0].header.stamp.nanosec < start_stamp):
                            continue
                        assert summary["status"] == "tf_unavailable"
                        assert not poses.candidates
                        assert poses.header.frame_id == "topic_contract_candidate_frame"
                        assert poses.tcp_frame == "test_tcp"
                        assert len(markers) == 1 and markers[0].action == Marker.DELETEALL
                        assert markers[0].header.frame_id == poses.header.frame_id
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
