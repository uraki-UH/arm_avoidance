"""上方把持launchの共通トピック出力と個別指定の結合確認。"""

import json
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
from std_msgs.msg import Float32MultiArray, String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def check_case(node, qos, root, enable_override, candidate_frame="topic_contract_candidate_frame"):
    prefix = "/topic_contract" if enable_override else ""
    topics = {
        "candidate_topic": prefix + "/grasp_pose_cands",
        "candidate_nodes_topic": prefix + "/grasp_pose_cands/nodes",
        "score_topic": prefix + "/grasp_pose_cand_scores",
        "summary_topic": prefix + "/grasp_pose_cands/summary",
    }
    received = {}
    subscriptions = []
    for key, msg_type in (
        ("candidate_topic", GraspCandidateArray),
        ("score_topic", Float32MultiArray),
        ("summary_topic", String),
        ("candidate_nodes_topic", MarkerArray),
    ):
        subscriptions.append(node.create_subscription(
            msg_type, topics[key],
            lambda msg, key=key: received.__setitem__(key, msg), qos))

    map_pub = node.create_publisher(TopologicalMap, "/topological_map", qos)
    cluster_pub = node.create_publisher(PlaneClusterArray, "/plane_clusters", qos)
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
    )
    config_parameters["candidate_frame"] = candidate_frame
    config_parameters["tcp_frame"] = "test_tcp"
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
                    scores = received["score_topic"]
                    if not poses.candidates:
                        continue
                    markers = received["candidate_nodes_topic"].markers
                    if len(markers) != 3:
                        continue
                    assert len(poses.candidates) == len(scores.data) == 1
                    assert summary["candidate_count"] == 1
                    assert summary["candidates"][0]["cluster_id"] == 1
                    assert summary["candidates"][0]["attached_node_num"] == 1
                    assert summary["candidates"][0]["attached_component_num"] == 1
                    expected_frame = candidate_frame or graph.header.frame_id
                    assert summary["candidate_frame"] == expected_frame
                    assert summary["tcp_frame"] == "test_tcp"
                    assert poses.header.frame_id == expected_frame
                    assert poses.tcp_frame == "test_tcp"
                    assert 0.0 < scores.data[0] <= 1.0
                    assert poses.update_id > 0
                    assert poses.candidates[0].id == 0
                    assert poses.candidates[0].state == GraspCandidate.UNKNOWN
                    assert poses.candidates[0].shape_score == scores.data[0]
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
                            assert markers[0].action == Marker.DELETEALL
                            assert len(markers) == (1 if is_blocked else 3)
                            assert len(poses.candidates) == (0 if is_blocked else 1)
                            assert summary["candidate_count"] == len(poses.candidates)
                            assert summary["rejected_approach_obstacle"] == int(is_blocked)
                            if not is_blocked:
                                assert summary["candidates"][0]["attached_node_num"] == 1
                            print(f"進入障害物の遷移確認: is_blocked={is_blocked}", flush=True)
                            break
                        else:
                            raise TimeoutError("障害物変更後の候補待機の時間超過")
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
                        assert not received["score_topic"].data
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
