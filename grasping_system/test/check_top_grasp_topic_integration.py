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
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray, String
import yaml


def check_case(node, qos, root, enable_override):
    prefix = "/topic_contract" if enable_override else ""
    topics = {
        "candidate_topic": prefix + "/grasp_pose_cands",
        "score_topic": prefix + "/grasp_pose_cand_scores",
        "summary_topic": prefix + "/grasp_pose_cands/summary",
    }
    received = {}
    subscriptions = []
    for key, msg_type in (
        ("candidate_topic", GraspCandidateArray),
        ("score_topic", Float32MultiArray),
        ("summary_topic", String),
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
    clusters = PlaneClusterArray()
    clusters.clusters = [cluster]
    params_path = root / "gng_vlut_system/config/ToPoDualArm.yaml"
    config_dir = tempfile.TemporaryDirectory(prefix="top_grasp_topic_contract_")
    if enable_override:
        # 名前付きYAMLとlaunch引数の出力先の整合
        config = yaml.safe_load(params_path.read_text())
        config["/top_grasp_surface_estimator"]["ros__parameters"].update(topics)
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
                    assert len(poses.candidates) == len(scores.data) == 1
                    assert summary["candidate_count"] == 1
                    assert summary["candidates"][0]["cluster_id"] == 1
                    assert poses.header.frame_id == graph.header.frame_id
                    assert 0.0 < scores.data[0] <= 1.0
                    assert poses.update_id > 0
                    assert poses.candidates[0].id == 0
                    assert poses.candidates[0].state == GraspCandidate.UNKNOWN
                    assert poses.candidates[0].shape_score == scores.data[0]
                    assert abs(poses.candidates[0].pose.position.z - 0.1) < 1.0e-6
                    assert node.count_publishers(topics["candidate_topic"]) == 1
                    assert node.count_publishers("/grasp_pose_cands/reachability") == 0
                    assert node.count_publishers("/grasp_pose_cands/reachability_markers") == 0
                    assert node.count_publishers("/grasp_pose_markers") == 0
                    assert not any(
                        "grasp_pose_marker_bridge" in name
                        for name in node.get_node_names())
                    print(f"確認成功: {topics}", flush=True)
                    return
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
    try:
        check_case(node, qos, root, False)
        check_case(node, qos, root, True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
