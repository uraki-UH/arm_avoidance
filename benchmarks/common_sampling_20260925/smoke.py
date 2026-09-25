"""共通化前後の実bag比較、把持候補一致・失効・試験プロセス終了の検証。"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from ais_gng_msgs.msg import TopologicalMap, TopologicalNode, TopologicalCluster
import yaml


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def xyz(cloud):
    offsets = {field.name: field.offset for field in cloud.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                     "offsets": [offsets[key] for key in ("x", "y", "z")], "itemsize": cloud.point_step})
    data = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width * cloud.height)
    return np.column_stack([data[key] for key in ("x", "y", "z")])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--trials", type=int, default=2)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--validation", action="store_true")
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "179"
    root = Path("/ros2_ws/src")
    output = root / "artifacts/common_sampling_20260925"
    config_dir = root / "ais_gng_cpu/src/ais_gng/config"
    settings = yaml.safe_load((config_dir / "gng_cpu/at128.yaml").read_text())["ais_gng_node"]["ros__parameters"]
    planes = yaml.safe_load((config_dir / "plane_cluster_incremental.yaml").read_text())["plane_cluster_incremental_node"]["ros__parameters"]
    settings.update({"plane_cluster." + key: value for key, value in planes.items()})
    settings.update({"input.local_coordinates": True, "input.point_cloud_num": 100000,
        "input.sampling_mode": "uniform", "input.visualize": False, "classify.human": False,
        "classify.car": False, "curve_clustering": False, "performance.log_interval_ms": 1,
        "grasp_attention.margin": 0.0, "grasp_attention.ratio": .3, "boundary_attention.ratio": .2,
        "grasp_attention.timeout_sec": 5.0, "boundary_attention.timeout_sec": 5.0})
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as db:
        topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        frames = [data for (data,) in db.execute(
            "select data from messages where topic_id=? order by timestamp limit ?", (topic_id, args.frames))]
    assert len(frames) == args.frames
    rclpy.init()
    observer = rclpy.create_node("common_sampling_observer")
    results = []
    try:
        for trial in range(args.trials):
            cases = [("before", False), ("after", False), ("before", True), ("after", True)]
            if trial % 2:
                cases.reverse()
            if args.validation:
                cases = [("after", True)]
            for version, enable_mixed in cases:
                case = f"{version}_{'mixed' if enable_mixed else 'unknown'}_{trial}"
                if args.validation:
                    case += "_validation"
                namespace = "/common_sampling_" + case
                params = dict(settings, **{"enable_grasp_attention": enable_mixed,
                    "enable_boundary_attention": enable_mixed, "input.topic_names": [namespace + "/input"],
                    "grasp_attention.topic": namespace + "/candidate"})
                config = output / (case + ".yaml")
                config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
                cache = {key: {} for key in ("map", "input", "grasp")}
                subscriptions = []
                for name, kind, topic in (("map", TopologicalMap, "/topological_map"),
                        ("input", PointCloud2, "/scan/transformed"), ("grasp", PointCloud2, "/downsampling/grasp")):
                    subscriptions.append(observer.create_subscription(kind, namespace + topic,
                        lambda msg, key=name: cache[key].__setitem__(stamp(msg), msg), qos_profile_sensor_data))
                publisher = observer.create_publisher(PointCloud2, namespace + "/input", 1)
                candidate_pub = observer.create_publisher(TopologicalMap, namespace + "/candidate", 1)
                process = None
                log_path = output / (case + ".log")
                with log_path.open("w") as log:
                    try:
                        env = os.environ.copy()
                        if version == "before":
                            env["LD_LIBRARY_PATH"] = str(output / "before") + ":" + env.get("LD_LIBRARY_PATH", "")
                        command = ["/ros2_ws/install/ais_gng/lib/ais_gng/ais_gng_cpu", "--ros-args",
                                   "-r", "__ns:=" + namespace, "--params-file", str(config)]
                        print("start:", version, " ".join(command), flush=True)
                        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                   env=env, start_new_session=True)
                        deadline = time.monotonic() + 20
                        while time.monotonic() < deadline:
                            assert process.poll() is None, case
                            if publisher.get_subscription_count() and (not enable_mixed or candidate_pub.get_subscription_count()):
                                break
                            rclpy.spin_once(observer, timeout_sec=.05)
                        assert publisher.get_subscription_count()
                        ready_at = time.monotonic() + 1
                        while time.monotonic() < ready_at:
                            rclpy.spin_once(observer, timeout_sec=.05)
                        selected_counts = []
                        for idx, data in enumerate(frames):
                            msg = deserialize_message(data, PointCloud2)
                            msg.header.stamp.sec = 1000 + idx // 10
                            msg.header.stamp.nanosec = idx % 10 * 100000000
                            msg.header.frame_id = "test_world"
                            has_candidate = not args.validation or idx < len(frames) - 4
                            if enable_mixed:
                                candidate = TopologicalMap(); candidate.header = msg.header
                                for node_id, pos in enumerate(((-20., -20., -2.), (20., 20., 4.))):
                                    node = TopologicalNode(); node.id = node_id
                                    node.pos.x, node.pos.y, node.pos.z = pos
                                    candidate.nodes.append(node)
                                cluster = TopologicalCluster(); cluster.nodes = [0, 1]
                                candidate.clusters.append(cluster)
                                if not has_candidate:
                                    candidate.header.stamp.sec = 1
                                candidate_pub.publish(candidate)
                                # 重点候補と点群の別購読間の受信順序の安定化。
                                time.sleep(.03)
                                msg.header.stamp.sec = 1000 + idx // 10
                            key = stamp(msg)
                            for values in cache.values():
                                values.clear()
                            publisher.publish(msg)
                            required = ("map", "input", "grasp") if enable_mixed else ("map", "input")
                            deadline = time.monotonic() + 12
                            while time.monotonic() < deadline:
                                assert process.poll() is None, (case, idx)
                                if all(key in cache[name] for name in required):
                                    break
                                rclpy.spin_once(observer, timeout_sec=.03)
                            assert all(key in cache[name] for name in required), (case, idx)
                            assert cache["map"][key].nodes
                            if enable_mixed:
                                points = xyz(cache["input"][key])
                                expected = points[np.all((points >= [-20, -20, -2]) & (points <= [20, 20, 4]), axis=1)]
                                if not has_candidate:
                                    expected = expected[:0]
                                actual = xyz(cache["grasp"][key])
                                assert np.array_equal(actual, expected), (case, idx, actual.shape, expected.shape)
                                selected_counts.append(len(actual))
                            assert observer.count_publishers(namespace + "/downsampling/nonplane") == 0
                    finally:
                        if process is not None:
                            for sig, wait_sec in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                                if process.poll() is not None:
                                    break
                                os.killpg(process.pid, sig)
                                try:
                                    process.wait(timeout=wait_sec)
                                except subprocess.TimeoutExpired:
                                    continue
                            assert process.poll() is not None
                            print(f"stopped: pid={process.pid} exit={process.returncode}", flush=True)
                        observer.destroy_publisher(publisher)
                        observer.destroy_publisher(candidate_pub)
                        for subscription in subscriptions:
                            observer.destroy_subscription(subscription)
                elapsed = [float(value) for value in re.findall(r"processing=([0-9.]+) ms", log_path.read_text())]
                assert len(elapsed) == args.frames, (case, len(elapsed))
                measured = elapsed[min(20, args.frames // 2):]
                result = {"case": case, "processing_ms": sum(measured) / len(measured),
                          "frames": len(elapsed), "selected_counts": selected_counts}
                results.append(result)
                print("PASS:", result, flush=True)
                name = "validation.json" if args.validation else "comparison.json"
                (output / name).write_text(json.dumps(results, indent=2) + "\n")
    finally:
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
