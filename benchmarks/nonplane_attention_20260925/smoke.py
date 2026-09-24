"""分離ROSドメインでの実点群入力・重点候補照合と試験ノードの後片付け。"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from ais_gng_msgs.msg import TopologicalMap
import yaml


def xyz(cloud):
    offsets = {field.name: field.offset for field in cloud.fields}
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [offsets[key] for key in ("x", "y", "z")],
                      "itemsize": cloud.point_step})
    data = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width * cloud.height)
    return np.column_stack([data[key] for key in ("x", "y", "z")])


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", default="artifacts/nonplane_attention_20260925")
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "174"
    root = Path("/ros2_ws/src")
    output = root / args.output
    output.mkdir(parents=True, exist_ok=True)
    config_dir = root / "ais_gng_cpu/src/ais_gng/config"
    settings = yaml.safe_load((config_dir / "gng_cpu/at128.yaml").read_text())["ais_gng_node"]["ros__parameters"]
    planes = yaml.safe_load((config_dir / "plane_cluster_incremental.yaml").read_text())["plane_cluster_incremental_node"]["ros__parameters"]
    settings.update({"plane_cluster." + key: value for key, value in planes.items()})
    settings.update({"input.local_coordinates": True, "input.point_cloud_num": 100000,
                     "input.sampling_mode": "uniform", "input.visualize": False,
                     "classify.human": False, "classify.car": False,
                     "surface_model.enable": False, "plane_cluster.direct_enabled": True,
                     "nonplane_attention.timeout_sec": 5.0})
    assert settings["nonplane_attention.enable_debug_points"] is True
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as db:
        topic = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        frames = [data for (data,) in db.execute(
            "select data from messages where topic_id=? order by timestamp limit 12", (topic,))]
    assert len(frames) == 12
    results = []
    rclpy.init()
    observer = rclpy.create_node("nonplane_attention_test_observer")
    try:
        for case, overrides, has_debug in (
                ("enabled", {}, True),
                ("size_rejected", {"nonplane_attention.min_component_nodes": 65534}, True),
                ("output_disabled", {"nonplane_component.direct_enabled": False}, True),
                ("debug_disabled", {"nonplane_attention.enable_debug_points": False}, False),
                ("disabled", {"enable_nonplane_attention": False}, False),
                ("plane_disabled", {"plane_cluster.direct_enabled": False}, False)):
            namespace = "/nonplane_attention_test_" + case
            params = dict(settings, **overrides)
            params["input.topic_names"] = [namespace + "/input"]
            config = output / (case + ".yaml")
            config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
            caches = {name: {} for name in ("map", "input", "selected")}
            subscriptions = []
            for name, msg_type, topic_name in (
                    ("map", TopologicalMap, "/topological_map"),
                    ("input", PointCloud2, "/scan/transformed"),
                    ("selected", PointCloud2, "/downsampling/nonplane")):
                subscriptions.append(observer.create_subscription(
                    msg_type, namespace + topic_name,
                    lambda message, key=name: caches[key].__setitem__(stamp(message), message),
                    qos_profile_sensor_data))
            publisher = observer.create_publisher(PointCloud2, namespace + "/input", 1)
            command = [args.executable, "--ros-args", "-r", "__ns:=" + namespace,
                       "--params-file", str(config)]
            log = (output / (case + ".log")).open("w")
            process = None
            try:
                print("start:", " ".join(command), flush=True)
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 15
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    assert process.poll() is None, case
                    rclpy.spin_once(observer, timeout_sec=.05)
                assert publisher.get_subscription_count() > 0, case
                # 入出力のDDS接続待ち。再送による同一フレーム二重学習の回避。
                ready_at = time.monotonic() + 1.0
                while time.monotonic() < ready_at:
                    assert process.poll() is None, case
                    rclpy.spin_once(observer, timeout_sec=.05)
                selected_counts = []
                has_small_components = False
                previous = None
                num_frames = 12 if case == "enabled" else 4
                for idx in range(num_frames):
                    message = deserialize_message(frames[idx], PointCloud2)
                    # 時刻逆行・大きな時刻差・座標系変更による旧候補の失効。
                    tick = idx if idx < 8 else (0 if idx == 8 else 100 + idx)
                    message.header.stamp.sec = 1000 + tick // 10
                    message.header.stamp.nanosec = tick % 10 * 100000000
                    message.header.frame_id = "test_world" if idx < 10 else "changed_world"
                    key = stamp(message)
                    for cache in caches.values():
                        cache.pop(key, None)
                    publisher.publish(message)
                    deadline = time.monotonic() + 10
                    required = ("map", "input", "selected") if has_debug else ("map", "input")
                    while not all(key in caches[name] for name in required) and time.monotonic() < deadline:
                        assert process.poll() is None, case
                        rclpy.spin_once(observer, timeout_sec=.03)
                    assert all(key in caches[name] for name in required), (case, idx)
                    graph = caches["map"][key]
                    if has_debug:
                        selected = xyz(caches["selected"][key])
                        selected_counts.append(len(selected))
                        if idx == 0 or case == "size_rejected" or idx in (8, 9, 10):
                            assert len(selected) == 0, (case, idx, len(selected))
                        elif case == "enabled":
                            counts = Counter(node.nonplane_component_id for node in previous.nodes
                                             if node.nonplane_component_id != 4294967295)
                            has_small_components |= any(value < 5 for value in counts.values())
                            anchors = np.array([[node.pos.x, node.pos.y, node.pos.z] for node in previous.nodes
                                                if node.nonplane_component_id in counts and
                                                counts[node.nonplane_component_id] >= 5])
                            points = xyz(caches["input"][key])
                            if len(anchors):
                                distances = cKDTree(anchors).query(points)[0]
                                expected = points[distances <= .3]
                                assert selected.shape == expected.shape, (idx, selected.shape, expected.shape)
                                assert np.allclose(selected, expected, rtol=0, atol=1e-6)
                                if idx in (1, 4, 7):
                                    # 追加近傍検索の独立計測用入力。XYZだけの保存、元bagの変更なし。
                                    selection_name = "selection.bin" if idx == 7 else f"selection_{idx:02d}.bin"
                                    with (output / selection_name).open("wb") as stream:
                                        stream.write(struct.pack("<II", len(anchors), len(points)))
                                        anchors.astype("<f4").tofile(stream)
                                        points.astype("<f4").tofile(stream)
                            else:
                                assert len(selected) == 0
                    else:
                        assert observer.count_publishers(namespace + "/downsampling/nonplane") == 0
                    previous = graph
                if case in ("enabled", "output_disabled"):
                    assert max(selected_counts) > 0, (case, selected_counts)
                if case == "enabled":
                    assert has_small_components, "実入力に小成分なし"
                if case == "output_disabled":
                    assert all(node.nonplane_component_id == 4294967295 for node in previous.nodes)
                results.append({"case": case, "selected_counts": selected_counts})
                print("PASS:", results[-1], flush=True)
            finally:
                if process is not None:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGTERM)
                            try:
                                process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=5)
                    print(f"stopped: pid={process.pid} exit={process.returncode}", flush=True)
                log.close()
                observer.destroy_publisher(publisher)
                for subscription in subscriptions:
                    observer.destroy_subscription(subscription)
        (output / "smoke_results.json").write_text(json.dumps(results, indent=2) + "\n")
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
