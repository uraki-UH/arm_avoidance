"""非平面重点削除後の実bag入力・通常出力・不要トピック不在の有限検証。"""
import argparse
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rcl_interfaces.srv import ListParameters
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt32MultiArray
from ais_gng_msgs.msg import TopologicalMap
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", required=True)
    parser.add_argument("--bag", required=True)
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID") == "178"
    root = Path("/ros2_ws/src")
    output = root / "artifacts/nonplane_attention_removal_20260925"
    output.mkdir(parents=True, exist_ok=True)
    config_dir = root / "ais_gng_cpu/src/ais_gng/config"
    settings = yaml.safe_load((config_dir / "gng_cpu/at128.yaml").read_text())["ais_gng_node"]["ros__parameters"]
    planes = yaml.safe_load((config_dir / "plane_cluster_incremental.yaml").read_text())["plane_cluster_incremental_node"]["ros__parameters"]
    settings.update({"plane_cluster." + key: value for key, value in planes.items()})
    settings.update({"input.local_coordinates": True, "input.point_cloud_num": 100000,
                     "input.sampling_mode": "uniform", "input.visualize": False,
                     "classify.human": False, "classify.car": False,
                     "curve_clustering": False, "plane_cluster.direct_enabled": True})
    with sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True) as db:
        topic_id = db.execute("select id from topics where name='/lidar_points'").fetchone()[0]
        frames = [data for (data,) in db.execute(
            "select data from messages where topic_id=? order by timestamp limit 12", (topic_id,))]
    assert len(frames) == 12
    rclpy.init()
    observer = rclpy.create_node("nonplane_removal_observer")
    try:
        cases = (("default", {}, True),
                 ("component_off", {"nonplane_component.direct_enabled": False}, False),
                 ("plane_off", {"plane_cluster.direct_enabled": False, "plane_clustering": False}, False))
        for case, overrides, has_components in cases:
            namespace = "/nonplane_removal_" + case
            params = dict(settings, **overrides)
            params["input.topic_names"] = [namespace + "/input"]
            params["nonplane_component.output_topic"] = namespace + "/components"
            config = output / (case + ".yaml")
            config.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
            maps, components = {}, {}
            subscriptions = [observer.create_subscription(TopologicalMap, namespace + "/topological_map",
                lambda msg: maps.__setitem__(msg.header.stamp.nanosec, msg), qos_profile_sensor_data),
                observer.create_subscription(UInt32MultiArray, namespace + "/components",
                lambda msg: components.__setitem__(msg.data[0], msg), qos_profile_sensor_data)]
            publisher = observer.create_publisher(PointCloud2, namespace + "/input", 1)
            client = observer.create_client(ListParameters, namespace + "/ais_gng_node/list_parameters")
            process = None
            with (output / (case + ".log")).open("w") as log:
                try:
                    command = [args.executable, "--ros-args", "-r", "__ns:=" + namespace,
                               "--params-file", str(config)]
                    print("start:", " ".join(command), flush=True)
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        assert process.poll() is None, case
                        if publisher.get_subscription_count() and client.service_is_ready() and observer.count_publishers(namespace + "/topological_map"):
                            break
                        rclpy.spin_once(observer, timeout_sec=.05)
                    assert publisher.get_subscription_count() and client.service_is_ready(), case
                    future = client.call_async(ListParameters.Request())
                    rclpy.spin_until_future_complete(observer, future, timeout_sec=5)
                    assert future.done(), case
                    names = future.result().result.names
                    assert not any("nonplane_attention" in name or "voxel_attention" in name for name in names), names
                    ready_at = time.monotonic() + 1
                    while time.monotonic() < ready_at:
                        rclpy.spin_once(observer, timeout_sec=.05)
                    for idx, data in enumerate(frames):
                        msg = deserialize_message(data, PointCloud2)
                        msg.header.stamp.sec = 1000 + idx // 10
                        msg.header.stamp.nanosec = idx % 10 * 100000000
                        msg.header.frame_id = "test_world"
                        key = msg.header.stamp.nanosec
                        maps.clear()
                        publisher.publish(msg)
                        deadline = time.monotonic() + 12
                        while time.monotonic() < deadline:
                            assert process.poll() is None, (case, idx)
                            if key in maps and (not has_components or maps[key].frame_number in components):
                                break
                            rclpy.spin_once(observer, timeout_sec=.03)
                        assert key in maps and maps[key].nodes, (case, idx)
                        if has_components:
                            result = components[maps[key].frame_number]
                            assert result.data[1] > 0, (case, idx)
                            assert any(node.nonplane_component_id != 4294967295 for node in maps[key].nodes)
                        else:
                            assert observer.count_publishers(namespace + "/components") == 0
                        assert observer.count_publishers(namespace + "/downsampling/nonplane") == 0
                    print(f"PASS: {case}, frames=12, removed_parameters=0, removed_publishers=0", flush=True)
                finally:
                    if process is not None:
                        for sig, timeout_sec in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                            if process.poll() is not None:
                                break
                            os.killpg(process.pid, sig)
                            try:
                                process.wait(timeout=timeout_sec)
                            except subprocess.TimeoutExpired:
                                continue
                        assert process.poll() is not None
                        print(f"stopped: pid={process.pid} exit={process.returncode}", flush=True)
                    observer.destroy_client(client)
                    observer.destroy_publisher(publisher)
                    for subscription in subscriptions:
                        observer.destroy_subscription(subscription)
    finally:
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
