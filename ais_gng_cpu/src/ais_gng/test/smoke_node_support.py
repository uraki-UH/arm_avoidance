"""独立ROSドメインでの配信確認。自身が起動したノードの確実な停止。"""
import argparse
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

os.environ["ROS_DOMAIN_ID"] = "187"
import rclpy
from rclpy.qos import qos_profile_sensor_data
from ais_gng_msgs.msg import TopologicalMap
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executable")
    parser.add_argument("input")
    parser.add_argument("directory")
    args = parser.parse_args()
    directory = Path(args.directory)
    directory.mkdir(exist_ok=True)
    rclpy.init()
    observer = rclpy.create_node("support_observer")
    results = {}
    try:
        for mode in ("baseline", "support"):
            namespace = f"/support_check_{mode}"
            maps, markers = [], []
            map_subscription = observer.create_subscription(TopologicalMap, namespace + "/topological_map", maps.append, 10)
            marker_subscription = observer.create_subscription(MarkerArray, namespace + "/node_support_ellipsoids", markers.append, 10)
            publisher = observer.create_publisher(PointCloud2, namespace + "/points", qos_profile_sensor_data)
            command = [args.executable, "--ros-args", "-r", "__ns:=" + namespace,
                       "-p", f"input.topic_names:=['{namespace}/points']",
                       "-p", "input.local_coordinates:=true", "-p", "input.point_cloud_num:=20000",
                       "-p", "node.learning_num:=5000", "-p", "node.num_max:=5000",
                       "-p", "node.covariance_enabled:=true", "-p", "node.eta_s1:=0.08",
                       "-p", "performance.log_interval_ms:=1", "-p", "classify.human:=false",
                       "-p", "classify.car:=false", "-p", "node.enable_support:=" + ("true" if mode == "support" else "false")]
            with open(directory / f"{mode}.log", "w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 20
                    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                        rclpy.spin_once(observer, timeout_sec=0.05)
                        if process.poll() is not None:
                            raise RuntimeError(f"{mode}ノードが起動時に終了しました")
                    if publisher.get_subscription_count() == 0:
                        raise RuntimeError("点群購読の開始を確認できません")
                    with open(args.input, "rb") as stream:
                        for frame in range(80):
                            stamp, point_num = struct.unpack("<dI", stream.read(12))
                            payload = stream.read(point_num * 12)
                            cloud = PointCloud2()
                            cloud.header.frame_id = "map"
                            cloud.header.stamp.sec = int(stamp) + 1
                            cloud.header.stamp.nanosec = int((stamp % 1) * 1e9)
                            cloud.height = 1; cloud.width = point_num
                            cloud.fields = [PointField(name=name, offset=idx * 4, datatype=PointField.FLOAT32, count=1)
                                            for idx, name in enumerate(("x", "y", "z"))]
                            cloud.point_step = 12; cloud.row_step = point_num * 12
                            cloud.is_dense = True; cloud.data = payload
                            publisher.publish(cloud)
                            deadline = time.monotonic() + 3
                            while len(maps) < frame + 1 and time.monotonic() < deadline:
                                rclpy.spin_once(observer, timeout_sec=0.01)
                            if len(maps) < frame + 1:
                                raise RuntimeError(f"frame={frame}の地図配信がありません")
                            rclpy.spin_once(observer, timeout_sec=0.02)
                    has_covariance = any(any(abs(value) > 0 for value in node.winner_point_covariance) for node in maps[-1].nodes)
                    support_num = max((sum(marker.action == 0 for marker in array.markers) for array in markers), default=0)
                    if not has_covariance or (mode == "support" and support_num == 0):
                        raise RuntimeError("共分散または支持領域の配信確認に失敗しました")
                    results[mode] = {"maps": len(maps), "marker_arrays": len(markers), "max_support_num": support_num,
                                     "has_covariance": has_covariance, "command": command}
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill(); process.wait()
                    results.setdefault(mode, {})["is_stopped"] = process.poll() is not None
            observer.destroy_subscription(map_subscription)
            observer.destroy_subscription(marker_subscription)
            observer.destroy_publisher(publisher)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        (directory / "results.json").write_text(json.dumps(results, ensure_ascii=False, indent=2))
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
