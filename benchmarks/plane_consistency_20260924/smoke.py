"""分離ROSドメインでの配布バイナリ起動・平面統合試験と子プロセスの後片付け。"""

import os
import math
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import yaml
from ais_gng_msgs.msg import PlaneClusterArray, TopologicalMap, TopologicalNode
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    assert os.environ.get("ROS_DOMAIN_ID") == "173"
    output_dir = Path("/ros2_ws/src/artifacts/plane_consistency_20260924")
    executable_dir = Path("/ros2_ws/install/ais_gng/lib/ais_gng")
    processes, logs = [], []
    rclpy.init()
    observer = rclpy.create_node("plane_consistency_smoke_observer")
    received = []
    settings = yaml.safe_load(Path(
        "/ros2_ws/src/ais_gng_cpu/src/ais_gng/config/plane_cluster_incremental.yaml"
    ).read_text())["plane_cluster_incremental_node"]["ros__parameters"]
    try:
        for executable, name, params in (
            ("ais_gng_cpu", "plane_consistency_smoke_cpu", [
                "plane_cluster.direct_enabled:=true", "plane_cluster.min_plane_width_ratio:=1.0",
                "input.topic_names:=[/plane_consistency/unused]"]),
            ("plane_cluster_incremental_node", "plane_consistency_smoke_plane", [
                "min_plane_width_ratio:=1.0", "input_topic:=/plane_consistency/map",
                "output_topic:=/plane_consistency/planes", "enable_nonplane_markers:=false",
                "surface_model.enable:=false"]),
        ):
            log = (output_dir / f"{name}.log").open("w")
            logs.append(log)
            command = [str(executable_dir / executable), "--ros-args", "-r", f"__node:={name}"]
            for param in params:
                command.extend(["-p", param])
            print("start:", " ".join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            client = observer.create_client(GetParameters, f"/{name}/get_parameters")
            assert client.wait_for_service(timeout_sec=12.0), name
            req = GetParameters.Request()
            names = ["min_plane_width_ratio", "growth_residual_ratio", "retention_residual_ratio",
                     "max_effective_spacing", "max_normalized_cluster_residual", "max_merge_side_residual_ratio",
                     "enable_fragment_merge", "max_fragment_nodes", "max_fragment_edge_ratio_th",
                     "max_fragment_residual_ratio_th", "min_fragment_merge_frames"]
            prefix = "plane_cluster." if executable == "ais_gng_cpu" else ""
            req.names = [prefix + key for key in names]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(observer, future, timeout_sec=5.0)
            assert future.done()
            for key, value in zip(names, future.result().values):
                expected = settings[key]
                actual = value.bool_value if isinstance(expected, bool) else (
                    value.integer_value if isinstance(expected, int) else value.double_value)
                assert actual == expected, (key, actual, expected)
            observer.destroy_client(client)
        observer.create_subscription(PlaneClusterArray, "/plane_consistency/planes", received.append, 10)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = observer.create_publisher(TopologicalMap, "/plane_consistency/map", qos)
        graph = TopologicalMap()
        graph.header.frame_id = "map"
        for row in range(6):
            for column in range(120):
                idx = row * 120 + column
                value = TopologicalNode()
                value.id = idx
                value.pos.x, value.pos.y = column * 0.05, row * 0.05
                value.normal.z = 1.0
                graph.nodes.append(value)
                if column:
                    graph.edges.extend([idx - 1, idx])
                if row:
                    graph.edges.extend([idx - 120, idx])
        for spacing in (0.005, 0.05, 2.0):
            for idx, value in enumerate(graph.nodes):
                value.pos.x = (idx % 120) * spacing
                value.pos.y = (idx // 120) * spacing
                value.pos.z = 0.02 * spacing * math.sin(1.7 * idx)
            first_frame = graph.frame_number
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                graph.frame_number += 1
                publisher.publish(graph)
                rclpy.spin_once(observer, timeout_sec=0.1)
                if (received and received[-1].frame_number > first_frame and
                        len(received[-1].clusters) == 1 and
                        len(received[-1].clusters[0].node_indices) == 720):
                    print(f"PASS: spacing={spacing} m, one 720-node noisy plane", flush=True)
                    break
            else:
                raise AssertionError(f"plane output missing: spacing={spacing}")

        # 新しいノードIDによる各試験の所属初期化。確認済み出力6フレームの連続一致。
        def expect_clusters(num_clusters):
            last_frame = graph.frame_number
            num_matches = 0
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                graph.frame_number += 1
                publisher.publish(graph)
                rclpy.spin_once(observer, timeout_sec=0.1)
                if not received or received[-1].frame_number <= last_frame:
                    continue
                result = received[-1]
                last_frame = result.frame_number
                if (len(result.clusters) == num_clusters and
                        sum(len(cluster.node_indices) for cluster in result.clusters) == len(graph.nodes)):
                    num_matches += 1
                    if num_matches == 6:
                        return
                else:
                    num_matches = 0
            raise AssertionError(f"interior patch output missing: expected={num_clusters}")

        for trial_idx, offset in enumerate((0.0, 0.10)):
            graph.nodes.clear()
            del graph.edges[:]
            angle = math.radians(1.0)
            for is_small, size in ((False, 41), (True, 5)):
                base_idx = len(graph.nodes)
                for row in range(size):
                    for column in range(size):
                        idx = base_idx + row * size + column
                        value = TopologicalNode()
                        value.id = 10000 + trial_idx * 2000 + idx
                        x = column * 0.5 - (1.0 if is_small else 10.0)
                        value.pos.x = x * math.cos(angle) if is_small else x
                        value.pos.y = row * 0.5 - (1.0 if is_small else 10.0)
                        value.pos.z = offset + x * math.sin(angle) if is_small else 0.0
                        value.normal.x = -math.sin(angle) if is_small else 0.0
                        value.normal.z = math.cos(angle) if is_small else 1.0
                        graph.nodes.append(value)
                        if column:
                            graph.edges.extend([idx - 1, idx])
                        if row:
                            graph.edges.extend([idx - size, idx])
            expect_clusters(2)
            for row in range(5):
                graph.edges.extend([(18 + row) * 41 + 17, 1681 + row * 5,
                                    (18 + row) * 41 + 23, 1681 + row * 5 + 4])
            expect_clusters(1 if offset == 0.0 else 2)
            print(f"PASS: 1706-node interior patch, 1 deg tilt, offset={offset} m", flush=True)

        # 1本接続の確認待ちを、各入力に対応する出力フレームで検証。
        graph.nodes.clear()
        del graph.edges[:]
        for size, origin_x in ((12, 0.0), (5, 0.60)):
            base_idx = len(graph.nodes)
            for row in range(size):
                for column in range(size):
                    idx = base_idx + row * size + column
                    value = TopologicalNode()
                    value.id = 20000 + idx
                    value.pos.x, value.pos.y = origin_x + column * 0.05, row * 0.05
                    value.normal.z = 1.0
                    graph.nodes.append(value)
                    if column:
                        graph.edges.extend([idx - 1, idx])
                    if row:
                        graph.edges.extend([idx - size, idx])
        expect_clusters(2)
        graph.edges.extend([11, 144])
        num_confirmation_frames = settings["min_fragment_merge_frames"]
        for frame in range(1, num_confirmation_frames + 1):
            graph.frame_number += 1
            publisher.publish(graph)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.1)
                if received and received[-1].frame_number == graph.frame_number:
                    break
            else:
                raise AssertionError("fragment confirmation output missing")
            expected = 1 if frame == num_confirmation_frames else 2
            assert len(received[-1].clusters) == expected, (frame, len(received[-1].clusters))
            assert sum(len(cluster.node_indices) for cluster in received[-1].clusters) == 169
        expect_clusters(1)
        print(f"PASS: single-edge fragment merge after {num_confirmation_frames} frames", flush=True)
        assert all(process.poll() is None for process in processes)
    finally:
        for process in reversed(processes):
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
        observer.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
