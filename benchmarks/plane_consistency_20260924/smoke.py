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
                     "enable_fragment_merge", "max_fragment_edge_ratio_th",
                     "max_fragment_residual_ratio_th", "min_fragment_merge_frames", "enable_directional_split",
                     "min_split_edge_angle_deg_th", "min_split_conflict_nodes", "min_split_conflict_ratio_th",
                     "max_isolated_frames", "enable_coplanar_absorption",
                     "max_absorption_edge_angle_deg_th", "max_absorption_edge_ratio_th"]
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

        # 入力1回に対応する出力の確認。確認待ち回数と孤立猶予の厳密な検証用。
        def expect_frame(num_clusters, num_nodes):
            graph.frame_number += 1
            publisher.publish(graph)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.1)
                if received and received[-1].frame_number == graph.frame_number:
                    break
            else:
                raise AssertionError("confirmation output missing")
            assert len(received[-1].clusters) == num_clusters, (graph.frame_number, len(received[-1].clusters))
            assert sum(len(cluster.node_indices) for cluster in received[-1].clusters) == num_nodes

        num_confirmation_frames = settings["min_fragment_merge_frames"]
        for frame in range(1, num_confirmation_frames + 1):
            expected = 1 if frame == num_confirmation_frames else 2
            expect_frame(expected, 169)
        expect_clusters(1)
        print(f"PASS: single-edge fragment merge after {num_confirmation_frames} frames", flush=True)

        # 同一平面の接続切れのみでは分割せず、面外根拠の連続成立時だけ分割。
        del graph.edges[-2:]
        for _ in range(settings["split_confirm_frames"] + 6):
            expect_frame(1, 169)
        print("PASS: coplanar component retained after disconnection", flush=True)
        for idx in range(7):
            value = TopologicalNode()
            value.id = 30000 + idx
            value.pos.x, value.pos.y = graph.nodes[144 + idx].pos.x, graph.nodes[144 + idx].pos.y
            value.pos.z = -0.05
            value.normal.x = 1.0
            graph.edges.extend([144 + idx, len(graph.nodes)])
            graph.nodes.append(value)
        for _ in range(settings["split_confirm_frames"]):
            expect_frame(1, 169)
        expect_frame(2, 169)
        print("PASS: off-plane external edges confirmed before split", flush=True)

        # 孤立ノードの前回情報による短期保持と、猶予切れ・再接続の確認。
        saved_edges = list(graph.edges)
        del graph.edges[:]
        for first, second in zip(saved_edges[::2], saved_edges[1::2]):
            if first != 0 and second != 0:
                graph.edges.extend([first, second])
        graph.nodes[0].normal.z = 0.0
        for _ in range(settings["max_isolated_frames"]):
            expect_frame(2, 169)
        expect_frame(2, 168)
        del graph.edges[:]
        graph.edges.extend(saved_edges)
        graph.nodes[0].normal.z = 1.0
        expect_frame(2, 169)
        print("PASS: isolated-node grace, expiry and reconnection", flush=True)

        # 未所属点の1本接続・距離緩和と、面外エッジを持つ小物体の拒否。
        for trial_idx, (num_contacts, has_conflict) in enumerate(((1, False), (2, False), (1, True))):
            graph.nodes.clear()
            del graph.edges[:]
            for idx in range(36):
                value = TopologicalNode()
                value.id = 40000 + trial_idx * 100 + idx
                value.pos.x, value.pos.y = (idx % 6) * 0.05, (idx // 6) * 0.05
                value.normal.z = 1.0
                graph.nodes.append(value)
                if idx % 6:
                    graph.edges.extend([idx - 1, idx])
                if idx >= 6:
                    graph.edges.extend([idx - 6, idx])
            expect_clusters(1)
            value = TopologicalNode()
            value.id = 40036 + trial_idx * 100
            value.pos.x, value.pos.y = 0.30, 0.10
            value.pos.z = 0.005 if num_contacts == 1 else 0.010
            value.normal.z = 1.0
            graph.nodes.append(value)
            graph.edges.extend([17, 36])
            if num_contacts == 2:
                graph.edges.extend([23, 36])
            if has_conflict:
                protrusion = TopologicalNode()
                protrusion.id = value.id + 1
                protrusion.pos.x, protrusion.pos.y = value.pos.x, value.pos.y
                protrusion.pos.z = value.pos.z + 0.05
                protrusion.normal.x = 1.0
                graph.nodes.append(protrusion)
                graph.edges.extend([36, 37])
            for _ in range(6):
                expect_frame(1, 36 if has_conflict else 37)
            print(f"PASS: coplanar absorption contacts={num_contacts}, conflict={has_conflict}", flush=True)
        # 点数上限なしの1本接続統合。1,170点と99点の平面、および段差の分離。
        for trial_idx, offset in enumerate((0.0, 0.04)):
            graph.nodes.clear()
            del graph.edges[:]
            for width, height, origin_x, origin_z in ((45, 26, 0.0, 0.0), (11, 9, 2.25, offset)):
                for row in range(height):
                    for column in range(width):
                        idx = len(graph.nodes)
                        value = TopologicalNode()
                        value.id = 50000 + trial_idx * 2000 + idx
                        value.pos.x, value.pos.y = origin_x + column * 0.05, row * 0.05
                        value.pos.z = origin_z
                        value.normal.z = 1.0
                        graph.nodes.append(value)
                        if column:
                            graph.edges.extend([idx - 1, idx])
                        if row:
                            graph.edges.extend([idx - width, idx])
            expect_clusters(2)
            graph.edges.extend([44, 1170])
            for frame in range(1, num_confirmation_frames + 1):
                expected = 1 if offset == 0.0 and frame == num_confirmation_frames else 2
                expect_frame(expected, 1269)
            expect_clusters(1 if offset == 0.0 else 2)
            print(f"PASS: single-edge merge, 1170+99 nodes, offset={offset} m", flush=True)
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
