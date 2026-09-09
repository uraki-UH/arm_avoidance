#!/usr/bin/env python3
"""隔離ROSドメインでのtopological_map境界属性・Viewer転送・入力停止の検証。"""

import argparse
import asyncio
import json
import os
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from ais_gng_msgs.msg import TopologicalMap
from sensor_msgs.msg import PointCloud2, PointField
from tornado.websocket import websocket_connect


def require(is_valid, message):
    if not is_valid:
        raise RuntimeError(message)


def decode_graph_packet(payload):
    if isinstance(payload, str):
        message = json.loads(payload)
        return message.get('graph') if message.get('type') == 'stream.graph' else None
    if len(payload) < 36 or payload[:4] != b'TMG1':
        return None
    _, version, _, tag_size, frame_size, _, num_nodes, num_edges, _, payload_size = struct.unpack_from('<IHH7I', payload)
    require(version == 1 and payload_size == len(payload) - 36, 'バイナリヘッダーの不一致')
    offset = 36 + tag_size
    frame = payload[offset:offset + frame_size].decode()
    offset += frame_size
    nodes = [{'id': struct.unpack_from('<H', payload, offset + idx * 84)[0],
              'is_boundary_candidate': bool(payload[offset + idx * 84 + 5]),
              'boundary_evidence': payload[offset + idx * 84 + 6]} for idx in range(num_nodes)]
    offset += num_nodes * 84
    edges = list(struct.unpack_from(f'<{num_edges}H', payload, offset))
    return {'frameId': frame, 'nodes': nodes, 'edges': edges}


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        os.killpg(process.pid, sig)
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError('検証プロセスの停止失敗')


async def check_case(executable, gateway_executable, enable_candidates, max_neighbors):
    command = [executable, '--ros-args', '-r', '__node:=boundary_test_gng']
    for parameter in [
        'node.num_max:=1024', 'node.learning_num:=4000',
        'input.topic_names:=[/boundary_test_points]', 'input.point_cloud_num:=2000',
        'input.local_coordinates:=true', 'classify.human:=false', 'classify.car:=false',
        'plane_cluster.direct_enabled:=false', 'nonplane_component.direct_enabled:=false',
        f'boundary.enable_candidates:={str(enable_candidates).lower()}',
        f'boundary.max_neighbors:={max_neighbors}',
    ]:
        command.extend(['-p', parameter])
    gateway_command = [gateway_executable, '--ros-args', '-r', '__node:=boundary_test_gateway', '-p', 'port:=19091']
    process = gateway = node = connection = None
    pending_message = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            for label, invocation in [('GNG', command), ('Viewer', gateway_command)]:
                print(f'{label}起動コマンド:', ' '.join(invocation), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
            gateway = subprocess.Popen(gateway_command, stdout=log, stderr=log, start_new_session=True)
            node = rclpy.create_node('boundary_test_driver')
            maps = []
            subscription = node.create_subscription(TopologicalMap, '/topological_map', maps.append, 10)
            publisher = node.create_publisher(PointCloud2, '/boundary_test_points', 10)
            cloud = PointCloud2()
            cloud.header.frame_id = 'boundary_test_world'
            cloud.height, cloud.width = 1, 900
            cloud.fields = [PointField(name=name, offset=4 * idx, datatype=PointField.FLOAT32, count=1)
                            for idx, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step, cloud.row_step, cloud.is_dense = 12, 10800, True
            cloud.data = b''.join(struct.pack('<fff', 1.0 + 0.04 * x, -0.6 + 0.04 * y, 0.2)
                                  for y in range(30) for x in range(30))
            deadline = time.monotonic() + 20
            next_publish = next_subscribe = 0
            checked_maps = num_streams = 0
            has_candidate = False
            while time.monotonic() < deadline:
                require(process.poll() is None and gateway.poll() is None, '検証プロセスの異常終了')
                if connection is None:
                    try:
                        connection = await websocket_connect('ws://127.0.0.1:19091', connect_timeout=0.3)
                        pending_message = connection.read_message()
                    except OSError:
                        await asyncio.sleep(0.05)
                        continue
                if time.monotonic() >= next_subscribe:
                    await connection.write_message(json.dumps({'id': 'boundary-test', 'method': 'sources.setActive',
                        'params': {'sourceId': '/topological_map', 'active': True}}))
                    next_subscribe = time.monotonic() + 0.5
                if time.monotonic() >= next_publish:
                    cloud.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(cloud)
                    next_publish = time.monotonic() + 0.06
                rclpy.spin_once(node, timeout_sec=0)
                for graph in maps[checked_maps:]:
                    require(graph.header.frame_id == cloud.header.frame_id, '座標系の不一致')
                    neighbors = [set() for _ in graph.nodes]
                    for idx in range(0, len(graph.edges), 2):
                        source, target = graph.edges[idx:idx + 2]
                        neighbors[source].add(target)
                        neighbors[target].add(source)
                    expected = [enable_candidates and len(adjacent) <= max_neighbors for adjacent in neighbors]
                    actual = [point.is_boundary_candidate for point in graph.nodes]
                    require(all(point.boundary_evidence == 0 for point in graph.nodes), '観測情報なしの証拠生成')
                    require(actual == expected, '学習グラフ次数と境界属性の不一致')
                    has_candidate |= any(actual)
                    checked_maps += 1
                if pending_message.done():
                    payload = pending_message.result()
                    require(payload is not None, 'WebSocket切断')
                    streamed = decode_graph_packet(payload)
                    if streamed is not None:
                        require(streamed['frameId'] == cloud.header.frame_id, 'Viewer転送時の座標系不一致')
                        require(all(isinstance(point.get('is_boundary_candidate'), bool)
                                    for point in streamed['nodes']), 'Viewerへの境界属性欠落')
                        streamed_neighbors = [set() for _ in streamed['nodes']]
                        for idx in range(0, len(streamed['edges']), 2):
                            source, target = streamed['edges'][idx:idx + 2]
                            streamed_neighbors[source].add(target)
                            streamed_neighbors[target].add(source)
                        streamed_actual = [point['is_boundary_candidate'] for point in streamed['nodes']]
                        require(all(point.get('boundary_evidence', 0) == 0 for point in streamed['nodes']), '不明証拠の転送不一致')
                        streamed_expected = [enable_candidates and len(adjacent) <= max_neighbors
                                             for adjacent in streamed_neighbors]
                        if streamed_actual != streamed_expected:
                            print('転送診断:', streamed['nodes'][:3], 'ROS受信:',
                                  [(len(graph.nodes), sum(point.is_boundary_candidate for point in graph.nodes))
                                   for graph in maps[-3:]], flush=True)
                        require(streamed_actual == streamed_expected,
                                f'Viewerへの転送値不一致: nodes={len(streamed_actual)}, '
                                f'actual={sum(streamed_actual)}, expected={sum(streamed_expected)}, '
                                f'diff={[idx for idx, pair in enumerate(zip(streamed_actual, streamed_expected)) if pair[0] != pair[1]][:10]}')
                        num_streams += 1
                        # ブラウザと同じ受信完了通知。次フレームの転送許可。
                        await connection.write_message(json.dumps({
                            'type': 'stream.topological_map.applied', 'topic': '/topological_map'}))
                    pending_message = connection.read_message()
                if checked_maps >= 8 and num_streams >= 3:
                    break
                await asyncio.sleep(0.005)
            require(checked_maps >= 8 and num_streams >= 3,
                    f'ROSまたはViewer検証フレーム不足: ROS={checked_maps}, WS={num_streams}')
            if enable_candidates and max_neighbors == 4:
                require(has_candidate, '境界候補未検出')
            topics = dict(node.get_topic_names_and_types())
            require('/boundary_candidates' not in topics and '/boundary_candidates/markers' not in topics,
                    '不要な専用トピックの残存')
            drain_end = time.monotonic() + 0.5
            while time.monotonic() < drain_end:
                rclpy.spin_once(node, timeout_sec=0)
                await asyncio.sleep(0.01)
            num_maps = len(maps)
            wait_end = time.monotonic() + 0.3
            while time.monotonic() < wait_end:
                rclpy.spin_once(node, timeout_sec=0)
                await asyncio.sleep(0.01)
            require(len(maps) == num_maps, '入力停止中の学習出力増加')
            require(subscription is not None, '購読保持の失敗')
            print(f'検証成功: enable={enable_candidates}, max_neighbors={max_neighbors}, ROS={checked_maps}, WS={num_streams}', flush=True)
        finally:
            if connection is not None:
                connection.close()
            if pending_message is not None and not pending_message.done():
                pending_message.cancel()
            stop_process(process)
            stop_process(gateway)
            if node is not None:
                node.destroy_node()
            log.seek(0)
            print('\n'.join(log.read().splitlines()[-6:]), flush=True)
            print('検証用GNG・Viewer: 停止・回収済み', flush=True)


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--gateway-executable', required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '189'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    try:
        for enable_candidates, max_neighbors in [(True, 4), (True, 0), (False, 4)]:
            await check_case(args.executable, args.gateway_executable, enable_candidates, max_neighbors)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
