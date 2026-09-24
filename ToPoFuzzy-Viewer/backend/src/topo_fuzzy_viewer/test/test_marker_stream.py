"""大容量Markerの送信待ち・最新反映・Graph併用・旧クライアント互換性の分離ROS試験。"""

import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

from test_nonplane_stream import Client


class RenderClient(Client):
    def __init__(self, port):
        super().__init__(port)
        self.frames = []
        self.marker_bytes = 0
        self.expected_points = None

    def until(self, predicate, sec=15):
        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            self.sock.settimeout(max(0.001, deadline - time.monotonic()))
            header = self.read(2)
            num = header[1] & 127
            if num == 126:
                num = struct.unpack('!H', self.read(2))[0]
            elif num == 127:
                num = struct.unpack('!Q', self.read(8))[0]
            data = self.read(num)
            value = json.loads(data) if header[0] & 15 == 1 else data
            if isinstance(value, dict) and value.get('type') == 'stream.marker_array':
                self.frames.append(value['markers'][0]['id'])
                self.marker_bytes += len(data)
                if self.expected_points is not None:
                    assert sum(len(marker.get('points', [])) for marker in value['markers']) == self.expected_points
                assert 'arrow_styles' in value, '省略後も復元可能な完全スナップショット'
            if isinstance(value, bytes) and value[:4] == b'TMG1':
                tag_size = struct.unpack_from('<I', value, 8)[0]
                self.send({'type': 'stream.topological_map.applied',
                           'topic': value[36:36 + tag_size].decode()})
            if predicate(value):
                return value
        raise TimeoutError('配信待機の時間超過')


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '94'
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node('marker_stream_test_20260925')
    topic = '/plane_clusters/markers/nodes'
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub = node.create_publisher(MarkerArray, topic, qos)
    graph_pub = node.create_publisher(TopologicalMap, '/topological_map', qos)
    clients = []
    gateway = None
    with tempfile.TemporaryDirectory(prefix='marker-stream-test-') as temp:
        with open(Path(temp) / 'gateway.log', 'w+') as log:
            try:
                # 実入力fixtureがある場合は同じ個数・座標の大容量Markerを使用。
                fixture = Path('/ros2_ws/src/artifacts/viewer_plane_markers_20260925/input.json')
                data = json.loads(fixture.read_text())['markers'] if fixture.exists() else [{
                    'type': 'line_list', 'points': [[idx * 0.01, 1., 2.] for idx in range(60000)]}]
                message = MarkerArray()
                for row in data:
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.ns = row.get('ns', 'test')
                    marker.type = {'line_list': Marker.LINE_LIST, 'points': Marker.POINTS}.get(row['type'], Marker.POINTS)
                    marker.pose.orientation.w = 1.
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.01
                    marker.color.r = marker.color.a = 1.
                    marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in row['points']]
                    message.markers.append(marker)

                with socket.socket() as probe:
                    probe.bind(('127.0.0.1', 0))
                    port = probe.getsockname()[1]
                env = dict(os.environ, ROS_LOG_DIR=temp)
                command = [os.environ.get('VIEWER_GATEWAY_EXECUTABLE',
                    '/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node'),
                    '--ros-args', '-p', f'port:={port}']
                print('START:', ' '.join(command), flush=True)
                gateway = subprocess.Popen(command, env=env, stdout=log, stderr=log, start_new_session=True)
                deadline = time.monotonic() + 15
                while not clients and time.monotonic() < deadline:
                    try:
                        clients.append(RenderClient(port))
                    except ConnectionRefusedError:
                        time.sleep(0.05)
                assert clients
                paced = clients[0]
                num_points = sum(len(marker.points) for marker in message.markers)
                paced.expected_points = num_points
                paced.until(lambda v: isinstance(v, dict) and v.get('type') == 'stream.capabilities')
                paced.send({'type': 'stream.marker_array.ready'})
                legacy = RenderClient(port)
                clients.append(legacy)
                legacy.expected_points = num_points
                legacy.until(lambda v: isinstance(v, dict) and v.get('type') == 'stream.capabilities')

                def active(is_active):
                    paced.send({'id': 'marker-switch', 'method': 'sources.setActive',
                                'params': {'sourceId': topic, 'active': is_active, 'removeLayer': not is_active}})
                    paced.until(lambda v: isinstance(v, dict) and v.get('id') == 'marker-switch')

                active(True)
                paced.send({'id': 'graph-on', 'method': 'sources.setActive',
                            'params': {'sourceId': '/topological_map', 'active': True}})
                paced.until(lambda v: isinstance(v, dict) and v.get('id') == 'graph-on')
                deadline = time.monotonic() + 10
                while marker_pub.get_subscription_count() == 0 or graph_pub.get_subscription_count() == 0:
                    assert time.monotonic() < deadline
                    rclpy.spin_once(node, timeout_sec=0.05)

                def publish(frame):
                    message.markers[0].id = frame
                    marker_pub.publish(message)
                    graph = TopologicalMap()
                    graph.header.frame_id = 'map'
                    graph.header.stamp.sec = graph.frame_number = frame
                    graph.nodes = [TopologicalNode(id=0)]
                    graph_pub.publish(graph)

                def is_marker(frame):
                    return lambda v: isinstance(v, dict) and v.get('type') == 'stream.marker_array' and v['markers'][0]['id'] == frame

                publish(1)
                paced.until(is_marker(1))
                paced.send({'type': 'request.state'})
                for frame in range(2, 9):
                    publish(frame)
                    legacy.until(is_marker(frame))
                paced.until(lambda v: isinstance(v, bytes) and v[:4] == b'TMG1' and struct.unpack_from('<I', v, 16)[0] == 8)
                assert paced.frames == [1], paced.frames
                print(json.dumps({'before_ack': paced.frames, 'legacy_frames': legacy.frames,
                    'paced_bytes': paced.marker_bytes, 'legacy_bytes': legacy.marker_bytes}), flush=True)
                paced.send({'type': 'stream.marker_array.applied', 'topic': topic})
                paced.until(is_marker(8))
                assert paced.frames == [1, 8], paced.frames
                print('PASS: ACK待ち1件、最新8へ直接更新、Graph併用、旧クライアントの連続受信', flush=True)

                publish(9)
                legacy.until(is_marker(9))
                active(False)
                active(True)
                paced.until(is_marker(9))
                print('PASS: 購読解除時の未完了ACK破棄と同じトピックの再開', flush=True)
                previous_num = len(paced.frames)
                paced.send({'type': 'stream.marker_array.applied', 'topic': topic})
                paced.send({'type': 'request.state'})
                paced.until(is_marker(9))
                assert len(paced.frames) == previous_num + 1
                print('PASS: 配信停止中のrequest.stateによる最新状態再取得と全点数保持', flush=True)
                paced.sock.close()
                clients.remove(paced)
                paced = RenderClient(port)
                clients.append(paced)
                paced.expected_points = num_points
                paced.until(is_marker(9))
                print('PASS: 再接続時の最新Marker復元', flush=True)
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read(), flush=True)
                raise
            finally:
                for client in clients:
                    client.sock.close()
                node.destroy_node()
                rclpy.shutdown()
                if gateway:
                    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                        try:
                            os.killpg(gateway.pid, sig)
                        except ProcessLookupError:
                            break
                        try:
                            gateway.wait(timeout=5)
                            break
                        except subprocess.TimeoutExpired:
                            pass
                    try:
                        os.killpg(gateway.pid, 0)
                    except ProcessLookupError:
                        print('STOPPED: 試験gateway・ROS購読・WebSocket接続', flush=True)
                    else:
                        raise AssertionError('試験gatewayのプロセス残存')


if __name__ == '__main__':
    main()
