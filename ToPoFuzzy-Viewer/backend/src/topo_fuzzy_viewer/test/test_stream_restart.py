"""配信元のプロセス停止・再起動と描画完了通知欠落の回帰検証。"""

import base64
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time


def publish(generation):
    import rclpy
    from ais_gng_msgs.msg import TopologicalMap, TopologicalNode
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import Marker, MarkerArray
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    rclpy.init()
    node = rclpy.create_node('stream_restart_publisher')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cloud_pub = node.create_publisher(PointCloud2, '/restart/points', 1)
    graph_pub = node.create_publisher(TopologicalMap, '/restart/Tmap', latched)
    marker_qos = latched if generation == 1 else QoSProfile(
        depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    marker_pub = node.create_publisher(MarkerArray, '/restart/markers', marker_qos)
    graph = TopologicalMap()
    graph.header.frame_id = 'map'
    vertex = TopologicalNode()
    vertex.id = generation
    vertex.pos.x = float(generation)
    graph.nodes = [vertex]
    graph_pub.publish(graph)
    cloud = PointCloud2()
    cloud.header.frame_id = 'map'
    cloud.height = cloud.width = 1
    cloud.fields = [PointField(name=axis, offset=idx * 4, datatype=7, count=1)
                    for idx, axis in enumerate('xyz')]
    cloud.point_step = cloud.row_step = 12
    cloud.data = struct.pack('<fff', generation, 0, 1)
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.id = generation
    marker.type = Marker.SPHERE
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(generation)
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.color.a = 1.0
    label = Marker()
    label.header = marker.header
    label.id = 1000 + generation
    label.type = Marker.TEXT_VIEW_FACING
    label.pose.orientation.w = 1.0
    label.scale.z = 0.025
    label.color.a = 1.0
    label.text = f'候補{generation}: local_point_budget\n接触未確認'
    timer = node.create_timer(0.1, lambda: (
        cloud_pub.publish(cloud), marker_pub.publish(MarkerArray(markers=[marker, label]))))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class Client:
    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(('GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n'
                           'Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n'
                           f'Sec-WebSocket-Key: {key}\r\n\r\n').encode())
        header = b''
        while not header.endswith(b'\r\n\r\n'):
            header += self.read(1)
        assert b'101 Switching Protocols' in header

    def read(self, num):
        data = bytearray()
        while len(data) < num:
            part = self.sock.recv(num - len(data))
            if not part:
                raise EOFError()
            data.extend(part)
        return bytes(data)

    def send(self, value):
        data = json.dumps(value).encode()
        mask = os.urandom(4)
        header = bytes([0x81, 0x80 | len(data)]) if len(data) < 126 else (
            bytes([0x81, 0xfe]) + struct.pack('!H', len(data)))
        self.sock.sendall(header + mask + bytes(v ^ mask[idx % 4] for idx, v in enumerate(data)))

    def receive(self):
        header = self.read(2)
        num = header[1] & 127
        if num == 126:
            num = struct.unpack('!H', self.read(2))[0]
        elif num == 127:
            num = struct.unpack('!Q', self.read(8))[0]
        data = self.read(num)
        return json.loads(data) if header[0] & 15 == 1 else data

    def until(self, predicate):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            value = self.receive()
            if predicate(value):
                return value
        raise AssertionError('配信待機の時間超過')


def stop(process):
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def main():
    # 通常Viewerと分離したROSドメイン・WebSocketポート。
    env = dict(os.environ, ROS_DOMAIN_ID='83', ROS_LOCALHOST_ONLY='1')
    port = 19091
    gateway = publisher = client = None
    topics = ['/restart/points', '/restart/Tmap', '/restart/markers']
    with tempfile.TemporaryFile() as log:
        try:
            gateway = subprocess.Popen([
                os.environ.get('GATEWAY_BINARY', '/ros2_ws/build/topo_fuzzy_viewer/viewer_ws_gateway_node'),
                '--ros-args', '-p', f'port:={port}'], env=env, stdout=log, stderr=log)
            publisher = subprocess.Popen([sys.executable, __file__, '--publisher', '1'],
                                         env=env, stdout=log, stderr=log)
            deadline = time.monotonic() + 15
            while client is None and time.monotonic() < deadline:
                try:
                    client = Client(port)
                except ConnectionRefusedError:
                    time.sleep(0.1)
            assert client is not None
            client.until(lambda v: isinstance(v, dict) and v.get('id') == 'sync_sources'
                         and set(topics) <= {s['id'] for s in v['result']['sources']})
            for topic in topics:
                client.send({'id': topic, 'method': 'sources.setActive',
                             'params': {'sourceId': topic, 'active': True}})

            def is_graph(value, generation):
                if not isinstance(value, bytes) or value[:4] != b'TMG1':
                    return False
                tag_size, frame_size = struct.unpack_from('<II', value, 8)
                return struct.unpack_from('<H', value, 36 + tag_size + frame_size)[0] == generation

            # 描画完了通知を意図的に送らず、旧世代を送信待ち状態に固定。
            client.until(lambda v: is_graph(v, 1))
            stop(publisher)
            deleted = set()
            def has_deleted(value):
                if isinstance(value, dict) and value.get('type') == 'stream.delete':
                    deleted.add(value['topic'])
                return set(topics) <= deleted
            client.until(has_deleted)
            client.send({'id': 'waiting', 'method': 'sources.list'})
            waiting = client.until(lambda v: isinstance(v, dict) and v.get('id') == 'waiting')
            assert set(topics) <= {s['id'] for s in waiting['result']['sources'] if s['active']}, waiting
            publisher = subprocess.Popen([sys.executable, __file__, '--publisher', '2'],
                                         env=env, stdout=log, stderr=log)
            generation = 2
            received = set()
            def has_new_data(value):
                if is_graph(value, generation):
                    received.add('graph')
                elif isinstance(value, bytes):
                    body = value[1 + value[0]:]
                    if len(body) >= 32 and struct.unpack_from('<I', body)[0] == 0x50434458:
                        if struct.unpack_from('<f', body, 20)[0] == generation:
                            received.add('cloud')
                elif value.get('type') == 'stream.marker_array':
                    if any(m['id'] == generation for m in value.get('markers', [])):
                        text_marker = next(m for m in value['markers'] if m['id'] == 1000 + generation)
                        assert text_marker['type'] == 'text'
                        assert text_marker['text'] == f'候補{generation}: local_point_budget\n接触未確認'
                        assert text_marker['scale'][2] == 0.025
                        received.add('marker')
                return received == {'graph', 'cloud', 'marker'}
            client.until(has_new_data)
            # 消失検出を待たない即時再起動と、再度の未ACK状態からの復帰。
            stop(publisher)
            generation = 3
            received.clear()
            publisher = subprocess.Popen([sys.executable, __file__, '--publisher', '3'],
                                         env=env, stdout=log, stderr=log)
            client.until(has_new_data)
            print('PASS: 全レイヤー削除、選択維持、未ACKグラフ・点群・QoS変更Marker、文字本文、即時再起動')
        except BaseException:
            log.seek(0)
            print(log.read().decode(errors='replace'), file=sys.stderr)
            raise
        finally:
            if client:
                client.sock.close()
            stop(publisher)
            stop(gateway)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--publisher':
        publish(int(sys.argv[2]))
    else:
        main()
