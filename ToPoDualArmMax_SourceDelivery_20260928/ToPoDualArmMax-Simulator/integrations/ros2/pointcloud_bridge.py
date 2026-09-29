"""ブラウザの点群をROS 2へ配信する独立HTTPブリッジ。GNG・FVGへの依存なし。"""
import argparse
import json
import math
import struct
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

root = Path(__file__).resolve().parents[2] / 'app'
topics = {'rgbd': '/sim/rgbd/points', 'object_full': '/sim/object/full_points',
          'object_visible': '/sim/object/visible_points'}
max_body_bytes = 13064008


def parse_packet(raw):
    if len(raw) < 8 or raw[:4] != b'TPC1':
        raise ValueError('TPC1形式の点群が必要です')
    size, = struct.unpack_from('<I', raw, 4)
    if size > 64000 or 8 + size > len(raw):
        raise ValueError('メタデータ長が不正です')
    meta = json.loads(raw[8:8 + size])
    if not isinstance(meta, dict):
        raise ValueError('メタデータはオブジェクト形式です')
    count = meta.get('count')
    offset = 8 + (size + 3) // 4 * 4
    if type(count) is not int or not 0 <= count <= 1000000 or len(raw) != offset + count * 12:
        raise ValueError('点数・データ長が不正です')
    if meta.get('source') not in topics or meta.get('frame_id') != 'base_footprint':
        raise ValueError('点群種別・座標系が不正です')
    data = raw[offset:]
    if any(not math.isfinite(v[0]) for v in struct.iter_unpack('<f', data)):
        raise ValueError('座標に非有限値があります')
    pose = meta.get('robot_pose', {})
    if not isinstance(pose, dict) or len(pose) > 64 or any(
            not isinstance(k, str) or type(v) not in (int, float) or not math.isfinite(v)
            for k, v in pose.items()):
        raise ValueError('関節角が不正です')
    if meta['source'] != 'rgbd':
        matrix = meta.get('object_to_world')
        if type(meta.get('object_id')) is not int or not isinstance(matrix, list) or len(matrix) != 16 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in matrix):
            raise ValueError('対象物体の姿勢が不正です')
    return meta, data


def make_handler(publish, allowed_origins):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(root), **kwargs)

        def end_headers(self):
            origin = self.headers.get('Origin')
            if origin in allowed_origins:
                self.send_header('Access-Control-Allow-Origin', origin)
                self.send_header('Vary', 'Origin')
            self.send_header('Cache-Control', 'no-store')
            super().end_headers()

        def respond(self, status, value):
            body = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            if self.headers.get('Origin') not in allowed_origins:
                return self.respond(403, {'error': '許可されていないOriginです'})
            self.send_response(204)
            self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-ToPo-Points')
            self.end_headers()

        def do_GET(self):
            if urlsplit(self.path).path == '/api/points/status':
                return self.respond(200, {'service': 'topo-pointcloud-bridge', 'topics': topics})
            super().do_GET()

        def do_POST(self):
            if urlsplit(self.path).path != '/api/points':
                return self.respond(404, {'error': 'APIが見つかりません'})
            if self.headers.get('Origin') not in allowed_origins or self.headers.get('X-ToPo-Points') != '1':
                self.close_connection = True
                return self.respond(403, {'error': '送信元・ヘッダーが不正です'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 8 <= length <= max_body_bytes:
                    raise ValueError('データ長が不正です')
                self.connection.settimeout(10)
                meta, data = parse_packet(self.rfile.read(length))
                result = publish(meta, data)
            except (ValueError, TypeError, OverflowError, TimeoutError) as error:
                self.close_connection = True
                return self.respond(400, {'error': str(error)})
            self.respond(200, result)
    return Handler


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, JointState
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8879)
    parser.add_argument('--allow-origin', action='append', default=[])
    args = parser.parse_args()
    allowed = set(args.allow_origin) | {f'http://{host}:{port}' for host in ('127.0.0.1', 'localhost')
                                       for port in (8877, args.port)}
    rclpy.init()
    node = Node('topo_browser_pointcloud_bridge')
    publishers = {key: node.create_publisher(PointCloud2, topic, 2) for key, topic in topics.items()}
    joints = node.create_publisher(JointState, '/sim/joint_states', 2)
    info = node.create_publisher(String, '/sim/points/info', 2)
    lock = threading.Lock()

    def publish(meta, data):
        with lock:
            stamp = node.get_clock().now().to_msg()
            msg = PointCloud2()
            msg.header.stamp = stamp
            msg.header.frame_id = meta['frame_id']
            msg.height = 1
            msg.width = meta['count']
            msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = len(data)
            msg.is_dense = True
            msg.data = data
            publishers[meta['source']].publish(msg)
            pose = JointState()
            pose.header = msg.header
            pose.name = list(meta.get('robot_pose', {}))
            pose.position = [float(v) for v in meta.get('robot_pose', {}).values()]
            joints.publish(pose)
            info.publish(String(data=json.dumps(dict(meta, stamp_sec=stamp.sec, stamp_nanosec=stamp.nanosec))))
            return {'topic': topics[meta['source']], 'count': msg.width, 'stamp_sec': stamp.sec, 'stamp_nanosec': stamp.nanosec}

    server = ThreadingHTTPServer((args.host, args.port), make_handler(publish, allowed))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'ROS 2点群ブリッジ: http://{args.host}:{args.port}/', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
