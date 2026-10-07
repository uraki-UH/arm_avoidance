"""ブラウザの点群をROS 2へ配信する独立HTTPブリッジ。GNG・FVGへの依存なし。"""
from array import array
import argparse
import json
import math
import struct
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from robot_exchange import validate_state, RobotExchange

root = Path(__file__).resolve().parents[2] / 'app'
topics = {'rgbd': '/sim/rgbd/points', 'object_full': '/sim/object/full_points',
          'object_visible': '/sim/object/visible_points', 'mid360': '/sim/lidar/points'}
max_body_bytes = 50000000


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
    if type(count) is not int or not 0 <= count <= 2073600:
        raise ValueError('点数・データ長が不正です')
    if meta.get('source') not in topics or meta.get('frame_id') != 'base_footprint':
        raise ValueError('点群種別・座標系が不正です')
    depth = meta.get('depth_image')
    num_pixels = 0
    if depth is not None:
        if meta['source'] != 'rgbd' or not isinstance(depth, dict):
            raise ValueError('深度画像はRGB-D全体のみ対応です')
        width, height = depth.get('width'), depth.get('height')
        if type(width) is not int or type(height) is not int or not 16 <= width <= 1920 or not 16 <= height <= 1080:
            raise ValueError('深度画像の寸法が不正です')
        if any(type(depth.get(k)) not in (float, int) or not math.isfinite(depth[k]) for k in ('fx', 'fy', 'ppx', 'ppy')) or depth['fx'] <= 0 or depth['fy'] <= 0:
            raise ValueError('内部パラメータが不正です')
        if depth.get('model') != 'none' or depth.get('coeffs') != [0, 0, 0, 0, 0]:
            raise ValueError('歪み補正済みの深度画像が必要です')
        num_pixels = width * height
    if len(raw) != offset + count * 12 + num_pixels * 4:
        raise ValueError('点群・深度画像のデータ長が不正です')
    if num_pixels and any(not math.isfinite(v[0]) or v[0] < 0 for v in struct.iter_unpack('<f', raw[offset + count * 12:])):
        raise ValueError('深度値が不正です')
    data = raw[offset:]
    if any(not math.isfinite(v[0]) for v in struct.iter_unpack('<f', data[:count * 12])):
        raise ValueError('座標に非有限値があります')
    pose = meta.get('robot_pose', {})
    if not isinstance(pose, dict) or len(pose) > 64 or any(
            not isinstance(k, str) or type(v) not in (int, float) or not math.isfinite(v)
            for k, v in pose.items()):
        raise ValueError('関節角が不正です')
    if meta['source'] in ('object_full', 'object_visible'):
        matrix = meta.get('object_to_world')
        if type(meta.get('object_id')) is not int or not isinstance(matrix, list) or len(matrix) != 16 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in matrix):
            raise ValueError('対象物体の姿勢が不正です')
    if meta.get('robot_state') is not None:
        validate_state(meta['robot_state'])
    if meta['source'] == 'mid360':
        state = meta.get('robot_state')
        if state is None or not any(t['child'] == 'sim_mid360_frame' and t['parent'] == 'base_footprint' for t in state['transforms']):
            raise ValueError('MID-360の取得時TFが必要です')
    return meta, data


def make_handler(publish, allowed_origins, publish_state=None, latest_trajectory=None):
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
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def do_OPTIONS(self):
            if self.headers.get('Origin') not in allowed_origins:
                return self.respond(403, {'error': '許可されていないOriginです'})
            self.send_response(204)
            self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-ToPo-Points')
            self.end_headers()

        def do_GET(self):
            if urlsplit(self.path).path == '/api/trajectory' and latest_trajectory:
                try:
                    model = parse_qs(urlsplit(self.path).query).get('model', [''])[0]
                    return self.respond(200, latest_trajectory(model))
                except ValueError as error:
                    return self.respond(400, {'error': str(error)})
            if urlsplit(self.path).path == '/api/points/status':
                return self.respond(200, {'service': 'topo-pointcloud-bridge', 'topics': topics})
            super().do_GET()

        def do_POST(self):
            if urlsplit(self.path).path not in ('/api/points', '/api/state'):
                return self.respond(404, {'error': 'APIが見つかりません'})
            if self.headers.get('Origin') not in allowed_origins or self.headers.get('X-ToPo-Points') != '1':
                self.close_connection = True
                return self.respond(403, {'error': '送信元・ヘッダーが不正です'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 8 <= length <= max_body_bytes:
                    raise ValueError('データ長が不正です')
                self.connection.settimeout(10)
                if urlsplit(self.path).path == '/api/state':
                    if length > 64000 or publish_state is None:
                        raise ValueError('状態データ長が不正です')
                    result = publish_state(json.loads(self.rfile.read(length)))
                else:
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
    from depth_output import create_depth_messages, depth_topics
    from sensor_msgs.msg import Image, CameraInfo
    depth_publishers = [node.create_publisher(kind, topic, 2) for kind, topic in
                        zip((Image, CameraInfo, PointCloud2), depth_topics)]
    lock = threading.Lock()
    exchange = RobotExchange(node, joints)

    def publish_state(state):
        with lock:
            return exchange.publish(state)


    def publish(meta, data):
        if meta.get('robot_state') is not None:
            exchange.validate(meta['robot_state'])
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
        msg.row_step = meta['count'] * 12
        msg.is_dense = True
        msg.data = array('B', data[:msg.row_step])
        # 深度の逆投影・配列構築中も姿勢送信が可能なロック範囲
        depth_messages = create_depth_messages(meta['depth_image'], data[msg.row_step:], stamp) if meta.get('depth_image') is not None else []
        with lock:
            publishers[meta['source']].publish(msg)
            for publisher, message in zip(depth_publishers, depth_messages):
                publisher.publish(message)
            if meta.get('robot_state') is not None:
                exchange.publish(meta['robot_state'], stamp)
            else:
                pose = JointState()
                pose.header = msg.header
                pose.name = list(meta.get('robot_pose', {}))
                pose.position = [float(v) for v in meta.get('robot_pose', {}).values()]
                joints.publish(pose)
            info.publish(String(data=json.dumps(dict(meta, stamp_sec=stamp.sec, stamp_nanosec=stamp.nanosec))))
        return {'topic': topics[meta['source']], 'depth_topics': depth_topics if meta.get('depth_image') else [], 'count': msg.width, 'stamp_sec': stamp.sec, 'stamp_nanosec': stamp.nanosec}

    server = ThreadingHTTPServer((args.host, args.port), make_handler(publish, allowed, publish_state, exchange.latest))
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
