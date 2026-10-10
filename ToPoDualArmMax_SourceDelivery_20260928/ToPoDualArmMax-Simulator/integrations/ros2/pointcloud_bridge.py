"""ブラウザの点群をROS 2へ配信する独立HTTPブリッジ。GNG・FVGへの依存なし。"""
import argparse
import json
import math
import struct
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from robot_exchange import validate_state, RobotExchange
from native_points import ensure_native, validate_payload

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
    has_color = 'color_format' in meta
    if has_color and (meta['color_format'] != 'rgb8_valid8' or meta['source'] not in ('rgbd', 'object_visible')):
        raise ValueError('色情報の形式・点群種別が不正です')
    color_offset = offset + count * 12 + num_pixels * 4
    if len(raw) != color_offset + (count * 4 if has_color else 0):
        raise ValueError('点群・深度画像のデータ長が不正です')
    data = raw[offset:]
    validate_payload(data, count, num_pixels, has_color)
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


def make_handler(publish, allowed_origins, publish_state=None, latest_trajectory=None, follow=None):
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
            self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-ToPo-Points, X-ToPo-Follow')
            self.end_headers()

        def do_GET(self):
            if urlsplit(self.path).path == '/api/trajectory' and latest_trajectory:
                try:
                    model = parse_qs(urlsplit(self.path).query).get('model', [''])[0]
                    return self.respond(200, latest_trajectory(model))
                except ValueError as error:
                    return self.respond(400, {'error': str(error)})
            if urlsplit(self.path).path == '/api/follow':
                return self.respond(200, follow.snapshot() if follow is not None else {'has_manager': False})
            if urlsplit(self.path).path == '/api/points/status':
                return self.respond(200, {'service': 'topo-pointcloud-bridge', 'protocol_version': 4, 'topics': topics, 'pointcloud_backend': 'cpp'})
            super().do_GET()

        def do_POST(self):
            if urlsplit(self.path).path == '/api/follow':
                if self.headers.get('Origin') not in allowed_origins or self.headers.get('X-ToPo-Follow') != '1':
                    return self.respond(403, {'error': '許可されていない構成操作です'})
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if follow is None or not 0 < size <= 4096:
                        raise ValueError('構成操作の入力不正')
                    data = json.loads(self.rfile.read(size))
                    if not isinstance(data, dict):
                        raise ValueError('構成操作の形式不正')
                    return self.respond(200, follow.perform(data))
                except (ValueError, TypeError, TimeoutError) as error:
                    return self.respond(400, {'error': str(error)})
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
    from sensor_msgs.msg import PointCloud2, JointState
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tf-topic', default='/tf', help='標準TFの配信先。/sim/tf指定で専用配信のみ')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8879)
    parser.add_argument('--allow-origin', action='append', default=[])
    args = parser.parse_args()
    if not 1024 <= args.port <= 65534:
        parser.error('HTTPポートは1024～65534。次のポートを関節WebSocketに使用します')
    from joint_stream import start_joint_stream
    allowed = set(args.allow_origin) | {f'http://{host}:{port}' for host in ('127.0.0.1', 'localhost')
                                       for port in (8877, args.port)}
    ensure_native()
    rclpy.init()
    node = Node('topo_browser_pointcloud_bridge')
    from lazy_output import output_registry
    # 実送信中のトピックだけのROS配信口。未使用センサの候補表示の防止
    outputs = output_registry(node)
    publishers = {key: outputs.create_publisher(PointCloud2, topic, 2) for key, topic in topics.items()}
    joints = outputs.create_publisher(JointState, '/sim/joint_states', 2)
    info = outputs.create_publisher(String, '/sim/points/info', 2)
    from depth_output import create_point_messages, depth_topics
    from sensor_msgs.msg import Image, CameraInfo
    depth_publishers = [outputs.create_publisher(kind, topic, 2) for kind, topic in
                        zip((Image, CameraInfo, PointCloud2), depth_topics)]
    lock = threading.Lock()
    exchange = RobotExchange(node, joints, args.tf_topic, outputs.create_publisher)
    from follow_bridge import follow_bridge
    follow = follow_bridge(node, outputs)

    def publish_state(state):
        with lock:
            return exchange.publish(state)


    def publish(meta, data):
        if meta.get('robot_state') is not None:
            exchange.validate(meta['robot_state'])
        stamp = node.get_clock().now().to_msg()
        # 画素ごとの検査・逆投影・色付けをC++で実行。通信・姿勢更新用GILの解放。
        msg, depth_messages = create_point_messages(meta, data, stamp)
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

    stop_joint_stream = start_joint_stream(node, exchange, args.host, args.port + 1, allowed, follow)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(publish, allowed, publish_state, exchange.latest, follow))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'ROS 2点群ブリッジ: http://{args.host}:{args.port}/', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_joint_stream()
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        outputs.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
