#!/usr/bin/env python3
"""既存Viewer RPCを利用した独立車両照合ノード。"""
import json
from concurrent.futures import ThreadPoolExecutor
from queue import Empty, Queue

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from vehicle_registration import load_models, register_vehicle


class ViewerVehicleRegistrationNode(Node):
    def __init__(self):
        super().__init__('viewer_vehicle_registration_node')
        default_path = get_package_share_directory('topo_fuzzy_viewer') + '/config/vehicle_models/models.json'
        path = self.declare_parameter('model_file', default_path).value
        self.models = load_models(path)
        self.response_pub = self.create_publisher(String, '/viewer/internal/rpc/response', 20)
        self.event_pub = self.create_publisher(String, '/viewer/internal/events/job', 20)
        self.request_sub = self.create_subscription(String, '/viewer/internal/rpc/request', self.handle_request, 20)
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.results = Queue()
        self.is_busy = False
        self.timer = self.create_timer(0.05, self.flush_results)
        self.get_logger().info('車両表面照合の準備完了（4モデル、選択時のみ計算）')

    def publish(self, publisher, payload):
        publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))

    def handle_request(self, message):
        try:
            request = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(request, dict) or request.get('method') != 'vehicle.register':
            return
        req_id = request.get('id', '')
        if self.is_busy:
            self.publish(self.response_pub, {'id': req_id, 'ok': False, 'error': {
                'code': 'BUSY', 'message': '他の車両照合を実行中です。完了後に再実行してください。', 'details': {}}})
            return
        self.is_busy = True
        params = request.get('params', {})
        self.executor_pool.submit(self.run_registration, req_id, params)

    def run_registration(self, req_id, params):
        try:
            def progress(value, label):
                self.results.put(('event', {'type': 'job.progress', 'jobId': req_id, 'sessionId': 'vehicle',
                                           'progress': int(value*100), 'stage': label}))
            if not isinstance(params, dict):
                raise ValueError('paramsはオブジェクトが必要です。')
            result = register_vehicle(params.get('snapshot'), self.models,
                                      params.get('dist_th', 0.25), params.get('support_dist_th', 0.35), progress)
            self.results.put(('event', {'type': 'job.completed', 'jobId': req_id, 'sessionId': 'vehicle',
                                       'durationMs': result['elapsed_ms'], 'message': '車両照合完了'}))
            self.results.put(('response', {'id': req_id, 'ok': True, 'result': result}))
        except Exception as error:
            code = 'INVALID_PARAMS' if isinstance(error, (ValueError, TypeError, KeyError)) else 'REGISTRATION_FAILED'
            payload = {'code': code, 'message': str(error), 'details': {}}
            self.results.put(('event', {'type': 'job.failed', 'jobId': req_id, 'sessionId': 'vehicle', 'error': payload}))
            self.results.put(('response', {'id': req_id, 'ok': False, 'error': payload}))

    def flush_results(self):
        while True:
            try:
                kind, payload = self.results.get_nowait()
            except Empty:
                return
            self.publish(self.event_pub if kind == 'event' else self.response_pub, payload)
            if kind == 'response':
                self.is_busy = False


def main():
    rclpy.init()
    node = ViewerVehicleRegistrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
