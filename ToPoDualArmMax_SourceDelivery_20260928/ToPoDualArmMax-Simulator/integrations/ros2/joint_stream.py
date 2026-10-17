"""関節角専用WebSocket。ROS入力の最新値保持と描画から独立した送信経路。"""
import asyncio
import json
import math
import threading
import time

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket
from robot_exchange import robot_base_frame


def start_joint_stream(node, exchange, host, port, origins, follow=None):
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    ready = threading.Event()
    context = {}
    cleanup_futures = set()
    cleanup_lock = threading.Lock()

    def track_cleanup(future):
        with cleanup_lock:
            cleanup_futures.add(future)
        def finished(result):
            if result.exception() is not None:
                node.get_logger().error('物理セッション終了失敗: '+str(result.exception()))
            with cleanup_lock:
                cleanup_futures.discard(result)
        future.add_done_callback(finished)

    class Handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.set_nodelay(True)
            self.subscription = None
            self.reference_subscription = None
            self.latest_reference = self.last_reference = None
            self.latest = None
            self.last_sent = None
            self.is_writing = False
            self.model = None
            self.last_publish = 0.
            self.enable_fresh_input = False
            self.is_read_only = False
            self.max_state_age_sec = .3
            self.last_stamp_sec = -math.inf
            self.timer = None
            self.data_lock = threading.Lock()
            context['clients'].add(self)

        def on_message(self, raw):
            try:
                data = json.loads(raw)
                if data.get('type') == 'config':
                    if self.model is not None:
                        raise ValueError('設定変更には再接続が必要です')
                    model, hz = data.get('model'), data.get('hz')
                    if model not in exchange.joint_names or type(hz) not in (int, float) or not math.isfinite(hz) or not 1 <= hz <= 200:
                        raise ValueError('モデルまたはHzが不正です')
                    topic = data.get('topic', '/joint_states')
                    self.is_read_only = data.get('is_read_only', False)
                    if type(self.is_read_only) is not bool:
                        raise ValueError('受信専用設定には真偽値が必要です')
                    if not isinstance(topic, str) or not topic.startswith('/') or (topic == '/sim/joint_states' and not self.is_read_only):
                        raise ValueError('入力は絶対トピック名を指定。/sim/joint_states は受信専用接続のみ使用可能です')
                    self.model, self.hz = model, hz
                    self.enable_fresh_input = data.get('enable_fresh_input') is True
                    max_age = data.get('max_state_age_sec', .3)
                    if type(max_age) not in (int, float) or not math.isfinite(max_age) or not 0 < max_age <= 5:
                        raise ValueError('実測鮮度の許容時間は0〜5 sの正数が必要です')
                    self.max_state_age_sec = max_age
                    if data.get('receive') is True:
                        self.subscription = node.create_subscription(JointState, topic, self.receive, qos_profile_sensor_data)
                        if self.is_read_only and topic=='/sim/joint_states':
                            from std_msgs.msg import String
                            from rclpy.qos import QoSProfile,DurabilityPolicy
                            self.reference_subscription = node.create_subscription(String,'/sim/nonstop/reference',self.receive_reference,
                                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    self.timer = tornado.ioloop.PeriodicCallback(self.flush_latest, 1000 / hz)
                    self.timer.start()
                    self.write_message({'type': 'ready'})
                elif data.get('type') == 'state' and self.model is not None:
                    if self.is_read_only:
                        raise ValueError('受信専用接続からの状態送信は禁止です')
                    state = data.get('state')
                    if not isinstance(state, dict) or state.get('robot_model') != self.model:
                        raise ValueError('状態のモデルが一致しません')
                    outputs = data.get('outputs')
                    if not isinstance(outputs, list) or any(value not in ('base', 'tf', 'joints') for value in outputs):
                        raise ValueError('状態送信項目が不正です')
                    now = time.monotonic()
                    if now - self.last_publish >= 1 / self.hz * .9:
                        if follow is not None:
                            follow.claim_simulator(self, state)
                        result = exchange.publish(state, outputs=outputs)
                        if follow is not None:
                            follow.publish_simulator(state, result)
                        self.last_publish = now
                    self.write_message({'type': 'ack'})
                elif data.get('type') == 'joints' and self.model is not None:
                    if self.is_read_only:
                        raise ValueError('受信専用接続からの関節送信は禁止です')
                    pose = data.get('pose')
                    if not isinstance(pose, dict) or not pose or not set(pose) <= exchange.joint_names[self.model] or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose.values()):
                        raise ValueError('関節角が不正です')
                    now = time.monotonic()
                    if now - self.last_publish >= 1 / self.hz * .9:
                        message = JointState()
                        message.header.stamp = node.get_clock().now().to_msg()
                        message.header.frame_id = robot_base_frame(self.model)
                        message.name = list(pose)
                        message.position = list(map(float, pose.values()))
                        exchange.joints.publish(message)
                        self.last_publish = now
                    self.write_message({'type': 'ack'})
                else:
                    raise ValueError('未対応の関節通信メッセージです')
            except (ValueError, TypeError, AttributeError, RuntimeError) as error:
                self.write_message({'type': 'error', 'error': str(error)})
                self.close()

        def receive(self, message):
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                return
            pose = {name: value for name, value in zip(message.name, message.position) if name in exchange.joint_names[self.model]}
            if not pose or any(not math.isfinite(v) for v in pose.values()):
                return
            stamp_sec = None
            try:
                stamp_sec = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            except AttributeError:
                if self.enable_fresh_input:
                    return
            motion = {}
            for field in ('velocity', 'effort'):
                values = getattr(message, field, ())
                # JointStateと同じ関節順の実測値。欠測・不正長・非有限値のゼロ補完なし。
                motion[field] = ({name: float(value) for name, value in zip(message.name, values) if name in pose}
                                 if len(values) == len(message.name) and all(math.isfinite(value) for value in values) else {})
            with self.data_lock:
                # 物理フォロワー用の実時刻と進行確認。再送キャッシュによる鮮度更新なし
                if self.enable_fresh_input and (not 0 <= time.time()-stamp_sec < self.max_state_age_sec or stamp_sec <= self.last_stamp_sec):
                    return
                self.last_stamp_sec = stamp_sec
                self.latest = {'type': 'joints', 'pose': pose, 'motion': motion,
                               'received': time.monotonic(), 'stamp_sec': stamp_sec}

        def receive_reference(self, message):
            """CLIと同じ保持付き基準の表示専用転送。FK・ボクセル化の再実行なし。"""
            try:
                if len(message.data)>262144:
                    return
                value = json.loads(message.data)
                if (not isinstance(value,dict) or value.get('format')!='topo-shuttle-reference/1'
                        or value.get('model')!=self.model or not isinstance(value.get('task'),dict)):
                    return
                with self.data_lock:
                    self.latest_reference = dict(type='task_reference',task=value['task'])
            except (ValueError,TypeError):
                return

        async def flush_latest(self):
            with self.data_lock:
                latest = self.latest
                reference = self.latest_reference
            has_reference = reference is not None and reference is not self.last_reference
            has_joints = latest is not None and latest is not self.last_sent and time.monotonic()-latest['received']<=max(.5,2/self.hz)
            if self.is_writing or not (has_reference or has_joints):
                return
            self.is_writing = True
            try:
                if has_reference:
                    await self.write_message(reference)
                    self.last_reference = reference
                if has_joints:
                    await self.write_message(latest)
                    self.last_sent = latest
            except tornado.websocket.WebSocketClosedError:
                pass
            finally:
                self.is_writing = False

        def on_close(self):
            if follow is not None:
                follow.release_simulator(self)
            if self.timer:
                self.timer.stop()
            if self.subscription is not None:
                node.destroy_subscription(self.subscription)
                self.subscription = None
            if self.reference_subscription is not None:
                node.destroy_subscription(self.reference_subscription)
                self.reference_subscription = None
            context['clients'].discard(self)

    def run():
        asyncio.set_event_loop(asyncio.new_event_loop())
        loop = tornado.ioloop.IOLoop.current()
        context.update(loop=loop, clients=set())
        try:
            from physics_stream import physics_handler
            from motor_stream import motor_handler
            from ros_plugins import plugin_handler
            server = tornado.httpserver.HTTPServer(tornado.web.Application([(r'/joints', Handler), (r'/plugins', plugin_handler(origins, context['clients'], node, track_cleanup)), (r'/physics', physics_handler(origins, context['clients'], node, track_cleanup)), (r'/motors', motor_handler(node, exchange, origins, context['clients']))], websocket_max_message_size=1048576))
            server.listen(port, address=host)
            context['server'] = server
        except Exception as error:
            context['error'] = error
            ready.set()
            loop.close()
            return
        ready.set()
        loop.start()
        loop.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    ready.wait()
    if 'error' in context:
        raise RuntimeError(f'関節WebSocketポート{port}を開始できません') from context['error']

    def stop():
        def close():
            for client in list(context['clients']):
                client.on_close()
                client.close()
            context['server'].stop()
            context['loop'].stop()
        context['loop'].add_callback(close)
        thread.join(timeout=3)
        # ROSノード破棄前の、所有launch・物理workerによる資源解放の完了待ち。
        with cleanup_lock:
            pending = list(cleanup_futures)
        for future in pending:
            future.result(timeout=20)
    return stop
