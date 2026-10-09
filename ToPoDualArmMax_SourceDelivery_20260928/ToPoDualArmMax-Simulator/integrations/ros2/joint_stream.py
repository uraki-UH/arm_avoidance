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


def start_joint_stream(node, exchange, host, port, origins):
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    ready = threading.Event()
    context = {}

    class Handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.set_nodelay(True)
            self.subscription = None
            self.latest = None
            self.last_sent = None
            self.is_writing = False
            self.model = None
            self.last_publish = 0.
            self.enable_fresh_input = False
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
                    if not isinstance(topic, str) or not topic.startswith('/') or topic == '/sim/joint_states':
                        raise ValueError('入力は絶対トピック名を指定。送信先 /sim/joint_states の自己受信は禁止です')
                    self.model, self.hz = model, hz
                    self.enable_fresh_input = data.get('enable_fresh_input') is True
                    max_age = data.get('max_state_age_sec', .3)
                    if type(max_age) not in (int, float) or not math.isfinite(max_age) or not 0 < max_age <= 5:
                        raise ValueError('実測鮮度の許容時間は0〜5 sの正数が必要です')
                    self.max_state_age_sec = max_age
                    if data.get('receive') is True:
                        self.subscription = node.create_subscription(JointState, topic, self.receive, qos_profile_sensor_data)
                    self.timer = tornado.ioloop.PeriodicCallback(self.flush_latest, 1000 / hz)
                    self.timer.start()
                    self.write_message({'type': 'ready'})
                elif data.get('type') == 'state' and self.model is not None:
                    state = data.get('state')
                    if not isinstance(state, dict) or state.get('robot_model') != self.model:
                        raise ValueError('状態のモデルが一致しません')
                    outputs = data.get('outputs')
                    if not isinstance(outputs, list) or any(value not in ('base', 'tf', 'joints') for value in outputs):
                        raise ValueError('状態送信項目が不正です')
                    now = time.monotonic()
                    if now - self.last_publish >= 1 / self.hz * .9:
                        exchange.publish(state, outputs=outputs)
                        self.last_publish = now
                    self.write_message({'type': 'ack'})
                elif data.get('type') == 'joints' and self.model is not None:
                    pose = data.get('pose')
                    if not isinstance(pose, dict) or not pose or not set(pose) <= exchange.joint_names[self.model] or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose.values()):
                        raise ValueError('関節角が不正です')
                    now = time.monotonic()
                    if now - self.last_publish >= 1 / self.hz * .9:
                        message = JointState()
                        message.header.stamp = node.get_clock().now().to_msg()
                        message.header.frame_id = 'base_footprint'
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
            if self.enable_fresh_input:
                try:
                    stamp_sec = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                except AttributeError:
                    return
            with self.data_lock:
                # 物理フォロワー用の実時刻と進行確認。再送キャッシュによる鮮度更新なし
                if self.enable_fresh_input and (not 0 <= time.time()-stamp_sec < self.max_state_age_sec or stamp_sec <= self.last_stamp_sec):
                    return
                self.last_stamp_sec = stamp_sec
                self.latest = {'type': 'joints', 'pose': pose, 'received': time.monotonic(), 'stamp_sec': stamp_sec}

        async def flush_latest(self):
            with self.data_lock:
                latest = self.latest
            if self.is_writing or latest is None or latest is self.last_sent or time.monotonic() - latest['received'] > max(.5, 2 / self.hz):
                return
            self.is_writing = True
            try:
                await self.write_message(latest)
                self.last_sent = latest
            except tornado.websocket.WebSocketClosedError:
                pass
            finally:
                self.is_writing = False

        def on_close(self):
            if self.timer:
                self.timer.stop()
            if self.subscription is not None:
                node.destroy_subscription(self.subscription)
                self.subscription = None
            context['clients'].discard(self)

    def run():
        asyncio.set_event_loop(asyncio.new_event_loop())
        loop = tornado.ioloop.IOLoop.current()
        context.update(loop=loop, clients=set())
        try:
            from physics_stream import physics_handler
            server = tornado.httpserver.HTTPServer(tornado.web.Application([(r'/joints', Handler), (r'/physics', physics_handler(origins, context['clients']))], websocket_max_message_size=1048576))
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
    return stop
