"""モータ値の読取り専用WebSocket。指令送信・トルクON/OFFへの接続なし。"""

import json
import math
import threading
import time
import tornado.ioloop
import tornado.websocket


def motor_payload(message, name_map, unit, message_type):
    """明示単位での値抽出。DynamixelのmA→A・deg→rad換算、列ごとの欠測保持。"""
    if message_type == 'dynamixel_present':
        names = [str(value) for value in message.id_list]
        fields = {'current_a': ('current_ma', 0.001),
                  'position_rad': ('position_deg', math.pi / 180),
                  'velocity_rad_sec': ('velocity_deg_s', math.pi / 180)}
        if unit not in fields:
            return None
        field, scale = fields[unit]
        values = getattr(message, field, [])
        stamp_sec = None
    else:
        names = list(message.name)
        field = {'position_rad': 'position', 'velocity_rad_sec': 'velocity',
                 'torque_nm': 'effort', 'current_a': 'effort'}.get(unit)
        if field is None:
            return None
        values = getattr(message, field, [])
        scale = 1.0
        stamp = getattr(getattr(message, 'header', None), 'stamp', None)
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9 if stamp and (stamp.sec or stamp.nanosec) else None
    if (len(names) > 128 or len(values) != len(names) or len(set(names)) != len(names)
            or not names or any(not math.isfinite(value) for value in values)):
        return None
    matched = [(name_map[name], value) for name, value in zip(names, values) if name in name_map]
    if len({name for name, _ in matched}) != len(matched):
        return None
    result = {name: float(value * scale) for name, value in matched}
    if not result or any(not math.isfinite(value) for value in result.values()):
        return None
    return {'values': result, 'unit': unit, 'stamp_sec': stamp_sec}


def motor_handler(node, exchange, origins, clients):
    """接続ごとの購読所有と最新値だけの転送。姿勢・物理セッションから独立。"""
    class handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.set_nodelay(True)
            self.subscriptions = []
            self.timer = None
            self.latest = {}
            self.lock = threading.Lock()
            self.is_writing = False
            self.has_config = False
            clients.add(self)

        def on_message(self, raw):
            try:
                config = json.loads(raw)
                if self.has_config or config.get('type') != 'config':
                    raise ValueError('監視は初回設定のみ対応、指令操作は未対応です')
                model = config.get('model')
                if model not in exchange.joint_names:
                    raise ValueError('モデルが不正です')
                aliases = config.get('name_map')
                if (not isinstance(aliases, dict) or not 1 <= len(aliases) <= 128
                        or any(not isinstance(name, str) or not name or len(name) > 128
                               or value not in exchange.joint_names[model] for name, value in aliases.items())):
                    raise ValueError('実機名・IDとURDF関節名の対応が不正です')
                inputs = config.get('inputs')
                if not isinstance(inputs, list) or not 1 <= len(inputs) <= 3:
                    raise ValueError('監視入力は1～3件です')
                units, topics, groups = set(), {}, {}
                for source in inputs:
                    unit, topic, kind = source.get('unit'), source.get('topic'), source.get('message_type')
                    if (unit not in ('motion', 'torque_nm', 'current_a') or unit in units
                            or not isinstance(topic, str) or not topic.startswith('/') or len(topic) > 256
                            or kind not in ('joint_state', 'dynamixel_present')
                            or (kind == 'dynamixel_present' and unit != 'current_a')
                            or (topic in topics and topics[topic] != kind)):
                        raise ValueError('入力トピック・型・単位が不正または重複です')
                    units.add(unit)
                    topics[topic] = kind
                    groups.setdefault((topic, kind), []).append(unit)
                for (topic, kind), selected_units in groups.items():
                    if 'torque_nm' in selected_units and 'current_a' in selected_units:
                        raise ValueError('同一effort列のトルク・電流への二重解釈は未対応です')
                    columns = []
                    for unit in selected_units:
                        columns.extend(['position_rad', 'velocity_rad_sec'] if unit == 'motion' else [unit])
                    if kind == 'dynamixel_present' and 'motion' not in units:
                        columns.extend(['position_rad', 'velocity_rad_sec'])
                    if kind == 'dynamixel_present':
                        from dynamixel_handler_msgs.msg import DynamixelPresent
                        message_class = DynamixelPresent
                    else:
                        from sensor_msgs.msg import JointState
                        message_class = JointState
                    from rclpy.qos import qos_profile_sensor_data
                    callback = lambda message, columns=tuple(columns), kind=kind: self.receive(message, aliases, columns, kind)
                    self.subscriptions.append(node.create_subscription(message_class, topic, callback, qos_profile_sensor_data))
                self.has_config = True
                self.timer = tornado.ioloop.PeriodicCallback(self.flush_latest, 100)
                self.timer.start()
                self.write_message({'type': 'ready'})
            except (ValueError, TypeError, AttributeError, ImportError, RuntimeError) as error:
                self.write_message({'type': 'error', 'error': str(error)})
                self.close()

        def receive(self, message, aliases, columns, kind):
            received = time.monotonic()
            with self.lock:
                for unit in columns:
                    payload = motor_payload(message, aliases, unit, kind)
                    if payload is not None:
                        payload['received'] = received
                        self.latest[unit] = payload

        async def flush_latest(self):
            if self.is_writing:
                return
            with self.lock:
                latest, self.latest = self.latest, {}
            if not latest:
                return
            self.is_writing = True
            try:
                samples = [value for value in latest.values() if time.monotonic() - value['received'] <= 1.0]
                if samples:
                    await self.write_message({'type': 'motor_state', 'samples': samples})
            except tornado.websocket.WebSocketClosedError:
                pass
            finally:
                self.is_writing = False

        def on_close(self):
            if self.timer:
                self.timer.stop()
            for subscription in self.subscriptions:
                node.destroy_subscription(subscription)
            self.subscriptions.clear()
            clients.discard(self)
    return handler
