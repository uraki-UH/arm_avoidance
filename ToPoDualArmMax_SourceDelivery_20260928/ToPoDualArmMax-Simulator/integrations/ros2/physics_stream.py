"""ブラウザごとの物理セッション。切断時に計算停止。"""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import math
import threading
import time
from enum import Enum
import tornado.ioloop
import tornado.websocket
from physics_scene import PhysicsScene


class command_source(str, Enum):
    browser = 'browser'
    ros = 'ros'
    leader = 'leader'
    avoidance = 'avoidance'
    api = 'api'


def trajectory_command(message, mode):
    """名前付きの単一点即時指令。時系列軌道再生とは独立した受信契約。"""
    if len(message.points) != 1 or not message.joint_names or len(set(message.joint_names)) != len(message.joint_names):
        raise ValueError('ROS指令には重複のない関節名と1点の目標が必要です')
    point = message.points[0]
    if point.time_from_start.sec != 0 or point.time_from_start.nanosec != 0 or point.accelerations:
        raise ValueError('即時指令では時刻指定・加速度指令は未対応です')
    result = dict(mode=mode)
    for field, values in (('position', point.positions), ('velocity', point.velocities), ('effort', point.effort)):
        if values:
            if len(values) != len(message.joint_names):
                raise ValueError('ROS指令の関節数が一致しません')
            result[field] = dict(zip(message.joint_names, values))
    return result


class physics_session:
    """単一workerによるMuJoCo所有。入力は最新1件、未処理キューの蓄積なし。"""
    def __init__(self, scene_factory, ros_node=None):
        self.scene_factory = scene_factory
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='topo-physics')
        self.lock = threading.Lock()
        self.close_event = threading.Event()
        self.scene = None
        self.pending_poses = None
        self.closed_future = None
        self.placement_id = 0
        self.ros_node = ros_node
        self.command_subscription = None
        self.pending_command = None
        self.command_id = 0

    def start(self, value):
        robot = value.get('robot') or {}
        has_avoidance = (value.get('avoidance') or {}).get('mode', 'none') != 'none'
        self.command_source = command_source(robot.get('command_source', 'leader' if robot.get('enable_leader_follow') else 'avoidance' if has_avoidance else 'browser'))
        if has_avoidance != (self.command_source == command_source.avoidance):
            raise ValueError('回避モジュールと指令元の設定が一致しません')
        if self.command_source == command_source.leader and robot.get('control_mode', 'position') != 'position':
            raise ValueError('リーダー姿勢の追従は位置制御専用です')
        self.command_received_sec = time.monotonic()
        self.is_command_stopped = False
        self.enable_command_timeout_hold = robot.get('enable_nonstop_simulation') is not True
        if not self.enable_command_timeout_hold and (self.command_source != command_source.ros
                or robot.get('command_topic') != '/sim/nonstop/joint_command'):
            raise ValueError('非停止実験は専用MuJoCo指令トピックのみの対応です')
        self.enable_leader_follow = robot.get('enable_leader_follow') is True
        self.max_leader_age_sec = robot.get('max_leader_age_sec', 1.)
        if (type(self.max_leader_age_sec) not in (int, float) or
                not math.isfinite(self.max_leader_age_sec) or not 0 < self.max_leader_age_sec <= 5):
            raise ValueError('リーダー実測の鮮度設定不正')
        if self.close_event.is_set():
            return False
        self.scene = self.scene_factory(value['bodies'], value.get('robot'), value.get('avoidance'))
        if self.command_source == command_source.ros:
            if self.ros_node is None or self.scene.robot is None:
                raise ValueError('ROS指令にはROS接続と物理ロボットが必要です')
            topic = robot.get('command_topic', '/sim/joint_command')
            if not isinstance(topic, str) or not topic.startswith('/') or topic.endswith('/') or '//' in topic:
                raise ValueError('ROS指令トピックは絶対名が必要です')
            from trajectory_msgs.msg import JointTrajectory
            from rclpy.qos import qos_profile_sensor_data
            mode = self.scene.robot.control_mode.value
            def receive(message):
                if self.close_event.is_set():
                    return
                try:
                    stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                    if stamp and not 0 <= self.ros_node.get_clock().now().nanoseconds*1e-9-stamp < self.max_leader_age_sec:
                        return
                    self.queue_command(dict(source='ros', command=trajectory_command(message, mode)))
                except (ValueError, AttributeError) as error:
                    self.ros_node.get_logger().error(str(error))
            self.command_subscription = self.ros_node.create_subscription(JointTrajectory, topic, receive, qos_profile_sensor_data)
        self.leader_began_sec = time.time()
        self.leader_stamp_sec = None
        self.is_leader_stopped = False
        return not self.close_event.is_set()

    def queue_poses(self, value):
        with self.lock:
            if not self.close_event.is_set():
                self.pending_poses = value

    def queue_command(self, value):
        if value.get('source') != self.command_source.value:
            raise ValueError('実行中の指令元と一致しません')
        if self.command_source == command_source.avoidance:
            raise ValueError('回避モジュール実行中の外部指令は受理できません')
        command_id = value.get('command_id', 0)
        if type(command_id) is not int or command_id < 0:
            raise ValueError('指令番号が不正です')
        with self.lock:
            if not self.close_event.is_set():
                self.pending_command = (value['command'], time.monotonic(), command_id)

    def apply_poses(self, value):
        self.scene.move(value['poses'])
        if not self.scene.robot or 'joints' not in value:
            return
        if self.command_source not in (command_source.browser, command_source.leader, command_source.avoidance):
            raise ValueError('この指令元では姿勢パケットによる目標更新はできません')
        if self.enable_leader_follow:
            stamp = value.get('leader_stamp_sec')
            if (self.is_leader_stopped or type(stamp) not in (int, float) or
                    not math.isfinite(stamp) or not 0 <= time.time()-stamp < self.max_leader_age_sec or
                    (self.leader_stamp_sec is not None and stamp < self.leader_stamp_sec)):
                return
        self.scene.set_goal(value['joints'])
        if self.enable_leader_follow:
            self.leader_stamp_sec = stamp

    def place(self, value):
        """計算worker内の配置確定。未処理の旧キネマティック姿勢と配置前フレームの識別。"""
        placement_id = value['placement_id']
        if type(placement_id) is not int or placement_id <= self.placement_id:
            raise ValueError('配置要求番号が不正です')
        if self.close_event.is_set():
            return None
        self.scene.place(value['pose'])
        with self.lock:
            if self.pending_poses is not None:
                self.pending_poses = {**self.pending_poses, 'poses': [pose for pose in self.pending_poses['poses'] if pose['id'] != value['pose']['id']]}
        self.placement_id = placement_id
        return dict(type='placed', placement_id=placement_id)

    def step(self):
        if self.close_event.is_set():
            return None
        with self.lock:
            value, self.pending_poses = self.pending_poses, None
            command, self.pending_command = self.pending_command, None
        if value is not None:
            self.apply_poses(value)
        if command is not None:
            if self.scene.robot is None:
                raise ValueError('物理ロボットがありません')
            self.scene.robot.set_command(command[0])
            self.command_received_sec = command[1]
            self.command_id = command[2]
            self.is_command_stopped = False
        if (self.enable_command_timeout_hold and self.command_source == command_source.ros
                and not self.is_command_stopped and time.monotonic()-self.command_received_sec >= self.max_leader_age_sec):
            self.scene.robot.hold_position()
            self.is_command_stopped = True
        # ブラウザ描画・入力再送に非依存の、worker内の実測失効判定
        stamp = self.leader_stamp_sec if self.leader_stamp_sec is not None else self.leader_began_sec
        if self.enable_leader_follow and not self.is_leader_stopped and time.time()-stamp >= self.max_leader_age_sec:
            self.is_leader_stopped = True
            self.scene.robot.hold_position()
        if self.close_event.is_set():
            return None
        result = self.scene.step()
        result['placement_id'] = self.placement_id
        result['control_mode'] = self.scene.robot.control_mode.value if self.scene.robot else None
        result['command_source'] = self.command_source.value
        result['is_command_stopped'] = self.is_command_stopped
        result['command_id'] = self.command_id
        if self.enable_leader_follow:
            result['is_leader_stopped'] = self.is_leader_stopped
        return None if self.close_event.is_set() else result

    def release(self):
        if self.command_subscription is not None:
            self.ros_node.destroy_subscription(self.command_subscription)
            self.command_subscription = None
        if self.scene is not None:
            avoidance = getattr(self.scene, 'avoidance', None)
            try:
                if avoidance is not None and hasattr(avoidance, 'close'):
                    avoidance.close()
            finally:
                self.scene = None

    def close(self):
        if self.close_event.is_set():
            return self.closed_future
        self.close_event.set()
        with self.lock:
            self.pending_poses = None
        # 進行中の構築・1周期だけの完了待ち。同じworkerでの所有資源解除
        self.closed_future = self.executor.submit(self.release)
        self.executor.shutdown(wait=False)
        return self.closed_future


def physics_handler(origins, clients, node=None, on_release=None):
    class Handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.set_nodelay(True)
            self.session = None
            self.timer = None
            self.is_writing = False
            self.is_ready = False
            clients.add(self)

        def report_error(self, error):
            try:
                self.write_message({'type': 'error', 'error': str(error)})
            except tornado.websocket.WebSocketClosedError:
                pass
            self.close()

        async def on_message(self, raw):
            try:
                value = json.loads(raw)
                if value['type'] == 'start' and self.session is None:
                    scene_factory = lambda bodies, robot, avoidance: PhysicsScene(bodies, robot, avoidance, ros_node=node)
                    session = self.session = physics_session(scene_factory, node)
                    has_started = await asyncio.wrap_future(session.executor.submit(session.start, value))
                    if not has_started or self.session is not session:
                        return
                    self.is_ready = True
                    self.timer = tornado.ioloop.PeriodicCallback(self.step, 10)
                    self.timer.start()
                    self.write_message({'type': 'ready', 'engine': 'MuJoCo', 'timestep_sec': .002, 'can_place_objects': True, 'can_command_robot': True,
                                        'control_mode': session.scene.robot.control_mode.value if session.scene.robot else None})
                elif value['type'] == 'poses' and self.is_ready:
                    self.session.queue_poses(value)
                elif value['type'] == 'command' and self.is_ready:
                    self.session.queue_command(value)
                elif value['type'] == 'place' and self.is_ready:
                    session = self.session
                    result = await asyncio.wrap_future(session.executor.submit(session.place, value))
                    if result is not None and self.session is session:
                        self.write_message(result)
                else:
                    raise ValueError('未対応の物理操作です')
            except (ValueError, TypeError, KeyError, ImportError, RuntimeError) as error:
                self.report_error(error)

        async def step(self):
            if not self.is_ready or self.is_writing:
                return
            self.is_writing = True
            session = self.session
            try:
                result = await asyncio.wrap_future(session.executor.submit(session.step))
                if result is not None and self.session is session:
                    await self.write_message(result)
            except tornado.websocket.WebSocketClosedError:
                pass
            except Exception as error:
                self.report_error(error)
            finally:
                self.is_writing = False

        def on_close(self):
            if self.timer:
                self.timer.stop()
            self.is_ready = False
            if self.session is not None:
                future = self.session.close()
                if on_release is not None:
                    on_release(future)
                self.session = None
            clients.discard(self)
    return Handler
