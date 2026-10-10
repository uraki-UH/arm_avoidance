"""ブラウザごとの物理セッション。切断時に計算停止。"""
import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import math
import threading
import time
import tornado.ioloop
import tornado.websocket
from physics_scene import PhysicsScene


class physics_session:
    """単一workerによるMuJoCo所有。入力は最新1件、未処理キューの蓄積なし。"""
    def __init__(self, scene_factory):
        self.scene_factory = scene_factory
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='topo-physics')
        self.lock = threading.Lock()
        self.close_event = threading.Event()
        self.scene = None
        self.pending_poses = None
        self.closed_future = None

    def start(self, value):
        robot = value.get('robot') or {}
        self.enable_leader_follow = robot.get('enable_leader_follow') is True
        self.max_leader_age_sec = robot.get('max_leader_age_sec', 1.)
        if (type(self.max_leader_age_sec) not in (int, float) or
                not math.isfinite(self.max_leader_age_sec) or not 0 < self.max_leader_age_sec <= 5):
            raise ValueError('リーダー実測の鮮度設定不正')
        if self.close_event.is_set():
            return False
        self.scene = self.scene_factory(value['bodies'], value.get('robot'), value.get('avoidance'))
        self.leader_began_sec = time.time()
        self.leader_stamp_sec = None
        self.is_leader_stopped = False
        return not self.close_event.is_set()

    def queue_poses(self, value):
        with self.lock:
            if not self.close_event.is_set():
                self.pending_poses = value

    def apply_poses(self, value):
        self.scene.move(value['poses'])
        if not self.scene.robot or 'joints' not in value:
            return
        if self.enable_leader_follow:
            stamp = value.get('leader_stamp_sec')
            if (self.is_leader_stopped or type(stamp) not in (int, float) or
                    not math.isfinite(stamp) or not 0 <= time.time()-stamp < self.max_leader_age_sec or
                    (self.leader_stamp_sec is not None and stamp < self.leader_stamp_sec)):
                return
            self.leader_stamp_sec = stamp
        self.scene.robot.targets = self.scene.robot.validate(value['joints'])

    def step(self):
        if self.close_event.is_set():
            return None
        with self.lock:
            value, self.pending_poses = self.pending_poses, None
        if value is not None:
            self.apply_poses(value)
        # ブラウザ描画・入力再送に非依存の、worker内の実測失効判定
        stamp = self.leader_stamp_sec if self.leader_stamp_sec is not None else self.leader_began_sec
        if self.enable_leader_follow and not self.is_leader_stopped and time.time()-stamp >= self.max_leader_age_sec:
            self.is_leader_stopped = True
            actual = self.scene.robot.state()
            self.scene.robot.targets = {name: actual[name] for name in self.scene.robot.independent}
            self.scene.robot.commands = dict(self.scene.robot.targets)
        if self.close_event.is_set():
            return None
        result = self.scene.step()
        if self.enable_leader_follow:
            result['is_leader_stopped'] = self.is_leader_stopped
        return None if self.close_event.is_set() else result

    def release(self):
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


def physics_handler(origins, clients):
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
                    session = self.session = physics_session(PhysicsScene)
                    has_started = await asyncio.wrap_future(session.executor.submit(session.start, value))
                    if not has_started or self.session is not session:
                        return
                    self.is_ready = True
                    self.timer = tornado.ioloop.PeriodicCallback(self.step, 10)
                    self.timer.start()
                    self.write_message({'type': 'ready', 'engine': 'MuJoCo', 'timestep_sec': .002})
                elif value['type'] == 'poses' and self.is_ready:
                    self.session.queue_poses(value)
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
                self.session.close()
                self.session = None
            clients.discard(self)
    return Handler
