"""ブラウザごとの物理セッション。切断時に計算停止。"""
import json
import math
import time
import tornado.ioloop
import tornado.websocket
from physics_scene import PhysicsScene


def physics_handler(origins, clients):
    class Handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.set_nodelay(True)
            self.scene = None
            self.timer = None
            self.is_writing = False
            self.enable_leader_follow = False
            self.is_leader_stopped = False
            self.leader_stamp_sec = None
            clients.add(self)

        def on_message(self, raw):
            try:
                value = json.loads(raw)
                if value['type'] == 'start' and self.scene is None:
                    robot = value.get('robot') or {}
                    self.enable_leader_follow = robot.get('enable_leader_follow') is True
                    self.max_leader_age_sec = robot.get('max_leader_age_sec', 1.)
                    if (type(self.max_leader_age_sec) not in (int, float) or
                            not math.isfinite(self.max_leader_age_sec) or not 0 < self.max_leader_age_sec <= 5):
                        raise ValueError('リーダー実測の鮮度設定不正')
                    self.scene = PhysicsScene(value['bodies'], value.get('robot'), value.get('avoidance'))
                    self.leader_began_sec = time.time()
                    self.timer = tornado.ioloop.PeriodicCallback(self.step, 10)
                    self.timer.start()
                    self.write_message({'type': 'ready', 'engine': 'MuJoCo', 'timestep_sec': .002})
                elif value['type'] == 'poses' and self.scene is not None:
                    self.scene.move(value['poses'])
                    if self.scene.robot and 'joints' in value:
                        if self.enable_leader_follow:
                            stamp = value.get('leader_stamp_sec')
                            if (self.is_leader_stopped or type(stamp) not in (int, float) or
                                    not math.isfinite(stamp) or not 0 <= time.time()-stamp < self.max_leader_age_sec or
                                    (self.leader_stamp_sec is not None and stamp < self.leader_stamp_sec)):
                                return
                            self.leader_stamp_sec = stamp
                        self.scene.robot.targets = self.scene.robot.validate(value['joints'])
                else:
                    raise ValueError('未対応の物理操作です')
            except (ValueError, TypeError, KeyError, ImportError, RuntimeError) as error:
                self.write_message({'type': 'error', 'error': str(error)})
                self.close()

        async def step(self):
            if self.scene is None or self.is_writing:
                return
            self.is_writing = True
            try:
                # ブラウザの描画停止・同一時刻の再送に非依存の物理側失効判定
                stamp = self.leader_stamp_sec if self.leader_stamp_sec is not None else self.leader_began_sec
                if self.enable_leader_follow and not self.is_leader_stopped and time.time()-stamp >= self.max_leader_age_sec:
                    self.is_leader_stopped = True
                    actual = self.scene.robot.state()
                    self.scene.robot.targets = {name: actual[name] for name in self.scene.robot.independent}
                    self.scene.robot.commands = dict(self.scene.robot.targets)
                result = self.scene.step()
                if self.enable_leader_follow:
                    result['is_leader_stopped'] = self.is_leader_stopped
                await self.write_message(result)
            except tornado.websocket.WebSocketClosedError:
                pass
            except Exception as error:
                self.write_message({'type': 'error', 'error': str(error)})
                self.close()
            finally:
                self.is_writing = False

        def on_close(self):
            if self.timer:
                self.timer.stop()
            self.scene = None
            clients.discard(self)
    return Handler
