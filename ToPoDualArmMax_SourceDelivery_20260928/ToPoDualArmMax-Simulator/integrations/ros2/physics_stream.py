"""ブラウザごとの物理セッション。切断時に計算停止。"""
import json
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
            clients.add(self)

        def on_message(self, raw):
            try:
                value = json.loads(raw)
                if value['type'] == 'start' and self.scene is None:
                    self.scene = PhysicsScene(value['bodies'], value.get('robot'))
                    self.timer = tornado.ioloop.PeriodicCallback(self.step, 10)
                    self.timer.start()
                    self.write_message({'type': 'ready', 'engine': 'MuJoCo', 'timestep_sec': .002})
                elif value['type'] == 'poses' and self.scene is not None:
                    self.scene.move(value['poses'])
                    if self.scene.robot and 'joints' in value:
                        self.scene.robot.targets = self.scene.robot.validate(value['joints'])
                else:
                    raise ValueError('未対応の物理操作です')
            except (ValueError, TypeError, KeyError, ImportError) as error:
                self.write_message({'type': 'error', 'error': str(error)})
                self.close()

        async def step(self):
            if self.scene is None or self.is_writing:
                return
            self.is_writing = True
            try:
                result = self.scene.step()
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
