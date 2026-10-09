"""標準・Longの静的障害物回避、トルク制限、障害物移動後の追従再開。"""
import sys
from pathlib import Path
import unittest
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from physics_scene import PhysicsScene


class FilterTest(unittest.TestCase):
    def test_static_obstacle(self):
        for model in ('long', 'standard'):
            with self.subTest(model=model):
                config = dict(model=model, position=[0, 0, 0], quaternion=[0, 0, 0, 1], pose={})
                probe = PhysicsScene([], config)
                joint = probe.robot.ids['L_joint2']
                probe.data.qpos[probe.model.jnt_qposadr[joint]] = .8
                probe.mujoco.mj_forward(probe.model, probe.data)
                tip = probe.mujoco.mj_name2id(probe.model, probe.mujoco.mjtObj.mjOBJ_BODY, 'robot_L_tcp')
                obstacle = dict(id='obstacle', mode='static', position=probe.data.xpos[tip].tolist(), quaternion=[0, 0, 0, 1], mass=.2,
                                geoms=[dict(position=[0, 0, 0], quaternion=[0, 0, 0, 1], size=[.025, .025, .025])])
                for mode in ('none', 'oscbf'):
                    scene = PhysicsScene([obstacle], config, dict(mode=mode))
                    scene.robot.targets['L_joint2'] = 1.2
                    geom = int(np.where(scene.model.geom_bodyid == scene.body_ids['obstacle'])[0][0])
                    num_contacts = 0
                    min_dist = float('inf')
                    start = time.perf_counter()
                    for _ in range(600):
                        frame = scene.step()
                        num_contacts += sum(geom in contact.geom for contact in scene.data.contact)
                        for name, idx in scene.robot.actuator_ids.items():
                            self.assertLessEqual(abs(scene.data.actuator_force[idx]), scene.robot.max_effort[name]+1e-8)
                        if scene.avoidance and frame['avoidance']['min_dist'] is not None:
                            min_dist = min(min_dist, frame['avoidance']['min_dist'])
                    elapsed = (time.perf_counter()-start)*1000/600
                    print(dict(model=model, mode=mode, contacts=num_contacts, min_proxy_dist=min_dist if mode=='oscbf' else None, update_ms=elapsed), flush=True)
                    if mode == 'none':
                        self.assertGreater(num_contacts, 0)
                    else:
                        self.assertEqual(num_contacts, 0)
                        self.assertGreater(min_dist, .05)
                        before = scene.robot.state()['L_joint2']
                        self.assertGreater(before, .1)
                        scene.model.body_pos[scene.body_ids['obstacle']] += [3, 0, 0]
                        scene.mujoco.mj_forward(scene.model, scene.data)
                        for _ in range(300):
                            scene.step()
                        self.assertGreater(scene.robot.state()['L_joint2'], before+.2)


if __name__ == '__main__':
    unittest.main()
