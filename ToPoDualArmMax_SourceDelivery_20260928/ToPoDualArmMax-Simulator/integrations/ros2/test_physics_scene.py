"""落下・接触・姿勢指定・不正入力の検証。"""
import unittest
from physics_scene import PhysicsScene


def body(name, mode, z):
    return dict(id=name, mode=mode, position=[0, 0, z], quaternion=[0, 0, 0, 1], mass=.2,
                geoms=[dict(position=[0, 0, 0], quaternion=[0, 0, 0, 1], size=[.1, .1, .1])])


class PhysicsTest(unittest.TestCase):
    def test_fall_and_contact(self):
        scene = PhysicsScene([body('table', 'static', .2), body('box', 'dynamic', .8)])
        initial = scene.step()['poses'][0]['position'][2]
        for _ in range(250):
            frame = scene.step()
        self.assertLess(frame['poses'][0]['position'][2], initial-.2)
        self.assertAlmostEqual(frame['poses'][0]['position'][2], .4, delta=.002)
        self.assertGreater(frame['contacts'], 0)
        self.assertAlmostEqual(frame['time_sec'], 2.51, places=6)

    def test_kinematic(self):
        scene = PhysicsScene([body('robot', 'kinematic', .5)])
        scene.move([dict(id='robot', position=[1, 0, .5], quaternion=[0, 0, 0, 1])])
        scene.step()
        self.assertAlmostEqual(scene.data.xpos[scene.body_ids['robot']][0], 1)

    def test_invalid_mass(self):
        value = body('box', 'dynamic', .5)
        value['mass'] = -1
        with self.assertRaises(ValueError):
            PhysicsScene([value])


if __name__ == '__main__':
    unittest.main()
