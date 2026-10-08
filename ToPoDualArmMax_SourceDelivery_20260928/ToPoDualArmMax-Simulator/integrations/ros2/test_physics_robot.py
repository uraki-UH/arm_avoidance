"""標準・Longの関節追従、拘束、トルク制限、URDF運動学の検証。"""
import unittest
import numpy as np
from physics_scene import PhysicsScene
from physics_robot import rotation, numbers
from test_physics_scene import body


def robot_config(model='long', **kwargs):
    return dict(model=model, position=[0, 0, 0], quaternion=[0, 0, 0, 1], pose={}, **kwargs)


class RobotTest(unittest.TestCase):
    def test_models_follow_and_limit_effort(self):
        for name in ('standard', 'long'):
            with self.subTest(model=name):
                scene = PhysicsScene([], robot_config(name))
                self.assertEqual(scene.model.nq, 21)
                self.assertEqual(scene.model.nu, 19)
                scene.robot.targets.update(neck_pan_joint=.4, L_joint2=.3, L_gripper_joint=.4)
                for _ in range(300):
                    frame = scene.step()
                    for joint, idx in scene.robot.actuator_ids.items():
                        self.assertLessEqual(abs(scene.data.actuator_force[idx]), scene.robot.max_effort[joint]+1e-9)
                self.assertAlmostEqual(frame['joints']['neck_pan_joint'], .4, delta=.005)
                self.assertAlmostEqual(frame['joints']['L_joint2'], .3, delta=.01)
                self.assertAlmostEqual(frame['joints']['L_gripper_mimic'], -frame['joints']['L_gripper_joint'], delta=.002)
                np.testing.assert_allclose(scene.data.xpos[scene.model.body('robot_base').id], [0, 0, 0], atol=1e-12)

    def test_robot_obstacle_contact_force(self):
        for name in ('standard', 'long'):
            config = robot_config(name)
            reference = PhysicsScene([], config)
            idx = reference.model.body('robot_L_link7').id
            geom = int(np.flatnonzero(reference.model.geom_bodyid == idx)[0])
            obstacle = body('obstacle', 'static', 0)
            obstacle['position'] = reference.data.geom_xpos[geom].tolist()
            obstacle['geoms'][0]['size'] = [.06, .06, .06]
            scene = PhysicsScene([obstacle], config)
            has_force = False
            for contact_idx, contact in enumerate(scene.data.contact):
                names = [scene.model.body(int(scene.model.geom_bodyid[g])).name for g in (contact.geom1, contact.geom2)]
                if 'obstacle' not in names or not any(x.startswith('robot_') for x in names):
                    continue
                force = np.zeros(6)
                scene.mujoco.mj_contactForce(scene.model, scene.data, contact_idx, force)
                has_force |= force[0] > 0
            self.assertTrue(has_force, name+'のロボットと障害物の接触反力')

    def test_joint_lock_and_urdf_limits(self):
        config = robot_config(locked_joints=['L_joint2'])
        config['pose'] = {'L_joint2': .2}
        scene = PhysicsScene([], config)
        scene.robot.targets['L_joint2'] = 1
        for _ in range(100):
            frame = scene.step()
        self.assertAlmostEqual(frame['joints']['L_joint2'], .2, delta=.001)
        self.assertLess(scene.robot.validate({'L_joint2': 100})['L_joint2'], 2.01)
        with self.assertRaises(ValueError):
            scene.robot.validate({'no_joint': 0})

    def test_urdf_forward_kinematics(self):
        import xml.etree.ElementTree as et
        from pathlib import Path
        app = Path(__file__).resolve().parents[2] / 'app'
        for name in ('standard', 'long'):
            config = robot_config(name)
            config['pose'] = {'L_joint2': .3, 'L_joint4': -.4, 'neck_pan_joint': .2}
            scene = PhysicsScene([], config)
            path = app / ('source.urdf' if name == 'long' else 'models/standard/source.urdf')
            tree = et.parse(path).getroot()
            transforms = {'base_footprint': np.eye(4)}
            pending = list(tree.findall('joint'))
            while pending:
                for joint in pending[:]:
                    parent = joint.find('parent').get('link')
                    if parent not in transforms:
                        continue
                    local = np.eye(4)
                    orig = joint.find('origin')
                    if orig is not None:
                        local[:3, :3] = rotation(numbers(orig.get('rpy', '0 0 0')))
                        local[:3, 3] = numbers(orig.get('xyz', '0 0 0'))
                    if joint.get('type') != 'fixed':
                        axis = np.array(numbers(joint.find('axis').get('xyz')))
                        axis /= np.linalg.norm(axis)
                        angle = scene.robot.state()[joint.get('name')]
                        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                        local[:3, :3] = local[:3, :3] @ (np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*cross@cross)
                    child = joint.find('child').get('link')
                    transforms[child] = transforms[parent] @ local
                    idx = scene.model.body('robot_'+child).id
                    np.testing.assert_allclose(scene.data.xpos[idx], transforms[child][:3, 3], atol=1e-9)
                    np.testing.assert_allclose(scene.data.xmat[idx].reshape(3,3), transforms[child][:3,:3], atol=1e-9)
                    pending.remove(joint)

    def test_object_hinge_slide_and_fixed(self):
        fixed = body('fixed', 'static', 1)
        hinge = body('hinge', 'hinge', 2)
        hinge['constraint'] = dict(axis=[0, 1, 0], pivot=[-.1, 0, 0], range=[-.2, .2])
        slide = body('slide', 'slide', 3)
        slide['constraint'] = dict(axis=[0, 0, 1], pivot=[0, 0, 0], range=[-.2, .2])
        scene = PhysicsScene([fixed, hinge, slide])
        for _ in range(200):
            scene.step()
        np.testing.assert_allclose(scene.data.xpos[scene.body_ids['fixed']], [0, 0, 1])
        self.assertLess(abs(scene.data.qpos[0]), .205)
        self.assertAlmostEqual(scene.data.xpos[scene.body_ids['slide']][2], 2.8, delta=.002)
        pivot = scene.data.xpos[scene.body_ids['hinge']]+scene.data.xmat[scene.body_ids['hinge']].reshape(3,3) @ np.array([-.1,0,0])
        np.testing.assert_allclose(pivot, [-.1,0,2], atol=1e-9)


if __name__ == '__main__':
    unittest.main()
