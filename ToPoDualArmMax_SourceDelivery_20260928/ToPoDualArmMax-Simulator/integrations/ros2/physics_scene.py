"""ブラウザ配置からのMuJoCo接触モデルと固定刻みシミュレーション。"""
import math
import xml.etree.ElementTree as et
from physics_robot import PhysicsRobot, text


def vector(value, length, bound=100):
    if not isinstance(value, list) or len(value) != length or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > bound for x in value):
        raise ValueError('物理モデルの数値が不正です')
    return value


def quaternion(value):
    vector(value, 4, 1.01)
    if abs(sum(x*x for x in value)-1) > .001:
        raise ValueError('姿勢の四元数が不正です')
    return [value[3], *value[:3]]


class PhysicsScene:
    def __init__(self, bodies, robot=None, avoidance=None, ros_node=None):
        import mujoco
        self.mujoco = mujoco
        if not isinstance(bodies, list) or not 0 <= len(bodies) <= 512:
            raise ValueError('物理物体数が不正です')
        root = et.Element('mujoco', model='browser_scene')
        et.SubElement(root, 'option', timestep='.002', gravity='0 0 -9.81', integrator='implicitfast')
        et.SubElement(root, 'compiler', angle='radian')
        world = et.SubElement(root, 'worldbody')
        et.SubElement(world, 'geom', type='plane', pos='0 0 -.14', size='10 10 .1', friction='.8 .005 .0001')
        self.dynamic = []
        self.kinematic = []
        ids = set()
        for value in bodies:
            name, mode = value['id'], value['mode']
            if not isinstance(name, str) or len(name) > 80 or name in ids or mode not in ('static', 'dynamic', 'kinematic', 'hinge', 'slide'):
                raise ValueError('物理物体IDまたはモードが不正です')
            ids.add(name)
            body = et.SubElement(world, 'body', name=name, pos=text(vector(value['position'], 3)), quat=text(quaternion(value['quaternion'])))
            if mode == 'dynamic':
                et.SubElement(body, 'freejoint')
                self.dynamic.append(name)
            elif mode in ('hinge', 'slide'):
                constraint = value.get('constraint', {})
                axis = vector(constraint.get('axis', [0, 0, 1]), 3, 1)
                if sum(x*x for x in axis) < .000001:
                    raise ValueError('拘束軸の長さがゼロです')
                limits = vector(constraint.get('range', [-1.570796327, 1.570796327] if mode == 'hinge' else [-.2, .2]), 2)
                if not limits[0] < limits[1] or not limits[0] <= 0 <= limits[1]:
                    raise ValueError('拘束範囲は初期位置0を含む昇順です')
                et.SubElement(body, 'joint', type=mode, axis=text(axis), pos=text(vector(constraint.get('pivot', [0, 0, 0]), 3)), limited='true', range=text(limits), damping='.1')
                self.dynamic.append(name)
            elif mode == 'kinematic':
                body.set('mocap', 'true')
                self.kinematic.append(name)
            geoms = value['geoms']
            if not isinstance(geoms, list) or not 1 <= len(geoms) <= 256:
                raise ValueError('衝突形状数が不正です')
            mass = value.get('mass', .2)
            if type(mass) not in (int, float) or not math.isfinite(mass) or not .001 <= mass <= 1000:
                raise ValueError('質量は0.001～1000 kgです')
            for geom in geoms:
                size = vector(geom['size'], 3, 10)
                if min(size) <= 0:
                    raise ValueError('衝突形状の寸法が不正です')
                et.SubElement(body, 'geom', type='box', pos=text(vector(geom['position'], 3)), quat=text(quaternion(geom['quaternion'])), size=text(size), mass=str(mass/len(geoms)), friction='.8 .005 .0001', contype='1', conaffinity='1')
        self.robot = None
        learned_models = None
        if robot is not None:
            fixed_joint_names = ()
            if avoidance and avoidance.get('mode') == 'gng':
                from gng_inputs import load_models, workspace
                learned_models = load_models(avoidance.get('params_file') or str(
                    workspace / 'gng_vlut_system/config/topo_dual_arm_max_long.yaml'))
                models = learned_models[1]
                active_names = {name for model in models.values() for name in model['joint_names']}
                # 学習の固定関節条件とMuJoCo拘束の一致。既存の関節固定機能の使用。
                fixed_joint_names = {name for model in models.values() for name in model['fixed_joints']} - active_names
            base = et.SubElement(world, 'body', name='robot_base', pos=text(vector(robot['position'], 3)),
                                 quat=text(quaternion(robot['quaternion'])))
            self.robot = PhysicsRobot(root, base, robot, mujoco, fixed_joint_names)
        self.model = mujoco.MjModel.from_xml_string(et.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        if self.robot:
            self.robot.bind(self.model, self.data)
        self.body_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ids}
        mujoco.mj_forward(self.model, self.data)
        self.avoidance = None
        if avoidance and avoidance.get('mode', 'none') != 'none':
            if not self.robot:
                raise ValueError('回避には関節動力学ロボットが必要です')
            if avoidance.get('mode') == 'oscbf':
                from oscbf_avoidance import create_filter
                self.avoidance = create_filter(self, avoidance)
            elif avoidance.get('mode') == 'gng':
                from gng_avoidance import gng_filter
                self.avoidance = gng_filter(self, avoidance, ros_node, robot, learned_models)
            else:
                raise ValueError('未登録の回避方式です')

    def move(self, poses):
        if not isinstance(poses, list) or len(poses) > 512:
            raise ValueError('姿勢指定数が不正です')
        updates = []
        for pose in poses:
            if pose['id'] not in self.kinematic:
                raise ValueError('姿勢指定対象ではありません')
            idx = self.model.body_mocapid[self.body_ids[pose['id']]]
            updates.append((idx, vector(pose['position'], 3), quaternion(pose['quaternion'])))
        for idx, position, quat in updates:
            self.data.mocap_pos[idx] = position
            self.data.mocap_quat[idx] = quat

    def step(self):
        if self.avoidance:
            self.avoidance.update()
        for _ in range(5):
            if self.robot:
                self.robot.control()
                if self.avoidance:
                    self.avoidance.apply()
            self.mujoco.mj_step(self.model, self.data)
        self.mujoco.mj_forward(self.model, self.data)
        poses = []
        for name in self.dynamic:
            idx = self.body_ids[name]
            q = self.data.xquat[idx]
            poses.append(dict(id=name, position=self.data.xpos[idx].tolist(), quaternion=[*q[1:].tolist(), float(q[0])]))
        return dict(avoidance=self.avoidance.status if self.avoidance else None, type='physics', time_sec=float(self.data.time), contacts=int(self.data.ncon), poses=poses, joints=self.robot.state() if self.robot else None, motion=self.robot.motion_state() if self.robot else None)
