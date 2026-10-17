"""ブラウザ配置からのMuJoCo接触モデルと固定刻みシミュレーション。"""
import math
import copy
from functools import lru_cache
import xml.etree.ElementTree as et
import hashlib
from pathlib import Path
import tempfile
from physics_robot import PhysicsRobot, text
from ros_plugins import physics_adapter


def vector(value, length, bound=100):
    if not isinstance(value, list) or len(value) != length or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x) > bound for x in value):
        raise ValueError('物理モデルの数値が不正です')
    return value


def quaternion(value):
    vector(value, 4, 1.01)
    if abs(sum(x*x for x in value)-1) > .001:
        raise ValueError('姿勢の四元数が不正です')
    return [value[3], *value[:3]]


def compiled_model(mujoco, root, cache_dir):
    """同一XML・衝突メッシュ・MuJoCo版のコンパイル済み物理モデル。"""
    xml = et.tostring(root,encoding='unicode')
    if cache_dir is None:
        return mujoco.MjModel.from_xml_string(xml)
    assets = tuple((filename,Path(filename).stat().st_mtime_ns,Path(filename).stat().st_size)
                   for filename in sorted({node.get('file') for node in root.findall('./asset/mesh')}))
    # MjDataだけでなく可変のMjModelも実験間で分離。未変更メッシュの再読込なし。
    return copy.copy(load_compiled_model(mujoco,xml,assets,str(cache_dir)))


@lru_cache(maxsize=2)
def load_compiled_model(mujoco, xml, assets, cache_dir):
    """直近2形状の静的物理モデル。メッシュ更新情報によるキャッシュ失効。"""
    digest = hashlib.sha256((mujoco.__version__+xml).encode())
    for filename,_,_ in assets:
        digest.update(Path(filename).read_bytes())
    directory = Path(cache_dir)
    filename = directory/(digest.hexdigest()+'.mjb')
    if filename.is_file():
        try:
            return mujoco.MjModel.from_binary_path(str(filename))
        except (ValueError,OSError):
            pass
    model = mujoco.MjModel.from_xml_string(xml)
    temporary = None
    try:
        directory.mkdir(parents=True,exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=directory,prefix='model-',suffix='.tmp',delete=False) as stream:
            temporary = Path(stream.name)
        mujoco.mj_saveModel(model,str(temporary),None)
        temporary.chmod(0o644)
        temporary.replace(filename)
    except OSError:
        pass
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return model


class PhysicsScene:
    def __init__(self, bodies, robot=None, avoidance=None, ros_node=None, *, model_cache_dir=None):
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
        self.body_modes = {}
        self.anchors = {}
        ids = set()
        for value in bodies:
            name, mode = value['id'], value['mode']
            if not isinstance(name, str) or len(name) > 80 or name in ids or mode not in ('static', 'dynamic', 'kinematic', 'hinge', 'slide'):
                raise ValueError('物理物体IDまたはモードが不正です')
            ids.add(name)
            self.body_modes[name] = mode
            placement = dict(pos=text(vector(value['position'], 3)), quat=text(quaternion(value['quaternion'])))
            parent = world
            if mode in ('hinge', 'slide'):
                # 拘束基準の可動アンカー。通常計算中は世界固定、直接配置時だけ更新
                self.anchors[name] = 'anchor_'+name
                parent = et.SubElement(world, 'body', name=self.anchors[name], mocap='true', **placement)
                placement = {}
            body = et.SubElement(parent, 'body', name=name, **placement)
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
            elif mode in ('static', 'kinematic'):
                body.set('mocap', 'true')
                if mode == 'kinematic':
                    self.kinematic.append(name)
            geoms = value['geoms']
            if not isinstance(geoms, list) or not 1 <= len(geoms) <= 256:
                raise ValueError('衝突形状数が不正です')
            mass = value.get('mass', .2)
            if type(mass) not in (int, float) or not math.isfinite(mass) or not .001 <= mass <= 1000:
                raise ValueError('質量は0.001～1000 kgです')
            for geom in geoms:
                kind = geom.get('type', 'box')
                if kind in ('sphere', 'capsule'):
                    radius = geom.get('radius')
                    if type(radius) not in (int, float) or not math.isfinite(radius) or not 0 < radius <= 10:
                        raise ValueError('球の半径が不正です')
                    size = [radius]
                    if kind == 'capsule':
                        half_length = geom.get('half_length')
                        if type(half_length) not in (int, float) or not math.isfinite(half_length) or not 0 < half_length <= 10:
                            raise ValueError('カプセルの半長の不正')
                        size.append(half_length)
                elif kind == 'box':
                    size = vector(geom['size'], 3, 10)
                    if min(size) <= 0:
                        raise ValueError('衝突形状の寸法が不正です')
                else:
                    raise ValueError('未対応の衝突形状: '+str(kind))
                et.SubElement(body, 'geom', type=kind, pos=text(vector(geom['position'], 3)), quat=text(quaternion(geom['quaternion'])), size=text(size), mass=str(mass/len(geoms)), friction='.8 .005 .0001', contype='1', conaffinity='1')
        self.robot = None
        adapter, avoidance, prepared = physics_adapter(avoidance, robot)
        if adapter is not None and robot.get('control_mode', 'position') != 'position':
            raise ValueError('現在の回避モジュールは位置制御専用です')
        if robot is not None:
            fixed_joint_names = prepared['fixed_joint_names'] if prepared else ()
            base = et.SubElement(world, 'body', name='robot_base', pos=text(vector(robot['position'], 3)),
                                 quat=text(quaternion(robot['quaternion'])))
            self.robot = PhysicsRobot(root, base, robot, mujoco, fixed_joint_names)
        self.model = compiled_model(mujoco,root,model_cache_dir)
        self.data = mujoco.MjData(self.model)
        if self.robot:
            self.robot.bind(self.model, self.data)
        self.body_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name) for name in ids}
        self.anchor_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, anchor) for name, anchor in self.anchors.items()}
        mujoco.mj_forward(self.model, self.data)
        self.avoidance = None
        if adapter is not None:
            self.avoidance = adapter.create(self, avoidance, ros_node, robot, prepared)

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
        return self.state()

    def set_goal(self, joints):
        """入力元共通の名目目標。回避ありの場合は回避側だけへの受渡し。"""
        if self.robot is None:
            raise ValueError('物理ロボットがありません')
        if self.avoidance is not None:
            if not callable(getattr(self.avoidance, 'set_targets', None)):
                raise ValueError('この回避モジュールは名目目標の更新に未対応です')
            self.avoidance.set_targets(joints)
        else:
            self.robot.set_targets(joints)

    def link_transform(self, name):
        """実測リンクのworld変換。回避処理からMuJoCo内部配列を隠す参照窓口。"""
        import numpy as np
        idx = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_BODY, 'robot_'+name)
        if idx < 0:
            raise ValueError('物理ロボットのリンクがありません: '+name)
        result = np.eye(4)
        result[:3, :3] = self.data.xmat[idx].reshape(3, 3)
        result[:3, 3] = self.data.xpos[idx]
        return result

    def place(self, pose):
        """対象物体だけの直接配置。速度・拘束変位の初期化、他物体・ロボット・時刻の保持。"""
        name = pose['id']
        if name not in self.body_modes or not name.startswith('object_'):
            raise ValueError('直接配置対象の物体がありません')
        position, quat = vector(pose['position'], 3), quaternion(pose['quaternion'])
        idx = self.body_ids[name]
        mode = self.body_modes[name]
        if mode == 'dynamic':
            joint = self.model.body_jntadr[idx]
            qpos = self.model.jnt_qposadr[joint]
            self.data.qpos[qpos:qpos+7] = [*position, *quat]
        else:
            anchor = self.anchor_ids.get(name, idx)
            mocap = self.model.body_mocapid[anchor]
            self.data.mocap_pos[mocap] = position
            self.data.mocap_quat[mocap] = quat
            if mode in ('hinge', 'slide'):
                joint = self.model.body_jntadr[idx]
                self.data.qpos[self.model.jnt_qposadr[joint]] = 0
        if mode in ('dynamic', 'hinge', 'slide'):
            dof = self.model.jnt_dofadr[self.model.body_jntadr[idx]]
            num_dofs = 6 if mode == 'dynamic' else 1
            self.data.qvel[dof:dof+num_dofs] = 0
            self.data.qacc_warmstart[dof:dof+num_dofs] = 0
        self.mujoco.mj_forward(self.model, self.data)

    def state(self):
        """同一計算状態の描画・モータ標本。"""
        poses = []
        for name in self.dynamic:
            idx = self.body_ids[name]
            q = self.data.xquat[idx]
            poses.append(dict(id=name, position=self.data.xpos[idx].tolist(), quaternion=[*q[1:].tolist(), float(q[0])]))
        return dict(avoidance=self.avoidance.status if self.avoidance else None, type='physics', time_sec=float(self.data.time), contacts=int(self.data.ncon), poses=poses, joints=self.robot.state() if self.robot else None, motion=self.robot.motion_state() if self.robot else None)
