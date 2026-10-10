"""同梱URDFに基づく固定基台の関節モデルとトルク制限付きPD駆動。"""
from pathlib import Path
from collections.abc import Mapping
import math
import xml.etree.ElementTree as et
from types import MappingProxyType


def numbers(value):
    return [float(x) for x in value.split()]


def text(values):
    return ' '.join(map(str, values))


def rotation(rpy):
    import numpy as np
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def origin(element, mujoco):
    import numpy as np
    node = element.find('origin')
    if node is None:
        return dict(pos='0 0 0', quat='1 0 0 0')
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation(numbers(node.get('rpy', '0 0 0'))).reshape(9))
    return dict(pos=node.get('xyz', '0 0 0'), quat=' '.join(map(str, quat)))


class PhysicsRobot:
    def __init__(self, root, base, config, mujoco):
        import numpy as np
        self.mujoco = mujoco
        model_name = config.get('model')
        if model_name not in ('standard', 'long'):
            raise ValueError('物理ロボットはstandard / longのみです')
        app = Path(__file__).resolve().parents[2] / 'app'
        path = app / ('models/standard/source.urdf' if model_name == 'standard' else 'source.urdf')
        urdf = et.parse(path).getroot()
        links = {x.get('name'): x for x in urdf.findall('link')}
        joints = urdf.findall('joint')
        self.joints = {x.get('name'): x for x in joints if x.get('type') != 'fixed'}
        self.independent = {name: joint for name, joint in self.joints.items() if joint.find('mimic') is None}
        locks = config.get('locked_joints', [])
        if not isinstance(locks, list) or any(x not in self.independent for x in locks):
            raise ValueError('固定対象の関節名が不正です')
        self.locks = tuple(locks)
        self.initial = MappingProxyType(self.validate(config.get('pose', {})))
        self._targets = dict(self.initial)
        self._commands = dict(self.initial)
        asset = et.SubElement(root, 'asset')
        equality = et.SubElement(root, 'equality')
        actuator = et.SubElement(root, 'actuator')
        self.max_effort = {}
        children = {}
        child_names = set()
        for joint in joints:
            children.setdefault(joint.find('parent').get('link'), []).append(joint)
            child_names.add(joint.find('child').get('link'))
        def add_link(parent, name, joint=None):
            body = et.SubElement(parent, 'body', name='robot_' + name, **(origin(joint, mujoco) if joint is not None else {}))
            if joint is not None and joint.get('type') != 'fixed':
                kind = joint.get('type')
                if kind not in ('revolute', 'continuous', 'prismatic'):
                    raise ValueError('未対応のURDF関節: ' + kind)
                attrs = dict(name=joint.get('name'), type='slide' if kind == 'prismatic' else 'hinge', axis=joint.find('axis').get('xyz'), damping='.1', armature='.001')
                limit = joint.find('limit')
                if kind != 'continuous':
                    attrs.update(limited='true', range=f"{limit.get('lower')} {limit.get('upper')}")
                et.SubElement(body, 'joint', **attrs)
            link = links[name]
            inertial = link.find('inertial')
            if inertial is not None:
                inertia = inertial.find('inertia')
                matrix = np.array([[float(inertia.get('i'+a+b, inertia.get('i'+b+a, '0'))) for b in 'xyz'] for a in 'xyz'])
                pose = origin(inertial, mujoco)
                # fullinertiaはボディ座標系。URDF慣性座標からの回転
                node = inertial.find('origin')
                rot = rotation(numbers(node.get('rpy', '0 0 0'))) if node is not None else np.eye(3)
                matrix = rot @ matrix @ rot.T
                et.SubElement(body, 'inertial', pos=pose['pos'], mass=inertial.find('mass').get('value'), fullinertia=text([matrix[0,0], matrix[1,1], matrix[2,2], matrix[0,1], matrix[0,2], matrix[1,2]]))
            for idx, collision in enumerate(link.findall('collision')):
                geometry = collision.find('geometry')
                mesh = geometry.find('mesh')
                if mesh is not None:
                    mesh_name = f'{name}_{idx}'
                    et.SubElement(asset, 'mesh', name=mesh_name, file=str(path.parent / mesh.get('filename')), scale=mesh.get('scale', '1 1 1'))
                    shape = dict(type='mesh', mesh=mesh_name)
                elif geometry.find('box') is not None:
                    shape = dict(type='box', size=text([x/2 for x in numbers(geometry.find('box').get('size'))]))
                elif geometry.find('sphere') is not None:
                    shape = dict(type='sphere', size=geometry.find('sphere').get('radius'))
                elif geometry.find('cylinder') is not None:
                    cylinder = geometry.find('cylinder')
                    shape = dict(type='cylinder', size=text([float(cylinder.get('radius')), float(cylinder.get('length'))/2]))
                else:
                    raise ValueError('同梱URDFの衝突形状が未対応です: '+name)
                et.SubElement(body, 'geom', **shape, **origin(collision, mujoco), contype='2', conaffinity='3' if config.get('enable_self_collision', False) else '1', friction='.8 .005 .0001')
            for child in children.get(name, []):
                add_link(body, child.find('child').get('link'), child)
        for name in links.keys() - child_names:
            add_link(base, name)
        for name, joint in self.joints.items():
            mimic = joint.find('mimic')
            if mimic is not None:
                et.SubElement(equality, 'joint', joint1=name, joint2=mimic.get('joint'), polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0")
            elif name in self.locks:
                et.SubElement(equality, 'joint', joint1=name, polycoef=f'{self.initial[name]} 0 0 0 0')
            else:
                effort = float(joint.find('limit').get('effort'))
                self.max_effort[name] = effort
                et.SubElement(actuator, 'general', name=name, joint=name, gainprm='40', biastype='affine', biasprm='0 -40 -3', forcelimited='true', forcerange=f'{-effort} {effort}')

    @property
    def targets(self) -> Mapping[str, float]:
        return MappingProxyType(self._targets)

    @property
    def commands(self) -> Mapping[str, float]:
        return MappingProxyType(self._commands)

    def validate(self, pose: Mapping[str, float], *, base_pose: Mapping[str, float] | None = None) -> dict[str, float]:
        if not isinstance(pose, Mapping) or any(name not in self.independent for name in pose):
            raise ValueError('関節目標の形式または関節名が不正です')
        result = {}
        for name, joint in self.independent.items():
            value = pose.get(name, base_pose[name] if base_pose is not None else 0)
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 10000:
                raise ValueError('関節角度が不正です')
            if joint.get('type') != 'continuous':
                limit = joint.find('limit')
                value = min(float(limit.get('upper')), max(float(limit.get('lower')), value))
            result[name] = value
        return result

    def set_targets(self, pose: Mapping[str, float], *, command_pose: Mapping[str, float] | None = None):
        """部分目標の反映と、検査済み経路の補間開始姿勢の一括更新。"""
        targets = self.validate(pose, base_pose=self._targets)
        commands = self._commands if command_pose is None else self.validate(command_pose, base_pose=self._commands)
        self._targets, self._commands = targets, commands

    def hold_position(self):
        """現在の実測関節姿勢による、目標と補間状態の同期。"""
        pose = {name: value for name, value in self.state().items() if name in self.independent}
        self.set_targets(pose, command_pose=pose)

    def bind(self, model, data):
        self.model, self.data = model, data
        self.ids = {name: self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joints}
        for name, joint in self.joints.items():
            mimic = joint.find('mimic')
            value = self.initial[name] if mimic is None else self.initial[mimic.get('joint')]*float(mimic.get('multiplier', '1'))+float(mimic.get('offset', '0'))
            data.qpos[model.jnt_qposadr[self.ids[name]]] = value
        self.actuator_ids = {name: self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.max_effort}

        # 不変の関節対応・速度上限の事前解決。制御周期内のXML検索を省略
        self.control_entries = [(name, idx, int(model.jnt_dofadr[self.ids[name]]),
                                 float(self.joints[name].find('limit').get('velocity')))
                                for name, idx in self.actuator_ids.items()]
        self.state_entries = [(name, int(model.jnt_qposadr[idx])) for name, idx in self.ids.items()]

    def control(self):
        timestep = self.model.opt.timestep
        for name, idx, dof, max_velocity in self.control_entries:
            max_step = max_velocity * timestep
            delta = self._targets[name]-self._commands[name]
            self._commands[name] += min(max(delta, -max_step), max_step)
            # トルク制限内のPDと重力・コリオリ補償。目標速度の制限と実速度は別
            self.data.ctrl[idx] = self._commands[name]+self.data.qfrc_bias[dof]/40

    def state(self):
        return {name: float(self.data.qpos[idx]) for name, idx in self.state_entries}

    def motion_state(self):
        """関節速度と関節自由度へ作用するアクチュエータ駆動力。接触力を除外。"""
        return {
            'velocity': {name: float(self.data.qvel[self.model.jnt_dofadr[idx]]) for name, idx in self.ids.items()},
            'effort': {name: float(self.data.qfrc_actuator[self.model.jnt_dofadr[idx]]) for name, idx in self.ids.items()},
        }
