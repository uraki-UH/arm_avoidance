"""URDFの質量・重心と関節姿勢による静的支持トルク。"""
import math
import xml.etree.ElementTree as et

import numpy as np


def vector(value):
    result = np.array(list(map(float, value.split())))
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('URDFの3次元ベクトル不正')
    return result


def rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1. - math.cos(angle)) * skew @ skew


def origin(item):
    result = np.eye(4)
    if item is not None:
        result[:3, 3] = vector(item.get('xyz', '0 0 0'))
        roll, pitch, yaw = vector(item.get('rpy', '0 0 0'))
        result[:3, :3] = (rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch)
                          @ rotation((1, 0, 0), roll))
    return result


class gravity_model:
    def __init__(self, urdf_path, names, gravity=(0., 0., -9.80665)):
        self.names = tuple(names)
        self.gravity = vector(' '.join(map(str, gravity)))
        root = et.parse(urdf_path).getroot()
        self.links, self.children, self.joints = {}, {}, {}
        for link in root.findall('link'):
            name, inertial = link.get('name'), link.find('inertial')
            if not name or name in self.links:
                raise ValueError('URDFリンク名の欠損・重複')
            mass = 0. if inertial is None else float(inertial.find('mass').get('value'))
            com = np.zeros(3) if inertial is None else origin(inertial.find('origin'))[:3, 3]
            if not math.isfinite(mass) or mass < 0:
                raise ValueError('URDF質量の不正: ' + name)
            self.links[name] = mass, com
        child_links = set()
        for joint in root.findall('joint'):
            name, kind = joint.get('name'), joint.get('type')
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            axis = joint.find('axis')
            axis = vector('1 0 0' if axis is None else axis.get('xyz'))
            if (not name or name in self.joints or parent not in self.links or child not in self.links
                    or child in child_links or kind not in ('fixed', 'revolute', 'continuous', 'prismatic')
                    or np.linalg.norm(axis) == 0):
                raise ValueError('URDF関節・接続の不正: ' + str(name))
            self.joints[name] = kind, child, origin(joint.find('origin')), axis / np.linalg.norm(axis), joint.find('mimic')
            self.children.setdefault(parent, []).append(name)
            child_links.add(child)
        roots = set(self.links) - child_links
        if len(roots) != 1 or len(set(names)) != len(names) or not set(names) <= self.joints.keys():
            raise ValueError('URDFルートまたは対象関節の不正')
        self.root = roots.pop()
        for name in names:
            if self.joints[name][0] not in ('revolute', 'continuous'):
                raise ValueError('支持トルク対象は回転関節のみ: ' + name)
        # トポロジーの循環・未接続リンクの事前確認
        self.evaluate({})

    def evaluate(self, positions):
        """基台座標の重力による支持トルク[N m]と位置エネルギー[J]。未指定角度は0。"""
        if any(name not in self.joints or not math.isfinite(q) for name, q in positions.items()):
            raise ValueError('重力計算の関節名・角度不正')
        torques = dict.fromkeys(self.names, 0.)
        seen, potential = set(), 0.
        def angle(name, chain=()):
            if name in chain:
                raise ValueError('URDF mimicの循環')
            mimic = self.joints[name][4]
            if mimic is not None:
                return float(mimic.get('multiplier', '1')) * angle(mimic.get('joint'), chain + (name,)) + float(mimic.get('offset', '0'))
            return positions.get(name, 0.)
        def visit(link, transform, ancestors):
            nonlocal potential
            if link in seen:
                raise ValueError('URDFリンク接続の循環')
            seen.add(link)
            mass, com = self.links[link]
            center = transform[:3, :3] @ com + transform[:3, 3]
            force = mass * self.gravity
            potential -= float(force @ center)
            for name, point, axis in ancestors:
                torques[name] -= float(axis @ np.cross(center - point, force))
            for name in self.children.get(link, ()):
                kind, child, offset, axis, _ = self.joints[name]
                frame = transform @ offset
                extra = ((name, frame[:3, 3], frame[:3, :3] @ axis),) if name in torques else ()
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(axis, angle(name))
                elif kind == 'prismatic':
                    motion[:3, 3] = axis * angle(name)
                visit(child, frame @ motion, ancestors + extra)
        visit(self.root, np.eye(4), ())
        if seen != set(self.links) or not all(math.isfinite(value) for value in (*torques.values(), potential)):
            raise ValueError('重力計算の未接続リンク・非有限値')
        return torques, potential
