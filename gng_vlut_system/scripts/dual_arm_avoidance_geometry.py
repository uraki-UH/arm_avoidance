#!/usr/bin/env python3
"""URDF外接球と接近カプセルによる局所退避の幾何計算。"""
from pathlib import Path
from itertools import product
import struct
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*(cross@cross)


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        result[:3, :3] = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
    return result


def mesh_vertices(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50*count:
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        return np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    return np.array([list(map(float, line.split()[1:])) for line in data.decode().splitlines()
                     if line.strip().startswith('vertex ')])


def read_joint_geometry(root):
    """URDF記載順の関節定義・独立関節名・位置制限・親リンク。"""
    joints, joint_names, limits, parents = [], [], [], {}
    for item in root.findall('joint'):
        name = item.get('name')
        kind = item.get('type')
        mimic = item.find('mimic')
        idx = -1
        if kind != 'fixed' and mimic is None:
            idx = len(joint_names)
            joint_names.append(name)
            limit = item.find('limit')
            limits.append((-np.pi, np.pi) if kind == 'continuous' else
                          (float(limit.get('lower')), float(limit.get('upper'))))
        axis = item.find('axis')
        parent, child = item.find('parent').get('link'), item.find('child').get('link')
        parents[child] = parent
        joints.append((name, parent, child, kind, origin(item.find('origin')),
                       np.fromstring(axis.get('xyz') if axis is not None else '1 0 0', sep=' '), idx, mimic))
    return joints, joint_names, np.asarray(limits), parents


def shape_vertices(shape, mesh_directory, link_name):
    """形状ローカル座標の頂点群。基本形状は外接箱の頂点群。"""
    mesh = shape.find('geometry/mesh')
    if mesh is not None:
        vertices = mesh_vertices(mesh_directory/mesh.get('filename'))
        vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        return vertices
    box = shape.find('geometry/box')
    if box is not None:
        size = np.fromstring(box.get('size', ''), sep=' ')
        if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
            raise ValueError(f'直方体寸法の不正: {link_name}')
        return np.asarray(list(product((-1, 1), repeat=3))) * size / 2
    sphere = shape.find('geometry/sphere')
    if sphere is not None:
        radius = float(sphere.get('radius'))
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError(f'球寸法の不正: {link_name}')
        return np.asarray(list(product((-1, 1), repeat=3))) * radius
    cylinder = shape.find('geometry/cylinder')
    if cylinder is not None:
        radius, length = float(cylinder.get('radius')), float(cylinder.get('length'))
        if not np.all(np.isfinite([radius, length])) or min(radius, length) <= 0:
            raise ValueError(f'円柱寸法の不正: {link_name}')
        return np.asarray(list(product((-1, 1), repeat=3))) * [radius, radius, length / 2]
    raise ValueError(f'未対応の形状: {link_name}')


def enclosing_spheres(link_name, vertices):
    """リンク座標の頂点群を覆うAABB球列。長軸方向の記載順。"""
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    axis = int(np.argmax(upper-lower))
    center = (lower+upper)/2
    half = (upper-lower)/2
    count = max(1, int(np.ceil(half[axis]*2/0.035)))
    spacing = half[axis]*2/count
    radius = float(np.sqrt(np.sum(np.delete(half, axis)**2)+(spacing/2)**2))
    spheres = []
    for part in range(count):
        point = center.copy()
        point[axis] = lower[axis]+spacing*(part+0.5)
        spheres.append((link_name, point, radius))
    return spheres


def read_link_spheres(root, mesh_directory):
    """URDF記載順のリンク外接球。衝突形状のない外装も監視対象。"""
    spheres = []
    for link in root.findall('link'):
        name = link.get('name')
        shapes = link.findall('collision') or link.findall('visual')
        for shape in shapes:
            vertices = shape_vertices(shape, mesh_directory, name)
            transform = origin(shape.find('origin'))
            vertices = vertices@transform[:3, :3].T + transform[:3, 3]
            spheres.extend(enclosing_spheres(name, vertices))
    return spheres


def compile_link_operations(root, joints, joint_names, parents):
    """木構造の接続順・mimic係数・軸行列の事前展開。"""
    roots = {link.get('name') for link in root.findall('link')} - set(parents)
    if len(roots) != 1:
        raise ValueError('URDFのルートリンクが一意ではありません')
    root_link = roots.pop()
    link_indices = {root_link: 0}
    operations = []
    remaining = list(joints)
    while remaining:
        pending = []
        for name, parent, child, kind, fixed, axis, idx, mimic in remaining:
            if parent not in link_indices:
                pending.append((name, parent, child, kind, fixed, axis, idx, mimic))
                continue
            child_idx = len(link_indices)
            link_indices[child] = child_idx
            multiplier, offset = 1.0, 0.0
            if mimic is not None:
                idx = joint_names.index(mimic.get('joint'))
                multiplier, offset = float(mimic.get('multiplier', '1')), float(mimic.get('offset', '0'))
            axis = axis/np.linalg.norm(axis)
            x, y, z = axis
            cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            operations.append((link_indices[parent], child_idx, kind, fixed, axis, idx,
                               multiplier, offset, cross, cross@cross))
        if len(pending) == len(remaining):
            raise ValueError('URDFの接続を解決できません')
        remaining = pending
    return root_link, link_indices, operations


def is_nearby_link_pair(first, second, parents):
    """自己干渉監視から除外する接続距離3以内のリンク対。"""
    ancestors = {}
    for depth in range(4):
        ancestors[first] = depth
        first = parents.get(first, first)
    for depth in range(4):
        if second in ancestors and depth+ancestors[second] <= 3:
            return True
        second = parents.get(second, second)
    return False


def build_collision_pairs(spheres, centers, radii, is_arm, parents, link_groups):
    """初期重複・隣接リンクを除いた自己干渉ペアと腕間ペア。"""
    pairs = []
    for first in range(len(spheres)):
        for second in range(first):
            if not (is_arm[first] or is_arm[second]):
                continue
            if is_nearby_link_pair(spheres[first][0], spheres[second][0], parents):
                continue
            gap = np.linalg.norm(centers[first]-centers[second])-radii[first]-radii[second]
            if gap > 0.015:
                pairs.append((first, second))
    # 左右の可動腕同士の干渉ペア。胴体・床・作業台との干渉とは区別
    inter_arm_pairs = [
        (first, second) for first, second in pairs
        if is_arm[first] and is_arm[second]
        and (link_groups[spheres[first][0]] != link_groups[spheres[second][0]] if link_groups is not None
             else {spheres[first][0][0], spheres[second][0][0]} == {'L', 'R'})
    ]
    return (np.asarray(pairs, dtype=int).reshape(-1, 2),
            np.asarray(inter_arm_pairs, dtype=int).reshape(-1, 2))


class robot_geometry:
    """外接球列による保守的なリンク形状。隣接・初期重複球を除いた自己干渉監視。"""

    def __init__(self, urdf_path, planning_groups=None):
        path = Path(urdf_path)
        root = ET.parse(path).getroot()
        self.joints, self.joint_names, self.limits, self.parents = read_joint_geometry(root)
        self.arm_indices = [i for i, name in enumerate(self.joint_names)
                            if name.startswith(('L_joint', 'R_joint'))]
        self.spheres = read_link_spheres(root, path.parent)
        self.root_link, self.link_indices, self.operations = compile_link_operations(
            root, self.joints, self.joint_names, self.parents)
        link_indices = self.link_indices
        self.sphere_links = np.array([link_indices[name] for name, _, _ in self.spheres])
        self.sphere_points = np.array([point for _, point, _ in self.spheres])
        self.radii = np.array([entry[2] for entry in self.spheres])
        self.is_arm = np.array([name.startswith(('L_', 'R_')) and 'shoulder' not in name
                                for name, _, _ in self.spheres])
        link_groups = None
        if planning_groups is not None:
            link_groups = {name: group['name'] for group in planning_groups for name in group['link_names']}
            self.is_arm = np.array([name in link_groups for name, _, _ in self.spheres])
            self.arm_indices = [self.joint_names.index(name) for group in planning_groups for name in group['joint_names']]
        self.self_pairs = np.empty((0, 2), dtype=int)
        centers = self.centers(np.zeros(len(self.joint_names)))
        self.self_pairs, self.inter_arm_pairs = build_collision_pairs(
            self.spheres, centers, self.radii, self.is_arm, self.parents, link_groups)

    def link_transforms(self, positions):
        transforms = np.empty((len(self.operations)+1, 4, 4))
        transforms[0] = np.eye(4)
        for parent, child, kind, fixed, axis, idx, multiplier, offset, cross, square in self.operations:
            if idx < 0:
                transforms[child] = transforms[parent]@fixed
                continue
            value = positions[idx]*multiplier+offset
            motion = np.eye(4)
            if kind in ('revolute', 'continuous'):
                motion[:3, :3] += np.sin(value)*cross+(1-np.cos(value))*square
            elif kind == 'prismatic':
                motion[:3, 3] = axis*value
            transforms[child] = transforms[parent]@fixed@motion
        return transforms

    def centers(self, positions):
        selected = self.link_transforms(positions)[self.sphere_links]
        return np.einsum('nij,nj->ni', selected[:, :3, :3], self.sphere_points)+selected[:, :3, 3]

    def clearance(self, positions, hand, elbow, radius):
        centers = self.centers(positions)
        segment = elbow-hand
        ratio = np.clip((centers-hand)@segment/max(float(segment@segment), 1e-12), 0, 1)
        closest = hand+ratio[:, None]*segment
        gap = np.linalg.norm(centers-closest, axis=1)-self.radii-radius
        idx = int(np.argmin(gap))
        return float(gap[idx]), centers, idx, closest[idx]

    def has_inter_arm_clearance(self, centers, min_clearance_th=0.005):
        first, second = self.inter_arm_pairs.T
        return not np.any(np.linalg.norm(centers[first]-centers[second], axis=1)
                          -self.radii[first]-self.radii[second] < min_clearance_th)

    def internal_clearances(self, centers):
        """自己干渉ペア・床・作業台の外接球表面間距離 [m]。"""
        first, second = self.self_pairs.T
        self_gaps = np.linalg.norm(centers[first]-centers[second], axis=1)-self.radii[first]-self.radii[second]
        floor_gaps = centers[self.is_arm, 2]-self.radii[self.is_arm]
        # Gazebo作業台の外接箱
        delta = np.maximum(np.abs(centers-np.array([0.70, 0, 0.20]))-np.array([0.175, 0.4, 0.2]), 0)
        table_gaps = np.linalg.norm(delta[self.is_arm], axis=1)-self.radii[self.is_arm]
        return np.concatenate((self_gaps, floor_gaps, table_gaps))

    def has_internal_clearance(self, centers, min_clearance_th=0.005):
        return not np.any(self.internal_clearances(centers) < min_clearance_th)

    def choose_step(self, positions, home, hand, elbow, radius, min_clearance, max_step,
                    min_internal_clearance_th=0.005):
        current_gap, _, _, _ = self.clearance(positions, hand, elbow, radius)
        def evaluate(candidate, max_cost=float('inf')):
            gap, centers, _, _ = self.clearance(candidate, hand, elbow, radius)
            cost = 200*max(0.0, min_clearance-gap)**2 + 0.003*float(np.sum((candidate-home)**2))
            if cost >= max_cost or not self.has_internal_clearance(centers, min_internal_clearance_th):
                return float('inf'), gap
            # 現姿勢から候補までの中間姿勢を含む余裕確認
            middle_gap, middle, _, _ = self.clearance((positions+candidate)/2, hand, elbow, radius)
            if not self.has_internal_clearance(middle, min_internal_clearance_th) or min(gap, middle_gap) < min(current_gap, min_clearance)-0.0002:
                return float('inf'), gap
            return cost, gap
        best = positions.copy()
        best_cost, _ = evaluate(best)
        candidates = [positions+np.clip(home-positions, -max_step, max_step)]
        for idx in self.arm_indices:
            for sign in (-1, 1):
                candidate = positions.copy()
                candidate[idx] += sign*max_step
                candidates.append(candidate)
        for candidate in candidates:
            candidate = np.clip(candidate, self.limits[:, 0], self.limits[:, 1])
            score, _ = evaluate(candidate, best_cost)
            if score < best_cost-1e-9:
                best_cost, best = score, candidate
        return best, bool(np.isfinite(best_cost))
