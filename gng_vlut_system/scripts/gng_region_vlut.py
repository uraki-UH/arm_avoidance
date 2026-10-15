#!/usr/bin/env python3
"""3D占有に応じた凸な姿勢領域ノードと、被覆付き状態ペアによる遷移判定。"""
import argparse
import ctypes
from dataclasses import dataclass
import itertools
import json
import os
from pathlib import Path
import time

import numpy as np

from dual_arm_avoidance_geometry import calculate_link_transforms
from gng_edge_vlut import (
    bridge_check, checked_positive, content_signature, edge_state, geometry_signature,
    load_index, packed_ids, pose_key, save_index, sphere_voxels)


_native_kernels = {}


def native_kernel():
    """C++数値部品の遅延読込み。参照実装の明示選択、未配置時の既定互換。"""
    mode = os.environ.get('GNG_REGION_KERNEL', 'auto')
    if mode not in ('auto', 'python', 'cpp'):
        raise ValueError('GNG_REGION_KERNELはauto・python・cpp')
    if mode == 'python':
        return None
    explicit_path = os.environ.get('GNG_REGION_NATIVE_LIBRARY', '')
    key = mode, explicit_path
    if key in _native_kernels:
        return _native_kernels[key]
    paths = ([Path(explicit_path)] if explicit_path else
             [Path(__file__).absolute().parent/'libgng_region_kernel.so']+
             [Path(prefix)/'lib/gng_vlut_system/libgng_region_kernel.so'
              for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if prefix])
    filename = next((path for path in paths if path.is_file()), None)
    if filename is None:
        if mode == 'cpp' or explicit_path:
            raise RuntimeError('C++領域カーネル未配置。ビルドと実行環境の確認が必要')
        _native_kernels[key] = None
        return None
    library = ctypes.CDLL(str(filename.absolute()))
    library.gng_region_kernel_version.restype = ctypes.c_int
    if library.gng_region_kernel_version() != 1:
        raise RuntimeError('C++領域カーネルのABI不一致')
    floats = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    integers = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
    flags = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
    library.gng_region_coefficients.argtypes = [ctypes.c_size_t]*3+[
        integers, integers, floats, integers, integers, floats, floats, floats,
        floats, flags, floats, floats, floats, floats]
    library.gng_region_coefficients.restype = ctypes.c_int
    library.gng_region_points_clear.argtypes = [ctypes.c_size_t]*3+[
        floats, floats, floats, floats, floats, ctypes.c_double, ctypes.c_double]
    library.gng_region_points_clear.restype = ctypes.c_int
    if hasattr(library, 'gng_region_accumulate_counts'):
        library.gng_region_accumulate_counts.argtypes = [ctypes.c_size_t]*4+[
            integers, integers, integers, flags, ctypes.c_int, integers]
        library.gng_region_accumulate_counts.restype = ctypes.c_int
    if hasattr(library, 'gng_region_accumulate_leaf_counts'):
        leaf_bits = np.ctypeslib.ndpointer(dtype=np.uint16, flags='C_CONTIGUOUS')
        library.gng_region_accumulate_leaf_counts.argtypes = [ctypes.c_size_t]*5+[
            integers, integers, integers, leaf_bits, integers, integers, ctypes.c_int, integers]
        library.gng_region_accumulate_leaf_counts.restype = ctypes.c_int
    _native_kernels[key] = library
    return library


def unpacked_cells(values):
    values = np.asarray(values, dtype=np.uint64)
    mask = np.uint64((1 << 21)-1)
    return np.column_stack([((values >> np.uint64(shift)) & mask).astype(np.int64)-(1 << 20)
                            for shift in (42, 21, 0)])


def unique_poses(values):
    """凸包の頂点候補。高次元の面列挙・全関節格子なし。"""
    return np.unique(np.asarray(values, dtype=float), axis=0)


def fit_quadratic(points, widths, active_indices, source_domains=()):
    """連動楕円体の近似と既存領域の包含。制約頂点・高次元格子の追加なし。"""
    points = unique_poses(points)
    reference = np.mean(points, axis=0)
    active = np.asarray(active_indices, dtype=int)
    delta = points[:, active]-reference[active]
    covariance = delta.T@delta/max(1, len(points))+np.diag(np.asarray(widths)**2)
    candidates = [covariance]
    if source_domains:
        candidates.append(covariance+sum(domain['generators'][active]@domain['generators'][active].T
                                        for domain in source_domains)/len(source_domains))
    choices = []
    for candidate in candidates:
        values, vectors = np.linalg.eigh(candidate)
        scale = max(float(np.max(values)), 1e-24)
        values = np.maximum(values, scale*1e-14)
        generators = np.zeros((len(reference), len(reference)))
        generators[np.ix_(active, active)] = vectors*np.sqrt(values)
        inverse = np.linalg.pinv(generators, rcond=1e-15)
        multiplier = max(1., float(np.max(np.linalg.norm((points-reference)@inverse.T, axis=1))))
        for domain in source_domains:
            # 三角不等式による既存楕円体全体の包含。登録済み姿勢だけの照合ではない条件。
            multiplier = max(multiplier, np.linalg.norm(inverse@(domain['reference']-reference))
                             +np.linalg.norm(inverse@domain['generators'], ord=2))
        generators *= multiplier+1e-10
        score = float(np.sum(np.linalg.norm(generators[active], axis=1)/np.maximum(widths, 1e-12)))
        choices.append((score, generators))
    return reference, min(choices, key=lambda item: item[0])[1]


def quadratic_contains(reference, generators, poses, limits):
    """楕円体・生成部分空間・関節制限の共通部分。"""
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    inverse = np.linalg.pinv(generators, rcond=1e-15)
    delta = poses-reference
    coefficients = delta@inverse.T
    residual = delta-coefficients@generators.T
    return (np.isfinite(poses).all(axis=1) & (np.linalg.norm(coefficients, axis=1) <= 1.+1e-9)
            & (np.max(np.abs(residual), axis=1) <= 1e-10)
            & np.all(poses >= limits[:, 0]-1e-10, axis=1) & np.all(poses <= limits[:, 1]+1e-10, axis=1))


def absolute_linear_bound(coefficients, generators, max_dev):
    """連動楕円体上の重み付き絶対偏差上界。Cauchy-SchwarzとGram行和上界。"""
    spreads = np.linalg.norm(generators, axis=1)
    weights = coefficients/np.maximum(spreads, 1e-30)
    gram = np.einsum('sj,jk,jl->skl', weights, generators, generators, optimize=True)
    eigen_upper = np.min(np.stack((np.max(np.sum(np.abs(gram), axis=2), axis=1),
                                  np.trace(gram, axis1=1, axis2=2))), axis=0)
    bound = np.sqrt(np.maximum(0., (coefficients@spreads)*eigen_upper))
    return np.minimum(bound, coefficients@max_dev)+1e-12


def point_envelope_clear(points, centers, radii, supports, normals, min_gap, cell_radius):
    """保守球と支持平面の共通包絡。実際の要求余裕による境界点の判定。"""
    library = native_kernel()
    if library is not None:
        buffers = [np.ascontiguousarray(value, dtype=float) for value in (points, centers, radii, supports, normals)]
        if (buffers[0].shape != (len(points), 3) or buffers[1].shape != (len(radii), 3)
                or buffers[2].shape != (len(radii),) or buffers[3].shape != (len(radii), len(normals))
                or buffers[4].shape != (len(normals), 3)):
            raise ValueError('C++点包絡の配列形状の不一致')
        answer = library.gng_region_points_clear(len(points), len(radii), len(normals), *buffers, min_gap, cell_radius)
        if answer in (0, 1):
            return bool(answer)
        if answer != 2:
            raise RuntimeError('C++点包絡の計算失敗')
    delta = points[:, None]-centers
    point_indices, sphere_indices = np.nonzero(np.sum(delta*delta, axis=2) <= (radii+min_gap+cell_radius+1e-9)**2)
    projections = delta[point_indices, sphere_indices]@normals.T
    return not np.any(np.all(projections <= supports[sphere_indices]+min_gap+cell_radius+1e-9, axis=1))


class region_geometry:
    """FK・解析Jacobian・全域微分上界の共用。点群・関節組合せの登録なし。"""
    def __init__(self, geometry, voxel_size, clearance_margin, max_num_cells, end_sec):
        self.geometry = geometry
        self.voxel_size = voxel_size
        self.clearance_margin = clearance_margin
        self.max_num_cells = max_num_cells
        self.end_sec = end_sec
        self.cache = {}
        self.enable_static_cache = False
        self.sphere_cache = {}
        self.native_anchor_cache = {}
        self.kernel_backend = 'cpp' if native_kernel() is not None else 'python'
        self.num_evaluations = 0
        self.paths = []
        operations = {item[1]: item for item in geometry.center_operations}
        shape = (len(geometry.radii), len(geometry.joint_names))
        self.rot_bounds, self.pris_bounds, self.angular_bounds = [np.zeros(shape) for _ in range(3)]
        for sphere_idx, child in enumerate(geometry.center_links):
            path = []
            reach = np.linalg.norm(geometry.center_points[sphere_idx])
            while child:
                operation = operations[child]
                parent, _, kind, fixed, _, idx, multiplier, offset, *_ = operation
                if kind in ('revolute', 'continuous'):
                    self.rot_bounds[sphere_idx, idx] += abs(multiplier)*reach
                    self.angular_bounds[sphere_idx, idx] += abs(multiplier)
                elif kind == 'prismatic':
                    self.pris_bounds[sphere_idx, idx] += abs(multiplier)
                    reach += np.max(np.abs(geometry.limits[idx]*multiplier+offset))
                else:
                    raise ValueError('未対応の関節形状')
                path.append((operation, reach))
                reach += np.linalg.norm(fixed[:3, 3])
                child = parent
            self.paths.append(path)
        self.global_bounds = self.rot_bounds+self.pris_bounds
        self.operation_indices = {operation[1]: idx for idx, operation in enumerate(geometry.center_operations)}
        self.descendants = {child: [] for child in self.operation_indices}
        for operation in geometry.center_operations:
            parent = operation[0]
            while parent:
                self.descendants[parent].append(operation)
                parent = operations[parent][0]
        self.operation_joints = np.asarray([operation[5] for operation in geometry.center_operations], dtype=int)
        self.native_layout = (
            self.operation_joints.astype(np.int64),
            np.asarray([int(operation[2] == 'prismatic') for operation in geometry.center_operations], dtype=np.int64),
            np.asarray([operation[6] for operation in geometry.center_operations], dtype=float),
            np.r_[0, np.cumsum([len(path) for path in self.paths], dtype=np.int64)],
            np.asarray([self.operation_indices[operation[1]] for path in self.paths for operation, _ in path], dtype=np.int64),
            np.asarray([reach for path in self.paths for _, reach in path], dtype=float))
        membership = np.zeros((len(self.paths), len(self.operation_joints)), dtype=bool)
        for sphere_idx, path in enumerate(self.paths):
            for operation, _ in path:
                membership[sphere_idx, self.operation_indices[operation[1]]] = True
        self.has_operation = membership
        a, b = geometry.self_pairs.T
        self.unshared_operations = ~(membership[a] & membership[b])
        operation_coeffs = np.zeros((len(geometry.radii), len(self.operation_joints)))
        for sphere_idx, path in enumerate(self.paths):
            for operation, reach in path:
                operation_coeffs[sphere_idx, self.operation_indices[operation[1]]] = abs(operation[6])*(reach if operation[2] != 'prismatic' else 1.)
        self.relative_global_coeffs = (operation_coeffs[a]+operation_coeffs[b])*self.unshared_operations
        normals = np.asarray([item for item in itertools.product((-1., 0., 1.), repeat=3) if any(item)])
        self.normals = normals/np.linalg.norm(normals, axis=1)[:, None]
        self.cell_support = voxel_size/2*np.abs(self.normals).sum(axis=1)

    def anchor(self, q):
        key = np.asarray(q, dtype='<f8').tobytes()
        if key in self.cache:
            return self.cache[key]
        geometry = self.geometry
        transforms = calculate_link_transforms(q, geometry.center_operations)
        centers = geometry.centers(q)
        jacobian = np.zeros((len(centers), 3, len(q)))
        axes, origins = {}, {}
        for parent, child, _, fixed, axis, *_ in geometry.center_operations:
            before = transforms[parent]@fixed
            axes[child], origins[child] = before[:3, :3]@axis, before[:3, 3]
        for sphere_idx, path in enumerate(self.paths):
            for operation, _ in path:
                _, child, kind, _, _, idx, multiplier, *_ = operation
                if kind in ('revolute', 'continuous'):
                    jacobian[sphere_idx, :, idx] += multiplier*np.cross(axes[child], centers[sphere_idx]-origins[child])
                else:
                    jacobian[sphere_idx, :, idx] += multiplier*axes[child]
        # 同一直線上の平行回転軸・軸方向直動による、回転レバー長の不変判定。
        line_preserved = {}
        for child, descendants in self.descendants.items():
            line_preserved[child] = all(
                np.linalg.norm(np.cross(axes[child], axes[item[1]])) < 1e-14
                and (item[2] == 'prismatic' or np.linalg.norm(np.cross(axes[child], origins[item[1]]-origins[child])) < 1e-14)
                for item in descendants)
        result = centers, jacobian, axes, origins, line_preserved
        self.cache[key] = result
        return result

    def bounds(self, reference, vertices, *, generators=None, box_half_width=None):
        """領域全体の中心変位とTaylor剰余上界。軸上不変回転の局所レバー短縮。"""
        delta = np.abs(vertices-reference)
        max_dev = (np.asarray(box_half_width) if box_half_width is not None else
                   np.max(delta, axis=0) if generators is None else
                   np.minimum(np.linalg.norm(generators, axis=1), np.max(np.abs(self.geometry.limits-reference[:, None]), axis=1)))
        centers, _, axes, origins, line_preserved = self.anchor(reference)
        local = np.zeros_like(self.global_bounds)
        operation_coeffs = np.zeros((len(centers), len(self.operation_joints)))
        library = native_kernel()
        if library is not None:
            key = np.asarray(reference, dtype='<f8').tobytes()
            if key not in self.native_anchor_cache:
                children = [operation[1] for operation in self.geometry.center_operations]
                self.native_anchor_cache[key] = (np.asarray([axes[child] for child in children]),
                    np.asarray([origins[child] for child in children]),
                    np.asarray([line_preserved[child] for child in children], dtype=np.uint8))
            result = library.gng_region_coefficients(len(centers), len(reference), len(self.operation_joints),
                *self.native_layout, np.ascontiguousarray(centers), *self.native_anchor_cache[key],
                self.global_bounds, np.ascontiguousarray(max_dev), local, operation_coeffs)
            if result != 0:
                raise RuntimeError('C++連続変位上界の計算失敗')
        for sphere_idx, path in (() if library is not None else enumerate(self.paths)):
            descendants = set()
            for operation, reach in path:
                _, child, kind, _, _, idx, multiplier, *_ = operation
                if kind in ('revolute', 'continuous'):
                    radius = np.linalg.norm(np.cross(axes[child], centers[sphere_idx]-origins[child]))
                    descendant_motion = sum(self.global_bounds[sphere_idx, item]*max_dev[item] for item in descendants)
                    if line_preserved[child]:
                        descendant_motion = 1e-12
                    coefficient = abs(multiplier)*min(reach, radius+descendant_motion)
                else:
                    coefficient = abs(multiplier)
                local[sphere_idx, idx] += coefficient
                operation_coeffs[sphere_idx, self.operation_indices[child]] = coefficient
                descendants.add(idx)
        def upper(coefficients):
            return (coefficients@max_dev if box_half_width is not None else
                    np.max(delta@coefficients.T, axis=0) if generators is None else
                    absolute_linear_bound(coefficients, generators, max_dev))
        movement = upper(local)
        angular = upper(self.angular_bounds)
        rot = upper(self.rot_bounds)
        pris = upper(self.pris_bounds)
        remainder = .5*angular*(rot+2*pris)
        a, b = self.geometry.self_pairs.T
        # 同じ祖先変換による剛体移動は相対距離に影響なし。残りの操作だけの上界。
        coefficients = (operation_coeffs[a]+operation_coeffs[b])*self.unshared_operations
        if generators is None and box_half_width is None:
            pair_motion = np.max(np.einsum('po,vo->pv', coefficients, delta[:, self.operation_joints]), axis=1)
        else:
            per_joint = np.zeros((len(coefficients), len(reference)))
            for idx, joint_idx in enumerate(self.operation_joints):
                per_joint[:, joint_idx] += coefficients[:, idx]
            pair_motion = upper(per_joint)
        return movement, remainder, pair_motion

    def envelope(self, reference, vertices, *, generators=None, max_num_occupied_cells=None, box_half_width=None):
        """基準球包絡と一次像＋二次剰余の共通部分。3D局所セル候補の一度生成。"""
        if time.monotonic() >= self.end_sec:
            raise TimeoutError('領域構築の時間予算超過')
        self.num_evaluations += 1
        vertices = unique_poses([reference, *vertices])
        geometry = self.geometry
        centers, jacobian, *_ = self.anchor(reference)
        if box_half_width is not None:
            if (generators is not None or np.shape(box_half_width) != np.shape(reference)
                    or not np.isfinite(box_half_width).all() or np.any(np.asarray(box_half_width) < 0)):
                raise ValueError('箱領域の半幅・連動制約の不正')
            movement, remainder, pair_motion = self.bounds(reference, vertices, box_half_width=box_half_width)
            supports = np.abs(np.einsum('nk,skj->snj', self.normals, jacobian))@box_half_width
        elif generators is None:
            movement, remainder, pair_motion = self.bounds(reference, vertices)
            images = np.einsum('vj,skj->svk', vertices-reference, jacobian)
            supports = np.max(images@self.normals.T, axis=1)
        else:
            # 連動方向を保持した一次像の支持関数と、関節偏差上界による全域剰余。
            movement, remainder, pair_motion = self.bounds(reference, vertices, generators=generators)
            linear = np.einsum('skj,jl->skl', jacobian, generators)
            supports = np.linalg.norm(np.einsum('nk,skl->snl', self.normals, linear), axis=2)
        core, occupied, outer, inner = set(), set(), set(), set()
        padding = 1e-10
        inflation = self.clearance_margin+self.voxel_size*np.sqrt(3)/2
        for idx in np.flatnonzero(geometry.is_arm):
            if time.monotonic() >= self.end_sec:
                raise TimeoutError('領域構築の時間予算超過')
            radius = geometry.radii[idx]
            is_fixed = (self.enable_static_cache and movement[idx] == 0. and remainder[idx] == 0.
                        and np.all(supports[idx] == 0.))
            cache_key = (int(idx), centers[idx].tobytes()) if is_fixed else None
            cached = self.sphere_cache.get(cache_key) if is_fixed else None
            if cached is not None:
                core.update(cached[0])
                occupied.update(cached[1])
                if max_num_occupied_cells is not None and len(occupied) > max_num_occupied_cells:
                    raise OverflowError('領域占有予算の早期超過')
                outer.update(cached[2])
                inner.update(cached[3])
                continue
            values = sphere_voxels(centers[idx], radius+movement[idx]+inflation+padding,
                                   self.voxel_size, self.max_num_cells)
            low = unpacked_cells(values)*self.voxel_size
            cell_centers = low+self.voxel_size/2
            near = np.maximum(np.maximum(low-centers[idx], centers[idx]-low-self.voxel_size), 0)
            near_sq = np.einsum('ij,ij->i', near, near)
            projected = np.einsum('ik,jk->ij', cell_centers-centers[idx], self.normals)-supports[idx]-self.cell_support
            has_outer = np.all(projected <= radius+remainder[idx]+inflation+padding, axis=1)
            has_occupied = ((near_sq <= (radius+movement[idx]+padding)**2)
                            & np.all(projected <= radius+remainder[idx]+padding, axis=1))
            far = np.maximum(np.abs(low-centers[idx]), np.abs(low+self.voxel_size-centers[idx]))
            has_inner = (radius > movement[idx]+padding) & (np.einsum('ij,ij->i', far, far) <= (max(0., radius-movement[idx]-padding))**2)
            core.update(map(int, values[near_sq <= (radius+padding)**2]))
            occupied.update(map(int, values[has_occupied]))
            if max_num_occupied_cells is not None and len(occupied) > max_num_occupied_cells:
                raise OverflowError('領域占有予算の早期超過')
            outer.update(map(int, values[has_outer]))
            inner.update(map(int, values[has_inner]))
            if is_fixed:
                self.sphere_cache[cache_key] = tuple(frozenset(map(int, item)) for item in
                    (values[near_sq <= (radius+padding)**2], values[has_occupied], values[has_outer], values[has_inner]))
        values = geometry.internal_clearances(centers)
        a, b = geometry.self_pairs.T
        num_pairs = len(a)
        relative = centers[a]-centers[b]
        normals = relative/np.maximum(np.linalg.norm(relative, axis=1, keepdims=True), 1e-12)
        gradient = np.einsum('pi,pij->pj', normals, jacobian[a]-jacobian[b])
        projected_gap = (-np.abs(gradient)@box_half_width if box_half_width is not None else
                         np.min(gradient@(vertices-reference).T, axis=1) if generators is None else
                         -np.linalg.norm(gradient@generators, axis=1))
        # 距離ノルムの支持平面と中心の二次剰余。変位ノルムだけの過細分の抑制。
        pair_lower = values[:num_pairs]+np.maximum(-pair_motion, projected_gap-remainder[a]-remainder[b])
        fixed_lower = []
        cursor = num_pairs
        selected = geometry.is_arm
        for kind, _, rotation, _ in geometry.environment_shapes:
            current = values[cursor:cursor+int(selected.sum())]-movement[selected]
            if kind == 'plane':
                plane_jacobian = np.einsum('k,skj->sj', rotation[:, 2], jacobian[selected])
                projected = (-np.abs(plane_jacobian)@box_half_width if box_half_width is not None else
                             np.min(plane_jacobian@(vertices-reference).T, axis=1) if generators is None else
                             -np.linalg.norm(plane_jacobian@generators, axis=1))
                current = np.maximum(current, values[cursor:cursor+int(selected.sum())]+projected-remainder[selected])
            fixed_lower.append(current)
            cursor += int(selected.sum())
        lower = np.concatenate([pair_lower]+fixed_lower)
        min_internal = float(np.min(lower)) if lower.size else 1e6
        selected = geometry.is_arm
        result = dict(vertices=vertices, core=core, occupied=occupied, outer=outer, inner=inner, min_internal=min_internal,
                      narrow_centers=centers[selected].copy(), narrow_radii=(geometry.radii+movement)[selected],
                      narrow_supports=(supports+geometry.radii[:, None]+remainder[:, None])[selected])
        if generators is not None:
            result.update(reference=reference, generators=generators)
        return result

    def line_internal_lower(self, reference, vertices, min_gap):
        """細長い凸領域の連続内部離隔。横幅の全域補正、FKだけの区間細分。"""
        result = self.line_internal_certificate(reference, vertices, min_gap)
        return None if result is None else result['min_internal']

    def line_internal_certificate(self, reference, vertices, min_gap=None):
        """連続内部離隔の下界と実姿勢の棄却証拠。点群・占有登録への依存なし。"""
        if len(vertices) < 2:
            return None
        centered = vertices-reference
        _, values, directions = np.linalg.svd(centered, full_matrices=False)
        if len(values) > 1 and values[1] > max(1e-12, values[0]*.05):
            return None
        projected = centered@directions[0]
        first, second = vertices[int(np.argmin(projected))], vertices[int(np.argmax(projected))]
        residual = centered-projected[:, None]*directions[0]
        extra_dev = np.max(np.abs(residual), axis=0)+np.max(np.abs(residual[[int(np.argmin(projected)), int(np.argmax(projected))]]), axis=0)
        extra_margin = self.relative_global_coeffs@extra_dev[self.operation_joints]
        for _ in self.geometry.environment_shapes:
            extra_margin = np.r_[extra_margin, (self.global_bounds@extra_dev)[self.geometry.is_arm]]
        count = 9
        lower = None
        min_sampled_internal = 1e6
        while count <= 129 and time.monotonic() < self.end_sec:
            positions = first+np.linspace(0., 1., count)[:, None]*(second-first)
            transforms = calculate_link_transforms(positions, self.geometry.center_operations)
            selected = transforms[:, self.geometry.center_links]
            centers = np.einsum('vnij,nj->vni', selected[:, :, :3, :3], self.geometry.center_points)+selected[:, :, :3, 3]
            velocity = np.zeros_like(centers)
            for operation_idx, operation in enumerate(self.geometry.center_operations):
                parent, _, kind, fixed, axis, joint_idx, multiplier, *_ = operation
                rate = multiplier*(second-first)[joint_idx]
                if rate == 0.:
                    continue
                before = transforms[:, parent]@fixed
                direction = before[:, :3, :3]@axis
                has_operation = self.has_operation[:, operation_idx]
                if kind == 'prismatic':
                    velocity[:, has_operation] += rate*direction[:, None]
                else:
                    velocity[:, has_operation] += rate*np.cross(direction[:, None], centers[:, has_operation]-before[:, None, :3, 3])
            delta = np.abs(second-first)/(2*(count-1))
            margins = self.relative_global_coeffs@delta[self.operation_joints]
            for _ in self.geometry.environment_shapes:
                margins = np.r_[margins, (self.global_bounds@delta)[self.geometry.is_arm]]
            gaps = self.geometry.internal_clearances(centers)
            if not gaps.size:
                return dict(min_internal=1e6, min_sampled_internal=1e6)
            min_sampled_internal = min(min_sampled_internal, float(np.min(gaps)))
            bounds = gaps-margins
            # 距離の支持平面による方向微分と全域二次剰余。ほぼ不変な離隔への過細分抑制。
            remainder = .5*(self.angular_bounds@delta)*((self.rot_bounds+2*self.pris_bounds)@delta)
            a, b = self.geometry.self_pairs.T
            relative = centers[:, a]-centers[:, b]
            normals = relative/np.maximum(np.linalg.norm(relative, axis=2, keepdims=True), 1e-12)
            derivative = np.einsum('vpi,vpi->vp', normals, velocity[:, a]-velocity[:, b])
            bounds[:, :len(a)] = np.maximum(bounds[:, :len(a)], gaps[:, :len(a)]
                -np.abs(derivative)/(2*(count-1))-remainder[a]-remainder[b])
            cursor = len(a)
            for kind, _, rotation, _ in self.geometry.environment_shapes:
                end = cursor+int(self.geometry.is_arm.sum())
                if kind == 'plane':
                    bounds[:, cursor:end] = np.maximum(bounds[:, cursor:end], gaps[:, cursor:end]
                        -np.abs(velocity[:, self.geometry.is_arm]@rotation[:, 2])/(2*(count-1))-remainder[self.geometry.is_arm])
                cursor = end
            lower = float(np.min(bounds-extra_margin))
            if min_gap is not None and lower >= min_gap:
                break
            count = (count-1)*2+1
        return None if lower is None else dict(min_internal=lower, min_sampled_internal=min_sampled_internal)


@dataclass
class region_node:
    reference: np.ndarray
    data: dict
    base_num_cells: int
    base_min_internal: float


class region_builder:
    """占有増分による成長・細分・統合。探索元GNGの書換えなし。"""
    def __init__(self, geometry, active_indices, *, voxel_size, clearance_margin, max_build_sec,
                 max_num_regions, max_num_cells, max_num_vertices, max_extra_cells_th,
                 min_internal_gap, max_num_splits, min_overlap_ratio_th, internal_policy='coupled'):
        self.kernel = region_geometry(geometry, voxel_size, clearance_margin, max_num_cells, time.monotonic()+max_build_sec)
        self.active_indices = active_indices
        self.max_num_regions, self.max_num_vertices = max_num_regions, max_num_vertices
        self.max_extra_cells_th, self.min_internal_gap = max_extra_cells_th, min_internal_gap
        self.max_num_splits, self.min_overlap_ratio_th = max_num_splits, min_overlap_ratio_th
        self.internal_policy = internal_policy
        self.nodes, self.parents, self.links = [], [], []
        self.num_merges = 0
        self.last_witness_gap = None

    def root(self, idx):
        while self.parents[idx] != idx:
            idx = self.parents[idx]
        return idx

    def add(self, reference):
        if len(self.nodes) >= self.max_num_regions:
            return None
        data = self.kernel.envelope(reference, [reference])
        idx = len(self.nodes)
        self.nodes.append(region_node(reference.copy(), data, len(data['occupied']), data['min_internal']))
        self.parents.append(idx)
        return idx

    def proposal(self, idx, vertices):
        node = self.nodes[self.root(idx)]
        vertices = unique_poses([*node.data['vertices'], *vertices])
        if len(vertices) > self.max_num_vertices:
            return None
        try:
            data = self.kernel.envelope(node.reference, vertices)
        except OverflowError:
            return None
        if len(data['occupied'])-node.base_num_cells > self.max_extra_cells_th:
            return None
        if (self.internal_policy == 'coupled' and node.base_min_internal >= self.min_internal_gap
                and data['min_internal'] < self.min_internal_gap):
            return None
        return data

    def grow(self, idx, vertices):
        idx = self.root(idx)
        data = self.proposal(idx, vertices)
        if data is None:
            return False
        self.nodes[idx].data = data
        return True

    def connect(self, first_idx, second_idx, first, second, depth=0):
        """各接続全体を覆う端点領域。片側clearでも遷移の通過判定が可能。"""
        first_idx, second_idx = self.root(first_idx), self.root(second_idx)
        can_first = self.grow(first_idx, [second])
        can_second = self.grow(second_idx, [first]) if second_idx != first_idx else can_first
        if can_first or can_second:
            link_idx = len(self.links)
            self.links.append((first_idx, second_idx, can_first, can_second, first.copy(), second.copy()))
            return [link_idx]
        if depth >= self.max_num_splits:
            return None
        middle = (first+second)/2
        middle_idx = self.add(middle)
        if middle_idx is None:
            return None
        left = self.connect(first_idx, middle_idx, first, middle, depth+1)
        if left is None:
            return None
        right = self.connect(middle_idx, second_idx, middle, second, depth+1)
        return None if right is None else left+right

    def widen(self, idx, max_num_growth_checks):
        """3D球中心Jacobianの低変化方向を優先した、占有予算付き拡張。"""
        node = self.nodes[self.root(idx)]
        _, jacobian, *_ = self.kernel.anchor(node.reference)
        selected = jacobian[self.kernel.geometry.is_arm][:, :, self.active_indices].reshape(-1, len(self.active_indices))
        # 各可動関節の小幅な内側領域。角度等間隔ではなく、ボクセル幅と3D変位での幅設定。
        limits = self.kernel.geometry.limits
        sensitivity = np.max(np.linalg.norm(jacobian[self.kernel.geometry.is_arm][:, :, self.active_indices], axis=1), axis=0)
        widths = np.minimum(np.diff(limits[self.active_indices], axis=1).ravel()/2,
                            self.kernel.voxel_size*.02/np.maximum(sensitivity, 1e-12))
        vertices = []
        for axis, width in zip(self.active_indices, widths):
            for sign in (-1., 1.):
                q = node.reference.copy()
                q[axis] = np.clip(q[axis]+sign*width, *limits[axis])
                vertices.append(q)
        self.grow(idx, vertices)
        _, _, directions = np.linalg.svd(selected, full_matrices=False)
        remaining = max_num_growth_checks
        for direction in directions[::-1]:
            full = np.zeros_like(node.reference)
            full[self.active_indices] = direction
            for sign in (-1., 1.):
                if remaining <= 0:
                    return
                vector = sign*full
                mask = np.abs(vector) > 1e-12
                lengths = np.where(vector[mask] > 0, limits[mask, 1]-node.reference[mask],
                                   limits[mask, 0]-node.reference[mask])/vector[mask]
                max_length = max(0., float(np.min(lengths)))
                if max_length == 0:
                    continue
                remaining -= 1
                self.grow(idx, [node.reference+vector*max_length])

    def merge(self):
        """隣接領域の共通占有率と、統合後の占有予算による限定統合。"""
        for first, second, *_ in self.links:
            first, second = self.root(first), self.root(second)
            if first == second:
                continue
            a, b = self.nodes[first], self.nodes[second]
            common = len(a.data['occupied'] & b.data['occupied'])
            union = len(a.data['occupied'] | b.data['occupied'])
            if not union or common/union < self.min_overlap_ratio_th:
                continue
            candidate = self.proposal(first, b.data['vertices'])
            if candidate is not None:
                self.nodes[first].data = candidate
                self.parents[second] = first
                self.num_merges += 1


class quadratic_builder(region_builder):
    """被覆を保持した二次領域の成長・資源削減統合・両端被覆接続。"""
    def widths(self, reference):
        _, jacobian, *_ = self.kernel.anchor(reference)
        sensitivity = np.max(np.linalg.norm(jacobian[self.kernel.geometry.is_arm][:, :, self.active_indices], axis=1), axis=0)
        span = np.diff(self.kernel.geometry.limits[self.active_indices], axis=1).ravel()
        return np.minimum(span*.001, self.kernel.voxel_size*.02/np.maximum(sensitivity, 1e-12))

    def add(self, reference):
        if len(self.nodes) >= self.max_num_regions:
            return None
        center, generators = fit_quadratic([reference], self.widths(reference), self.active_indices)
        data = self.kernel.envelope(center, [reference], generators=generators)
        idx = len(self.nodes)
        self.nodes.append(region_node(reference.copy(), data, len(data['occupied']), data['min_internal']))
        self.parents.append(idx)
        return idx

    def include(self, pose):
        """全姿勢の保持を前提とした占有近傍の共鳴。未包含時の保護ノード追加。"""
        roots = [idx for idx in range(len(self.nodes)) if self.root(idx) == idx]
        for idx in roots:
            data = self.nodes[idx].data
            if quadratic_contains(data['reference'], data['generators'], [pose], self.kernel.geometry.limits)[0]:
                return idx
        target = self.kernel.geometry.centers(pose)[self.kernel.geometry.is_arm]
        order = sorted(roots, key=lambda idx: np.max(np.linalg.norm(
            self.kernel.anchor(self.nodes[idx].data['reference'])[0][self.kernel.geometry.is_arm]-target, axis=1)))
        for idx in order[:3]:
            if (self.internal_policy == 'separate' or self.nodes[idx].data['min_internal'] >= self.min_internal_gap) and self.grow(idx, [pose]):
                return idx
        return self.add(pose)

    def proposal(self, idx, vertices, source_domains=()):
        node = self.nodes[self.root(idx)]
        if not source_domains and quadratic_contains(node.data['reference'], node.data['generators'], vertices,
                                                     self.kernel.geometry.limits).all():
            return node.data
        points = unique_poses([*node.data['vertices'], *vertices])
        if len(points) > self.max_num_vertices:
            return None
        domains = (node.data, *source_domains)
        reference, generators = fit_quadratic(points, self.widths(node.reference), self.active_indices, domains)
        if not quadratic_contains(reference, generators, points, self.kernel.geometry.limits).all():
            return None
        try:
            data = self.kernel.envelope(reference, points, generators=generators)
        except OverflowError:
            return None
        if len(data['occupied'])-node.base_num_cells > self.max_extra_cells_th:
            return None
        if (self.internal_policy == 'coupled' and max(domain['min_internal'] for domain in domains) >= self.min_internal_gap
                and data['min_internal'] < self.min_internal_gap):
            return None
        return data

    def connect(self, first_idx, second_idx, first, second, depth=0):
        """接続区間全体の端点領域被覆。安全ラベルには全接続の成立も必要。"""
        first_idx, second_idx = self.root(first_idx), self.root(second_idx)
        can_first = self.grow(first_idx, [second])
        can_second = self.grow(second_idx, [first]) if second_idx != first_idx else can_first
        if can_first or can_second:
            idx = len(self.links)
            self.links.append((first_idx, second_idx, can_first, can_second, first.copy(), second.copy()))
            return [idx]
        if depth >= self.max_num_splits:
            return None
        middle = (first+second)/2
        middle_idx = self.add(middle)
        if middle_idx is None:
            return None
        left = self.connect(first_idx, middle_idx, first, middle, depth+1)
        if left is None:
            return None
        right = self.connect(middle_idx, second_idx, middle, second, depth+1)
        return None if right is None else left+right

    def merge(self):
        """占有IoUではなく、連続被覆を保持した逆引き関係の削減。"""
        candidates = set(tuple(sorted((a, b))) for a, b, *_ in self.links if a != b)
        for first, second in sorted(candidates, key=lambda pair: sum(len(self.nodes[idx].data['outer']) for idx in pair)):
            first, second = self.root(first), self.root(second)
            if first == second:
                continue
            a, b = self.nodes[first], self.nodes[second]
            candidate = self.proposal(first, b.data['vertices'], (b.data,))
            if candidate is None or len(candidate['outer']) > len(a.data['outer'])+len(b.data['outer']):
                continue
            self.nodes[first].data = candidate
            self.parents[second] = first
            self.num_merges += 1

    def widen(self, idx, max_num_growth_checks):
        """低占有変化方向の限定探索。失敗時の全領域再構築なし。"""
        node = self.nodes[self.root(idx)]
        _, jacobian, *_ = self.kernel.anchor(node.reference)
        selected = jacobian[self.kernel.geometry.is_arm][:, :, self.active_indices].reshape(-1, len(self.active_indices))
        _, _, directions = np.linalg.svd(selected, full_matrices=False)
        for direction in directions[::-1][:max_num_growth_checks]:
            full = np.zeros_like(node.reference)
            full[self.active_indices] = direction
            for sign in (-1., 1.):
                vector = sign*full
                is_moving = np.abs(vector) > 1e-12
                limits = self.kernel.geometry.limits
                lengths = np.where(vector[is_moving] > 0, limits[is_moving, 1]-node.reference[is_moving],
                                   limits[is_moving, 0]-node.reference[is_moving])/vector[is_moving]
                length = max(0., float(np.min(lengths)))
                for _ in range(3):
                    if self.grow(idx, [node.reference+vector*length]):
                        break
                    length *= .5


class occupancy_builder(region_builder):
    """占有変化による共鳴と連続被覆保持。領域形状の形式より接続・索引効率の優先。"""
    def include(self, pose):
        roots = [idx for idx in range(len(self.nodes)) if self.root(idx) == idx]
        for idx in roots:
            if np.any(np.all(self.nodes[idx].data['vertices'] == pose, axis=1)):
                return idx
        target = self.kernel.geometry.centers(pose)[self.kernel.geometry.is_arm]
        order = sorted(roots, key=lambda idx: np.max(np.linalg.norm(
            self.kernel.anchor(self.nodes[idx].data.get('reference', self.nodes[idx].reference))[0][self.kernel.geometry.is_arm]-target, axis=1)))
        for idx in order[:3]:
            if (self.internal_policy == 'separate' or self.nodes[idx].data['min_internal'] >= self.min_internal_gap) and self.grow(idx, [pose]):
                return idx
        return self.add(pose)

    def connect(self, first_idx, second_idx, first, second, depth=0):
        """実中間姿勢の不成立を先行棄却。不可行区間の際限のない領域細分の抑制。"""
        centers = self.kernel.geometry.centers((first+second)/2)
        values = self.kernel.geometry.internal_clearances(centers)
        gap = float(np.min(values)) if values.size else 1e6
        if gap < self.min_internal_gap:
            self.last_witness_gap = gap if self.last_witness_gap is None else min(self.last_witness_gap, gap)
            return None
        return super().connect(first_idx, second_idx, first, second, depth)

    def proposal(self, idx, vertices):
        node = self.nodes[self.root(idx)]
        points = unique_poses([*node.data['vertices'], *vertices])
        if len(points) > self.max_num_vertices:
            return None
        # FK上界による包絡中心の選択。セル登録は採用した中心だけの一度生成。
        references = [node.reference, np.mean(points, axis=0)]
        estimates = []
        for reference in references:
            movement, _, _ = self.kernel.bounds(reference, points)
            score = float(np.sum((self.kernel.geometry.radii[self.kernel.geometry.is_arm]+movement[self.kernel.geometry.is_arm])**3))
            estimates.append((score, reference))
        reference = min(estimates, key=lambda item: item[0])[1]
        try:
            data = self.kernel.envelope(reference, points, max_num_occupied_cells=node.base_num_cells+self.max_extra_cells_th)
        except OverflowError:
            return None
        data.update(reference=reference.copy(), vertices=points)
        if len(data['occupied'])-node.base_num_cells > self.max_extra_cells_th:
            return None
        if self.internal_policy == 'coupled' and node.data['min_internal'] >= self.min_internal_gap and data['min_internal'] < self.min_internal_gap:
            lower = self.kernel.line_internal_lower(reference, points, self.min_internal_gap)
            if lower is None or lower < self.min_internal_gap:
                return None
            data['min_internal'] = max(data['min_internal'], lower)
        return data

    def merge(self):
        """連続姿勢集合の包含を保持した、隣接領域の資源削減統合。"""
        candidates = set(tuple(sorted((a, b))) for a, b, *_ in self.links if a != b)
        for first, second in sorted(candidates):
            first, second = self.root(first), self.root(second)
            if first == second:
                continue
            a, b = self.nodes[first], self.nodes[second]
            candidate = self.proposal(first, b.data['vertices'])
            if candidate is None or len(candidate['outer']) > len(a.data['outer'])+len(b.data['outer']):
                continue
            if self.internal_policy == 'coupled' and b.data['min_internal'] >= self.min_internal_gap and candidate['min_internal'] < self.min_internal_gap:
                continue
            self.nodes[first].data = candidate
            self.parents[second] = first
            self.num_merges += 1


def flat_sets(nodes, name):
    values = [np.asarray(sorted(node.data[name]), dtype=np.uint64) for node in nodes]
    return np.concatenate(values) if values else np.array([], dtype=np.uint64), np.r_[0, np.cumsum([len(value) for value in values], dtype=np.int64)]


def build_index(geometry, nodes, edges, *, active_joint_names=None, voxel_size=.05, clearance_margin=.02,
                max_num_regions=256, max_num_edges=512, max_num_relations=500000, max_build_sec=60.,
                max_num_cells=100000, max_num_vertices=64, max_extra_cells_th=96,
                min_internal_gap=.005, max_num_splits=8, max_num_growth_checks=2, min_overlap_ratio_th=.9,
                construction_method='legacy', required_poses=(), internal_policy='coupled'):
    """高次元の疎な領域ノード構築。元GNG・専用辺占有索引への依存なし。"""
    start_sec = time.monotonic()
    if construction_method not in ('legacy', 'quadratic_cover', 'occupancy_cover'):
        raise ValueError('領域構築方式の不正')
    enable_cover = construction_method != 'legacy'
    if internal_policy not in ('coupled', 'separate') or (not enable_cover and internal_policy != 'coupled'):
        raise ValueError('内部干渉分離にはoccupancy_cover・quadratic_cover・efficientの選択が必要')
    required = np.asarray(required_poses, dtype=float).reshape(-1, len(geometry.joint_names))
    if (not np.isfinite(required).all() or np.any(required < geometry.limits[:, 0]) or np.any(required > geometry.limits[:, 1])):
        raise ValueError('保護対象姿勢の関節制限・実数形式の不正')
    if len(required) and not enable_cover:
        raise ValueError('保護姿勢指定にはoccupancy_cover・quadratic_cover・efficientの選択が必要')
    edges = list(edges)
    num_original_nodes, num_original_edges = len(nodes), len(edges)
    protected_ids = []
    if enable_cover and len(required):
        nodes = {idx: np.asarray(q, dtype=float) for idx, q in nodes.items()}
        original_ids = sorted(nodes)
        original_positions = np.asarray([nodes[idx] for idx in original_ids])
        original_centers = geometry.centers(original_positions)[:, geometry.is_arm]
        for q in required:
            same = [idx for idx, value in nodes.items() if np.array_equal(value, q)]
            idx = same[0] if same else max(nodes, default=-1)+1
            nodes[idx] = q.copy()
            protected_ids.append(idx)
            center = geometry.centers(q)[geometry.is_arm]
            order = np.argsort(np.max(np.linalg.norm(original_centers-center, axis=2), axis=1))
            edges.extend((idx, original_ids[row]) for row in order[:3] if idx != original_ids[row])
    for name, value in (('voxel_size', voxel_size), ('max_build_sec', max_build_sec), ('min_internal_gap', min_internal_gap)):
        checked_positive(value, name)
    if not np.isfinite(clearance_margin) or clearance_margin < 0 or not 0 <= min_overlap_ratio_th <= 1:
        raise ValueError('領域の余裕・共通占有率設定の不正')
    for name, value in (('max_num_regions', max_num_regions), ('max_num_edges', max_num_edges),
                        ('max_num_relations', max_num_relations), ('max_num_cells', max_num_cells),
                        ('max_num_vertices', max_num_vertices), ('max_extra_cells_th', max_extra_cells_th),
                        ('max_num_splits', max_num_splits), ('max_num_growth_checks', max_num_growth_checks)):
        if type(value) is not int or value < (2 if name == 'max_num_vertices' else 0 if name in (
                'max_extra_cells_th', 'max_num_splits', 'max_num_growth_checks') else 1):
            raise ValueError(name+'の整数予算の不正')
    ids = sorted(nodes)
    positions = np.asarray([nodes[idx] for idx in ids], dtype=float)
    if (not ids or positions.shape != (len(ids), len(geometry.joint_names)) or not np.isfinite(positions).all()
            or any(type(idx) is not int or idx < 0 for idx in ids)):
        raise ValueError('領域構築元のノード形式の不正')
    if np.any(positions < geometry.limits[:, 0]-1e-8) or np.any(positions > geometry.limits[:, 1]+1e-8):
        raise ValueError('領域構築元の関節可動域違反')
    active = [geometry.joint_names.index(name) for name in (active_joint_names or geometry.joint_names)]
    if not active or len(set(active)) != len(active) or not np.any(geometry.is_arm):
        raise ValueError('領域構築の対象関節・監視形状の不正')
    lookup = {idx: row for row, idx in enumerate(ids)}
    pairs = sorted(set(tuple(sorted(pair)) for pair in edges))
    if any(len(pair) != 2 or pair[0] == pair[1] or any(idx not in lookup for idx in pair) for pair in pairs):
        raise ValueError('領域構築元の接続形式の不正')
    pairs = np.asarray([(lookup[a], lookup[b]) for a, b in pairs], dtype=np.int64).reshape(-1, 2)
    builder_type = {'legacy': region_builder, 'quadratic_cover': quadratic_builder,
                    'occupancy_cover': occupancy_builder}[construction_method]
    builder = builder_type(geometry, active, voxel_size=voxel_size, clearance_margin=clearance_margin,
        max_build_sec=max_build_sec, max_num_regions=max_num_regions, max_num_cells=max_num_cells,
        max_num_vertices=max_num_vertices, max_extra_cells_th=max_extra_cells_th,
        min_internal_gap=min_internal_gap, max_num_splits=max_num_splits, min_overlap_ratio_th=min_overlap_ratio_th,
        internal_policy=internal_policy)
    builder.kernel.enable_static_cache = enable_cover
    if enable_cover:
        builder.kernel.end_sec = start_sec+max_build_sec*.8
    seed_regions = np.full(len(ids), -1, dtype=np.int64)
    routes = [[] for _ in pairs]
    protected_regions = []
    edge_witness_gap = np.full(len(pairs), 1e6)
    edge_internal_lower = np.zeros(len(pairs))
    has_internal_bound = np.zeros(len(pairs), dtype=bool)
    internal_certificate_ms = 0.
    if enable_cover:
        # 不可行な接続の細分への予算消費の抑制。棄却根拠は実姿勢の離隔値のみ。
        for idx, (a, b) in enumerate(pairs):
            samples = positions[a]+np.linspace(0., 1., 9)[:, None]*(positions[b]-positions[a])
            gaps = geometry.internal_clearances(geometry.centers(samples))
            edge_witness_gap[idx] = float(np.min(gaps)) if gaps.size else 1e6
    try:
        for row, q in enumerate(positions):
            idx = builder.include(q) if enable_cover else builder.add(q)
            if idx is None:
                break
            seed_regions[row] = idx
        if enable_cover:
            protected_regions = [int(seed_regions[lookup[idx]]) for idx in protected_ids
                                 if seed_regions[lookup[idx]] >= 0]
        else:
            for q in required:
                idx = builder.add(q)
                if idx is None:
                    raise ValueError('保護対象姿勢の領域登録予算不足')
                protected_regions.append(idx)
        if enable_cover and np.all(seed_regions >= 0):
            # 種姿勢保持後の区間証明。全体構築予算内の限定枠、未証明は従来検査。
            begin = time.monotonic()
            previous_end_sec = builder.kernel.end_sec
            builder.kernel.end_sec = min(previous_end_sec, begin+min(5., max_build_sec*.1))
            try:
                for edge_idx, (a, b) in enumerate(pairs[:max_num_edges]):
                    if time.monotonic() >= builder.kernel.end_sec:
                        break
                    if edge_witness_gap[edge_idx] < min_internal_gap:
                        continue
                    certificate = builder.kernel.line_internal_certificate((positions[a]+positions[b])/2,
                        positions[[a, b]], min_gap=min_internal_gap+1e-9)
                    if certificate is not None:
                        edge_internal_lower[edge_idx] = certificate['min_internal']
                        edge_witness_gap[edge_idx] = min(edge_witness_gap[edge_idx], certificate['min_sampled_internal'])
                        has_internal_bound[edge_idx] = True
            finally:
                builder.kernel.end_sec = previous_end_sec
                internal_certificate_ms = (time.monotonic()-begin)*1000
        for edge_idx, (a, b) in enumerate(pairs[:max_num_edges]):
            if seed_regions[a] < 0 or seed_regions[b] < 0:
                continue
            if enable_cover and edge_witness_gap[edge_idx] < min_internal_gap:
                continue
            builder.last_witness_gap = None
            try:
                route = builder.connect(seed_regions[a], seed_regions[b], positions[a], positions[b])
            finally:
                if enable_cover and builder.last_witness_gap is not None:
                    edge_witness_gap[edge_idx] = min(edge_witness_gap[edge_idx], builder.last_witness_gap)
            if route is not None:
                routes[edge_idx] = route
        if not enable_cover:
            builder.merge()
            for idx in range(len(builder.nodes)):
                if builder.root(idx) == idx:
                    builder.widen(idx, max_num_growth_checks)
    except (TimeoutError, OverflowError):
        if enable_cover and (np.any(seed_regions < 0) or len(protected_regions) != len(required)):
            raise RuntimeError('種姿勢・保護対象姿勢を保持できない構築予算')
    if enable_cover and (np.any(seed_regions < 0) or len(protected_regions) != len(required)):
        raise RuntimeError('種姿勢・保護対象姿勢を保持できない構築予算')
    if enable_cover:
        builder.kernel.end_sec = start_sec+max_build_sec
        try:
            builder.merge()
            for idx in range(len(builder.nodes)):
                if builder.root(idx) == idx:
                    builder.widen(idx, max_num_growth_checks)
        except (TimeoutError, OverflowError):
            pass
    roots = sorted({builder.root(idx) for idx in range(len(builder.nodes))})
    compact = {idx: row for row, idx in enumerate(roots)}
    region_nodes = [builder.nodes[idx] for idx in roots]
    for row, idx in enumerate(seed_regions):
        if idx >= 0:
            seed_regions[row] = compact[builder.root(int(idx))]
    # 状態更新索引はボクセル→領域だけ。接続は被覆フラグ・端点・ルートのみ。
    entries = []
    has_index = np.zeros(len(region_nodes), dtype=bool)
    for row, node in enumerate(region_nodes):
        if len(entries)+len(node.data['outer']) <= max_num_relations:
            entries.extend((value, row, value in node.data['inner']) for value in node.data['outer'])
            has_index[row] = True
    entries.sort()
    voxels, starts = np.unique(np.array([entry[0] for entry in entries], dtype=np.uint64), return_index=True)
    links = builder.links
    connection_pairs = np.asarray([(compact[builder.root(a)], compact[builder.root(b)]) for a, b, *_ in links], dtype=np.int64).reshape(-1, 2)
    region_positions = np.asarray([node.data.get('reference', node.reference) for node in region_nodes], dtype=float).reshape(-1, positions.shape[1])
    vertices = np.concatenate([node.data['vertices'] for node in region_nodes]) if region_nodes else np.empty((0, positions.shape[1]))
    core_ids, core_pointers = flat_sets(region_nodes, 'core')
    occupied_ids, occupied_pointers = flat_sets(region_nodes, 'occupied')
    arrays = dict(node_ids=np.asarray(ids, dtype=np.int64), positions=positions, edges=pairs, seed_regions=seed_regions,
        region_positions=region_positions, vertices=vertices,
        vertex_pointers=np.r_[0, np.cumsum([len(node.data['vertices']) for node in region_nodes], dtype=np.int64)],
        min_internal=np.asarray([node.data['min_internal'] for node in region_nodes]), has_index=has_index,
        core_ids=core_ids, core_pointers=core_pointers, occupied_ids=occupied_ids, occupied_pointers=occupied_pointers,
        connection_pairs=connection_pairs, full_coverage=np.asarray([(a, b) for _, _, a, b, *_ in links], dtype=bool).reshape(-1, 2),
        connection_positions=np.asarray([[first, second] for *_, first, second in links], dtype=float).reshape(-1, 2, positions.shape[1]),
        route_connections=np.asarray([idx for route in routes for idx in route], dtype=np.int64),
        route_pointers=np.r_[0, np.cumsum([len(route) for route in routes], dtype=np.int64)],
        voxel_ids=voxels, pointers=np.r_[starts, len(entries)].astype(np.int64),
        entry_regions=np.asarray([entry[1] for entry in entries], dtype=np.int64),
        is_inner=np.asarray([entry[2] for entry in entries], dtype=bool))
    if enable_cover:
        arrays.update(joint_limits=geometry.limits.copy(), required_positions=required,
                      required_regions=np.asarray([compact[builder.root(idx)] for idx in protected_regions], dtype=np.int64),
                      edge_witness_gap=edge_witness_gap,
                      edge_internal_lower=edge_internal_lower, has_internal_bound=has_internal_bound,
                      narrow_centers=np.asarray([node.data['narrow_centers'] for node in region_nodes]),
                      narrow_radii=np.asarray([node.data['narrow_radii'] for node in region_nodes]),
                      narrow_supports=np.asarray([node.data['narrow_supports'] for node in region_nodes]), normals=builder.kernel.normals)
    if construction_method == 'quadratic_cover':
        arrays['generators'] = np.asarray([node.data['generators'] for node in region_nodes])
    metadata = dict(format='gng_region_vlut_v1', joint_names=geometry.joint_names,
        active_joint_names=[geometry.joint_names[idx] for idx in active], geometry_signature=geometry_signature(geometry),
        voxel_size=voxel_size, clearance_margin=clearance_margin, max_cell_radius=voxel_size*np.sqrt(3)/2,
        max_extra_cells_th=max_extra_cells_th, min_internal_gap=min_internal_gap, num_merges=builder.num_merges,
        num_evaluations=builder.kernel.num_evaluations, build_ms=(time.monotonic()-start_sec)*1000,
        region_type='convex_combination_of_joint_vertices', model_scope='ideal_sphere_geometry_static_environment',
        is_full_configuration_coverage=False, is_real_robot_certified=False)
    metadata['kernel_backend'] = builder.kernel.kernel_backend
    if enable_cover:
        metadata.update(construction_method=construction_method,
                        internal_policy=internal_policy, has_internal_certificates=True,
                        internal_certificate_path='joint_linear_segment', internal_certificate_ms=internal_certificate_ms,
                        has_implicit_residual=True, is_bidirectional_coverage=False,
                        safety_definition='clear_region_and_all_incident_routes',
                        has_point_refinement=True,
                        num_original_nodes=num_original_nodes, num_original_edges=num_original_edges)
    if construction_method == 'quadratic_cover':
        metadata['region_type'] = 'joint_ellipsoid_with_limits'
    metadata['content_signature'] = content_signature(metadata, arrays)
    return region_vlut(metadata, arrays)


def probe_environments(geometry, positions, edges, *, num_scenes=32, seed=0):
    """遠方・腕付近の球／棒状点配置。構築用と独立検証用の種の分離口。"""
    generator = np.random.default_rng(seed)
    scenes = []
    for idx in range(num_scenes):
        if idx % 4 == 0 or not len(edges):
            scenes.append(np.array([[5., 5., 5.]]))
            continue
        first, second = edges[int(generator.integers(len(edges)))]
        q = positions[first]+generator.uniform()*(positions[second]-positions[first])
        centers = geometry.centers(q)[geometry.is_arm]
        center = centers[int(generator.integers(len(centers)))]+generator.normal(0., .12, 3)
        direction = generator.normal(size=3)
        direction /= np.linalg.norm(direction)
        points = center+np.linspace(-.035, .035, 5)[:, None]*direction
        scenes.append(points)
    return scenes


def task_guard_poses(geometry, nodes, active_joint_names, *, max_num_guards=8, num_proposals=256, seed=0):
    """関節端・折畳み・手先位置／向きの低被覆候補。有限標本での保護姿勢選択。"""
    if type(max_num_guards) is not int or max_num_guards < 0 or type(num_proposals) is not int or num_proposals < 1:
        raise ValueError('手先保護候補数の整数条件の不正')
    if max_num_guards == 0:
        return np.empty((0, len(geometry.joint_names)))
    generator = np.random.default_rng(seed)
    active = [geometry.joint_names.index(name) for name in active_joint_names]
    original = np.asarray(list(nodes.values()), dtype=float)
    proposals = np.repeat(original[0][None], num_proposals+2*len(active), axis=0)
    proposals[:num_proposals, active] = generator.uniform(geometry.limits[active, 0], geometry.limits[active, 1], (num_proposals, len(active)))
    for row, idx in enumerate(active):
        proposals[num_proposals+2*row:num_proposals+2*row+2, idx] = geometry.limits[idx]
    gaps = geometry.internal_clearances(geometry.centers(proposals))
    proposals = proposals[np.min(gaps, axis=1) >= .005]
    if not len(proposals):
        return proposals
    children = {joint[6]: joint[2] for joint in geometry.joints if joint[6] in active}
    groups = {}
    for idx, child in children.items():
        path = []
        link = child
        while link in geometry.parents:
            path.append(link)
            link = geometry.parents[link]
        ancestors = [(joint_idx, name) for joint_idx, name in children.items() if name in path]
        root = max(ancestors, key=lambda item: path.index(item[1]))[0]
        if root not in groups or len(path) > groups[root][0]:
            groups[root] = len(path), geometry.link_indices[child]
    task_links = [value[1] for value in groups.values()]
    combined = np.vstack((original, proposals))
    transforms = geometry.link_transforms(combined)[:, task_links]
    position_scale = max(.1, float(np.max(np.linalg.norm(transforms[:, :, :3, 3], axis=2))))
    features = np.concatenate((transforms[:, :, :3, 3]/position_scale,
                               transforms[:, :, :3, :3].reshape(len(combined), len(task_links), 9)/np.sqrt(2)), axis=2).reshape(len(combined), -1)
    existing, candidate = features[:len(original)], features[len(original):]
    min_dist = np.min(np.sum((candidate[:, None]-existing[None])**2, axis=2), axis=1)
    chosen = []
    for _ in range(min(max_num_guards, len(proposals))):
        row = int(np.argmax(min_dist))
        if min_dist[row] <= 1e-12:
            break
        chosen.append(proposals[row])
        min_dist = np.minimum(min_dist, np.sum((candidate-candidate[row])**2, axis=1))
        min_dist[row] = -1.
    return np.asarray(chosen).reshape(-1, len(geometry.joint_names))


def evaluate_quality(model, geometry, scenes, *, min_gap=.01, min_internal_gap=.005, num_samples=65, cell_radius=0.):
    """同一姿勢グラフに対する接続保持・要確認・連続区間下界・資源量の評価。"""
    from scipy.spatial import cKDTree
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import shortest_path
    if num_samples < 2 or not len(scenes) or not len(model.arrays['edges']):
        raise ValueError('評価には環境配置と接続区間が必要')
    arrays = model.arrays
    positions, edges = arrays['positions'], arrays['edges']
    kernel = region_geometry(geometry, model.metadata['voxel_size'], model.metadata['clearance_margin'],
                             100000, time.monotonic()+60.)
    samples = positions[edges[:, 0], None]+np.linspace(0., 1., num_samples)[None, :, None]*(
        positions[edges[:, 1]]-positions[edges[:, 0]])[:, None]
    centers = geometry.centers(samples.reshape(-1, positions.shape[1])).reshape(len(edges), num_samples, -1, 3)
    movement = np.abs(positions[edges[:, 1]]-positions[edges[:, 0]])@kernel.global_bounds.T/(2*(num_samples-1))
    operation_coeffs = np.zeros((len(geometry.radii), len(kernel.operation_joints)))
    for sphere_idx, path in enumerate(kernel.paths):
        for operation, reach in path:
            operation_coeffs[sphere_idx, kernel.operation_indices[operation[1]]] = abs(operation[6])*(reach if operation[2] != 'prismatic' else 1.)
    first, second = geometry.self_pairs.T
    pair_coeffs = (operation_coeffs[first]+operation_coeffs[second])*kernel.unshared_operations
    pair_motion = np.abs(positions[edges[:, 1]]-positions[edges[:, 0]])[:, kernel.operation_joints]@pair_coeffs.T/(2*(num_samples-1))
    internal_lower = []
    for idx, item in enumerate(centers):
        margins = pair_motion[idx]
        for _ in geometry.environment_shapes:
            margins = np.r_[margins, movement[idx, geometry.is_arm]]
        internal_lower.append(float(np.min(geometry.internal_clearances(item)-margins)))
    internal_lower = np.asarray(internal_lower)
    internal_samples = np.asarray([float(np.min(geometry.internal_clearances(item))) for item in centers])
    costs = np.linalg.norm((positions[edges[:, 1]]-positions[edges[:, 0]])/np.diff(geometry.limits, axis=1).ravel(), axis=1)
    num_reference_clear = num_reference_blocked = num_reference_unknown = 0
    num_unknown_safe = num_false_clear = num_false_blocked = 0
    num_reference_pairs = num_retained_pairs = 0
    num_reference_protected_pairs = num_retained_protected_pairs = 0
    num_transition_violations = 0
    stretch_values, update_values, walk_values = [], [], []
    protected = arrays.get('required_positions', np.empty((0, positions.shape[1])))
    protected_keys = {q.astype('<f8').tobytes() for q in protected}
    is_protected = np.asarray([q.astype('<f8').tobytes() in protected_keys for q in positions])
    has_protected_endpoint = is_protected[:, None] | is_protected[None, :]

    def distances(mask):
        selected = edges[mask]
        values = costs[mask]
        rows = np.r_[selected[:, 0], selected[:, 1]]
        columns = np.r_[selected[:, 1], selected[:, 0]]
        graph = csr_matrix((np.r_[values, values], (rows, columns)), shape=(len(positions), len(positions)))
        return shortest_path(graph, directed=False)

    for points in scenes:
        tree = cKDTree(np.asarray(points, dtype=float))
        separation = tree.query(centers[:, :, geometry.is_arm].reshape(-1, 3))[0].reshape(len(edges), num_samples, -1)
        separation -= geometry.radii[geometry.is_arm]
        separation -= cell_radius
        is_reference_clear = ((np.min(separation-movement[:, None, geometry.is_arm], axis=(1, 2)) >= min_gap)
                              & (internal_lower >= min_internal_gap))
        is_reference_blocked = ((np.min(separation, axis=(1, 2)) < min_gap)
                                | (internal_samples < min_internal_gap))
        begin = time.perf_counter()
        model.update_cloud(tree)
        model.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)
        update_values.append((time.perf_counter()-begin)*1000)
        changed = model.stats['num_changed_voxels']
        walk_values.append(changed)
        answers = [model.query(geometry, positions[a], positions[b], tree, cell_radius, min_gap, min_internal_gap) for a, b in edges]
        is_index_clear = np.asarray([value is True for value in answers])
        is_index_blocked = np.asarray([value is False for value in answers])
        graph_states = model.graph_states(min_gap, min_internal_gap, cell_radius=cell_radius)
        source_owners = arrays['seed_regions'][edges]
        has_safe_owner = np.any((source_owners >= 0) & (graph_states[np.maximum(source_owners, 0)] == int(edge_state.clear)), axis=1)
        num_transition_violations += int((has_safe_owner & ~is_index_clear).sum())
        num_reference_clear += int(is_reference_clear.sum())
        num_reference_blocked += int(is_reference_blocked.sum())
        num_reference_unknown += int((~is_reference_clear & ~is_reference_blocked).sum())
        num_unknown_safe += int((is_reference_clear & ~is_index_clear & ~is_index_blocked).sum())
        num_false_clear += int((is_reference_blocked & is_index_clear).sum())
        num_false_blocked += int((is_reference_clear & is_index_blocked).sum())
        reference_dist = distances(is_reference_clear)
        index_dist = distances(is_index_clear)
        has_reference_pair = np.triu(np.isfinite(reference_dist), k=1)
        is_retained = has_reference_pair & np.isfinite(index_dist)
        num_reference_pairs += int(has_reference_pair.sum())
        num_retained_pairs += int(is_retained.sum())
        num_reference_protected_pairs += int((has_reference_pair & has_protected_endpoint).sum())
        num_retained_protected_pairs += int((is_retained & has_protected_endpoint).sum())
        is_valid = is_retained & (reference_dist > 0)
        stretch_values.extend((index_dist[is_valid]/reference_dist[is_valid]).tolist())
    if not num_reference_clear or not num_reference_pairs:
        raise ValueError('独立な安全参照区間・接続対が不足した評価条件')
    owners = model.locate(positions)
    protected_owners = model.locate(protected)
    result = dict(num_regions=len(arrays['region_positions']), num_relations=len(arrays['entry_regions']),
                num_constraint_values=int(arrays['generators'].size if 'generators' in arrays else arrays['vertices'].size),
                num_seed_poses=len(positions), num_uncovered_seed_poses=int((owners < 0).sum()),
                num_protected_poses=len(protected), num_uncovered_protected_poses=int((protected_owners < 0).sum()),
                num_reference_clear=num_reference_clear, num_reference_blocked=num_reference_blocked,
                num_reference_unknown=num_reference_unknown, num_unknown_safe=num_unknown_safe,
                unknown_safe_ratio=num_unknown_safe/num_reference_clear,
                num_false_clear=num_false_clear, num_false_blocked=num_false_blocked,
                num_transition_violations=num_transition_violations,
                num_reference_pairs=num_reference_pairs, num_retained_pairs=num_retained_pairs,
                reachability_ratio=num_retained_pairs/num_reference_pairs,
                num_reference_protected_pairs=num_reference_protected_pairs,
                num_retained_protected_pairs=num_retained_protected_pairs,
                max_path_stretch=max(stretch_values) if stretch_values else None,
                update_p95_ms=float(np.percentile(update_values, 95)),
                num_changed_voxels=sum(walk_values), build_ms=model.metadata['build_ms'],
                num_raw_bytes=sum(value.nbytes for value in arrays.values()), num_scenes=len(scenes),
                min_gap=min_gap, min_internal_gap=min_internal_gap, cell_radius=cell_radius)
    if num_reference_protected_pairs:
        result['protected_reachability_ratio'] = num_retained_protected_pairs/num_reference_protected_pairs
    return result


def build_efficient_index(geometry, nodes, edges, *, num_selection_scenes=24, selection_seed=13,
                          max_num_task_guards=8, min_reachability_ratio_th=.98, max_unknown_safe_ratio_th=.05, **options):
    """安全・被覆を必須条件とした、限定候補の接続性／要確認／資源Pareto選択。"""
    begin = time.monotonic()
    if not 0 <= min_reachability_ratio_th <= 1 or not 0 <= max_unknown_safe_ratio_th <= 1:
        raise ValueError('構築選択の到達性・要確認率条件の不正')
    budgets = sorted(set(max(1, int(options.get('max_extra_cells_th', 96))//divisor) for divisor in (4, 2, 1)))
    edges = list(edges)
    time_budget = options.get('max_build_sec', 60.)/len(budgets)
    active_names = options.get('active_joint_names') or geometry.joint_names
    guards = task_guard_poses(geometry, nodes, active_names, max_num_guards=max_num_task_guards, seed=selection_seed)
    required = np.asarray(options.get('required_poses', ())).reshape(-1, len(geometry.joint_names))
    options = dict(options, required_poses=np.vstack((required, guards)))
    candidates = []
    failures = []
    for budget in budgets:
        settings = dict(options, construction_method='occupancy_cover', max_extra_cells_th=budget, max_build_sec=time_budget)
        try:
            model = build_index(geometry, nodes, edges, **settings)
        except (RuntimeError, ValueError) as error:
            failures.append(dict(max_extra_cells_th=budget, reason=str(error)))
            continue
        scenes = probe_environments(geometry, model.arrays['positions'], model.arrays['edges'],
                                    num_scenes=num_selection_scenes, seed=selection_seed)
        metrics = evaluate_quality(model, geometry, scenes)
        if any(metrics[name] for name in ('num_false_clear', 'num_false_blocked', 'num_uncovered_seed_poses',
                                         'num_uncovered_protected_poses', 'num_transition_violations')):
            continue
        objective = (metrics['num_reference_protected_pairs']-metrics['num_retained_protected_pairs'],
                     metrics['num_reference_pairs']-metrics['num_retained_pairs'], metrics['num_unknown_safe'],
                     metrics['num_regions'], metrics['num_relations'])
        candidates.append((objective, model, metrics))
    if not candidates:
        raise RuntimeError('安全・種姿勢被覆条件を満たす構築候補なし')
    frontier = [item for item in candidates if not any(
        all(a <= b for a, b in zip(other[0], item[0])) and any(a < b for a, b in zip(other[0], item[0]))
        for other in candidates)]
    eligible = [item for item in frontier if item[2]['reachability_ratio'] >= min_reachability_ratio_th
                and item[2].get('protected_reachability_ratio', 1.) >= min_reachability_ratio_th
                and item[2]['unknown_safe_ratio'] <= max_unknown_safe_ratio_th]
    _, selected, metrics = (min(eligible, key=lambda item: (item[0][3], item[0][4], *item[0][:3])) if eligible else
                             min(frontier, key=lambda item: item[0]))
    metadata = dict(selected.metadata, selection_method='bounded_pareto', selection_seed=selection_seed,
                    selection_metrics=metrics, pareto_metrics=[item[2] for item in frontier],
                    selection_failures=failures, is_independent_quality_validation=False)
    metadata['num_task_guards'] = len(guards)
    metadata.update(can_meet_selection_targets=bool(eligible), min_reachability_ratio_th=min_reachability_ratio_th,
                    max_unknown_safe_ratio_th=max_unknown_safe_ratio_th, selection_build_ms=(time.monotonic()-begin)*1000,
                    candidate_metrics=[item[2] for item in candidates])
    metadata['content_signature'] = content_signature(metadata, selected.arrays)
    return region_vlut(metadata, selected.arrays)


class region_vlut:
    """領域占有の3値状態と、状態ペア／その列だけの遷移判定。専用辺占有なし。"""
    format_name = 'gng_region_vlut_v1'

    def __init__(self, metadata, arrays):
        self.metadata, self.arrays = metadata, arrays
        if metadata.get('format') != self.format_name or metadata.get('content_signature') != content_signature(metadata, arrays):
            raise ValueError('領域VLUTの形式・破損検査の不一致')
        self.validate()
        self.edge_lookup = {pose_key(arrays['positions'][a], arrays['positions'][b]): idx for idx, (a, b) in enumerate(arrays['edges'])}
        self.pose_owners = {q.astype('<f8').tobytes(): int(idx) for q, idx in zip(arrays['positions'], arrays['seed_regions']) if idx >= 0}
        self.counts = np.zeros((2, len(arrays['region_positions'])), dtype=np.int64)
        self.occupied, self.environment_key, self.has_environment = set(), None, False
        self.state_key, self.connection_states = None, None
        self.online_domains = {}
        self.environment_points, self.environment_point_ids = None, None
        self.point_key, self.point_clear = None, None
        self.domain_inverses = (np.linalg.pinv(arrays['generators'], rcond=1e-15)
                                if metadata['region_type'] == 'joint_ellipsoid_with_limits' else None)
        self.stats = dict(num_updates=0, num_registered_environments=0, num_fast_clear=0,
                          num_fast_blocked=0, num_fallback=0, num_changed_voxels=0, num_residual_queries=0,
                          num_online_domains=0, num_point_refinements=0, num_internal_checks_skipped=0,
                          num_internal_poses_skipped=0)

    def validate(self):
        arrays, metadata = self.arrays, self.metadata
        if metadata.get('region_type') not in ('convex_combination_of_joint_vertices', 'joint_ellipsoid_with_limits'):
            raise ValueError('領域VLUTの制約型の不一致')
        if sum(value.nbytes for value in arrays.values()) > 128*1024*1024:
            raise ValueError('領域VLUTのメモリ予算超過')
        num_joints = len(metadata['joint_names'])
        num_nodes = len(arrays['positions'])
        num_regions = len(arrays['region_positions'])
        num_edges = len(arrays['edges'])
        num_connections = len(arrays['connection_pairs'])
        for name, shape in (('positions', (num_nodes, num_joints)), ('region_positions', (num_regions, num_joints)),
                            ('vertices', (len(arrays['vertices']), num_joints)), ('edges', (num_edges, 2)),
                            ('node_ids', (num_nodes,)), ('seed_regions', (num_nodes,)), ('min_internal', (num_regions,)),
                            ('has_index', (num_regions,)), ('connection_pairs', (num_connections, 2)),
                            ('full_coverage', (num_connections, 2)), ('connection_positions', (num_connections, 2, num_joints))):
            if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                raise ValueError('領域VLUTの配列形式の不正: '+name)
        for name in ('has_index', 'full_coverage', 'is_inner'):
            if arrays[name].dtype != np.dtype(bool):
                raise ValueError('領域VLUTの真偽配列型の不正')
        for name in ('positions', 'region_positions', 'vertices', 'connection_positions', 'min_internal'):
            if not np.issubdtype(arrays[name].dtype, np.floating):
                raise ValueError('領域VLUTの実数配列型の不正')
        for name in ('node_ids', 'seed_regions', 'edges', 'connection_pairs', 'entry_regions', 'route_connections',
                     'vertex_pointers', 'core_pointers', 'occupied_pointers', 'pointers', 'route_pointers'):
            if not np.issubdtype(arrays[name].dtype, np.integer):
                raise ValueError('領域VLUTの整数配列型の不正')
        for name in ('voxel_ids', 'core_ids', 'occupied_ids'):
            if arrays[name].ndim != 1 or arrays[name].dtype != np.dtype(np.uint64):
                raise ValueError('領域VLUTのボクセル配列型の不正')
        for name, size in (('edges', num_nodes), ('connection_pairs', num_regions), ('entry_regions', num_regions),
                           ('route_connections', num_connections)):
            if np.any(arrays[name] < 0) or np.any(arrays[name] >= size):
                raise ValueError('領域VLUTの参照範囲の不正: '+name)
        if (np.any(arrays['seed_regions'] < -1) or np.any(arrays['seed_regions'] >= num_regions)
                or len(np.unique(arrays['node_ids'])) != num_nodes or np.any(arrays['node_ids'] < 0)
                or not np.all(arrays['full_coverage'].any(axis=1))):
            raise ValueError('領域VLUTのノード・被覆形式の不正')
        for name, size, total in (('vertex_pointers', num_regions, len(arrays['vertices'])),
                ('core_pointers', num_regions, len(arrays['core_ids'])), ('occupied_pointers', num_regions, len(arrays['occupied_ids'])),
                ('pointers', len(arrays['voxel_ids']), len(arrays['entry_regions'])),
                ('route_pointers', num_edges, len(arrays['route_connections']))):
            pointers = arrays[name]
            if pointers.shape != (size+1,) or pointers[0] != 0 or pointers[-1] != total or np.any(np.diff(pointers) < 0):
                raise ValueError('領域VLUTのCSR形式の不正: '+name)
        if (arrays['entry_regions'].ndim != 1 or arrays['is_inner'].shape != arrays['entry_regions'].shape
                or np.any(arrays['voxel_ids'][1:] <= arrays['voxel_ids'][:-1]) or np.any(np.diff(arrays['vertex_pointers']) < 1)):
            raise ValueError('領域VLUTの逆引き・頂点形式の不正')
        if metadata.get('has_internal_certificates', False):
            if (metadata.get('internal_certificate_path') != 'joint_linear_segment'
                    or metadata.get('internal_policy') not in ('coupled', 'separate')):
                raise ValueError('内部離隔証明の方式不一致')
            for name in ('edge_internal_lower', 'has_internal_bound'):
                if name not in arrays or arrays[name].shape != (num_edges,) or not np.isfinite(arrays[name]).all():
                    raise ValueError('内部離隔証明の配列形式の不正: '+name)
            if (arrays['has_internal_bound'].dtype != np.dtype(bool)
                    or not np.issubdtype(arrays['edge_internal_lower'].dtype, np.floating)
                    or np.any(arrays['edge_internal_lower'][arrays['has_internal_bound']]
                              > arrays['edge_witness_gap'][arrays['has_internal_bound']]+1e-9)):
                raise ValueError('内部離隔証明の下界・実姿勢証拠の不一致')
            for idx, (a, b) in enumerate(arrays['edges']):
                start, end = arrays['route_pointers'][idx:idx+2]
                points = arrays['connection_positions'][arrays['route_connections'][start:end]].reshape(-1, num_joints)
                if not len(points):
                    continue
                delta = arrays['positions'][b]-arrays['positions'][a]
                ratios = (points-arrays['positions'][a])@delta/max(float(delta@delta), 1e-30)
                if (np.any(ratios < -1e-12) or np.any(ratios > 1.+1e-12)
                        or np.max(np.abs(points-arrays['positions'][a]-ratios[:, None]*delta)) > 1e-12):
                    raise ValueError('内部離隔証明と保存経由区間の不一致')
        for name in ('voxel_size', 'max_cell_radius'):
            checked_positive(metadata[name], name)
        if not np.isfinite(metadata['clearance_margin']) or metadata['clearance_margin'] < 0:
            raise ValueError('領域VLUTの離隔設定の不正')
        if metadata['region_type'] == 'joint_ellipsoid_with_limits':
            for name, shape in (('generators', (num_regions, num_joints, num_joints)), ('joint_limits', (num_joints, 2)),
                                ('required_positions', (len(arrays['required_regions']), num_joints)),
                                ('required_regions', (len(arrays['required_positions']),)), ('edge_witness_gap', (num_edges,))):
                if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                    raise ValueError('二次領域の配列形式の不正: '+name)
            if (np.any(arrays['joint_limits'][:, 0] >= arrays['joint_limits'][:, 1])
                    or np.any(arrays['required_regions'] < 0) or np.any(arrays['required_regions'] >= num_regions)
                    or np.any(arrays['seed_regions'] < 0) or not arrays['full_coverage'].any(axis=1).all()):
                raise ValueError('二次領域の被覆・関節制約の不正')
            for poses, owners in ((arrays['positions'], arrays['seed_regions']),
                                  (arrays['required_positions'], arrays['required_regions'])):
                for q, idx in zip(poses, owners):
                    if not quadratic_contains(arrays['region_positions'][idx], arrays['generators'][idx], [q], arrays['joint_limits'])[0]:
                        raise ValueError('種姿勢または保護対象姿勢の包含欠落')
        elif metadata.get('has_implicit_residual', False):
            for name, shape in (('joint_limits', (num_joints, 2)), ('required_positions', (len(arrays['required_regions']), num_joints)),
                                ('required_regions', (len(arrays['required_positions']),)), ('edge_witness_gap', (num_edges,))):
                if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                    raise ValueError('被覆保護領域の配列形式の不正: '+name)
            if (np.any(arrays['seed_regions'] < 0) or np.any(arrays['required_regions'] < 0)
                    or np.any(arrays['required_regions'] >= num_regions)
                    or np.any(arrays['joint_limits'][:, 0] >= arrays['joint_limits'][:, 1])):
                raise ValueError('被覆保護領域の参照形式の不正')
            for poses, owners in ((arrays['positions'], arrays['seed_regions']), (arrays['required_positions'], arrays['required_regions'])):
                for q, idx in zip(poses, owners):
                    if not np.any(np.all(self.region_vertices(idx) == q, axis=1)):
                        raise ValueError('種姿勢または保護対象姿勢の包含欠落')
        if metadata.get('has_point_refinement', False):
            num_spheres = arrays['narrow_radii'].shape[1]
            for name, shape in (('narrow_radii', (num_regions, num_spheres)), ('narrow_centers', (num_regions, num_spheres, 3)),
                                ('narrow_supports', (num_regions, num_spheres, 26)), ('normals', (26, 3))):
                if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                    raise ValueError('点包絡細分の配列形式の不正: '+name)
            if np.any(arrays['narrow_radii'] < 0) or not np.allclose(np.linalg.norm(arrays['normals'], axis=1), 1., atol=1e-10, rtol=0.):
                raise ValueError('点包絡細分の半径の不正')

    def region_vertices(self, idx):
        start, end = self.arrays['vertex_pointers'][idx:idx+2]
        return self.arrays['vertices'][start:end]

    def constraints(self, idx):
        """q=q0+Gλ、λの非負条件と総和1による全関節の連動一次制約。"""
        reference = self.arrays['region_positions'][idx]
        if self.domain_inverses is not None:
            generators, inverse = self.arrays['generators'][idx], self.domain_inverses[idx]
            return dict(reference=reference.copy(), generators=generators.copy(), quadratic_matrix=inverse.T@inverse,
                        subspace_matrix=np.eye(len(reference))-generators@inverse, max_quadratic_value=1.,
                        min_internal=float(self.arrays['min_internal'][idx]),
                        min_position=self.arrays['joint_limits'][:, 0].copy(), max_position=self.arrays['joint_limits'][:, 1].copy())
        return dict(reference=reference.copy(), generators=(self.region_vertices(idx)-reference).T,
                    min_coeff=0., sum_coeff=1., min_internal=float(self.arrays['min_internal'][idx]))

    def locate(self, poses):
        """明示領域の包含検査。未包含は-1で、実現不能を意味しない結果。"""
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        num_joints = len(self.metadata['joint_names'])
        if poses.shape[1] != num_joints or not np.isfinite(poses).all():
            raise ValueError('包含検査の姿勢形式の不正')
        owners = np.full(len(poses), -1, dtype=np.int64)
        if self.domain_inverses is not None:
            for idx, (reference, inverse, generators) in enumerate(zip(self.arrays['region_positions'], self.domain_inverses,
                                                                       self.arrays['generators'])):
                pending = np.flatnonzero(owners < 0)
                if not len(pending):
                    break
                delta = poses[pending]-reference
                coefficients = delta@inverse.T
                has_member = ((np.linalg.norm(coefficients, axis=1) <= 1.+1e-9)
                              & (np.max(np.abs(delta-coefficients@generators.T), axis=1) <= 1e-10))
                owners[pending[has_member]] = idx
            limits = self.arrays['joint_limits']
            owners[np.any((poses < limits[:, 0]-1e-10) | (poses > limits[:, 1]+1e-10), axis=1)] = -1
            return owners
        from scipy.optimize import linprog
        for idx in range(len(self.arrays['region_positions'])):
            vertices = self.region_vertices(idx)
            min_position, max_position = vertices.min(axis=0), vertices.max(axis=0)
            pending = np.flatnonzero((owners < 0) & np.all(poses >= min_position-1e-10, axis=1)
                                    & np.all(poses <= max_position+1e-10, axis=1))
            equality = np.vstack(((vertices-vertices[0]).T, np.ones(len(vertices))))
            for row in pending:
                result = linprog(np.zeros(len(vertices)), A_eq=equality, b_eq=np.r_[poses[row]-vertices[0], 1.],
                                 bounds=(0., None), method='highs')
                if result.success and np.max(np.abs(equality@result.x-np.r_[poses[row]-vertices[0], 1.])) <= 1e-9:
                    owners[row] = idx
        return owners

    def residual_constraints(self):
        """関節範囲から明示領域の和集合を除いた暗黙残余。未登録姿勢の切捨てなし。"""
        if not self.metadata.get('has_implicit_residual', False):
            return None
        return dict(min_position=self.arrays['joint_limits'][:, 0].copy(),
                    max_position=self.arrays['joint_limits'][:, 1].copy(), excluded_region_ids=np.arange(len(self.counts[0])),
                    state=int(edge_state.unknown), has_index=False, has_feasibility_certificate=False)

    def constraints_for_pose(self, geometry, pose):
        """有効姿勢の連続領域または残余からの局所補完。予算超過時も点制約での保持。"""
        if not self.metadata.get('has_implicit_residual', False) or geometry_signature(geometry) != self.metadata['geometry_signature']:
            raise ValueError('局所補完の形状・二次領域条件の不一致')
        q = np.asarray(pose, dtype=float)
        if (q.shape != (len(geometry.joint_names),) or not np.isfinite(q).all()
                or np.any(q < geometry.limits[:, 0]) or np.any(q > geometry.limits[:, 1])):
            raise ValueError('局所補完の関節姿勢の不正')
        key = q.astype('<f8').tobytes()
        if key in self.online_domains:
            self.stats['num_residual_queries'] += 1
            return self.domain_constraints(self.online_domains[key])
        if not geometry.has_internal_clearance(geometry.centers(q), 0.):
            raise ValueError('局所補完の姿勢に自己・固定環境干渉あり')
        idx = int(self.locate([q])[0])
        if idx >= 0:
            self.pose_owners[key] = idx
            return self.constraints(idx)
        if key not in self.online_domains:
            active = [geometry.joint_names.index(name) for name in self.metadata['active_joint_names']]
            kernel = region_geometry(geometry, self.metadata['voxel_size'], self.metadata['clearance_margin'],
                                     100000, time.monotonic()+.1)
            _, jacobian, *_ = kernel.anchor(q)
            sensitivity = np.max(np.linalg.norm(jacobian[geometry.is_arm][:, :, active], axis=1), axis=0)
            widths = np.minimum(np.diff(geometry.limits[active], axis=1).ravel()*.001,
                                kernel.voxel_size*.02/np.maximum(sensitivity, 1e-12))
            reference, generators = fit_quadratic([q], widths, active)
            try:
                data = kernel.envelope(reference, [q], generators=generators)
            except (TimeoutError, OverflowError):
                generators = np.zeros_like(generators)
                data = dict(reference=q.copy(), generators=generators, has_index=False)
            inverse = np.linalg.pinv(generators, rcond=1e-15)
            data.update(quadratic_matrix=inverse.T@inverse, subspace_matrix=np.eye(len(q))-generators@inverse,
                        max_quadratic_value=1., min_position=geometry.limits[:, 0].copy(), max_position=geometry.limits[:, 1].copy())
            if len(self.online_domains) >= 32:
                self.online_domains.pop(next(iter(self.online_domains)))
            self.online_domains[key] = data
            self.stats['num_online_domains'] = len(self.online_domains)
        self.stats['num_residual_queries'] += 1
        return self.domain_constraints(self.online_domains[key])

    @staticmethod
    def domain_constraints(data):
        """局所補完の制約項目だけの複製。内部包絡・点索引の非公開。"""
        names = ('reference', 'generators', 'quadratic_matrix', 'subspace_matrix', 'max_quadratic_value',
                 'min_position', 'max_position', 'min_internal', 'has_index')
        return {name: data[name].copy() if isinstance(data[name], np.ndarray) else data[name]
                for name in names if name in data}

    def update(self, occupied_ids):
        self.environment_key = None
        self.state_key = None
        self.environment_points, self.environment_point_ids = None, None
        self.point_key, self.point_clear = None, None
        current = set(map(int, occupied_ids))
        arrays = self.arrays
        for values, sign in ((current-self.occupied, 1), (self.occupied-current, -1)):
            if not values or not len(arrays['voxel_ids']):
                continue
            values = np.asarray(sorted(values), dtype=np.uint64)
            positions = np.searchsorted(arrays['voxel_ids'], values)
            valid = positions < len(arrays['voxel_ids'])
            positions, values = positions[valid], values[valid]
            positions = positions[arrays['voxel_ids'][positions] == values]
            self.accumulate_counts(positions, sign)
        if np.any(self.counts < 0):
            raise ValueError('領域VLUTの接触カウンタ不整合')
        self.stats['num_changed_voxels'] = len(current ^ self.occupied)
        self.stats['num_updates'] += 1
        self.occupied, self.has_environment = current, True

    def accumulate_counts(self, positions, sign):
        """投稿形式ごとの差分集計部品。入力点群の登録はupdate_cloudの1回のみ。"""
        arrays = self.arrays
        accumulate = getattr(native_kernel(), 'gng_region_accumulate_counts', None)
        if accumulate is not None:
            answer = accumulate(len(positions), len(arrays['voxel_ids']), len(arrays['entry_regions']),
                len(self.counts[0]), np.ascontiguousarray(positions, dtype=np.int64),
                np.ascontiguousarray(arrays['pointers'], dtype=np.int64),
                np.ascontiguousarray(arrays['entry_regions'], dtype=np.int64),
                np.ascontiguousarray(arrays['is_inner']).view(np.uint8), sign, self.counts)
            if answer != 0:
                raise ValueError('C++領域VLUTの差分入力の不正')
            return
        for start in range(0, len(positions), 1024):
            indices = np.concatenate([np.arange(arrays['pointers'][idx], arrays['pointers'][idx+1]) for idx in positions[start:start+1024]])
            regions = arrays['entry_regions'][indices]
            self.counts[0] += sign*np.bincount(regions, minlength=len(self.counts[0]))
            self.counts[1] += sign*np.bincount(regions[arrays['is_inner'][indices]], minlength=len(self.counts[1]))

    def posting_regions(self, row):
        """1ボクセルに属する基本領域。階層方式では葉ビットからの局所復号。"""
        return self.arrays['entry_regions'][slice(*self.arrays['pointers'][row:row+2])]

    def update_cloud(self, cloud_tree):
        if self.environment_key is cloud_tree:
            return
        points = np.asarray(cloud_tree.data)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('領域照会の環境座標の不正')
        ids = packed_ids(np.floor(points/self.metadata['voxel_size']).astype(np.int64))
        self.update(ids)
        self.environment_points, self.environment_point_ids = points, ids
        self.environment_key = cloud_tree
        self.stats['num_registered_environments'] += 1

    def refine_points(self, min_gap, cell_radius):
        """粗索引の境界領域だけの点包絡検査。追加FK・入力膨張・再ボクセル化なし。"""
        key = min_gap, cell_radius
        if self.point_key == key:
            return self.point_clear
        is_clear = self.counts[0] == 0
        affected = np.flatnonzero(~is_clear & self.arrays['has_index'])
        if not len(affected):
            self.point_key, self.point_clear = key, is_clear
            return is_clear
        arrays = self.arrays
        rows = {int(idx): [] for idx in affected}
        saturated = set()
        positions = np.searchsorted(arrays['voxel_ids'], self.environment_point_ids)
        is_valid = positions < len(arrays['voxel_ids'])
        is_valid[is_valid] &= arrays['voxel_ids'][positions[is_valid]] == self.environment_point_ids[is_valid]
        for row in np.flatnonzero(is_valid):
            idx = positions[row]
            for owner in self.posting_regions(idx):
                owner = int(owner)
                if owner not in rows or owner in saturated:
                    continue
                rows[owner].append(row)
                if len(rows[owner]) > 128:
                    saturated.add(owner)
        for idx, point_rows in rows.items():
            if idx in saturated or not point_rows:
                continue
            is_clear[idx] = point_envelope_clear(self.environment_points[point_rows], arrays['narrow_centers'][idx],
                                              arrays['narrow_radii'][idx], arrays['narrow_supports'][idx],
                                              arrays['normals'], min_gap, cell_radius)
        self.stats['num_point_refinements'] += len(affected)
        self.point_key, self.point_clear = key, is_clear
        return is_clear

    def external_region_states(self, min_gap=0., *, cell_radius=None):
        """外部点群だけの3値判定。内部離隔との論理的な分離。"""
        if not np.isfinite(min_gap) or min_gap < 0:
            raise ValueError('領域状態の離隔条件の不正')
        states = np.full(len(self.counts[0]), int(edge_state.unknown), dtype=np.uint8)
        if not self.has_environment:
            return states
        valid = self.arrays['has_index']
        states[valid & (self.counts[1] > 0)] = int(edge_state.blocked)
        if min_gap <= self.metadata['clearance_margin']:
            has_clear = self.counts[0] == 0
            if self.metadata.get('has_point_refinement', False) and self.environment_points is not None:
                radius = self.metadata['max_cell_radius'] if cell_radius is None else cell_radius
                if not np.isfinite(radius) or radius < 0 or radius > self.metadata['max_cell_radius']+1e-10:
                    raise ValueError('点包絡細分の入力半径条件の不一致')
                has_clear = self.refine_points(min_gap, radius)
            states[valid & has_clear & (self.counts[1] == 0)] = int(edge_state.clear)
        return states

    def region_states(self, min_gap=0., min_internal_gap=0., *, cell_radius=None):
        """従来互換の外部環境・内部離隔の共通状態。"""
        if not np.isfinite(min_internal_gap) or min_internal_gap < 0:
            raise ValueError('領域状態の内部離隔条件の不正')
        states = self.external_region_states(min_gap, cell_radius=cell_radius)
        states[(states == int(edge_state.clear)) & (self.arrays['min_internal'] < min_internal_gap+1e-9)] = int(edge_state.unknown)
        return states

    def pair_states(self, states):
        """被覆端点のclearで通過、全領域blockedで棄却、その他は要確認。"""
        pairs = self.arrays['connection_pairs']
        states = np.asarray(states)
        if states.shape != (len(self.counts[0]),) or np.any(states < 0) or np.any(states > int(edge_state.blocked)):
            raise ValueError('領域状態ペアの入力形式の不正')
        values = states[pairs]
        results = np.full(len(pairs), int(edge_state.unknown), dtype=np.uint8)
        results[np.any((values == int(edge_state.clear)) & self.arrays['full_coverage'], axis=1)] = int(edge_state.clear)
        results[np.any(values == int(edge_state.blocked), axis=1)] = int(edge_state.blocked)
        return results

    def graph_states(self, min_gap=0., min_internal_gap=0., *, cell_radius=None):
        """領域安全と全接続通過の共通条件。安全―要確認の遷移保証用ラベル。"""
        raw = self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)
        states = raw.copy()
        connections = self.pair_states(raw)
        uncertain = self.arrays['connection_pairs'][connections != int(edge_state.clear)].ravel()
        uncertain = uncertain[states[uncertain] == int(edge_state.clear)]
        states[uncertain] = int(edge_state.unknown)
        for idx, (first, second) in enumerate(self.arrays['edges']):
            start, end = self.arrays['route_pointers'][idx:idx+2]
            can_pass = end > start and np.all(connections[self.arrays['route_connections'][start:end]] == int(edge_state.clear))
            if 'edge_witness_gap' in self.arrays and self.arrays['edge_witness_gap'][idx] < min_internal_gap:
                can_pass = False
            if not can_pass:
                owners = self.arrays['seed_regions'][[first, second]]
                owners = owners[owners >= 0]
                states[owners[states[owners] == int(edge_state.clear)]] = int(edge_state.unknown)
        return states

    def query(self, geometry, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap):
        return self.query_checks(geometry, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap).can_pass

    def query_checks(self, geometry, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap):
        """形状・全関節端点・要求余裕の同一照合による区間判定と内部離隔証明。"""
        if any(not np.isfinite(value) or value < 0 for value in (min_gap, min_internal_gap)):
            raise ValueError('領域照会の余裕距離の不正')
        if geometry_signature(geometry) != self.metadata['geometry_signature']:
            self.stats['num_fallback'] += 1
            return bridge_check()
        can_skip_internal = False
        edge_idx = self.edge_lookup.get(pose_key(first, second))
        if self.metadata.get('has_internal_certificates', False):
            if edge_idx is not None and self.arrays['has_internal_bound'][edge_idx]:
                can_skip_internal = bool(self.arrays['edge_internal_lower'][edge_idx] >= min_internal_gap+1e-9)
            if not can_skip_internal:
                owners = [self.pose_owners.get(np.asarray(q, dtype='<f8').tobytes(), -1) for q in (first, second)]
                if owners[0] >= 0 and owners[0] == owners[1]:
                    can_skip_internal = bool(self.arrays['min_internal'][owners[0]] >= min_internal_gap+1e-9)
        can_pass = self._query_validated(first, second, cloud_tree, cell_radius, min_gap, min_internal_gap,
                                       can_skip_internal=can_skip_internal)
        return bridge_check(can_pass, can_skip_internal)

    def _query_validated(self, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap, *, can_skip_internal):
        """照合済み形状の区間照会。全体内部証明時だけの外部環境単独判定。"""
        edge_idx = self.edge_lookup.get(pose_key(first, second))
        if (cloud_tree is None
                or not np.isfinite(cell_radius) or cell_radius < 0 or cell_radius > self.metadata['max_cell_radius']+1e-10
                or min_gap > self.metadata['clearance_margin']):
            self.stats['num_fallback'] += 1
            return None
        if 'edge_witness_gap' in self.arrays and edge_idx is not None and self.arrays['edge_witness_gap'][edge_idx] < min_internal_gap:
            self.stats['num_fast_blocked'] += 1
            return False
        if edge_idx is None:
            if self.metadata.get('has_implicit_residual', False):
                self.update_cloud(cloud_tree)
                # 局所包絡の生成は明示APIのみ。通常照会でのFK・セル再構築の抑制。
                owners = [self.pose_owners.get(np.asarray(q, dtype='<f8').tobytes(), -1) for q in (first, second)]
                states = self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)
                if owners[0] >= 0 and owners[0] == owners[1] and states[owners[0]] == int(edge_state.clear):
                    self.stats['num_fast_clear'] += 1
                    return True
                for q in (first, second):
                    data = self.online_domains.get(np.asarray(q, dtype='<f8').tobytes())
                    if (data is None or not data.get('has_index', True) or data['min_internal'] < min_internal_gap+1e-9
                            or not quadratic_contains(data['reference'], data['generators'], [first, second], self.arrays['joint_limits']).all()):
                        continue
                    if data['outer'].isdisjoint(self.occupied):
                        self.stats['num_fast_clear'] += 1
                        return True
                    rows = np.flatnonzero(np.isin(self.environment_point_ids, np.asarray(list(data['outer']), dtype=np.uint64)))
                    if len(rows) <= 128 and point_envelope_clear(self.environment_points[rows], data['narrow_centers'], data['narrow_radii'],
                                                                data['narrow_supports'], self.arrays['normals'], min_gap, cell_radius):
                        self.stats['num_fast_clear'] += 1
                        return True
            if self.domain_inverses is not None:
                self.update_cloud(cloud_tree)
                states = self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)
                for idx in np.flatnonzero(states == int(edge_state.clear)):
                    if quadratic_contains(self.arrays['region_positions'][idx], self.arrays['generators'][idx],
                                          [first, second], self.arrays['joint_limits']).all():
                        self.stats['num_fast_clear'] += 1
                        return True
            self.stats['num_fallback'] += 1
            return None
        start, end = self.arrays['route_pointers'][edge_idx:edge_idx+2]
        if start == end:
            self.stats['num_fallback'] += 1
            return None
        self.update_cloud(cloud_tree)
        key = min_gap, min_internal_gap, cell_radius, can_skip_internal
        if key != self.state_key:
            states = (self.external_region_states(min_gap, cell_radius=cell_radius) if can_skip_internal else
                      self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius))
            self.connection_states = self.pair_states(states)
            self.state_key = key
        pairs = self.connection_states[self.arrays['route_connections'][start:end]]
        if np.any(pairs == int(edge_state.blocked)):
            self.stats['num_fast_blocked'] += 1
            return False
        if np.all(pairs == int(edge_state.clear)):
            self.stats['num_fast_clear'] += 1
            return True
        self.stats['num_fallback'] += 1
        return None

    def save(self, path, *, allow_replace=False):
        save_index(self.metadata, self.arrays, path, allow_replace=allow_replace)

    @classmethod
    def load(cls, path):
        metadata, arrays = load_index(path)
        if cls is region_vlut and metadata.get('format') in ('urdf_vrm_v1', 'urdf_vrm_hierarchy_v1'):
            from urdf_vrm import urdf_hierarchy, urdf_vrm
            factory = urdf_hierarchy if metadata['format'] == 'urdf_vrm_hierarchy_v1' else urdf_vrm
            return factory(metadata, arrays)
        return cls(metadata, arrays)


def main():
    parser = argparse.ArgumentParser(description='3D占有増分による姿勢領域ノードの追加構築')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', default='')
    parser.add_argument('--urdf', default='')
    parser.add_argument('--avoidance-config', default='')
    parser.add_argument('--voxel-size', type=float, default=.05)
    parser.add_argument('--clearance-margin', type=float, default=.02)
    parser.add_argument('--max-extra-cells-th', type=int, default=96)
    parser.add_argument('--max-num-regions', type=int, default=256)
    parser.add_argument('--max-build-sec', type=float, default=60.)
    parser.add_argument('--construction-method', choices=('legacy', 'quadratic_cover', 'occupancy_cover', 'efficient'), default='legacy')
    parser.add_argument('--internal-policy', choices=('coupled', 'separate'), default='coupled')
    parser.add_argument('--min-internal-gap', type=float, default=.005)
    parser.add_argument('--required-poses', default='')
    parser.add_argument('--max-num-growth-checks', type=int, default=2)
    parser.add_argument('--max-num-task-guards', type=int, default=8)
    parser.add_argument('--min-reachability-ratio-th', type=float, default=.98)
    parser.add_argument('--max-unknown-safe-ratio-th', type=float, default=.05)
    parser.add_argument('--allow-replace', action='store_true')
    args = parser.parse_args()
    from dual_arm_avoidance_geometry import robot_geometry
    inputs = sorted(Path(args.input).rglob('edge_vlut_recipe.json')) if Path(args.input).is_dir() else [Path(args.input)]
    if not inputs or (len(inputs) > 1 and args.output):
        raise ValueError('構築レシピ未配置、または複数レシピへの単一出力指定')
    config = {}
    if args.avoidance_config:
        import yaml
        config = yaml.safe_load(Path(args.avoidance_config).read_text())['dual_arm_avoidance_demo']
    for filename in inputs:
        recipe = json.loads(filename.read_text())
        if recipe.get('format') != 'gng_edge_vlut_recipe_v1':
            raise ValueError('構築レシピの形式不一致')
        geometry = robot_geometry(args.urdf or recipe['urdf_path'], config.get('planning_groups'))
        active = [geometry.joint_names.index(name) for name in recipe['joint_names']]
        fixed = np.asarray([recipe.get('fixed_joints', {}).get(name, 0.) for name in geometry.joint_names])
        nodes = {}
        for node in recipe['nodes']:
            if type(node['id']) is not int or node['id'] < 0 or node['id'] in nodes or len(node['angles']) != len(active):
                raise ValueError('構築レシピのノード・角度の不正')
            q = fixed.copy()
            q[active] = node['angles']
            nodes[node['id']] = q
        builder = build_efficient_index if args.construction_method == 'efficient' else build_index
        options = (dict(max_num_task_guards=args.max_num_task_guards, min_reachability_ratio_th=args.min_reachability_ratio_th,
                        max_unknown_safe_ratio_th=args.max_unknown_safe_ratio_th) if args.construction_method == 'efficient'
                   else dict(construction_method=args.construction_method))
        model = builder(geometry, nodes, recipe['edges'], active_joint_names=recipe['joint_names'],
            voxel_size=args.voxel_size, clearance_margin=args.clearance_margin,
            max_extra_cells_th=args.max_extra_cells_th, max_num_regions=args.max_num_regions, max_build_sec=args.max_build_sec,
            max_num_growth_checks=args.max_num_growth_checks,
            internal_policy=args.internal_policy,
            min_internal_gap=args.min_internal_gap,
            required_poses=json.loads(Path(args.required_poses).read_text()) if args.required_poses else (), **options)
        model.save(args.output or filename.with_name('region_vlut.npz'), allow_replace=args.allow_replace)
        print(json.dumps(dict(num_regions=len(model.arrays['region_positions']), num_connections=len(model.arrays['connection_pairs']),
            num_covered_edges=int(np.count_nonzero(np.diff(model.arrays['route_pointers']))),
            internal_policy=model.metadata.get('internal_policy', 'coupled'),
            num_certified_internal_segments=int(np.count_nonzero(model.arrays['has_internal_bound'] &
                (model.arrays['edge_internal_lower'] >= args.min_internal_gap+1e-9)))
                if model.metadata.get('has_internal_certificates', False) else 0,
            num_relations=len(model.arrays['entry_regions']), num_merges=model.metadata['num_merges'], build_ms=model.metadata['build_ms'],
            selection_metrics=model.metadata.get('selection_metrics'), can_meet_selection_targets=model.metadata.get('can_meet_selection_targets'),
            num_protected_poses=len(model.arrays.get('required_positions', ())))))


if __name__ == '__main__':
    main()
