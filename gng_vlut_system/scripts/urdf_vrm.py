#!/usr/bin/env python3
"""URDF由来の連続角度箱と保守占有索引。未索引領域を保持した追加方式。"""
import argparse
import hashlib
import heapq
import json
from pathlib import Path
import time
from types import SimpleNamespace
import xml.etree.ElementTree as element_tree

import numpy as np

from dual_arm_avoidance_geometry import robot_geometry
from gng_edge_vlut import bridge_check, content_signature, edge_state, geometry_signature
from gng_region_vlut import flat_sets, region_geometry, region_vlut
from gng_region_vlut import native_kernel


def resolve_resource(value, relative_to):
    """設定ファイル・URDFのローカル解決。Docker共有パスの相対部分の保持。"""
    value = Path(value)
    candidates = [value, Path(relative_to)/value]
    suffix = str(value).removeprefix('/ros2_ws/src/')
    for parent in (Path(__file__).resolve().parent, *Path(__file__).resolve().parents):
        candidates.extend((parent/suffix, parent/'share'/'gng_vlut_system'/suffix, parent/'config'/value.name,
                           parent/'share'/'gng_vlut_system'/'config'/value.name))
    result = next((path.resolve() for path in candidates if path.is_file()), None)
    if result is None:
        raise FileNotFoundError(f'ローカル資源未配置: {value}')
    return result


def profile_groups(root, profiles):
    """URDFの接続に基づく独立関節列・可動リンク群。手首・mimic形状の省略なし。"""
    joints = root.findall('joint')
    parents = {joint.find('child').get('link'): joint for joint in joints}
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint.find('child').get('link'))
    groups = []
    for name, profile in profiles.items():
        link, chain, visited = profile['eef'], [], set()
        while link != profile['root']:
            if link not in parents or link in visited:
                raise ValueError(f'計画グループの経路未解決: {name}: {link}')
            visited.add(link)
            joint = parents[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        excluded = set(profile.get('dof_exclude', ()))
        names = [joint.get('name') for joint in reversed(chain)
                 if joint.get('type') != 'fixed' and joint.find('mimic') is None and joint.get('name') not in excluded]
        moving_roots = [joint.find('child').get('link') for joint in chain if joint.get('name') in names]
        moving, pending = set(), list(moving_roots)
        while pending:
            link = pending.pop()
            if link not in moving:
                moving.add(link)
                pending.extend(children.get(link, ()))
        groups.append(dict(name=name, joint_names=names, link_names=sorted(moving)))
    return groups


def strict_self_pairs(geometry, exclusion_pairs=()):
    """同一剛体・隣接剛体・明示除外だけを除いた自己干渉ペア。初期姿勢依存の除外なし。"""
    fixed_parents = {child: parent for _, parent, child, kind, *_ in geometry.joints if kind == 'fixed'}
    def body(link):
        while link in fixed_parents:
            link = fixed_parents[link]
        return link
    names = list(dict.fromkeys(body(name) for name, *_ in geometry.spheres))
    ids = {name: idx for idx, name in enumerate(names)}
    can_skip = np.eye(len(names), dtype=bool)
    for _, parent, child, *_ in geometry.joints:
        first, second = body(parent), body(child)
        if first in ids and second in ids:
            can_skip[ids[first], ids[second]] = can_skip[ids[second], ids[first]] = True
    for pair in exclusion_pairs:
        links = pair.split('|')
        if len(links) != 2 or any(link not in geometry.link_indices for link in links):
            raise ValueError('自己干渉の明示除外リンクの不正')
        first, second = body(links[0]), body(links[1])
        if first in ids and second in ids:
            can_skip[ids[first], ids[second]] = can_skip[ids[second], ids[first]] = True
    bodies = np.asarray([ids[body(name)] for name, *_ in geometry.spheres])
    first, second = np.tril_indices(len(bodies), -1)
    is_selected = ((geometry.is_arm[first] | geometry.is_arm[second])
                   & ~can_skip[bodies[first], bodies[second]])
    return np.column_stack((first[is_selected], second[is_selected])).astype(np.int64)


def read_geometry(urdf, groups, exclusion_pairs=(), floor_z=None):
    """構築・Viewer・検証で同じ形状・自己干渉規則の再現。"""
    environment = () if floor_z is None else (('plane', np.asarray([0., 0., floor_z]), np.eye(3), np.zeros(3)),)
    geometry = robot_geometry(str(urdf), groups, environment)
    if not len(geometry.radii) or not np.any(geometry.is_arm):
        raise ValueError('計画リンクの衝突形状・外接形状なし')
    if (not geometry.joint_names or geometry.limits.shape != (len(geometry.joint_names), 2)
            or not np.isfinite(geometry.limits).all() or np.any(np.diff(geometry.limits, axis=1) <= 0)
            or not np.isfinite(geometry.radii).all() or np.any(geometry.radii <= 0)
            or not np.isfinite(geometry.centers(np.zeros(len(geometry.joint_names)))).all()):
        raise ValueError('URDFの位置制限・形状・変換の不正')
    geometry.self_pairs = strict_self_pairs(geometry, exclusion_pairs)
    return geometry


def box_neighbors(lower, upper, active):
    """共通面の索引による隣接候補。代表姿勢間の通過保証とは別の接続。"""
    pairs = set()
    for axis in active:
        lower_faces = {}
        for idx, value in enumerate(lower[:, axis]):
            lower_faces.setdefault(float(value), []).append(idx)
        others = [idx for idx in active if idx != axis]
        for first, value in enumerate(upper[:, axis]):
            for second in lower_faces.get(float(value), ()):
                if first != second and np.all(np.minimum(upper[first, others], upper[second, others])
                                              > np.maximum(lower[first, others], lower[second, others])):
                    pairs.add(tuple(sorted((first, second))))
    return np.asarray(sorted(pairs), dtype=np.int64).reshape(-1, 2)


def segment_box_hits(first, second, lower, upper):
    """線形関節区間と複数箱の連続交差。関節角度サンプルの列挙なし。"""
    delta = second-first
    is_moving = delta != 0
    can_include = np.all((first[~is_moving] >= lower[:, ~is_moving]) &
                         (first[~is_moving] <= upper[:, ~is_moving]), axis=1)
    if not is_moving.any():
        return np.flatnonzero(can_include)
    a = (lower[:, is_moving]-first[is_moving])/delta[is_moving]
    b = (upper[:, is_moving]-first[is_moving])/delta[is_moving]
    enter = np.maximum(0., np.max(np.minimum(a, b), axis=1))
    leave = np.minimum(1., np.min(np.maximum(a, b), axis=1))
    return np.flatnonzero(can_include & (enter <= leave+1e-12))


def partition_boxes(kernel, lower, upper, seeds, active, *, max_num_parents,
                    max_num_leaves_per_parent, max_num_relations, max_extra_cells_th):
    """親分割・親内細分の共通木。床・自己干渉・予算超過による領域の削除なし。"""
    tree_lower, tree_upper, children = [lower], [upper], [[-1, -1]]
    widths = upper[active]-lower[active]
    motion = np.max(kernel.global_bounds[kernel.geometry.is_arm][:, active], axis=0)
    queue = []
    seed_members = {0: tuple(range(len(seeds)))}

    def offer(node, depth):
        lo, hi = tree_lower[node], tree_upper[node]
        nearest = np.clip(seeds[:, active], lo[active], hi[active])
        dist = float(np.min(np.sum(((nearest-seeds[:, active])/widths)**2, axis=1)))
        heapq.heappush(queue, (0 if seed_members[node] else 1, dist, -depth, node))

    def split(node, depth):
        lo, hi = tree_lower[node], tree_upper[node]
        half = (hi-lo)/2
        sensitivity = motion
        if has_hierarchy:
            # 球表面積と中心Jacobianによる占有変化量の低コスト近似。安全証明とは別の分割順位。
            _, jacobian, *_ = kernel.anchor((lo+hi)/2)
            selected = kernel.geometry.is_arm
            sensitivity = np.sum(kernel.geometry.radii[selected, None]**2*
                                 np.linalg.norm(jacobian[selected][:, :, active], axis=1), axis=0)
        scores = np.maximum(sensitivity*half[active], half[active]/widths*1e-12)
        axis = active[int(np.argmax(scores))]
        mid = (lo[axis]+hi[axis])/2
        if not lo[axis] < mid < hi[axis]:
            return ()
        first = len(tree_lower)
        children[node] = [first, first+1]
        for side in (0, 1):
            child_lo, child_hi = lo.copy(), hi.copy()
            if side == 0:
                child_hi[axis] = mid
            else:
                child_lo[axis] = mid
            tree_lower.append(child_lo)
            tree_upper.append(child_hi)
            children.append([-1, -1])
            seed_members[first+side] = tuple(idx for idx in seed_members[node]
                                            if (seeds[idx, axis] >= mid) == bool(side))
            offer(first+side, depth+1)
        return first, first+1

    offer(0, 0)
    has_hierarchy = max_num_leaves_per_parent > 1
    if has_hierarchy:
        # 親分割では占有生成なし。異なる細分条件での同一箱の再評価の抑制。
        while queue and (len(tree_lower)+1)//2 < max_num_parents and time.monotonic() < kernel.end_sec:
            _, _, negative_depth, node = heapq.heappop(queue)
            split(node, -negative_depth)
    parent_nodes = np.flatnonzero(np.asarray(children)[:, 0] < 0) if has_hierarchy else None
    node_parents = {int(node): idx for idx, node in enumerate(parent_nodes)} if has_hierarchy else {}
    num_parent_leaves = np.ones(len(parent_nodes), dtype=int) if has_hierarchy else None
    indexed, num_relations, num_evaluations = {}, 0, 0
    while queue and time.monotonic() < kernel.end_sec:
        _, _, negative_depth, node = heapq.heappop(queue)
        lo, hi = tree_lower[node], tree_upper[node]
        reference = (lo+hi)/2
        data = None
        try:
            num_evaluations += 1
            data = kernel.envelope(reference, [reference], box_half_width=(hi-lo)/2,
                                   max_num_occupied_cells=max_extra_cells_th+kernel.max_num_cells)
            if len(data['occupied']-data['core']) > max_extra_cells_th or num_relations+len(data['outer']) > max_num_relations:
                data = None
        except (OverflowError, TimeoutError):
            pass
        if data is not None:
            indexed[node] = data
            num_relations += len(data['outer'])
            continue
        parent = node_parents.get(node)
        has_capacity = (num_parent_leaves[parent] < max_num_leaves_per_parent if has_hierarchy else
                        (len(tree_lower)+1)//2 < max_num_parents)
        if not has_capacity or time.monotonic() >= kernel.end_sec:
            continue
        child_nodes = split(node, -negative_depth)
        if has_hierarchy and child_nodes:
            num_parent_leaves[parent] += 1
            node_parents.update({child: parent for child in child_nodes})
    return tree_lower, tree_upper, children, indexed, num_evaluations, parent_nodes, node_parents


def build_model(geometry, active_joint_names, *, fixed_positions=None, seed_poses=(),
                max_num_regions=128, max_build_sec=30., max_num_relations=500000,
                max_num_cells=16384, max_extra_cells_th=1024, voxel_size=.05, clearance_margin=.02,
                max_num_leaves_per_parent=1):
    """関節全組合せの格子なしの適応箱分割。時間・件数超過部分も残余として保持。"""
    if (type(max_num_regions) is not int or not 1 <= max_num_regions <= 2048
            or type(max_num_leaves_per_parent) is not int or not 1 <= max_num_leaves_per_parent <= 16
            or max_num_regions*max_num_leaves_per_parent > 2048
            or type(max_num_cells) is not int or max_num_cells < 1
            or type(max_extra_cells_th) is not int or max_extra_cells_th < 0
            or type(max_num_relations) is not int or max_num_relations < 1
            or not np.isfinite([max_build_sec, voxel_size, clearance_margin]).all()
            or min(max_build_sec, voxel_size) <= 0 or clearance_margin < 0):
        raise ValueError('URDF箱構築の予算・格子条件の不正')
    if not active_joint_names or len(set(active_joint_names)) != len(active_joint_names):
        raise ValueError('対象関節名の空集合・重複')
    active = [geometry.joint_names.index(name) for name in active_joint_names]
    fixed = (np.clip(np.zeros(len(geometry.joint_names)), geometry.limits[:, 0], geometry.limits[:, 1])
             if fixed_positions is None else np.asarray(fixed_positions, dtype=float))
    if fixed.shape != (len(geometry.joint_names),) or not np.isfinite(fixed).all() or np.any(
            (fixed < geometry.limits[:, 0]) | (fixed > geometry.limits[:, 1])):
        raise ValueError('基準姿勢の関節制限違反')
    seeds = np.asarray([fixed, *seed_poses], dtype=float)
    if seeds.ndim != 2 or seeds.shape[1] != len(fixed) or not np.isfinite(seeds).all():
        raise ValueError('優先姿勢の形式不正')
    inactive = np.asarray([idx for idx in range(len(fixed)) if idx not in active], dtype=np.int64)
    if np.any(seeds < geometry.limits[:, 0]) or np.any(seeds > geometry.limits[:, 1]) or np.any(seeds[:, inactive] != fixed[inactive]):
        raise ValueError('優先姿勢の関節制限・固定関節の不一致')
    lower, upper = fixed.copy(), fixed.copy()
    lower[active], upper[active] = geometry.limits[active, 0], geometry.limits[active, 1]
    if np.any(upper[active] <= lower[active]):
        raise ValueError('対象関節の位置範囲なし')
    begin = time.monotonic()
    kernel = region_geometry(geometry, voxel_size, clearance_margin, max_num_cells, begin+max_build_sec)
    kernel.enable_static_cache = True
    widths = upper[active]-lower[active]
    tree_lower, tree_upper, children, indexed, num_evaluations, parent_nodes, node_parents = partition_boxes(
        kernel, lower, upper, seeds, active, max_num_parents=max_num_regions,
        max_num_leaves_per_parent=max_num_leaves_per_parent, max_num_relations=max_num_relations,
        max_extra_cells_th=max_extra_cells_th)
    leaf_nodes = np.flatnonzero(np.asarray(children)[:, 0] < 0)
    lower, upper = np.asarray(tree_lower)[leaf_nodes], np.asarray(tree_upper)[leaf_nodes]
    references = (lower+upper)/2
    num_regions, num_joints, num_spheres = len(leaf_nodes), len(fixed), int(geometry.is_arm.sum())
    arrays = dict(box_lower=lower, box_upper=upper, tree_lower=np.asarray(tree_lower), tree_upper=np.asarray(tree_upper),
        tree_children=np.asarray(children, dtype=np.int64), leaf_nodes=leaf_nodes, active_indices=np.asarray(active, dtype=np.int64),
        joint_limits=geometry.limits.copy(), fixed_positions=fixed, positions=references, region_positions=references,
        node_ids=np.arange(num_regions), seed_regions=np.arange(num_regions), vertices=references,
        vertex_pointers=np.arange(num_regions+1), min_internal=np.full(num_regions, -1e6),
        has_index=np.asarray([int(node) in indexed for node in leaf_nodes], dtype=bool),
        narrow_centers=np.zeros((num_regions, num_spheres, 3)), narrow_radii=np.zeros((num_regions, num_spheres)),
        narrow_supports=np.zeros((num_regions, num_spheres, len(kernel.normals))), normals=kernel.normals)
    entries, nodes = [], []
    for idx, node in enumerate(leaf_nodes):
        data = indexed.get(int(node), dict(core=set(), occupied=set(), outer=set(), inner=set()))
        nodes.append(SimpleNamespace(data=data))
        if not arrays['has_index'][idx]:
            continue
        arrays['min_internal'][idx] = data['min_internal']
        for name in ('narrow_centers', 'narrow_radii', 'narrow_supports'):
            arrays[name][idx] = data[name]
        # 外接球は実メッシュの内包証明ではないため、URDF方式の衝突確定索引は空集合。
        entries.extend((voxel, idx) for voxel in data['outer'])
    entries.sort()
    ids = np.asarray([item[0] for item in entries], dtype=np.uint64)
    voxels, starts = np.unique(ids, return_index=True)
    arrays.update(voxel_ids=voxels, pointers=np.r_[starts, len(entries)].astype(np.int64),
        entry_regions=np.asarray([item[1] for item in entries], dtype=np.int64),
        is_inner=np.zeros(len(entries), dtype=bool))
    for name in ('core', 'occupied'):
        arrays[name+'_ids'], arrays[name+'_pointers'] = flat_sets(nodes, name)
    edges = box_neighbors(lower, upper, active)
    arrays.update(edges=edges, connection_pairs=edges.copy(), full_coverage=np.zeros((len(edges), 2), dtype=bool),
        route_connections=np.empty(0, dtype=np.int64), route_pointers=np.zeros(len(edges)+1, dtype=np.int64))
    volumes = np.prod((upper[:, active]-lower[:, active])/widths, axis=1)
    metadata = dict(format='urdf_vrm_v1', region_type='joint_box', construction_method='urdf_boxes',
        joint_names=geometry.joint_names, active_joint_names=list(active_joint_names), geometry_signature=geometry_signature(geometry),
        voxel_size=voxel_size, clearance_margin=clearance_margin, max_cell_radius=voxel_size*np.sqrt(3)/2,
        has_point_refinement=True, has_implicit_residual=True, internal_policy='strict_urdf_sphere_envelope',
        has_full_domain_partition=True, is_full_domain_indexed=bool(arrays['has_index'].all()),
        is_full_configuration_coverage=False,
        indexed_domain_volume_ratio=float(volumes[arrays['has_index']].sum()),
        num_evaluations=num_evaluations, max_build_sec=max_build_sec, kernel_backend=kernel.kernel_backend,
        max_num_regions=max_num_regions, max_num_relations=max_num_relations,
        max_num_cells=max_num_cells, max_extra_cells_th=max_extra_cells_th, build_ms=(time.monotonic()-begin)*1000,
        model_scope='urdf_outer_spheres_static_environment', is_real_robot_certified=False,
        continuous_joint_scope='finite_urdf_chart_outside_requires_fallback')
    if parent_nodes is not None:
        add_hierarchy(metadata, arrays, parent_nodes, node_parents, max_num_leaves_per_parent)
    metadata['content_signature'] = content_signature(metadata, arrays)
    factory = urdf_hierarchy if parent_nodes is not None else urdf_vrm
    return factory(metadata, arrays)


def add_hierarchy(metadata, arrays, parent_nodes, node_parents, max_num_leaves_per_parent):
    """親・葉の対応と葉ビット付きCSRへの圧縮。占有包絡の再生成なし。"""
    leaf_parents = np.asarray([node_parents[int(node)] for node in arrays['leaf_nodes']], dtype=np.int64)
    parent_groups = [np.flatnonzero(leaf_parents == idx) for idx in range(len(parent_nodes))]
    parent_leaves = np.concatenate(parent_groups).astype(np.int64)
    parent_pointers = np.r_[0, np.cumsum([len(group) for group in parent_groups])].astype(np.int64)
    local_bits = np.zeros(len(leaf_parents), dtype=np.uint16)
    for group in parent_groups:
        local_bits[group] = np.asarray([1 << idx for idx in range(len(group))], dtype=np.uint16)
    owners, bits, pointers = [], [], [0]
    for row in range(len(arrays['voxel_ids'])):
        grouped = {}
        for leaf in arrays['entry_regions'][slice(*arrays['pointers'][row:row+2])]:
            parent = int(leaf_parents[leaf])
            grouped[parent] = grouped.get(parent, 0) | int(local_bits[leaf])
        for parent, value in sorted(grouped.items()):
            owners.append(parent)
            bits.append(value)
        pointers.append(len(owners))
    arrays.update(parent_nodes=np.asarray(parent_nodes, dtype=np.int64), leaf_parents=leaf_parents,
        parent_leaf_pointers=parent_pointers, parent_leaf_regions=parent_leaves,
        entry_regions=np.asarray(owners, dtype=np.int64), entry_leaf_bits=np.asarray(bits, dtype=np.uint16),
        pointers=np.asarray(pointers, dtype=np.int64), is_inner=np.zeros(len(owners), dtype=bool))
    parent_edges = set()
    for first, second in arrays['edges']:
        pair = tuple(sorted((int(leaf_parents[first]), int(leaf_parents[second]))))
        if pair[0] != pair[1]:
            parent_edges.add(pair)
    arrays['parent_edges'] = np.asarray(sorted(parent_edges), dtype=np.int64).reshape(-1, 2)
    metadata.update(format='urdf_vrm_hierarchy_v1', construction_method='hierarchical_cover',
        has_parent_leaf_hierarchy=True, max_num_leaves_per_parent=max_num_leaves_per_parent,
        num_parents=len(parent_nodes), num_leaves=len(leaf_parents),
        has_point_refinement=True, split_metric='sphere_surface_jacobian',
        parent_scope='connected_joint_boxes_with_internal_linear_leaves')


class urdf_vrm(region_vlut):
    """既存の点群登録・差分集計を共用するURDF箱方式。追加の入力ボクセル化なし。"""
    format_name = 'urdf_vrm_v1'

    def validate(self):
        arrays, metadata = self.arrays, self.metadata
        num_regions, num_joints = len(arrays['region_positions']), len(metadata['joint_names'])
        if (metadata.get('region_type') != 'joint_box'
                or not 1 <= num_regions <= 2048 or not 1 <= num_joints <= 128
                or len(set(metadata['joint_names'])) != num_joints
                or sum(value.nbytes for value in arrays.values()) > 128*1024*1024):
            raise ValueError('URDF箱索引の件数・メモリ予算超過')
        active = arrays['active_indices']
        if (active.ndim != 1 or not len(active) or not np.issubdtype(active.dtype, np.integer)
                or len(np.unique(active)) != len(active) or np.any(active < 0) or np.any(active >= num_joints)
                or metadata['active_joint_names'] != [metadata['joint_names'][idx] for idx in active]
                or arrays['joint_limits'].shape != (num_joints, 2)
                or not np.isfinite(arrays['joint_limits']).all() or np.any(np.diff(arrays['joint_limits'], axis=1) <= 0)
                or arrays['fixed_positions'].shape != (num_joints,)
                or not np.isfinite(arrays['fixed_positions']).all()
                or np.any(arrays['fixed_positions'] < arrays['joint_limits'][:, 0])
                or np.any(arrays['fixed_positions'] > arrays['joint_limits'][:, 1])):
            raise ValueError('URDF箱の関節制約の不正')
        for name in ('voxel_size', 'max_cell_radius'):
            if not np.isfinite(metadata[name]) or metadata[name] <= 0:
                raise ValueError('URDF箱の格子条件の不正')
        if not np.isfinite(metadata['clearance_margin']) or metadata['clearance_margin'] < 0:
            raise ValueError('URDF箱の離隔条件の不正')
        for name in ('box_lower', 'box_upper', 'positions', 'region_positions'):
            if arrays[name].shape != (num_regions, num_joints) or not np.isfinite(arrays[name]).all():
                raise ValueError('URDF箱配列の不正: '+name)
        if np.any(arrays['box_lower'] > arrays['box_upper']):
            raise ValueError('URDF箱の上下限不正')
        lower, upper, children = arrays['tree_lower'], arrays['tree_upper'], arrays['tree_children']
        if (lower.shape != upper.shape or lower.ndim != 2 or lower.shape[1] != num_joints
                or len(lower) != 2*num_regions-1 or children.shape != (len(lower), 2)
                or not np.issubdtype(children.dtype, np.integer)
                or not np.isfinite(lower).all() or not np.isfinite(upper).all()):
            raise ValueError('分割木の形式不正')
        expected_lower, expected_upper = arrays['fixed_positions'].copy(), arrays['fixed_positions'].copy()
        expected_lower[active], expected_upper[active] = arrays['joint_limits'][active, 0], arrays['joint_limits'][active, 1]
        if not np.array_equal(lower[0], expected_lower) or not np.array_equal(upper[0], expected_upper):
            raise ValueError('構築対象窓と分割木の不一致')
        seen = {0}
        for node, pair in enumerate(children):
            if pair[0] == -1:
                if pair[1] != -1:
                    raise ValueError('分割木の葉形式不正')
                continue
            if np.any(pair <= node) or np.any(pair >= len(children)) or pair[0] == pair[1] or any(int(idx) in seen for idx in pair):
                raise ValueError('分割木の接続不正')
            seen.update(map(int, pair))
            first, second = map(int, pair)
            axes = np.flatnonzero(upper[first] != upper[node])
            if (len(axes) != 1 or not np.array_equal(lower[first], lower[node])
                    or not np.array_equal(upper[second], upper[node])):
                raise ValueError('分割木の領域欠落')
            axis = axes[0]
            expected = lower[node].copy()
            expected[axis] = upper[first, axis]
            if not lower[node, axis] < expected[axis] < upper[node, axis] or not np.array_equal(lower[second], expected):
                raise ValueError('分割木の切断位置不正')
        leaves = np.flatnonzero(children[:, 0] == -1)
        if (seen != set(range(len(lower))) or not np.array_equal(leaves, arrays['leaf_nodes'])
                or not np.array_equal(lower[leaves], arrays['box_lower']) or not np.array_equal(upper[leaves], arrays['box_upper'])):
            raise ValueError('分割木の未到達領域')
        if (arrays['has_index'].shape != (num_regions,) or arrays['has_index'].dtype != bool
                or arrays['min_internal'].shape != (num_regions,) or not np.isfinite(arrays['min_internal']).all()):
            raise ValueError('URDF箱状態の形式不正')
        if not np.array_equal(arrays['region_positions'], (arrays['box_lower']+arrays['box_upper'])/2):
            raise ValueError('URDF箱と基準姿勢の不一致')
        for name in ('edges', 'connection_pairs'):
            values = arrays[name]
            if (values.ndim != 2 or values.shape[1] != 2 or not np.issubdtype(values.dtype, np.integer)
                    or np.any(values < 0) or np.any(values >= num_regions)):
                raise ValueError('URDF箱の隣接候補形式不正')
        if (not np.array_equal(arrays['positions'], arrays['region_positions'])
                or not np.array_equal(arrays['seed_regions'], np.arange(num_regions))
                or not np.array_equal(arrays['connection_pairs'], arrays['edges'])
                or arrays['full_coverage'].shape != (len(arrays['edges']), 2) or arrays['full_coverage'].any()
                or len(arrays['route_connections']) or not np.array_equal(arrays['route_pointers'], np.zeros(len(arrays['edges'])+1))):
            raise ValueError('URDF箱の未保証接続への通過証明付与')
        self.validate_postings(num_regions)
        for name in ('core', 'occupied'):
            offsets, values = arrays[name+'_pointers'], arrays[name+'_ids']
            if (offsets.shape != (num_regions+1,) or not np.issubdtype(offsets.dtype, np.integer)
                    or values.ndim != 1 or values.dtype != np.uint64
                    or offsets[0] != 0 or offsets[-1] != len(values) or np.any(np.diff(offsets) < 0)):
                raise ValueError('保存占有の形式不正')
        if arrays['narrow_radii'].ndim != 2:
            raise ValueError('URDF点包絡の半径配列の不正')
        num_spheres = arrays['narrow_radii'].shape[1]
        for name, shape in (('narrow_centers', (num_regions, num_spheres, 3)),
                            ('narrow_radii', (num_regions, num_spheres)),
                            ('narrow_supports', (num_regions, num_spheres, len(arrays['normals'])))):
            if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                raise ValueError('URDF点包絡の形式不正')
        if (np.any(arrays['narrow_radii'] < 0) or arrays['normals'].ndim != 2 or arrays['normals'].shape[1] != 3
                or not len(arrays['normals']) or not np.isfinite(arrays['normals']).all()
                or not np.allclose(np.linalg.norm(arrays['normals'], axis=1), 1., atol=1e-10, rtol=0.)):
            raise ValueError('URDF点包絡の半径・方向の不正')

    def validate_postings(self, num_regions):
        """単層CSRの検証。階層方式では同じ形状検証と別の投稿形式を使用。"""
        arrays = self.arrays
        ids, pointers, owners = arrays['voxel_ids'], arrays['pointers'], arrays['entry_regions']
        if (ids.ndim != 1 or owners.ndim != 1 or ids.dtype != np.uint64
                or np.any(ids[1:] <= ids[:-1]) or pointers.shape != (len(ids)+1,)
                or pointers[0] != 0 or pointers[-1] != len(owners) or np.any(pointers[1:] < pointers[:-1])
                or not np.issubdtype(pointers.dtype, np.integer) or not np.issubdtype(owners.dtype, np.integer)
                or np.any(owners < 0) or np.any(owners >= num_regions)
                or arrays['is_inner'].shape != owners.shape or arrays['is_inner'].dtype != bool or arrays['is_inner'].any()):
            raise ValueError('URDF箱の逆引き形式不正')
        if np.any(~arrays['has_index'][owners]):
            raise ValueError('未索引領域への関係登録')

    def region_vertices(self, idx):
        """表示用の軸方向有限例。箱の全頂点・全姿勢の列挙ではない値。"""
        reference = self.arrays['region_positions'][idx]
        half = (self.arrays['box_upper'][idx]-self.arrays['box_lower'][idx])/2
        directions = np.diag(half)[self.arrays['active_indices']]
        return np.vstack((reference, reference+directions, reference-directions))

    def constraints(self, idx):
        return dict(reference=self.arrays['region_positions'][idx].copy(),
                    min_position=self.arrays['box_lower'][idx].copy(), max_position=self.arrays['box_upper'][idx].copy(),
                    has_index=bool(self.arrays['has_index'][idx]), min_internal=float(self.arrays['min_internal'][idx]))

    def locate(self, poses):
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        if poses.shape[1] != len(self.metadata['joint_names']) or not np.isfinite(poses).all():
            raise ValueError('包含姿勢の形式不正')
        owners = np.full(len(poses), -1, dtype=np.int64)
        for idx, (lower, upper) in enumerate(zip(self.arrays['box_lower'], self.arrays['box_upper'])):
            is_inside = (owners < 0) & np.all((poses >= lower) & (poses <= upper), axis=1)
            owners[is_inside] = idx
        return owners

    def residual_constraints(self):
        return [self.constraints(int(idx)) for idx in np.flatnonzero(~self.arrays['has_index'])]

    def constraints_for_pose(self, geometry, pose):
        if geometry_signature(geometry) != self.metadata['geometry_signature']:
            raise ValueError('URDF箱制約の形状不一致')
        q = np.asarray(pose, dtype=float)
        owners = self.locate(q)
        if owners[0] >= 0:
            return self.constraints(int(owners[0]))
        # 対象窓外の姿勢も、未検査の点制約としての保持。局所FK・占有再構築なし。
        return dict(reference=q.copy(), min_position=q.copy(), max_position=q.copy(), has_index=False,
                    has_feasibility_certificate=False, state=int(edge_state.unknown))

    def segment_regions(self, first, second):
        """線形関節区間と箱の連続交差。離散姿勢サンプル・辺占有索引なし。"""
        first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
        if (first.shape != (len(self.metadata['joint_names']),) or second.shape != first.shape
                or not np.isfinite(first).all() or not np.isfinite(second).all()):
            raise ValueError('区間端点の形式不正')
        if np.any(self.locate([first, second]) < 0):
            return None
        return segment_box_hits(first, second, self.arrays['box_lower'], self.arrays['box_upper'])

    def query_checks(self, geometry, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap):
        if (not np.isfinite([cell_radius, min_gap, min_internal_gap]).all() or min(cell_radius, min_gap, min_internal_gap) < 0):
            raise ValueError('URDF照会の離隔条件不正')
        if geometry_signature(geometry) != self.metadata['geometry_signature'] or cloud_tree is None:
            self.stats['num_fallback'] += 1
            return bridge_check()
        regions = self.segment_regions(first, second)
        if regions is None or not len(regions):
            self.stats['num_fallback'] += 1
            return bridge_check()
        can_skip_internal = bool(np.all(self.arrays['has_index'][regions]) and
            np.all(self.arrays['min_internal'][regions] >= min_internal_gap+1e-9))
        if cell_radius > self.metadata['max_cell_radius'] or min_gap > self.metadata['clearance_margin']:
            return bridge_check(None, can_skip_internal)
        self.update_cloud(cloud_tree)
        states = self.external_region_states(min_gap, cell_radius=cell_radius)
        if can_skip_internal and np.all(states[regions] == int(edge_state.clear)):
            self.stats['num_fast_clear'] += 1
            return bridge_check(True, True)
        self.stats['num_fallback'] += 1
        return bridge_check(None, can_skip_internal)


class urdf_hierarchy(urdf_vrm):
    """親と内部葉の階層。床・自己干渉の証明は全照会で共通の葉状態に集約。"""
    format_name = 'urdf_vrm_hierarchy_v1'

    def validate_postings(self, num_regions):
        arrays, metadata = self.arrays, self.metadata
        roots, leaf_parents = arrays['parent_nodes'], arrays['leaf_parents']
        offsets, regions = arrays['parent_leaf_pointers'], arrays['parent_leaf_regions']
        num_parents = len(roots)
        if (not 1 <= num_parents <= metadata['max_num_regions'] or metadata.get('num_parents') != num_parents
                or metadata.get('num_leaves') != num_regions
                or not 2 <= metadata.get('max_num_leaves_per_parent', 0) <= 16):
            raise ValueError('親・内部葉の件数予算不正')
        for values in (roots, leaf_parents, offsets, regions):
            if values.ndim != 1 or not np.issubdtype(values.dtype, np.integer):
                raise ValueError('親・葉対応の整数形式不正')
        if (offsets.shape != (num_parents+1,) or offsets[0] != 0 or offsets[-1] != num_regions
                or np.any(np.diff(offsets) < 1) or np.any(np.diff(offsets) > metadata['max_num_leaves_per_parent'])
                or leaf_parents.shape != (num_regions,) or np.any(leaf_parents < 0) or np.any(leaf_parents >= num_parents)
                or not np.array_equal(np.sort(regions), np.arange(num_regions))
                or np.any(roots < 0) or np.any(roots >= len(arrays['tree_children']))):
            raise ValueError('親・葉対応の欠落・重複')
        node_parents = np.full(len(arrays['tree_children']), -1, dtype=np.int64)
        for parent, root in enumerate(roots):
            pending = [int(root)]
            while pending:
                node = pending.pop()
                if node_parents[node] >= 0:
                    raise ValueError('親領域の木の重複')
                node_parents[node] = parent
                if arrays['tree_children'][node, 0] >= 0:
                    pending.extend(map(int, arrays['tree_children'][node]))
            if np.any(leaf_parents[regions[offsets[parent]:offsets[parent+1]]] != parent):
                raise ValueError('親の内部葉一覧の不一致')
        if not np.array_equal(node_parents[arrays['leaf_nodes']], leaf_parents):
            raise ValueError('親と葉の連続領域の不一致')
        edges = arrays['parent_edges']
        if (edges.ndim != 2 or edges.shape[1] != 2 or not np.issubdtype(edges.dtype, np.integer)
                or np.any(edges < 0) or np.any(edges >= num_parents)):
            raise ValueError('親の隣接候補の形式不正')
        ids, pointers, owners, bits = (arrays[name] for name in ('voxel_ids', 'pointers', 'entry_regions', 'entry_leaf_bits'))
        if (ids.ndim != 1 or ids.dtype != np.uint64 or np.any(ids[1:] <= ids[:-1])
                or pointers.shape != (len(ids)+1,) or not np.issubdtype(pointers.dtype, np.integer)
                or pointers[0] != 0 or pointers[-1] != len(owners) or np.any(np.diff(pointers) < 0)
                or owners.ndim != 1 or not np.issubdtype(owners.dtype, np.integer)
                or np.any(owners < 0) or np.any(owners >= num_parents)
                or bits.dtype != np.uint16 or bits.shape != owners.shape or np.any(bits == 0)
                or arrays['is_inner'].shape != owners.shape or arrays['is_inner'].dtype != bool or arrays['is_inner'].any()):
            raise ValueError('親・葉ビット逆引きの形式不正')
        for row in range(len(ids)):
            group = owners[slice(*pointers[row:row+2])]
            if np.any(group[1:] <= group[:-1]):
                raise ValueError('同一セルの親IDの重複・順序不正')
        for owner, value in zip(owners, bits):
            group = regions[offsets[owner]:offsets[owner+1]]
            if int(value) >> len(group) or any(not arrays['has_index'][group[bit]]
                    for bit in range(len(group)) if (int(value) >> bit) & 1):
                raise ValueError('未索引葉・範囲外葉への投稿登録')

    def posting_regions(self, row):
        arrays = self.arrays
        leaves = []
        for entry in range(*arrays['pointers'][row:row+2]):
            parent, bits = arrays['entry_regions'][entry], int(arrays['entry_leaf_bits'][entry])
            start = arrays['parent_leaf_pointers'][parent]
            while bits:
                bit = (bits & -bits).bit_length()-1
                leaves.append(int(arrays['parent_leaf_regions'][start+bit]))
                bits &= bits-1
        return leaves

    def accumulate_counts(self, positions, sign):
        arrays = self.arrays
        accumulate = getattr(native_kernel(), 'gng_region_accumulate_leaf_counts', None)
        if accumulate is not None:
            answer = accumulate(len(positions), len(arrays['voxel_ids']), len(arrays['entry_regions']),
                len(arrays['parent_nodes']), len(self.counts[0]), np.ascontiguousarray(positions, dtype=np.int64),
                arrays['pointers'], arrays['entry_regions'], arrays['entry_leaf_bits'],
                arrays['parent_leaf_pointers'], arrays['parent_leaf_regions'], sign, self.counts)
            if answer != 0:
                raise ValueError('C++親・葉カウンタの差分入力不正')
            return
        for row in positions:
            np.add.at(self.counts[0], self.posting_regions(int(row)), sign)

    def locate(self, poses):
        """全域分割木による所属。未索引・静的境界の葉も有効な担当として保持。"""
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        arrays = self.arrays
        if poses.shape[1] != len(self.metadata['joint_names']) or not np.isfinite(poses).all():
            raise ValueError('包含姿勢の形式不正')
        owners = np.full(len(poses), -1, dtype=np.int64)
        pending = np.flatnonzero(np.all((poses >= arrays['tree_lower'][0]) & (poses <= arrays['tree_upper'][0]), axis=1))
        nodes = np.zeros(len(pending), dtype=np.int64)
        lookup = np.full(len(arrays['tree_children']), -1, dtype=np.int64)
        lookup[arrays['leaf_nodes']] = np.arange(len(arrays['leaf_nodes']))
        while len(pending):
            children = arrays['tree_children'][nodes]
            is_leaf = children[:, 0] < 0
            owners[pending[is_leaf]] = lookup[nodes[is_leaf]]
            pending, nodes, children = pending[~is_leaf], nodes[~is_leaf], children[~is_leaf]
            if not len(pending):
                break
            axes = np.argmax(arrays['tree_upper'][children[:, 0]] != arrays['tree_upper'][nodes], axis=1)
            cuts = arrays['tree_upper'][children[:, 0], axes]
            side = (poses[pending, axes] >= cuts).astype(np.int64)
            nodes = children[np.arange(len(children)), side]
        return owners

    def parent_states(self, min_gap=0., min_internal_gap=0., *, cell_radius=None):
        """親全体の状態。親の要確認による安全な内部葉の破棄なし。"""
        states = self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)
        values = np.full(len(self.arrays['parent_nodes']), int(edge_state.unknown), dtype=np.uint8)
        for parent in range(len(values)):
            group = self.arrays['parent_leaf_regions'][slice(*self.arrays['parent_leaf_pointers'][parent:parent+2])]
            if np.all(states[group] == int(edge_state.clear)):
                values[parent] = int(edge_state.clear)
            elif np.all(states[group] == int(edge_state.blocked)):
                values[parent] = int(edge_state.blocked)
        return values

    def segment_regions(self, first, second):
        """親の粗交差から、該当する内部葉だけの連続交差。入力点群の再登録なし。"""
        first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
        num_joints = len(self.metadata['joint_names'])
        if (first.shape != (num_joints,) or second.shape != first.shape
                or not np.isfinite(first).all() or not np.isfinite(second).all()):
            raise ValueError('区間端点の形式不正')
        if np.any(self.locate([first, second]) < 0):
            return None
        arrays = self.arrays
        roots = arrays['parent_nodes']
        parents = segment_box_hits(first, second, arrays['tree_lower'][roots], arrays['tree_upper'][roots])
        if not len(parents):
            return np.empty(0, dtype=np.int64)
        leaves = np.concatenate([arrays['parent_leaf_regions'][slice(*arrays['parent_leaf_pointers'][idx:idx+2])]
                                 for idx in parents])
        hits = segment_box_hits(first, second, arrays['box_lower'][leaves], arrays['box_upper'][leaves])
        return np.sort(leaves[hits])

    def parent_constraints(self, parent_idx):
        """親内の一次制約成分。安全部分の和集合を凸包に置換しないAPI。"""
        if not 0 <= parent_idx < len(self.arrays['parent_nodes']):
            raise ValueError('親IDの範囲外')
        leaves = self.arrays['parent_leaf_regions'][slice(*self.arrays['parent_leaf_pointers'][parent_idx:parent_idx+2])]
        return dict(parent_idx=int(parent_idx), aggregation='union',
                    leaf_regions=leaves.copy(), components=[self.constraints(int(idx)) for idx in leaves])

    def linear_constraints(self, leaf_idx, *, min_gap=0., min_internal_gap=0., cell_radius=None):
        """選択した凸葉のQP向けA・上下限。環境未受信・静的未証明時は要確認。"""
        if not 0 <= leaf_idx < len(self.counts[0]):
            raise ValueError('葉IDの範囲外')
        values = self.constraints(leaf_idx)
        values.update(matrix=np.eye(len(self.metadata['joint_names'])),
            min_value=values['min_position'].copy(), max_value=values['max_position'].copy(),
            parent_idx=int(self.arrays['leaf_parents'][leaf_idx]),
            state=int(self.region_states(min_gap, min_internal_gap, cell_radius=cell_radius)[leaf_idx]))
        return values


def main():
    parser = argparse.ArgumentParser(description='実URDFからの予算付き連続箱・占有索引構築。実機出力なし')
    parser.add_argument('--params-file', default='')
    parser.add_argument('--avoidance-config', default='')
    parser.add_argument('--urdf', default='')
    parser.add_argument('--group', default='left_arm')
    parser.add_argument('--joint-names', nargs='+')
    parser.add_argument('--output', default='')
    parser.add_argument('--max-num-regions', type=int, default=128)
    parser.add_argument('--construction-method', choices=('hierarchical_cover', 'urdf_boxes'), default='hierarchical_cover')
    parser.add_argument('--max-num-leaves-per-parent', type=int, default=16)
    parser.add_argument('--max-build-sec', type=float, default=30.)
    parser.add_argument('--max-extra-cells-th', type=int, default=1024)
    parser.add_argument('--max-num-relations', type=int, default=500000)
    parser.add_argument('--voxel-size', type=float, default=.05)
    parser.add_argument('--clearance-margin', type=float, default=.02)
    parser.add_argument('--floor-z', type=float)
    parser.add_argument('--allow-replace', action='store_true')
    args = parser.parse_args()
    if args.construction_method == 'hierarchical_cover' and not 2 <= args.max_num_leaves_per_parent <= 16:
        parser.error('階層方式の内部葉上限は2〜16個の指定が必要')
    if args.avoidance_config and args.floor_z is not None:
        parser.error('--avoidance-configと--floor-zの同時指定不可')
    import yaml
    config, config_path = {}, None
    if args.params_file or not args.urdf:
        config_path = resolve_resource(args.params_file or 'topo_dual_arm_max_long.yaml', Path(__file__).parent.parent/'config')
        config = yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
    urdf = resolve_resource(args.urdf or config['urdf_path'], config_path.parent if config_path else Path.cwd())
    root = element_tree.parse(urdf).getroot()
    groups = profile_groups(root, config.get('gng', {}).get('profiles', {})) if not args.joint_names else []
    if args.joint_names or not groups:
        active = args.joint_names or [joint.get('name') for joint in root.findall('joint')
                                     if joint.get('type') != 'fixed' and joint.find('mimic') is None]
        moving = {joint.find('child').get('link') for joint in root.findall('joint') if joint.get('name') in active}
        changed = True
        while changed:
            previous = len(moving)
            moving.update(joint.find('child').get('link') for joint in root.findall('joint') if joint.find('parent').get('link') in moving)
            changed = len(moving) != previous
        groups = [dict(name='selected', joint_names=active, link_names=sorted(moving))]
    else:
        active = next((group['joint_names'] for group in groups if group['name'] == args.group), None)
        if active is None:
            parser.error('計画グループ未配置')
    exclusions = config.get('collision', {}).get('self_collision_exclusion_pairs', ())
    policy = 'adjacent_rigid_bodies_and_explicit_pairs_only'
    avoidance_path = None
    if args.avoidance_config:
        avoidance_path = resolve_resource(args.avoidance_config, Path(__file__).parent.parent/'config')
        avoidance = yaml.safe_load(avoidance_path.read_text())['dual_arm_avoidance_demo']
        groups = avoidance.get('planning_groups')
        # 回避実行系と同じ既存の固定環境・監視リンク・自己干渉規則。
        geometry = robot_geometry(str(urdf), groups)
        exclusions = ()
        policy = 'runtime_existing_policy'
    else:
        geometry = read_geometry(urdf, groups, exclusions, args.floor_z)
    model = build_model(geometry, active, max_num_regions=args.max_num_regions, max_build_sec=args.max_build_sec,
        max_num_relations=args.max_num_relations, max_extra_cells_th=args.max_extra_cells_th,
        voxel_size=args.voxel_size, clearance_margin=args.clearance_margin,
        max_num_leaves_per_parent=args.max_num_leaves_per_parent if args.construction_method == 'hierarchical_cover' else 1)
    model.metadata.update(urdf_path=str(urdf), source_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        planning_groups=groups, self_collision_exclusion_pairs=list(exclusions), floor_z=args.floor_z,
        self_collision_policy=policy, avoidance_config_path=str(avoidance_path) if avoidance_path else None)
    if avoidance_path:
        model.metadata['internal_policy'] = 'runtime_existing_sphere_envelope'
    model.metadata['content_signature'] = content_signature(model.metadata, model.arrays)
    robot_name = str(config.get('robot_name', urdf.stem))
    if any(value in robot_name for value in ('/', '\\', '..')):
        raise ValueError('保存先用ロボット名の不正')
    basename = 'urdf_hierarchy_' if args.construction_method == 'hierarchical_cover' else 'urdf_vrm_'
    output = Path(args.output) if args.output else Path('gng_results')/robot_name/(basename+args.group+'.npz')
    if output.suffix.lower() != '.npz' or output.resolve() in (urdf, config_path, avoidance_path):
        raise ValueError('URDF・設定と異なるnpz保存先の指定が必要')
    model.save(output, allow_replace=args.allow_replace)
    print(json.dumps(dict(output=str(output), urdf=str(urdf), active_joint_names=active,
        num_joints=len(geometry.joint_names), num_spheres=len(geometry.radii), num_self_pairs=len(geometry.self_pairs),
        num_regions=len(model.counts[0]), num_indexed_regions=int(model.arrays['has_index'].sum()),
        num_parents=model.metadata.get('num_parents', len(model.counts[0])),
        construction_method=model.metadata['construction_method'],
        self_collision_policy=model.metadata['self_collision_policy'],
        indexed_domain_volume_ratio=model.metadata['indexed_domain_volume_ratio'],
        num_relations=len(model.arrays['entry_regions']), build_ms=model.metadata['build_ms'],
        has_full_domain_partition=True, has_hardware_output=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
