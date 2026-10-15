#!/usr/bin/env python3
"""既存GNGに追加可能な、予算付き辺掃引の疎な逆引き索引。"""
import argparse
from dataclasses import dataclass
from enum import IntEnum
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import zipfile

import numpy as np


class edge_state(IntEnum):
    clear = 0
    unknown = 1
    blocked = 2


@dataclass(frozen=True)
class bridge_check:
    """外部環境を含む区間判定と、内部離隔の再検査省略可否。"""
    can_pass: bool | None = None
    can_skip_internal: bool = False


def checked_positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
        raise ValueError(name + 'には有限の正数が必要です')
    return value


def packed_ids(cells):
    """原点0・21 bit各軸の内部索引。ROS入力の符号化とは独立。"""
    cells = np.asarray(cells, dtype=np.int64).reshape(-1, 3)
    if np.any(cells < -(1 << 20)) or np.any(cells >= (1 << 20)):
        raise ValueError('辺索引の座標範囲外')
    unsigned = (cells + (1 << 20)).astype(np.uint64)
    return (unsigned[:, 0] << np.uint64(42)) | (unsigned[:, 1] << np.uint64(21)) | unsigned[:, 2]


def geometry_signature(geometry):
    """球形状・運動連鎖・対象リンク・内部検査規則の照合値。"""
    digest = hashlib.sha256()
    digest.update(json.dumps(geometry.joint_names, ensure_ascii=False).encode())
    for values in (geometry.radii, geometry.center_points, geometry.center_links, geometry.limits,
                   geometry.is_arm, geometry.self_pairs):
        array = np.asarray(values)
        digest.update(str((array.shape, array.dtype.str)).encode())
        digest.update(array.tobytes())
    for operation in geometry.center_operations:
        for value in operation:
            digest.update(np.asarray(value).tobytes() if isinstance(value, np.ndarray) else repr(value).encode())
    for kind, position, rotation, half_size in geometry.environment_shapes:
        digest.update(kind.encode())
        for value in (position, rotation, half_size):
            digest.update(np.asarray(value).tobytes())
    return digest.hexdigest()


def center_motion_bounds(geometry):
    """関節ごとの球中心変位上界。全関節格子・関節組合せの列挙なし。"""
    result = np.zeros((len(geometry.radii), len(geometry.joint_names)))
    operations = {operation[1]: operation for operation in geometry.center_operations}
    for sphere_idx, child in enumerate(geometry.center_links):
        reach = np.linalg.norm(geometry.center_points[sphere_idx])
        while child:
            parent, _, kind, fixed, _, joint_idx, multiplier, offset, *_ = operations[child]
            if kind in ('revolute', 'continuous'):
                result[sphere_idx, joint_idx] += abs(multiplier)*reach
            elif kind == 'prismatic':
                result[sphere_idx, joint_idx] += abs(multiplier)
                reach += np.max(np.abs(geometry.limits[joint_idx]*multiplier+offset))
            else:
                raise ValueError('未対応の可動関節')
            reach += np.linalg.norm(fixed[:3, 3])
            child = parent
    return result


def sphere_voxels(center, radius, voxel_size, max_num_cells, *, is_inner=False):
    """球と閉セル箱の交差。局所外接箱だけの登録候補。"""
    first = np.floor((center-radius)/voxel_size).astype(np.int64)-1
    last = np.floor((center+radius)/voxel_size).astype(np.int64)+1
    size = last-first+1
    if np.prod(size.astype(float)) > max_num_cells:
        raise OverflowError('球の登録予算超過')
    axes = [np.arange(first[idx], last[idx]+1) for idx in range(3)]
    cells = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    low = cells*voxel_size
    gap = (np.maximum(np.abs(low-center), np.abs(low+voxel_size-center)) if is_inner else
           np.maximum(np.maximum(low-center, center-low-voxel_size), 0))
    margin = -1e-12 if is_inner else 1e-12
    return packed_ids(cells[np.einsum('ij,ij->i', gap, gap) <= radius*radius+margin])


def internal_margin(geometry, centers, movement):
    """各サンプルと中間変位上界による内部離隔の下界。理想球モデル用。"""
    first, second = geometry.self_pairs.T
    margins = movement[first]+movement[second]
    for kind, *_ in geometry.environment_shapes:
        if kind not in ('plane', 'box'):
            raise ValueError('未対応の固定環境形状')
        margins = np.concatenate((margins, movement[geometry.is_arm]))
    values = geometry.internal_clearances(centers)
    return float(np.min(values-margins)) if values.size else 1e6


def refined_internal_margin(geometry, first, second, total_motion, centers, max_num_samples, min_gap, end_sec):
    """内部離隔だけの適応的細分。FKの追加だけ、掃引ボクセルの再登録なし。"""
    count = len(centers)
    margin = internal_margin(geometry, centers, total_motion/(2*(count-1)))
    num_samples = 0
    while margin < min_gap and count < max_num_samples and time.monotonic() < end_sec:
        count = min(2*(count-1)+1, max_num_samples)
        samples = first+np.linspace(0, 1, count)[:, None]*(second-first)
        centers = geometry.centers(samples)
        margin = internal_margin(geometry, centers, total_motion/(2*(count-1)))
        num_samples += count
    return margin, num_samples


def pose_key(first, second):
    """向きに依存しない固定線形軌道の端点キー。近似一致による流用なし。"""
    keys = [np.asarray(q, dtype='<f8').tobytes() for q in (first, second)]
    return tuple(sorted(keys))


def content_signature(metadata, arrays):
    """別成果物の破損検出。形状照合とは独立した配列・条件の照合。"""
    digest = hashlib.sha256(json.dumps({key: value for key, value in metadata.items() if key != 'content_signature'},
                                     sort_keys=True, ensure_ascii=False).encode())
    for name in sorted(arrays):
        array = np.asarray(arrays[name])
        digest.update(name.encode())
        digest.update(str((array.shape, array.dtype.str)).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def save_index(metadata, arrays, path, *, allow_replace=False):
    """追加バックエンド共通の原子的保存。既存GNG・ノードVLUTとは別成果物。"""
    path = Path(path)
    if path.exists() and not allow_replace:
        raise FileExistsError('索引の保存先が既に存在します: '+str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name+'.', suffix='.tmp', delete=False) as stream:
        temporary = Path(stream.name)
        try:
            np.savez_compressed(stream, metadata=np.asarray(json.dumps(metadata)), **arrays)
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
            if allow_replace:
                temporary.replace(path)
            else:
                os.link(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


def load_index(path):
    """追加バックエンド共通の展開予算・非pickle読込み。"""
    with zipfile.ZipFile(path) as archive:
        if sum(item.file_size for item in archive.infolist()) > 128*1024*1024:
            raise ValueError('索引の展開予算超過')
    with np.load(path, allow_pickle=False) as archive:
        metadata = json.loads(str(archive['metadata']))
        arrays = {name: archive[name].copy() for name in archive.files if name != 'metadata'}
    return metadata, arrays


def build_index(geometry, nodes, edges, *, voxel_size=.05, clearance_margin=.02,
                max_num_samples=128, max_num_edges=512, max_num_relations=500000,
                max_build_sec=60., max_num_cells=100000, min_internal_gap=.01):
    """全時間包絡の事前構築。予算未処理辺は要確認、元グラフの保持。"""
    checked_positive(voxel_size, 'voxel_size')
    checked_positive(max_build_sec, 'max_build_sec')
    checked_positive(min_internal_gap, 'min_internal_gap')
    if not np.isfinite(clearance_margin) or clearance_margin < 0:
        raise ValueError('clearance_marginには非負数が必要です')
    for name, value in (('max_num_samples', max_num_samples), ('max_num_edges', max_num_edges),
                        ('max_num_relations', max_num_relations), ('max_num_cells', max_num_cells)):
        if type(value) is not int or value < (2 if name == 'max_num_samples' else 1):
            raise ValueError(name + 'には整数の予算が必要です')
    ids = sorted(nodes)
    if not ids or len(set(geometry.joint_names)) != len(geometry.joint_names):
        raise ValueError('ノード・関節名の不正')
    positions = np.asarray([nodes[idx] for idx in ids], dtype=float)
    if positions.shape != (len(ids), len(geometry.joint_names)) or not np.all(np.isfinite(positions)):
        raise ValueError('全関節姿勢の個数・数値の不正')
    if np.any(positions < geometry.limits[:, 0]-1e-8) or np.any(positions > geometry.limits[:, 1]+1e-8):
        raise ValueError('保存姿勢の可動域違反')
    lookup = {idx: row for row, idx in enumerate(ids)}
    pairs = sorted(set(tuple(sorted(pair)) for pair in edges))
    if any(len(pair) != 2 or pair[0] == pair[1] or any(idx not in lookup for idx in pair) for pair in pairs):
        raise ValueError('接続辺の不正')
    pairs = np.asarray([(lookup[first], lookup[second]) for first, second in pairs], dtype=np.int64).reshape(-1, 2)
    bounds = center_motion_bounds(geometry)
    radii = geometry.radii
    selected = np.flatnonzero(geometry.is_arm)
    if not len(selected):
        raise ValueError('環境衝突対象の球がありません')
    start_sec = time.monotonic()
    has_index = np.zeros(len(pairs), dtype=bool)
    min_internal = np.full(len(pairs), -1e6)
    # 所有ノードと64辺ブロック。高次数ノードも複数ブロックでの保持。
    loads = np.zeros(len(ids), dtype=int)
    blocks, bit_ids, block_lookup = [], [], {}
    for first, second in pairs:
        owner = first if loads[first] <= loads[second] else second
        key = (int(owner), int(loads[owner]//64))
        if key not in block_lookup:
            block_lookup[key] = len(block_lookup)
        blocks.append(block_lookup[key])
        bit_ids.append(int(loads[owner] % 64))
        loads[owner] += 1
    entries = {}
    num_samples = 0
    num_internal_samples = 0
    environment_radius = voxel_size*np.sqrt(3)/2
    for edge_idx, (first, second) in enumerate(pairs):
        if edge_idx >= max_num_edges or time.monotonic()-start_sec >= max_build_sec:
            break
        delta = np.abs(positions[second]-positions[first])
        total_motion = bounds@delta
        count = max(2, int(np.ceil(np.max(total_motion)/voxel_size))+1)
        if count > max_num_samples:
            continue
        samples = positions[first]+np.linspace(0, 1, count)[:, None]*(positions[second]-positions[first])
        centers = geometry.centers(samples)
        movement = total_motion/(2*(count-1))
        inner, outer = set(), set()
        is_complete = True
        try:
            for pose in centers:
                if time.monotonic()-start_sec >= max_build_sec:
                    is_complete = False
                    break
                for idx in selected:
                    # 内側は球に完全包含されるセル。再格子化した点の位置に依存しない衝突証拠。
                    inner.update(map(int, sphere_voxels(pose[idx], radii[idx], voxel_size, max_num_cells, is_inner=True)))
                    outer.update(map(int, sphere_voxels(pose[idx], radii[idx]+movement[idx]+clearance_margin+environment_radius,
                                                      voxel_size, max_num_cells)))
                    if len(outer) > max_num_relations:
                        raise OverflowError('辺の逆引き登録予算超過')
        except OverflowError:
            is_complete = False
        if not is_complete:
            continue
        block, bit = blocks[edge_idx], 1 << bit_ids[edge_idx]
        new_keys = [(voxel, block) for voxel in outer if (voxel, block) not in entries]
        if len(entries)+len(new_keys) > max_num_relations:
            continue
        for voxel in outer:
            old_outer, old_inner = entries.get((voxel, block), (0, 0))
            entries[voxel, block] = (old_outer | bit, old_inner | (bit if voxel in inner else 0))
        has_index[edge_idx] = True
        min_internal[edge_idx], additional_samples = refined_internal_margin(
            geometry, positions[first], positions[second], total_motion, centers,
            max_num_samples, min_internal_gap, start_sec+max_build_sec)
        num_internal_samples += additional_samples
        num_samples += count
    keys = sorted(entries)
    voxels, starts = np.unique(np.asarray([key[0] for key in keys], dtype=np.uint64), return_index=True)
    arrays = dict(node_ids=np.asarray(ids, dtype=np.int64), positions=positions, edges=pairs,
                  has_index=has_index, min_internal=min_internal, edge_blocks=np.asarray(blocks, dtype=np.int64),
                  edge_bits=np.asarray(bit_ids, dtype=np.int64), voxel_ids=voxels,
                  pointers=np.append(starts, len(keys)).astype(np.int64),
                  entry_blocks=np.asarray([key[1] for key in keys], dtype=np.int64),
                  outer_bits=np.asarray([entries[key][0] for key in keys], dtype=np.uint64),
                  inner_bits=np.asarray([entries[key][1] for key in keys], dtype=np.uint64))
    metadata = dict(format='gng_edge_vlut_v1', joint_names=geometry.joint_names, geometry_signature=geometry_signature(geometry),
                    voxel_size=voxel_size, clearance_margin=clearance_margin, max_cell_radius=environment_radius,
                    num_blocks=len(block_lookup), num_samples=num_samples, num_internal_samples=num_internal_samples,
                    build_ms=(time.monotonic()-start_sec)*1000,
                    model_scope='ideal_sphere_geometry_linear_joint_path', is_real_robot_certified=False)
    metadata['content_signature'] = content_signature(metadata, arrays)
    return edge_vlut(metadata, arrays)


class edge_vlut:
    """疎CSRの逆引きと、消去も扱う接触カウンタ。環境のボクセル登録は照会間で共有。"""
    def __init__(self, metadata, arrays):
        self.metadata, self.arrays = metadata, arrays
        if metadata.get('format') != 'gng_edge_vlut_v1':
            raise ValueError('辺VLUTの形式不一致')
        if metadata.get('content_signature') != content_signature(metadata, arrays):
            raise ValueError('辺VLUTの破損・条件不整合')
        edges, positions = arrays['edges'], arrays['positions']
        for name in ('node_ids', 'edges', 'edge_blocks', 'edge_bits', 'pointers', 'entry_blocks'):
            if not np.issubdtype(arrays[name].dtype, np.integer):
                raise ValueError('辺VLUTの整数配列型の不正')
        if (arrays['has_index'].dtype != np.dtype(bool)
                or any(arrays[name].dtype != np.dtype(np.uint64) for name in ('voxel_ids', 'outer_bits', 'inner_bits'))):
            raise ValueError('辺VLUTのビット配列型の不正')
        num_edges, num_entries, num_voxels = len(edges), len(arrays['entry_blocks']), len(arrays['voxel_ids'])
        if (positions.ndim != 2 or positions.shape[1] != len(metadata['joint_names'])
                or not np.all(np.isfinite(positions)) or edges.shape != (num_edges, 2)
                or np.any(edges < 0) or np.any(edges >= len(positions))
                or type(metadata['num_blocks']) is not int or not 0 <= metadata['num_blocks'] <= num_edges
                or arrays['node_ids'].shape != (len(positions),)
                or len(np.unique(arrays['node_ids'])) != len(positions)):
            raise ValueError('辺VLUTの姿勢・辺形式の不正')
        for name in ('has_index', 'min_internal', 'edge_blocks', 'edge_bits'):
            if arrays[name].shape != (num_edges,):
                raise ValueError('辺VLUTの辺配列形式の不正')
        if (np.any(arrays['edge_bits'] < 0) or np.any(arrays['edge_bits'] >= 64)
                or np.any(arrays['edge_blocks'] < 0) or np.any(arrays['edge_blocks'] >= metadata['num_blocks'])
                or not np.all(np.isfinite(arrays['min_internal']))):
            raise ValueError('辺VLUTの辺ビット指定の不正')
        pointers = arrays['pointers']
        if (pointers.shape != (num_voxels+1,) or pointers[0] != 0 or pointers[-1] != num_entries
                or np.any(np.diff(pointers) < 0) or np.any(arrays['voxel_ids'][1:] <= arrays['voxel_ids'][:-1])):
            raise ValueError('辺VLUTのCSR形式の不正')
        if (arrays['voxel_ids'].shape != (num_voxels,)
                or any(arrays[name].shape != (num_entries,) for name in ('entry_blocks', 'outer_bits', 'inner_bits'))
                or np.any(arrays['entry_blocks'] < 0) or np.any(arrays['entry_blocks'] >= metadata['num_blocks'])
                or np.any(arrays['inner_bits'] & ~arrays['outer_bits'])):
            raise ValueError('辺VLUTの逆引き形式の不正')
        for name in ('voxel_size', 'max_cell_radius'):
            checked_positive(metadata[name], name)
        if not np.isfinite(metadata['clearance_margin']) or metadata['clearance_margin'] < 0:
            raise ValueError('辺VLUTの離隔設定の不正')
        if sum(value.nbytes for value in arrays.values()) + 2*metadata['num_blocks']*64*8 > 128*1024*1024:
            raise ValueError('辺VLUTの保存・実行メモリ予算超過')
        self.edge_lookup = {pose_key(arrays['positions'][first], arrays['positions'][second]): idx
                            for idx, (first, second) in enumerate(arrays['edges'])}
        self.counts = np.zeros((2, metadata['num_blocks'], 64), dtype=np.int64)
        self.occupied = set()
        self.environment_key = None
        self.has_environment = False
        self.bit_shifts = np.arange(64, dtype=np.uint64)
        self.stats = dict(num_updates=0, num_registered_environments=0, num_fast_clear=0,
                          num_fast_blocked=0, num_fallback=0, num_changed_voxels=0)

    def update(self, occupied_ids):
        # ID直接更新と点群照会を混在した場合の、旧スナップショットキーの失効。
        self.environment_key = None
        current = set(map(int, occupied_ids))
        added, removed = current-self.occupied, self.occupied-current
        arrays = self.arrays
        touched = set()
        for values, sign in ((added, 1), (removed, -1)):
            if not values or not len(arrays['voxel_ids']):
                continue
            values = np.asarray(sorted(values), dtype=np.uint64)
            positions = np.searchsorted(arrays['voxel_ids'], values)
            valid = positions < len(arrays['voxel_ids'])
            positions = positions[valid]
            values = values[valid]
            positions = positions[arrays['voxel_ids'][positions] == values]
            for start in range(0, len(positions), 1024):
                indices = np.concatenate([np.arange(arrays['pointers'][idx], arrays['pointers'][idx+1])
                                          for idx in positions[start:start+1024]])
                for part in range(0, len(indices), 2048):
                    chunk = indices[part:part+2048]
                    blocks = arrays['entry_blocks'][chunk]
                    touched.update(map(int, blocks))
                    for row, name in enumerate(('outer_bits', 'inner_bits')):
                        rows, bits = np.nonzero((arrays[name][chunk, None] >> self.bit_shifts) & np.uint64(1))
                        np.add.at(self.counts[row], (blocks[rows], bits), sign)
        if touched and np.any(self.counts[:, list(touched)] < 0):
            raise ValueError('辺VLUTの接触カウンタ不整合')
        self.occupied = current
        self.has_environment = True
        self.stats['num_updates'] += 1
        self.stats['num_changed_voxels'] = len(added)+len(removed)

    def update_cloud(self, cloud_tree):
        """受信スナップショットごとの1回の索引照合。格子条件ごとの共用。"""
        if self.environment_key is cloud_tree:
            return
        points = np.asarray(cloud_tree.data)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise ValueError('環境座標の不正')
        self.update(packed_ids(np.floor(points/self.metadata['voxel_size']).astype(np.int64)))
        self.environment_key = cloud_tree
        self.stats['num_registered_environments'] += 1

    def classify(self, edge_idx):
        if not self.has_environment or not self.arrays['has_index'][edge_idx]:
            return edge_state.unknown
        block, bit = self.arrays['edge_blocks'][edge_idx], self.arrays['edge_bits'][edge_idx]
        if self.counts[1, block, bit]:
            return edge_state.blocked
        return edge_state.unknown if self.counts[0, block, bit] else edge_state.clear

    def query(self, geometry, first, second, cloud_tree, cell_radius, min_gap, min_internal_gap):
        """条件一致時のみの高速回答。未登録・形状変更・余裕不足は従来検査へ委譲。"""
        if any(not np.isfinite(value) or value < 0 for value in (min_gap, min_internal_gap)):
            raise ValueError('照会の離隔距離の不正')
        edge_idx = self.edge_lookup.get(pose_key(first, second))
        if (edge_idx is None or geometry_signature(geometry) != self.metadata['geometry_signature']
                or not np.isfinite(cell_radius) or cell_radius < 0 or cell_radius > self.metadata['max_cell_radius']+1e-10
                or cloud_tree is None):
            self.stats['num_fallback'] += 1
            return None
        self.update_cloud(cloud_tree)
        state = self.classify(edge_idx)
        if state == edge_state.blocked:
            self.stats['num_fast_blocked'] += 1
            return False
        if (state == edge_state.clear and min_gap <= self.metadata['clearance_margin']
                and self.arrays['min_internal'][edge_idx] >= min_internal_gap+1e-9):
            self.stats['num_fast_clear'] += 1
            return True
        self.stats['num_fallback'] += 1
        return None

    def save(self, path, *, allow_replace=False):
        """別成果物への原子的保存。既存のGNG・ノードVLUTの書換えなし。"""
        save_index(self.metadata, self.arrays, path, allow_replace=allow_replace)

    @classmethod
    def load(cls, path):
        return cls(*load_index(path))


def query_backend(state, first, second, min_gap):
    """既定node_vlutの無変更と、追加バックエンドの遅延読込み。"""
    return query_backend_checks(state, first, second, min_gap).can_pass


vlut_backends = ('node_vlut', 'edge_vlut', 'region_vlut', 'hierarchy_vlut')


def validate_backend_config(config):
    """衝突照会バックエンドと必要ファイルの共通検査。"""
    kind = config.get('vlut_backend', 'node_vlut')
    if kind not in vlut_backends:
        raise ValueError('vlut_backendは'+'・'.join(vlut_backends))
    if kind != 'node_vlut':
        filename = config.get(kind+'_file')
        if not isinstance(filename, (str, Path)) or not str(filename).strip():
            raise ValueError(kind+'には'+kind+'_fileが必要です')
    return kind


def get_query_backend(state):
    """遅延読込みとキャッシュ。専用階層選択では形式・実行形状の厳密な照合。"""
    kind = validate_backend_config(state.config)
    if kind == 'node_vlut':
        return None
    filename = state.config[kind+'_file']
    attribute = kind+'_backend'
    backend = getattr(state, attribute, None)
    if backend is None:
        factory = edge_vlut
        if kind == 'region_vlut':
            from gng_region_vlut import region_vlut
            factory = region_vlut
        elif kind == 'hierarchy_vlut':
            from urdf_vrm import urdf_hierarchy
            factory = urdf_hierarchy
        backend = factory.load(filename)
        if kind == 'hierarchy_vlut' and geometry_signature(state.geometry) != backend.metadata['geometry_signature']:
            raise ValueError('階層VLUTと実行形状・固定環境・自己干渉規則が不一致です。'
                             '実行時と同じURDF・--avoidance-configでの再構築が必要です')
        setattr(state, attribute, backend)
    return backend


def query_backend_checks(state, first, second, min_gap):
    """同一照合結果による区間可否と内部離隔証明。既定ノード照会の維持。"""
    kind = state.config.get('vlut_backend', 'node_vlut')
    backend = get_query_backend(state)
    if backend is None:
        return bridge_check()
    arguments = (state.geometry, first, second, state.cloud_tree, state.cell_radius, min_gap,
                 state.config.get('min_planning_clearance_th', .005))
    if kind in ('region_vlut', 'hierarchy_vlut'):
        return backend.query_checks(*arguments)
    return bridge_check(backend.query(*arguments))


def main():
    parser = argparse.ArgumentParser(description='既存GNGの別保存レシピから辺VLUTを追加構築')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', default='')
    parser.add_argument('--urdf', default='')
    parser.add_argument('--avoidance-config', default='')
    parser.add_argument('--voxel-size', type=float, default=.05)
    parser.add_argument('--clearance-margin', type=float, default=.02)
    parser.add_argument('--max-num-edges', type=int, default=512)
    parser.add_argument('--max-build-sec', type=float, default=60.)
    parser.add_argument('--allow-replace', action='store_true')
    args = parser.parse_args()
    from dual_arm_avoidance_geometry import robot_geometry
    inputs = sorted(Path(args.input).rglob('edge_vlut_recipe.json')) if Path(args.input).is_dir() else [Path(args.input)]
    if not inputs or (len(inputs) > 1 and args.output):
        raise ValueError('レシピ未配置、または複数レシピへの単一保存先指定')
    for filename in inputs:
        build_recipe(filename, args, robot_geometry)


def build_recipe(filename, args, geometry_factory):
    recipe = json.loads(filename.read_text())
    if recipe.get('format') != 'gng_edge_vlut_recipe_v1':
        raise ValueError('構築レシピの形式不一致')
    config = {}
    if args.avoidance_config:
        import yaml
        config = yaml.safe_load(Path(args.avoidance_config).read_text())['dual_arm_avoidance_demo']
    geometry = geometry_factory(args.urdf or recipe['urdf_path'], config.get('planning_groups'))
    fixed = np.asarray([recipe.get('fixed_joints', {}).get(name, 0.) for name in geometry.joint_names])
    selected = [geometry.joint_names.index(name) for name in recipe['joint_names']]
    if len(set(selected)) != len(selected):
        raise ValueError('構築レシピの関節名重複')
    nodes = {}
    for item in recipe['nodes']:
        if (type(item['id']) is not int or item['id'] < 0 or item['id'] in nodes
                or len(item['angles']) != len(selected)):
            raise ValueError('構築レシピのノードID・角度数の不正')
        q = fixed.copy()
        q[selected] = item['angles']
        nodes[item['id']] = q
    model = build_index(geometry, nodes, recipe['edges'], voxel_size=args.voxel_size,
                        clearance_margin=args.clearance_margin, max_num_edges=args.max_num_edges,
                        max_build_sec=args.max_build_sec)
    model.save(args.output or str(filename.with_name('edge_vlut.npz')), allow_replace=args.allow_replace)
    print(json.dumps(dict(num_edges=len(model.arrays['edges']), num_indexed_edges=int(model.arrays['has_index'].sum()),
                          num_relations=len(model.arrays['entry_blocks']), build_ms=model.metadata['build_ms'])))


if __name__ == '__main__':
    main()
