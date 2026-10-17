"""記録自己除去・環境の裏付け分類と、独立した自己ボクセル接触評価。"""
import copy
import ctypes
import os
from array import array
from collections import deque
from enum import Enum
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree


class observation_kind(str, Enum):
    unavailable = 'unavailable'
    clear = 'clear'
    contact = 'contact'
    uncertain = 'uncertain'


def stamp_sec(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def same_grid(left, right):
    """自己マスクと環境の座標・索引契約。原点を含む照合。"""
    return (left.header.frame_id.lstrip('/') == right.header.frame_id.lstrip('/')
            and abs(left.voxel_size-right.voxel_size) < 1e-9
            and all(getattr(left, name) == getattr(right, name) for name in (
                'x_shift', 'y_shift', 'z_shift', 'offset', 'origin_x', 'origin_y', 'origin_z')))


def select_voxels(message, ids):
    """元の格子定義・ID順・ラベルを保持した部分集合。"""
    if message.labels and len(message.labels) != len(message.data):
        raise ValueError('ボクセルのラベル数が不正です')
    kept = [idx for idx, value in enumerate(message.data) if value in ids]
    # 大きな配列の全複製を避けた部分抽出。書換え対象headerの所有権は独立。
    result = copy.copy(message)
    result.header = copy.deepcopy(message.header)
    result.data = array('q', (message.data[idx] for idx in kept))
    result.labels = array('B', (message.labels[idx] for idx in kept)) if message.labels else array('B')
    return result


def recorded_environment(roi, mask):
    """記録ロボットの残像除去。シミュレーション自己姿勢への非依存。"""
    if not same_grid(roi, mask):
        raise ValueError('記録ROIと記録自己マスクの座標・ボクセル規格が異なります')
    return select_voxels(roi, set(roi.data)-set(mask.data))


def voxel_cells(message):
    """近接分類用の整数セル座標。21 bit索引の復号。"""
    if (not np.isfinite(message.voxel_size) or message.voxel_size <= 0
            or sorted((message.x_shift, message.y_shift, message.z_shift)) != [0, 21, 42]
            or not np.all(np.isfinite([message.origin_x, message.origin_y, message.origin_z]))
            or any(value < 0 for value in message.data)):
        raise ValueError('評価用ボクセル格子の不正')
    ids = np.unique(np.asarray(message.data, dtype=np.int64))
    mask = (1 << 21)-1
    cells = np.column_stack([((ids >> shift) & mask)-message.offset for shift in
                             (message.x_shift, message.y_shift, message.z_shift)])
    return ids, cells


class workspace_mask:
    """基台座標の固定禁止領域。机の範囲外を許可する箱の和集合と、任意の高さ制限。"""
    def __init__(self, params, frame_id):
        config = params.get('workspace_mask', {})
        self.enable_mask = config.get('enable_mask', False)
        if not isinstance(self.enable_mask, bool):
            raise ValueError('固定領域マスクの有効設定の不正')
        self.enable_min_z = config.get('enable_min_z', False)
        self.min_z = float(config.get('min_z', 0.))
        self.boxes = np.asarray(config.get('forbidden_boxes', []), dtype=float)
        self.cell_padding = float(np.float32(params['gng']['vlut_resolution']))*np.sqrt(3.)/2
        self.braking_prediction_sec = float(config.get('braking_prediction_sec', .2))
        if (not isinstance(self.enable_min_z, bool) or not np.isfinite(self.min_z)
                or not np.isfinite(self.braking_prediction_sec) or self.braking_prediction_sec <= 0
                or self.boxes.ndim != 1 or self.boxes.size % 6 or self.boxes.size > 6*128
                or not np.isfinite(self.boxes).all()):
            raise ValueError('固定領域マスクの高さ・箱配列の不正')
        self.boxes = self.boxes.reshape(-1, 2, 3)
        if np.any(self.boxes[:, 0] >= self.boxes[:, 1]):
            raise ValueError('固定領域マスクの箱寸法の不正')
        if self.enable_mask and config.get('frame_id', 'base_link') != frame_id:
            raise ValueError('固定領域マスクと制御ロボットの基準座標系の不一致')

    def can_bridge(self, geometry, first, second):
        """区間内の外接球列による保守的近似。既存形状の利用、点群・ボクセルの再登録なし。"""
        if not self.enable_mask or not (self.enable_min_z or len(self.boxes)):
            return True
        count = max(1, int(np.ceil(np.max(np.abs(second-first))/.05)))
        poses = first+(second-first)*np.linspace(0., 1., count+1)[:, None]
        paths = [poses]
        max_velocity = getattr(geometry, 'max_velocity', None)
        if max_velocity is not None:
            # 各軸が個別の速度上限で到達する軌道も検査。直線補間だけの見落し防止。
            duration = np.max(np.abs(second-first)/max_velocity)
            count = max(1, int(np.ceil(duration*np.max(max_velocity)/.05)))
            times = np.linspace(0., duration, count+1)
            paths.append(first+np.sign(second-first)*np.minimum(
                np.abs(second-first), times[:, None]*max_velocity))
        radii = geometry.radii[geometry.is_arm]+self.cell_padding
        for path in paths:
            centers = geometry.centers(path)[:, geometry.is_arm]
            # 隣接サンプル間の包絡AABB。端点だけの照合による机の貫通の防止。
            low = np.minimum(centers[:-1], centers[1:])-radii[None, :, None]
            high = np.maximum(centers[:-1], centers[1:])+radii[None, :, None]
            if self.enable_min_z and np.any(low[..., 2] < self.min_z):
                return False
            for box_low, box_high in self.boxes:
                if np.any(np.all((high > box_low) & (low < box_high), axis=-1)):
                    return False
        return True

    def has_braking_room(self, geometry, positions, velocity):
        """実測速度の継続予測による減速余地の近似。実機の制動性能保証は対象外。"""
        if not self.enable_mask:
            return True
        return self.can_bridge(geometry, positions, positions+velocity*self.braking_prediction_sec)


class sensor_observation:
    """生点群の一回登録と、TF専用の観測側自己マスク。制御対象の関節値への非依存。"""
    def __init__(self, params, settings):
        explicit = os.environ.get('GNG_OBSERVATION_NATIVE_LIBRARY', '')
        paths = [Path(explicit)] if explicit else [
            Path(prefix)/'lib/gng_vlut_system/libgng_observation_kernel.so'
            for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if prefix]
        path = next((path for path in paths if path.is_file()), None)
        if path is None:
            raise RuntimeError('観測入力部品未配置。gng_observation_kernelのビルド・installが必要です')
        self.library = ctypes.CDLL(str(path.absolute()))
        floats = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        integers = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
        handle, size = ctypes.c_void_p, ctypes.c_size_t
        self.library.gng_observation_create.argtypes = [ctypes.c_char_p]*3+[ctypes.c_double, integers]
        self.library.gng_observation_create.restype = handle
        self.library.gng_observation_destroy.argtypes = [handle]
        self.library.gng_observation_error.restype = ctypes.c_char_p
        for name in ('gng_observation_num_links', 'gng_observation_mask_capacity'):
            getattr(self.library, name).argtypes = [handle]
            getattr(self.library, name).restype = size
        self.library.gng_observation_link.argtypes = [handle, size]
        self.library.gng_observation_link.restype = ctypes.c_char_p
        self.library.gng_observation_build.argtypes = [handle, size, floats, floats, floats, floats,
                                                      integers, size, integers, size, integers]
        self.library.gng_observation_build.restype = ctypes.c_int
        layout = params['voxel_idx_shift']
        self.layout = np.array([layout[name] for name in ('x_shift', 'y_shift', 'z_shift', 'offset')], dtype=np.int64)
        # 保存済みVLUTと同じfloat32格子。PointCloud2の登録はこの一格子のみ。
        self.step = float(np.float32(params['gng']['vlut_resolution']))
        self.bounds = np.ascontiguousarray([
            *settings.get('visibility_min_bounds', [-1.2, -1.2, -1.2]),
            *settings.get('visibility_max_bounds', [1.2, 1.2, 1.2])], dtype=np.float64)
        if self.bounds.shape != (6,) or not np.isfinite(self.bounds).all() or np.any(self.bounds[:3] >= self.bounds[3:]):
            raise ValueError('観測ROI範囲の不正')
        self.handle = self.library.gng_observation_create(*[
            str(params.get(name, '')).encode() for name in ('urdf_path', 'resource_root_dir', 'mesh_root_dir')],
            self.step, self.layout)
        if not self.handle:
            raise RuntimeError(self.library.gng_observation_error().decode())
        try:
            self.links = [self.library.gng_observation_link(self.handle, idx).decode()
                          for idx in range(self.library.gng_observation_num_links(self.handle))]
            self.mask_ids = np.empty(self.library.gng_observation_mask_capacity(self.handle), dtype=np.int64)
        except Exception:
            self.close()
            raise
        self.roi_ids = np.empty(0, dtype=np.int64)
        self.num_updates = 0
        self.total_build_ms = self.max_build_ms = 0.
        self.details = {}

    def build(self, cloud, points, sensor_tf, link_tf, frame):
        """同じ観測時刻・同じ出力格子のROIと自己マスク。格子IDの後段共有。"""
        from voxel_msgs.msg import Voxel
        begin = time.perf_counter()
        if len(self.roi_ids) < len(points):
            self.roi_ids = np.empty(len(points), dtype=np.int64)
        sensor_tf = np.ascontiguousarray(sensor_tf, dtype=np.float64)
        link_tf = np.ascontiguousarray(link_tf, dtype=np.float64)
        if sensor_tf.shape != (4, 4) or link_tf.shape != (len(self.links), 4, 4):
            raise ValueError('観測TFの個数不一致')
        counts = np.zeros(2, dtype=np.int64)
        code = self.library.gng_observation_build(self.handle, len(points), points, sensor_tf, link_tf,
            self.bounds, self.roi_ids, len(self.roi_ids), self.mask_ids, len(self.mask_ids), counts)
        if code:
            raise ValueError(self.library.gng_observation_error().decode())
        def message(ids):
            result = Voxel()
            result.header = copy.deepcopy(cloud.header)
            result.header.frame_id = frame
            result.voxel_size = self.step
            result.x_shift, result.y_shift, result.z_shift, result.offset = map(int, self.layout)
            result.data = array('q', ids)
            return result
        roi, mask = message(self.roi_ids[:counts[0]]), message(self.mask_ids[:counts[1]])
        self.num_updates += 1
        elapsed_ms = (time.perf_counter()-begin)*1000
        self.total_build_ms += elapsed_ms
        self.max_build_ms = max(self.max_build_ms, elapsed_ms)
        self.details = dict(num_updates=self.num_updates, num_points=len(points), num_roi_voxels=int(counts[0]),
                            num_self_voxels=int(counts[1]), build_ms=elapsed_ms,
                            build_mean_ms=self.total_build_ms/self.num_updates, build_max_ms=self.max_build_ms)
        return roi, mask

    def close(self):
        if self.handle:
            self.library.gng_observation_destroy(self.handle)
            self.handle = None


class recorded_self_filter:
    """記録時刻の近傍マスク選択と、自己境界候補の分離。再ボクセル化なし。"""
    def __init__(self, padding=.02, max_pair_age_sec=.05, max_age_sec=1.):
        if (not np.isfinite(padding) or not 0 <= padding <= .05
                or not np.isfinite(max_pair_age_sec) or not 0 <= max_pair_age_sec <= .2):
            raise ValueError('記録自己除去の余裕幅または時刻差が不正です')
        self.padding, self.max_pair_age_sec, self.max_age_sec = padding, max_pair_age_sec, max_age_sec
        self.masks = deque(maxlen=32)
        self.cached_mask = self.mask_tree = None
        self.num_boundary_voxels = 0
        self.num_matched_frames = self.max_num_boundary_voxels = 0
        self.max_pair_age_ms = 0.
        self.details = dict(num_core_removed_voxels=0, num_boundary_voxels=0, pair_age_ms=None)

    @staticmethod
    def stamp_key(message):
        return message.header.stamp.sec*1000000000+message.header.stamp.nanosec

    def add_mask(self, message):
        now = time.monotonic()
        if self.masks and self.stamp_key(message) < self.stamp_key(self.masks[-1][0])-1000000000:
            # bag巻戻しをまたぐマスク・空間索引の持越し防止。
            self.masks.clear()
            self.cached_mask = self.mask_tree = None
        self.masks.append((message, now))

    def match(self, roi):
        now = time.monotonic()
        candidates = [message for message, received in self.masks
                      if now-received <= self.max_age_sec and len(message.data)]
        if not candidates:
            return None
        stamp = self.stamp_key(roi)
        mask = min(candidates, key=lambda message: abs(self.stamp_key(message)-stamp))
        if abs(self.stamp_key(mask)-stamp)*1e-9 > self.max_pair_age_sec:
            return None
        if not same_grid(roi, mask):
            raise ValueError('記録ROIと自己マスクの格子不一致')
        return mask

    def split(self, roi, mask):
        """自己確定部・境界候補・境界外の分離。境界候補の安全扱い禁止。"""
        raw = recorded_environment(roi, mask)
        ids, cells = voxel_cells(raw)
        has_boundary = np.zeros(len(ids), dtype=bool)
        if len(ids) and len(mask.data) and self.padding > 0:
            if mask is not self.cached_mask:
                _, mask_cells = voxel_cells(mask)
                self.mask_tree = cKDTree(mask_cells)
                self.cached_mask = mask
            # 既存格子の中心間距離だけの照合。膨張セル集合の全生成なし。
            radius = self.padding/roi.voxel_size+1e-7
            distances = self.mask_tree.query(cells, distance_upper_bound=radius)[0]
            has_boundary = distances <= radius
        boundary_ids = set(map(int, ids[has_boundary]))
        environment = select_voxels(raw, set(map(int, ids[~has_boundary])))
        boundary = select_voxels(raw, boundary_ids)
        self.num_boundary_voxels = len(boundary_ids)
        self.num_matched_frames += 1
        self.max_num_boundary_voxels = max(self.max_num_boundary_voxels, self.num_boundary_voxels)
        pair_age_ms = abs(self.stamp_key(roi)-self.stamp_key(mask))*1e-6
        self.max_pair_age_ms = max(self.max_pair_age_ms, pair_age_ms)
        self.details = dict(num_core_removed_voxels=len(roi.data)-len(raw.data),
                            num_boundary_voxels=len(boundary_ids),
                            pair_age_ms=pair_age_ms, num_matched_frames=self.num_matched_frames,
                            max_num_boundary_voxels=self.max_num_boundary_voxels, max_pair_age_ms=self.max_pair_age_ms)
        return environment, boundary, raw


class environment_observation:
    """環境の空間・時間裏付けと、同時刻近傍の自己マスク照合。"""
    def __init__(self, max_pair_age_sec=.2, max_age_sec=1.):
        self.max_pair_age_sec, self.max_age_sec = max_pair_age_sec, max_age_sec
        self.previous = self.environment = self.self_mask = None
        self.previous_tree = None
        self.previous_source_stamp_sec = self.source_stamp_sec = None
        self.supported_ids, self.uncertain_ids = set(), set()
        self.boundary_ids = set()
        self.environment_received_sec = self.self_received_sec = None
        self.last_pair = None
        self.last_valid_stamp_sec = None
        self.num_samples = self.num_contact_samples = self.num_uncertain_samples = 0
        self.max_num_overlap_voxels = self.max_num_raw_overlap_voxels = 0
        self.num_boundary_samples = self.max_num_boundary_overlap_voxels = 0
        self.covered_sec = 0.
        self.details = dict(state=observation_kind.unavailable.value, is_valid=False,
                            message='環境・実測自己ボクセルの入力待機')

    def preprocess(self, message, source_stamp_sec=None, *, boundary_ids=()):
        """境界外の26近傍・前観測による裏付け。自己境界・孤立点は未確定として保持。"""
        ids, cells = voxel_cells(message)
        self.boundary_ids = set(map(int, boundary_ids)) & set(map(int, ids))
        is_environment = ~np.isin(ids, np.asarray(list(self.boundary_ids), dtype=np.int64))
        environment_ids, environment_cells = ids[is_environment], cells[is_environment]
        source_stamp_sec = stamp_sec(message) if source_stamp_sec is None else source_stamp_sec
        tree = cKDTree(environment_cells)
        has_support = np.zeros(len(environment_ids), dtype=bool)
        if len(environment_ids) > 1:
            has_support = tree.query(environment_cells, k=2, p=np.inf)[0][:, 1] <= 1
        if (self.previous is not None and same_grid(message, self.previous)
                and 0 < source_stamp_sec-self.previous_source_stamp_sec <= self.max_age_sec
                and self.previous_tree.n and len(environment_ids)):
            has_support |= self.previous_tree.query(environment_cells, p=np.inf)[0] <= 1
        self.previous, self.previous_tree = message, tree
        self.previous_source_stamp_sec = self.source_stamp_sec = source_stamp_sec
        self.supported_ids = set(map(int, environment_ids[has_support]))
        self.uncertain_ids = set(map(int, ids))-self.supported_ids
        self.environment = message
        self.environment_received_sec = time.monotonic()
        return select_voxels(message, self.supported_ids), select_voxels(message, self.uncertain_ids)

    def on_self(self, message):
        voxel_cells(message)
        self.self_mask = message
        self.self_received_sec = time.monotonic()

    def evaluate(self):
        """自己除去前の環境と実測自己マスクの重なり。未確定接触は成功から除外。"""
        environment, mask = self.environment, self.self_mask
        if environment is None or mask is None:
            return None
        now = time.monotonic()
        pair_age_sec = abs(stamp_sec(environment)-stamp_sec(mask))
        if (not environment.data or not mask.data or now-self.environment_received_sec > self.max_age_sec
                or now-self.self_received_sec > self.max_age_sec
                or pair_age_sec > self.max_pair_age_sec or not same_grid(environment, mask)):
            self.last_valid_stamp_sec = None
            self.details = dict(state=observation_kind.unavailable.value, is_valid=False,
                                message='環境・自己マスクの空入力、鮮度、時刻または格子不一致')
            return None
        key = (stamp_sec(environment), stamp_sec(mask))
        if self.last_pair == key:
            return None
        self.last_pair = key
        body = set(mask.data)
        contact = self.supported_ids & body
        uncertain = self.uncertain_ids & body
        boundary = self.boundary_ids & body
        state = (observation_kind.contact if contact else
                 observation_kind.uncertain if uncertain else observation_kind.clear)
        self.num_samples += 1
        self.num_contact_samples += bool(contact)
        self.num_uncertain_samples += bool(uncertain)
        self.num_boundary_samples += bool(boundary)
        self.max_num_boundary_overlap_voxels = max(self.max_num_boundary_overlap_voxels, len(boundary))
        self.max_num_overlap_voxels = max(self.max_num_overlap_voxels, len(contact))
        self.max_num_raw_overlap_voxels = max(self.max_num_raw_overlap_voxels, len(contact)+len(uncertain))
        current_sec = max(key)
        if self.last_valid_stamp_sec is not None and 0 < current_sec-self.last_valid_stamp_sec <= self.max_pair_age_sec:
            self.covered_sec += current_sec-self.last_valid_stamp_sec
        self.last_valid_stamp_sec = current_sec
        self.details = dict(state=state.value, is_valid=True, message='',
            num_environment_voxels=len(self.supported_ids), num_uncertain_voxels=len(self.uncertain_ids),
            num_boundary_voxels=len(self.boundary_ids),
            num_isolated_voxels=len(self.uncertain_ids-self.boundary_ids),
            num_boundary_overlap_voxels=len(boundary), num_isolated_overlap_voxels=len(uncertain-boundary),
            num_overlap_voxels=len(contact), num_uncertain_overlap_voxels=len(uncertain),
            num_raw_overlap_voxels=len(contact)+len(uncertain), pair_age_ms=pair_age_sec*1000,
            environment_stamp_sec=key[0], self_stamp_sec=key[1])
        return select_voxels(environment, contact), select_voxels(environment, uncertain)

    def status(self):
        self.evaluate()
        history = (observation_kind.contact if self.num_contact_samples else
                   observation_kind.uncertain if self.num_uncertain_samples else
                   observation_kind.clear if self.num_samples else observation_kind.unavailable)
        return dict(self.details, num_samples=self.num_samples, num_contact_samples=self.num_contact_samples,
                    num_uncertain_samples=self.num_uncertain_samples, covered_sec=self.covered_sec,
                    source_stamp_sec=self.source_stamp_sec, history_state=history.value,
                    max_num_overlap_voxels=self.max_num_overlap_voxels,
                    num_boundary_samples=self.num_boundary_samples,
                    max_num_boundary_overlap_voxels=self.max_num_boundary_overlap_voxels,
                    max_num_raw_overlap_voxels=self.max_num_raw_overlap_voxels)
