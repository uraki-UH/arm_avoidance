"""記録自己除去・環境の裏付け分類と、独立した自己ボクセル接触評価。"""
import copy
from array import array
from collections import deque
from enum import Enum
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
