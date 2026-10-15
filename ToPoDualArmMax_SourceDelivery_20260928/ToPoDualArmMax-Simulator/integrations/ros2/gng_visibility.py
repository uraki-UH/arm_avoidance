"""既存ROI格子の三状態観測と、新規進入領域のC++照会。"""
import ctypes
from collections import Counter, deque
import os
from pathlib import Path
import threading
import time

import numpy as np

from gng_observation import stamp_sec


def visibility_kernel():
    """通常installまたは明示パスからの数値部品読込み。未配置時の暗黙な無効化なし。"""
    explicit = os.environ.get('GNG_VISIBILITY_NATIVE_LIBRARY', '')
    paths = [Path(explicit)] if explicit else [
        Path(prefix)/'lib/gng_vlut_system/libgng_visibility_kernel.so'
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if prefix]
    filename = next((path for path in paths if path.is_file()), None)
    if filename is None:
        raise RuntimeError('観測カーネル未配置。gng_visibility_kernelのビルド・installが必要です')
    library = ctypes.CDLL(str(filename.absolute()))
    library.gng_visibility_kernel_version.restype = ctypes.c_int
    if library.gng_visibility_kernel_version() != 3:
        raise RuntimeError('観測カーネルのABI不一致')
    floats = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    integers = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
    flags = np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')
    pixels = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
    depths = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
    size = ctypes.c_size_t
    library.gng_visibility_build.argtypes = [size, floats, floats, size, integers, size, integers,
                                            ctypes.c_double, floats, integers, size, ctypes.c_int, flags, integers]
    library.gng_visibility_build.restype = ctypes.c_int
    library.gng_visibility_check.argtypes = [flags, ctypes.c_double, floats, integers, size, floats,
                                            floats, size, floats, floats, size, integers]
    library.gng_visibility_check.restype = ctypes.c_int
    library.gng_visibility_projection.argtypes = [ctypes.c_double, floats, integers, floats,
                                                  floats, ctypes.c_int, pixels, depths]
    library.gng_visibility_projection.restype = ctypes.c_int
    library.gng_visibility_shadow_build.argtypes = [size, floats, size, integers, size, integers,
        ctypes.c_double, floats, integers, floats, floats, ctypes.c_int, pixels, depths, size, flags, integers]
    library.gng_visibility_shadow_build.restype = ctypes.c_int
    return library


def cloud_points(message):
    """PointCloud2の配置・行余白・バイト順を保持したXYZ抽出。全フィールドの辞書化なし。"""
    fields = {field.name: field for field in message.fields}
    formats, offsets = [], []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('観測点群にはfloat32/float64のXYZが必要です')
        code = 'f4' if field.datatype == 7 else 'f8'
        if field.offset < 0 or field.offset+int(code[1:]) > message.point_step:
            raise ValueError('観測点群のフィールド配置が不正です')
        formats.append(('>' if message.is_bigendian else '<')+code)
        offsets.append(field.offset)
    if (message.width*message.height > 2000000 or message.row_step < message.width*message.point_step
            or len(message.data) < message.height*message.row_step):
        raise ValueError('観測点群の配列長が不正です')
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats, offsets=offsets, itemsize=message.point_step))
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step))
    points = np.column_stack([values[name].ravel() for name in ('x', 'y', 'z')]).astype(np.float64)
    has_return = np.all(np.isfinite(points), axis=1) & (np.sum(points*points, axis=1) > 1e-12)
    return np.ascontiguousarray(points[has_return])


class roi_visibility:
    """フレームごとに失効する観測スナップショット。過去freeの無期限蓄積なし。"""
    def __init__(self, settings):
        self.library = visibility_kernel()
        self.max_age_sec = .3
        self.max_pair_age_sec = .15
        self.max_ray_steps = 2000000
        self.max_query_steps = 200000
        self.enable_ray_reuse = True
        self.model = settings.get('visibility_model', 'fov_shadow')
        if self.model not in ('fov_shadow', 'ray'):
            raise ValueError('未対応の観測方式: '+str(self.model))
        # MID-360の公称視野・距離。センサ座標系の上下角と近距離死角。
        self.sensor_limits = np.array([.1, 40., np.tan(np.deg2rad(-7.)), np.tan(np.deg2rad(52.))])
        self.cube_resolution = 128
        self.projection = None
        self.num_projection_builds = 0
        self.min_bounds = np.asarray(settings.get('visibility_min_bounds', [-1.2, -1.2, -1.2]), dtype=float)
        self.max_bounds = np.asarray(settings.get('visibility_max_bounds', [1.2, 1.2, 1.2]), dtype=float)
        if (self.min_bounds.shape != (3,) or self.max_bounds.shape != (3,)
                or not np.all(np.isfinite([self.min_bounds, self.max_bounds]))
                or np.any(self.min_bounds >= self.max_bounds)):
            raise ValueError('観測ROI範囲の不正')
        self.lock = threading.Lock()
        self.points = deque(maxlen=8)
        self.current = None
        self.num_updates = self.num_rejections = self.num_queries = 0
        self.total_build_ms = self.max_build_ms = self.total_query_ms = self.max_query_ms = 0.
        self.message = 'センサ観測待機'
        self.last_reason = 'unavailable'
        self.rejection_counts = Counter()
        self.details = {}

    def add_points(self, message):
        with self.lock:
            has_rollback = bool(self.points and stamp_sec(message) < stamp_sec(self.points[-1][0])-1.)
            if has_rollback:
                self.points.clear()
                self.current = None
            self.points.append((message, time.monotonic()))
            return has_rollback

    def match(self, roi):
        with self.lock:
            now = time.monotonic()
            candidates = [item for item in self.points if now-item[1] <= self.max_age_sec]
        if not candidates:
            return None
        result = min(candidates, key=lambda item: abs(stamp_sec(item[0])-stamp_sec(roi)))
        return result if abs(stamp_sec(result[0])-stamp_sec(roi)) <= self.max_pair_age_sec else None

    def invalidate(self, message):
        with self.lock:
            self.current = None
            self.message = message

    def has_fresh_observation(self):
        with self.lock:
            return self.current is not None and time.monotonic()-self.current[4] <= self.max_age_sec

    def grid_projection(self, step, origin, layout, transform):
        """格子・センサ取付姿勢だけの座標キャッシュ。観測状態の持越しなし。"""
        key = step, origin.tobytes(), layout.tobytes(), self.sensor_limits.tobytes(), self.cube_resolution
        # TF合成の浮動小数点丸めだけの同一視。取付位置・姿勢変化の追従は維持。
        if (self.projection is not None and self.projection[0] == key
                and np.allclose(self.projection_transform, transform, rtol=0., atol=1e-12)):
            return self.projection[1:]
        size = int(np.prod(layout[3:6]))
        pixels, depths = np.empty(size, dtype=np.int32), np.empty(size, dtype=np.float32)
        code = self.library.gng_visibility_projection(step, origin, layout, transform,
            self.sensor_limits, self.cube_resolution, pixels, depths)
        if code:
            raise ValueError('視野座標キャッシュの格子契約違反')
        self.projection = key, pixels, depths
        self.projection_transform = transform.copy()
        self.num_projection_builds += 1
        return pixels, depths

    def build(self, roi, supported, cloud, transform, received_sec, *, self_mask=None, control=None):
        """既存IDの再利用と、視野推定／実光線の選択。フレームごとの状態更新。"""
        begin = time.perf_counter()
        transform = np.ascontiguousarray(transform, dtype=np.float64)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError('観測座標変換の不正')
        # 格子契約の検証だけ。ID復号・再登録の重複なし。
        if (not np.isfinite(roi.voxel_size) or not .001 <= roi.voxel_size <= 1.
                or sorted((roi.x_shift, roi.y_shift, roi.z_shift)) != [0, 21, 42]
                or not np.all(np.isfinite([roi.origin_x, roi.origin_y, roi.origin_z]))):
            raise ValueError('観測格子の不正')
        step = float(roi.voxel_size)
        origin = np.array([roi.origin_x, roi.origin_y, roi.origin_z], dtype=np.float64)
        low = np.floor((self.min_bounds-origin)/step).astype(np.int64)
        high = np.ceil((self.max_bounds-origin)/step).astype(np.int64)
        shape = high-low
        if (np.any(shape > 512) or np.any(shape <= 0) or np.prod(shape, dtype=np.float64) > 8000000
                or np.any(low+roi.offset < 0) or np.any(high+roi.offset > (1 << 21))):
            raise ValueError('観測ROIのセル数上限超過')
        layout = np.array([*low, *shape, roi.x_shift, roi.y_shift, roi.z_shift, roi.offset], dtype=np.int64)
        points = cloud_points(cloud)
        if not len(points):
            self.invalidate('有効な測距なし')
            return
        for other in (self_mask, control):
            if other is not None and any(getattr(other, name) != getattr(roi, name) for name in (
                    'voxel_size', 'origin_x', 'origin_y', 'origin_z', 'x_shift', 'y_shift', 'z_shift', 'offset')):
                raise ValueError('遮蔽・障害物入力とROIの格子不一致')
        blocked = np.asarray(roi.data, dtype=np.int64)
        obstacles = np.asarray((control if control is not None and self.model == 'fov_shadow' else supported).data,
                               dtype=np.int64)
        states = np.empty(int(np.prod(shape)), dtype=np.uint8)
        counts = np.zeros(4, dtype=np.int64)
        pixels = None
        cache_begin = time.perf_counter()
        if self.model == 'fov_shadow':
            pixels, depths = self.grid_projection(step, origin, layout, transform)
            cache_ms = (time.perf_counter()-cache_begin)*1000
            if self_mask is not None:
                blocked = np.concatenate((blocked, np.asarray(self_mask.data, dtype=np.int64)))
            code = self.library.gng_visibility_shadow_build(len(points), points, len(blocked), blocked,
                len(obstacles), obstacles, step, origin, layout, transform, self.sensor_limits,
                self.cube_resolution, pixels, depths, self.max_ray_steps, states, counts)
        else:
            cache_ms = 0.
            points = np.ascontiguousarray(points@transform[:3, :3].T+transform[:3, 3])
            sensor = np.ascontiguousarray(transform[:3, 3])
            code = self.library.gng_visibility_build(len(points), points, sensor, len(blocked), blocked,
                len(obstacles), obstacles, step, origin, layout, self.max_ray_steps, int(self.enable_ray_reuse), states, counts)
        if code < 0:
            raise ValueError('観測カーネルの格子・点群契約違反')
        elapsed_ms = (time.perf_counter()-begin)*1000
        with self.lock:
            self.num_updates += 1
            self.total_build_ms += elapsed_ms
            self.max_build_ms = max(self.max_build_ms, elapsed_ms)
            snapshot = (states, step, origin, layout, received_sec)
            if pixels is not None:
                snapshot += (pixels,)
            self.current = None if code else snapshot
            self.message = '観測処理上限による無効化' if code else ''
            self.details = dict(num_free_voxels=int(counts[2]), num_obstacle_voxels=int(counts[3]),
                num_rays=int(counts[0]) if self.model == 'ray' else 0,
                num_occluders=int(counts[0]) if self.model == 'fov_shadow' else 0,
                num_unknown_voxels=len(states)-int(counts[2]+counts[3]),
                num_ray_steps=int(counts[1]) if self.model == 'ray' else 0,
                num_projection_steps=int(counts[1]) if self.model == 'fov_shadow' else 0,
                visibility_model=self.model, is_inferred_free=self.model == 'fov_shadow',
                cache_build_ms=cache_ms, num_projection_builds=self.num_projection_builds,
                source_stamp_sec=stamp_sec(cloud), pair_age_ms=abs(stamp_sec(cloud)-stamp_sec(roi))*1000,
                voxel_size_m=step, min_bounds_m=(origin+low*step).tolist(),
                max_bounds_m=(origin+high*step).tolist(), build_ms=elapsed_ms)

    def can_bridge(self, geometry, first, second):
        """開始身体以外へ新規進入するセルの観測確認。障害物・予算超過の即時棄却。"""
        if np.array_equal(first, second):
            return True
        begin = time.perf_counter()
        with self.lock:
            snapshot = self.current
        reason = 'unavailable'
        code = 1
        if snapshot is not None and time.monotonic()-snapshot[4] <= self.max_age_sec:
            states, step, origin, layout, _ = snapshot[:5]
            count = max(1, int(np.ceil(np.max(np.abs(second-first))/.025)))
            if count > 4096:
                reason = 'budget'
            else:
                poses = first+(second-first)*np.linspace(0, 1, count+1)[:, None]
                centers = geometry.centers(poses)[:, geometry.is_arm]
                initial = np.ascontiguousarray(centers[0])
                radii = np.ascontiguousarray(geometry.radii[geometry.is_arm]+.01)
                # サンプル間の中心線分を覆う球。関節補間曲線に対する厳密な連続証明ではない近似。
                padding = np.ascontiguousarray(np.linalg.norm(np.diff(centers, axis=0), axis=2)*.5)
                middle = np.ascontiguousarray((centers[:-1]+centers[1:])*.5)
                counts = np.zeros(2, dtype=np.int64)
                code = self.library.gng_visibility_check(states, step, origin, layout, len(radii),
                    initial, radii, count, middle, padding, self.max_query_steps, counts)
                if code < 0:
                    raise ValueError('観測照会の姿勢・形状契約違反')
                reason = ('free', 'unknown', 'obstacle', 'budget', 'outside_roi')[code]
                if code == 1 and len(snapshot) > 5 and snapshot[5] is not None and counts[1] >= 0:
                    pixel = snapshot[5][int(counts[1])]
                    reason = 'outside_fov' if pixel == -1 else 'blind_zone' if pixel == -2 else 'occluded'
        elapsed_ms = (time.perf_counter()-begin)*1000
        with self.lock:
            self.num_queries += 1
            self.num_rejections += int(code != 0)
            self.total_query_ms += elapsed_ms
            self.max_query_ms = max(self.max_query_ms, elapsed_ms)
            self.last_reason = reason
            if code:
                self.rejection_counts[reason] += 1
        return code == 0

    def status(self):
        with self.lock:
            age = None if self.current is None else max(0., time.monotonic()-self.current[4])
            return dict(self.details, is_enabled=True, is_fresh=age is not None and age <= self.max_age_sec,
                age_sec=age, message=self.message, last_reason=self.last_reason,
                rejection_counts=dict(self.rejection_counts),
                num_updates=self.num_updates, num_queries=self.num_queries, num_rejections=self.num_rejections,
                build_mean_ms=self.total_build_ms/max(1, self.num_updates), build_max_ms=self.max_build_ms,
                query_mean_ms=self.total_query_ms/max(1, self.num_queries), query_max_ms=self.max_query_ms)

    def free_message(self, roi):
        """購読時だけのfreeセル配信。未列挙セルは公開範囲内でunknown。"""
        import copy
        from array import array
        with self.lock:
            snapshot = self.current
        result = copy.copy(roi)
        result.header = copy.deepcopy(roi.header)
        result.labels = array('B')
        result.data = array('q')
        if snapshot is None or time.monotonic()-snapshot[4] > self.max_age_sec:
            return result
        states, _, _, layout, _ = snapshot[:5]
        flat = np.flatnonzero(states == 1)
        cells = np.column_stack(np.unravel_index(flat, layout[3:6]))+layout[:3]+layout[9]
        ids = np.zeros(len(cells), dtype=np.int64)
        for axis in range(3):
            ids |= cells[:, axis] << layout[6+axis]
        result.data.frombytes(ids.tobytes())
        return result
