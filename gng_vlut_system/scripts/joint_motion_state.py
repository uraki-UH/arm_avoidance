"""関節の位置・速度・加速度・ジャークと、任意次数の観測微分。"""
from collections import deque
from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


motion_fields = ('position', 'velocity', 'acceleration', 'jerk')


@dataclass(frozen=True)
class joint_motion_state:
    """同一時刻の関節状態。未取得・未計算はNone、推定項目は別途保持。"""
    stamp_sec: float
    position: Optional[float] = None
    velocity: Optional[float] = None
    acceleration: Optional[float] = None
    jerk: Optional[float] = None
    estimated_fields: tuple = ()


class joint_motion_tracker:
    """関節別の短い観測履歴。制御出力・制限値・ROSへの依存なし。"""

    def __init__(self, max_derivative_order=3, min_sample_period_sec=0.0001,
                 max_sample_gap_sec=0.5, continuous_joint_names=()):
        if type(max_derivative_order) is not int or not 0 <= max_derivative_order <= 3:
            raise ValueError('max_derivative_orderは0・1・2・3のいずれかが必要です')
        if (not math.isfinite(min_sample_period_sec) or min_sample_period_sec <= 0 or
                not math.isfinite(max_sample_gap_sec) or max_sample_gap_sec < min_sample_period_sec):
            raise ValueError('観測周期と履歴間隔には整合した正の有限値が必要です')
        self._max_derivative_order = max_derivative_order
        self._min_sample_period_sec = min_sample_period_sec
        self._max_sample_gap_sec = max_sample_gap_sec
        self._continuous_joint_names = frozenset(continuous_joint_names)
        self._last_stamp = {}
        self._history = {}

    def reset(self):
        """再生開始・座標系変更などに伴う全観測履歴の破棄。"""
        self._last_stamp.clear()
        self._history.clear()

    def update(self, stamp_sec, names, positions=(), velocities=(), accelerations=(), jerks=()):
        """指定関節だけの更新結果。入力済みの値を優先し、欠損項目のみ推定。"""
        if not math.isfinite(stamp_sec) or stamp_sec < 0:
            raise ValueError('観測時刻には非負の有限値が必要です')
        if len(set(names)) != len(names) or any(not isinstance(name, str) or not name for name in names):
            raise ValueError('関節名には重複しない空でない文字列が必要です')
        arrays = (positions, velocities, accelerations, jerks)
        if any(len(values) not in (0, len(names)) for values in arrays):
            raise ValueError('関節値の配列は空または関節名と同じ長さが必要です')
        # 入力全体の検証後に履歴を更新。非有限値は関節・項目ごとの欠損
        rows = [[float(values[idx]) if len(values) and math.isfinite(values[idx]) else None
                 for values in arrays] for idx in range(len(names))]
        result = {}
        for name, values in zip(names, rows):
            previous_stamp = self._last_stamp.get(name)
            if previous_stamp is not None:
                period_sec = stamp_sec - previous_stamp
                if 0 <= period_sec < self._min_sample_period_sec:
                    continue
                if period_sec < 0 or period_sec > self._max_sample_gap_sec:
                    self._history.pop(name, None)
            self._last_stamp[name] = stamp_sec
            estimated_fields = []
            if self._max_derivative_order:
                observed = tuple(values)
                if name not in self._history:
                    self._history[name] = [deque(maxlen=self._max_derivative_order - order + 1)
                                           for order in range(self._max_derivative_order)]
                history = self._history[name]
                for order, samples in enumerate(history):
                    value = observed[order]
                    if value is None:
                        samples.clear()
                        continue
                    if order == 0 and name in self._continuous_joint_names and samples:
                        value = samples[-1][1] + math.remainder(value - samples[-1][1], 2 * math.pi)
                    samples.append((stamp_sec, value))
                for order in range(1, self._max_derivative_order + 1):
                    if observed[order] is not None:
                        continue
                    # 推定値同士の再微分を回避。現在取得済みの最も高い次数が微分元
                    source_order = next((source for source in range(order - 1, -1, -1)
                                         if observed[source] is not None), None)
                    if source_order is None:
                        continue
                    derivative_order = order - source_order
                    samples = history[source_order]
                    if len(samples) <= derivative_order:
                        continue
                    value = self._differentiate(samples, derivative_order)
                    if value is not None:
                        values[order] = value
                        estimated_fields.append(motion_fields[order])
            result[name] = joint_motion_state(stamp_sec, *values, tuple(estimated_fields))
        return result

    @staticmethod
    def _differentiate(samples, derivative_order):
        """不等間隔の後退補間を現在時刻で微分。時刻の平行移動と正規化付き。"""
        times, values = np.asarray(samples, dtype=float).T
        span_sec = times[-1] - times[0]
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                coefficients = np.polynomial.polynomial.polyfit(
                    (times - times[-1]) / span_sec, values - values[-1], len(samples) - 1)
                value = coefficients[derivative_order] * math.factorial(derivative_order) / span_sec**derivative_order
        except (FloatingPointError, np.linalg.LinAlgError):
            return None
        return float(value) if math.isfinite(value) else None
