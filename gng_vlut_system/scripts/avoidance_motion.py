"""回避動作の入力フラグ・優先順位・目標生成部品。ROS非依存。"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable

import numpy as np


class motion_kind(str, Enum):
    """動作選択の共通語彙。診断・既存設定の文字列表現との互換。"""
    stopped = 'stopped'
    fault = 'fault'
    monitoring = 'monitoring'
    avoiding = 'avoiding'
    waiting_for_clearance = 'waiting_for_clearance'
    returning = 'returning'

    def __str__(self):
        return self.value


@dataclass(frozen=True)
class motion_flags:
    is_stop_requested: bool = False
    has_valid_input: bool = True
    has_active_joints: bool = False
    has_safe_neighbors: bool = False
    can_finish_retreat: bool = False
    can_return: bool = False
    is_home: bool = False


def select_motion(flags: motion_flags) -> motion_kind:
    """停止・入力失効を最優先とする動作選択。"""
    if flags.is_stop_requested:
        return motion_kind.stopped
    if not flags.has_valid_input:
        return motion_kind.fault
    if not flags.has_active_joints:
        return motion_kind.monitoring
    if not flags.has_safe_neighbors or not flags.can_finish_retreat:
        return motion_kind.avoiding
    if not flags.can_return:
        return motion_kind.waiting_for_clearance
    return motion_kind.monitoring if flags.is_home else motion_kind.returning


@dataclass(frozen=True)
class motion_input:
    """動作部品への姿勢スナップショットと制御刻み幅。"""
    positions: np.ndarray
    home: np.ndarray
    step: float


@dataclass(frozen=True)
class motion_result:
    """共通制約の適用前の候補目標と生成結果。"""
    target: np.ndarray
    has_candidate: bool
    is_gng_target: bool = False


motion_component = Callable[[motion_input], motion_result]


def hold_target(request: motion_input) -> motion_result:
    """現在姿勢の保持目標。"""
    return motion_result(request.positions.copy(), True)


def return_target(request: motion_input) -> motion_result:
    """対象関節の復帰目標。速度・区間安全検査は共通出力段。"""
    return motion_result(request.home.copy(), True)


def stop_target(request: motion_input) -> motion_result:
    """停止姿勢の保持目標。駆動停止ラッチ自体は実行系の管轄。"""
    return hold_target(request)


@dataclass(frozen=True)
class motion_components:
    """ROSノードに依存しない動作部品。退避計算のみ実行系からの注入。"""
    retreat: motion_component
    returning: motion_component = return_target
    hold: motion_component = hold_target
    stop: motion_component = stop_target
    _components: dict = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        components = {
            motion_kind.avoiding: self.retreat,
            motion_kind.returning: self.returning,
            motion_kind.waiting_for_clearance: self.hold,
            motion_kind.monitoring: self.hold,
            motion_kind.stopped: self.stop,
        }
        if not all(callable(component) for component in components.values()):
            raise ValueError('動作部品には呼出し可能な関数が必要です')
        object.__setattr__(self, '_components', components)

    def execute(self, action: motion_kind | str, request: motion_input) -> motion_result:
        action = motion_kind(action)
        if action is motion_kind.fault:
            return motion_result(request.positions.copy(), False)
        return self._components[action](request)
