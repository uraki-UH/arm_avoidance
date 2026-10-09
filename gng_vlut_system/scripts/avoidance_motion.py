"""回避動作の入力フラグ・優先順位・目標生成部品。ROS非依存。"""
from dataclasses import dataclass
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class motion_flags:
    is_stop_requested: bool = False
    has_valid_input: bool = True
    has_active_joints: bool = False
    has_safe_neighbors: bool = False
    can_finish_retreat: bool = False
    can_return: bool = False
    is_home: bool = False


def select_motion(flags: motion_flags) -> str:
    """停止・入力失効を最優先とする動作選択。"""
    if flags.is_stop_requested:
        return 'stopped'
    if not flags.has_valid_input:
        return 'fault'
    if not flags.has_active_joints:
        return 'monitoring'
    if not flags.has_safe_neighbors or not flags.can_finish_retreat:
        return 'avoiding'
    if not flags.can_return:
        return 'waiting_for_clearance'
    return 'monitoring' if flags.is_home else 'returning'


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

    def execute(self, action: str, request: motion_input) -> motion_result:
        if action == 'fault':
            return motion_result(request.positions.copy(), False)
        component = {'avoiding': self.retreat, 'returning': self.returning,
                     'waiting_for_clearance': self.hold, 'monitoring': self.hold,
                     'stopped': self.stop}[action]
        return component(request)
