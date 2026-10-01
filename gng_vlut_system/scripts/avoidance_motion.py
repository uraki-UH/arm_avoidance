"""回避動作の入力フラグ・優先順位・目標生成部品。ROS非依存。"""
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class motion_flags:
    is_stop_requested: bool = False
    has_valid_input: bool = True
    has_active_joints: bool = False
    has_safe_neighbors: bool = False
    can_finish_retreat: bool = False
    can_return: bool = False
    is_home: bool = False


def select_motion(flags):
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


def hold_target(state, home, step):
    """現在姿勢の保持目標。"""
    return state.positions.copy(), True, False


def return_target(state, home, step):
    """対象関節の復帰目標。速度・区間安全検査は共通出力段。"""
    return home.copy(), True, False


def stop_target(state, home, step):
    """停止姿勢の保持目標。駆動停止ラッチ自体は実行系の管轄。"""
    return state.positions.copy(), True, False


def retreat_target(state, home, step):
    """グラフ退避部品への委譲。"""
    return state.retreat_target(step)


@dataclass(frozen=True)
class motion_components:
    """各動作の差替え口。戻り値は目標姿勢・候補有無・GNG経路使用。"""
    retreat: Callable = retreat_target
    returning: Callable = return_target
    hold: Callable = hold_target
    stop: Callable = stop_target

    def execute(self, action, state, home, step):
        if action == 'fault':
            return state.positions.copy(), False, False
        component = {'avoiding': self.retreat, 'returning': self.returning,
                     'waiting_for_clearance': self.hold, 'monitoring': self.hold,
                     'stopped': self.stop}[action]
        return component(state, home, step)
