"""排他的な回避・リーダー追従の実測検査と速度制限。"""

import math

from joint_command_model import joint_command_model


def is_finite(value):
    """真偽値を除く有限な時刻・関節値の判定。"""
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except OverflowError:
        return False


class mode_model:
    """ROSに依存しないモード状態・実測保持・リーダー目標補間。"""

    def __init__(self, urdf_path, max_joint_velocity=0.3, max_state_age_sec=0.5):
        if (not is_finite(max_joint_velocity) or max_joint_velocity <= 0 or
                not is_finite(max_state_age_sec) or max_state_age_sec <= 0):
            raise ValueError('速度制限・実測有効期間には正の有限値が必要です')
        self.model = joint_command_model(urdf_path)
        self.max_joint_velocity = max_joint_velocity
        self.max_state_age_sec = max_state_age_sec
        self.mode = 'stopped'
        self.measured = {}
        self.max_velocity = math.inf
        self.commanded = {}
        self.has_state = False
        self.has_leader = False
        self.state_stamp_sec = None
        self.state_received_sec = None
        self.leader_stamp_sec = None
        self.leader_received_sec = None
        self.stationary_start_sec = None
        self.entered_sec = None
        self.last_sim_sec = None
        # 開始可否用の入力キャッシュと、切替後の有効目標の分離
        self.leader = {}
        self.targets = {}
        self.target_received_sec = {}
        self.has_new_leader = False
        self.has_leader_failure = False

    def is_fresh(self, now):
        return (self.has_state and is_finite(now) and self.state_received_sec is not None and
                0 <= now-self.state_received_sec <= self.max_state_age_sec)

    def has_fresh_leader(self, now):
        return (self.has_leader and is_finite(now) and self.leader_received_sec is not None and
                0 <= now-self.leader_received_sec <= self.max_state_age_sec)

    def invalidate_state(self):
        """異常入力後の実測・連続静止判定の失効。"""
        self.has_state = False
        self.max_velocity = math.inf
        self.stationary_start_sec = None

    def invalidate_leader(self):
        """異常入力後の開始可否キャッシュ・旧目標の失効。"""
        self.has_leader = False
        self.has_leader_failure = True
        self.leader.clear()
        self.targets.clear()
        self.target_received_sec.clear()

    def update_state(self, names, positions, velocities, stamp_sec, now):
        try:
            if (not is_finite(stamp_sec) or not is_finite(now) or
                    len(names) != len(velocities) or not names or
                    not all(is_finite(value) for value in positions) or
                    not all(is_finite(value) for value in velocities) or
                    not set(self.model.independent_names).issubset(names) or
                    (self.state_stamp_sec is not None and stamp_sec <= self.state_stamp_sec) or
                    (self.state_received_sec is not None and now < self.state_received_sec)):
                raise ValueError('実測関節・速度・更新時刻が不正です')
            values = self.model.feedback_positions(names, positions)
        except (ValueError, TypeError, OverflowError):
            self.invalidate_state()
            return False
        has_continuous_state = self.is_fresh(now)
        self.measured = values
        self.max_velocity = max(abs(value) for value in velocities)
        self.state_stamp_sec, self.state_received_sec = stamp_sec, now
        self.has_state = True
        if self.max_velocity > 0.01:
            self.stationary_start_sec = None
        elif self.stationary_start_sec is None or not has_continuous_state:
            self.stationary_start_sec = stamp_sec
        return True

    def update_leader(self, names, positions, stamp_sec, now):
        try:
            if (not names or not is_finite(stamp_sec) or not is_finite(now) or
                    not all(is_finite(value) for value in positions) or
                    (self.leader_stamp_sec is not None and stamp_sec <= self.leader_stamp_sec) or
                    (self.leader_received_sec is not None and now < self.leader_received_sec)):
                raise ValueError('リーダー関節・更新時刻が不正です')
            values = self.model.canonical_positions(names, positions)
            if any(not self.model.bounds[name][0] <= value <= self.model.bounds[name][1]
                   for name, value in values.items()):
                raise ValueError('リーダー目標が可動範囲外です')
        except (ValueError, TypeError, OverflowError):
            self.invalidate_leader()
            return False
        self.leader = values
        self.leader_stamp_sec, self.leader_received_sec = stamp_sec, now
        self.has_leader, self.has_leader_failure = True, False
        if self.mode == 'leader':
            self.targets.update(values)
            self.target_received_sec.update({name: now for name in values})
            self.has_new_leader = True
        return True

    def is_stationary(self, now):
        return (self.is_fresh(now) and self.max_velocity <= 0.01 and
                self.stationary_start_sec is not None and
                self.state_stamp_sec-self.stationary_start_sec >= 0.25-1e-12)

    def enter(self, mode, now):
        if mode not in ('hold', 'leader', 'avoidance'):
            raise ValueError('未対応の制御モードです: '+str(mode))
        if not self.is_fresh(now):
            raise ValueError('切替には新鮮な全関節実測が必要です')
        if mode == 'leader' and not self.has_fresh_leader(now):
            raise ValueError('切替には新鮮なリーダー入力が必要です')
        if mode == self.mode:
            return
        self.mode, self.entered_sec = mode, now
        self.commanded = dict(self.measured)
        self.targets.clear()
        self.target_received_sec.clear()
        self.has_new_leader, self.has_leader_failure = False, False
        self.last_sim_sec = None

    def stop(self):
        self.mode = 'stopped'
        self.commanded.clear()
        self.invalidate_leader()
        self.has_new_leader = False
        self.entered_sec, self.last_sim_sec = None, None

    def command(self, now, sim_sec):
        if self.mode in ('stopped', 'avoidance'):
            return None
        if not self.is_fresh(now):
            raise ValueError('全関節実測が失効しています')
        if not is_finite(sim_sec) or (self.last_sim_sec is not None and sim_sec < self.last_sim_sec):
            raise ValueError('指令時刻が不正です')
        duration_sec = 0.0 if self.last_sim_sec is None else min(sim_sec-self.last_sim_sec, 0.05)
        self.last_sim_sec = sim_sec
        if self.mode == 'leader':
            if self.has_leader_failure:
                raise ValueError('リーダー入力が不正です')
            if not self.has_new_leader:
                if now-self.entered_sec > 0.5:
                    raise ValueError('切替後のリーダー入力が未受信です')
                return dict(self.commanded)
            if not self.has_fresh_leader(now):
                raise ValueError('リーダー入力が失効しています')
            # 部分指令の省略関節に対する旧目標の無期限保持防止
            targets = {name: value for name, value in self.targets.items()
                       if now-self.target_received_sec[name] <= self.max_state_age_sec}
            self.commanded = self.model.step(self.commanded, targets, duration_sec, self.max_joint_velocity)
        return dict(self.commanded)
