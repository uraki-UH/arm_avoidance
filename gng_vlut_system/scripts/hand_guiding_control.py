"""手動操作の外力判定・保持目標・保持トルクとROS入力の共通部品。"""
import math
import time


def numeric_values(value, num, name):
    """スカラまたは軸順配列の有限な非負数値。"""
    values = [value] * num if isinstance(value, (int, float)) else value
    if (not hasattr(values, '__len__') or len(values) != num or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) or v < 0 for v in values)):
        raise ValueError(name + ': 非負の有限値または軸順配列が必要です')
    return list(map(float, values))


class adaptive_hold:
    """操作中の保持解除と、静止後の新しい角度での保持。モータ座標・N m。"""

    def __init__(self, groups, hold_gain, max_hold_torque, min_release_effort_th,
                 max_hold_effort_th, max_hold_velocity_th, min_hold_duration_sec,
                 release_ramp_sec, hold_ramp_sec):
        self.groups = tuple(groups)
        num = len(groups)
        if not num:
            raise ValueError('保持対象の軸が必要です')
        self.hold_gain = numeric_values(hold_gain, num, 'hold_gain_nm_per_rad')
        self.max_hold_torque = numeric_values(max_hold_torque, num, 'max_hold_torque_nm')
        self.min_release_effort_th = numeric_values(min_release_effort_th, num, 'min_release_effort_th')
        self.max_hold_effort_th = numeric_values(max_hold_effort_th, num, 'max_hold_effort_th')
        values = (max_hold_velocity_th, min_hold_duration_sec, release_ramp_sec, hold_ramp_sec)
        if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) or v <= 0 for v in values) or
                any(gain <= 0 or limit <= 0 or start <= stop for gain, limit, start, stop in
                    zip(self.hold_gain, self.max_hold_torque, self.min_release_effort_th, self.max_hold_effort_th))):
            raise ValueError('保持ゲイン・上限・時間・速度の正数と、解除／保持の閾値差が必要です')
        self.max_hold_velocity_th = max_hold_velocity_th
        self.min_hold_duration_sec = min_hold_duration_sec
        self.release_ramp_sec = release_ramp_sec
        self.hold_ramp_sec = hold_ramp_sec
        self.group_axes = [tuple(idx for idx, key in enumerate(groups) if key == group)
                           for group in dict.fromkeys(groups)]
        self.is_guiding = [False] * len(self.group_axes)
        self.blend = [0.] * len(self.group_axes)
        self.still_sec = [None] * len(self.group_axes)
        self.target = None
        self.last_sec = None

    def update(self, now, positions, velocities, external_effort=None, allow_guiding=None):
        """1制御周期の保持トルク。外力入力と操作許可のどちらか一方。"""
        num = len(self.groups)
        samples = (positions, velocities) + (() if external_effort is None else (external_effort,))
        if (any(len(values) != num or any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in values) for values in samples) or
                isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now) or
                (self.last_sec is not None and now < self.last_sec) or
                (external_effort is None) == (allow_guiding is None) or
                (allow_guiding is not None and not isinstance(allow_guiding, bool))):
            raise ValueError('保持制御の時刻・軸数・有限値・操作入力が不正です')
        if self.target is None:
            self.target = list(positions)
        duration_sec = 0. if self.last_sec is None else now - self.last_sec
        self.last_sec = now
        torques = [0.] * num
        for group_idx, axes in enumerate(self.group_axes):
            has_release = (allow_guiding if external_effort is None else
                           any(abs(external_effort[idx]) >= self.min_release_effort_th[idx] for idx in axes))
            can_hold = ((not allow_guiding if external_effort is None else
                         all(abs(external_effort[idx]) <= self.max_hold_effort_th[idx] for idx in axes)) and
                        all(abs(velocities[idx]) <= self.max_hold_velocity_th for idx in axes) and
                        self.blend[group_idx] == 0.)
            if has_release:
                self.is_guiding[group_idx] = True
                self.still_sec[group_idx] = None
            elif self.is_guiding[group_idx]:
                if not can_hold:
                    self.still_sec[group_idx] = None
                elif self.still_sec[group_idx] is None:
                    self.still_sec[group_idx] = now
                elif now - self.still_sec[group_idx] >= self.min_hold_duration_sec:
                    # 手動操作後の角度の一度だけの取り込み。保持中の垂れへの追従なし
                    self.is_guiding[group_idx] = False
                    self.still_sec[group_idx] = None
                    for idx in axes:
                        self.target[idx] = positions[idx]
            if self.is_guiding[group_idx]:
                self.blend[group_idx] = max(0., self.blend[group_idx] - duration_sec / self.release_ramp_sec)
            else:
                self.blend[group_idx] = min(1., self.blend[group_idx] + duration_sec / self.hold_ramp_sec)
            for idx in axes:
                torque = self.hold_gain[idx] * (self.target[idx] - positions[idx])
                torques[idx] = self.blend[group_idx] * max(-self.max_hold_torque[idx], min(self.max_hold_torque[idx], torque))
        return torques

    def report(self):
        return {'is_guiding': list(self.is_guiding), 'hold_blend': list(self.blend),
                'hold_target_rad': None if self.target is None else list(self.target)}


class guiding_input:
    """保持制御の設定と外力／操作許可入力。電流指令・モータモード変更なし。"""

    def __init__(self, node):
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        self.node = node
        fixed = ParameterDescriptor(read_only=True)
        numeric = ParameterDescriptor(read_only=True, dynamic_typing=True)
        def param(name, value, descriptor=fixed):
            return node.declare_parameter(name, value, descriptor).value
        self.source = param('interaction_source', 'manual')
        self.max_interaction_age_sec = param('max_interaction_age_sec', .3, numeric)
        if (self.source not in ('manual', 'external_effort') or
                isinstance(self.max_interaction_age_sec, bool) or
                not isinstance(self.max_interaction_age_sec, (int, float)) or
                not math.isfinite(self.max_interaction_age_sec) or self.max_interaction_age_sec <= 0):
            raise ValueError('操作入力の種類と正の有効期限が必要です')
        groups = (0,) * 2 if len(node.ids) == 2 else (0,) * 7 + (1,) * 7
        self.control = adaptive_hold(groups,
            param('hold_gain_nm_per_rad', .5, numeric), param('max_hold_torque_nm', .2, numeric),
            param('min_release_effort_th', .15, numeric), param('max_hold_effort_th', .05, numeric),
            param('max_hold_velocity_th', .02, numeric), param('min_hold_duration_sec', .3, numeric),
            param('release_ramp_sec', .15, numeric), param('hold_ramp_sec', .5, numeric))
        self.sample_sec = -math.inf
        self.sample_stamp = -1
        self.sample = None
        if self.source == 'manual':
            topic = param('manual_guiding_topic', '~/allow_guiding')
            self.subscription = node.create_subscription(Bool, topic, self.on_manual, qos_profile_sensor_data)
        else:
            topic = param('external_effort_topic', '~/external_effort')
            self.subscription = node.create_subscription(JointState, topic, self.on_effort, qos_profile_sensor_data)
        self.topic = self.subscription.topic_name

    def on_manual(self, message):
        self.sample, self.sample_sec = message.data, time.monotonic()

    def on_effort(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (time.time_ns() - stamp) * 1e-9
        names = tuple(map(str, self.node.ids))
        if (message.header.frame_id != 'dynamixel_motor' or len(set(message.name)) != len(message.name) or
                len(message.name) != len(message.effort) or not set(names) <= set(message.name) or
                any(not math.isfinite(value) for value in message.effort) or
                not 0 <= age < self.max_interaction_age_sec):
            self.sample = None
            return
        if stamp <= self.sample_stamp:
            return
        values = dict(zip(message.name, message.effort))
        self.sample = [values[name] for name in names]
        self.sample_stamp = stamp
        self.sample_sec = time.monotonic() - age

    def has_fresh_input(self, now):
        return self.sample is not None and 0 <= now - self.sample_sec < self.max_interaction_age_sec

    def check_ready(self, now):
        if not self.has_fresh_input(now):
            raise ValueError('手動操作の許可／外力入力の欠測または失効')
        if self.node.count_publishers(self.topic) != 1:
            raise ValueError('手動操作入力の配信元は1個が必要です')

    def update(self, now):
        self.check_ready(now)
        positions = [self.node.joints[motor_id][2] for motor_id in self.node.ids]
        velocities = [self.node.joints[motor_id][1] for motor_id in self.node.ids]
        if self.source == 'manual':
            return self.control.update(now, positions, velocities, allow_guiding=self.sample)
        return self.control.update(now, positions, velocities, external_effort=self.sample)

    def report(self):
        return {'interaction_source': self.source, 'interaction_topic': self.topic,
                'has_fresh_interaction': self.has_fresh_input(time.monotonic()), **self.control.report()}


def setup_guiding(node):
    """選択した制御だけの生成。重力補償モードでは追加入力・保持計算なし。"""
    from rcl_interfaces.msg import ParameterDescriptor
    mode = node.declare_parameter('control_mode', 'gravity', ParameterDescriptor(read_only=True)).value
    if mode not in ('gravity', 'adaptive_hold'):
        raise ValueError('control_modeにはgravityまたはadaptive_holdが必要です')
    return None if mode == 'gravity' else guiding_input(node)
