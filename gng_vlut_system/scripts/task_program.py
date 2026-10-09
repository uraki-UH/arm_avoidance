"""ROS非依存のタスク設定・方式登録・中断可能な逐次実行。"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable
import xml.etree.ElementTree as et


class task_kind(str, Enum):
    move = 'move'
    hold = 'hold'


class run_state(str, Enum):
    idle = 'idle'
    running = 'running'
    stopping = 'stopping'
    paused = 'paused'
    succeeded = 'succeeded'
    canceled = 'canceled'
    failed = 'failed'


@dataclass(frozen=True)
class joint_bound:
    min_position: float
    max_position: float
    max_velocity: float


@dataclass(frozen=True)
class joint_sample:
    stamp_sec: float
    positions: tuple
    velocities: tuple


@dataclass(frozen=True)
class motion_point:
    time_sec: float
    positions: tuple
    velocities: tuple
    accelerations: tuple


@dataclass(frozen=True)
class task_spec:
    kind: task_kind
    method: str
    targets: tuple
    planner: Callable
    duration_sec: float = 0.0


@dataclass(frozen=True)
class task_program:
    joint_names: tuple
    bounds: tuple
    tasks: tuple
    limits: dict
    input_sources: tuple = ()


def checked_mapping(value, allowed, label):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError(f'{label}: 辞書と既知キーが必要です ({allowed})')
    return value


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{label}: 有限の正値が必要です')
    return float(value)


def read_joint_bounds(path):
    """URDF内の独立関節と位置・速度制限。"""
    result = {}
    for joint in et.parse(path).getroot().findall('joint'):
        if joint.get('type') == 'fixed' or joint.find('mimic') is not None:
            continue
        limit = joint.find('limit')
        if limit is None or joint.get('type') not in ('revolute', 'prismatic', 'continuous'):
            raise ValueError('制限付きの独立関節が必要です')
        lower, upper = (-math.inf, math.inf) if joint.get('type') == 'continuous' else (
            float(limit.get('lower')), float(limit.get('upper')))
        if math.isnan(lower) or math.isnan(upper) or lower >= upper:
            raise ValueError('関節位置制限が不正です')
        name = joint.get('name')
        if not name or name in result:
            raise ValueError('関節名が不正または重複しています')
        result[name] = joint_bound(lower, upper, positive(float(limit.get('velocity')), name))
    if not result:
        raise ValueError('独立関節が必要です')
    return result


def direct_targets(item, poses, paths):
    checked_mapping(item, ('kind', 'method', 'target'), 'move/direct')
    return (poses[item['target']],)


def waypoint_targets(item, poses, paths):
    checked_mapping(item, ('kind', 'method', 'path'), 'move/waypoints')
    return tuple(poses[name] for name in paths[item['path']])


def position_targets(item, poses, paths):
    checked_mapping(item, ('kind', 'method', 'duration_sec'), 'hold/position')
    return ({},)


@dataclass(frozen=True)
class task_method:
    """方式単位の差替え境界。設定解釈と実行時の区間計画。"""

    resolve_targets: Callable
    plan: Callable


def load_program(data, joint_bounds, methods=None, components=None):
    """未知設定・未実装方式・関節制限違反の起動前検査。"""
    methods = task_methods if methods is None else methods
    checked_mapping(data, ('version', 'defaults', 'methods', 'inputs', 'limits', 'poses', 'paths', 'tasks'), 'program')
    input_sources = ()
    if 'inputs' in data:
        from task_components import bind_sources
        components, input_sources = bind_sources(data['inputs'], tuple(joint_bounds), components)
    if 'methods' in data:
        from task_components import configured_methods
        methods = configured_methods(data['methods'], methods, components, tuple(joint_bounds))
    if type(data.get('version')) is not int or data['version'] != 1:
        raise ValueError('version: 1 が必要です')
    defaults = {'move': 'direct', 'hold': 'position'}
    defaults.update(checked_mapping(data.get('defaults', {}), defaults, 'defaults'))
    for kind, method in defaults.items():
        if not isinstance(method, str) or (task_kind(kind), method) not in methods:
            raise ValueError(f'未対応方式: {kind}/{method}')
    limits = dict(max_velocity=0.2, max_acceleration=0.5, max_feedback_age_sec=0.5,
                  max_task_sec=60.0, max_stopping_sec=5.0, max_position_error_th=0.025,
                  max_stopped_velocity_th=0.03, settle_sec=0.25)
    limits.update(checked_mapping(data.get('limits', {}), limits, 'limits'))
    limits = {key: positive(value, key) for key, value in limits.items()}
    if not joint_bounds:
        raise ValueError('独立関節が必要です')
    poses = data.get('poses', {})
    if not isinstance(poses, dict):
        raise ValueError('poses: 辞書が必要です')
    for name, pose in poses.items():
        if not isinstance(name, str) or not name:
            raise ValueError('姿勢名が不正です')
        checked_mapping(pose, joint_bounds, name)
        if not pose:
            raise ValueError(f'空の姿勢: {name}')
        for joint, value in pose.items():
            bound = joint_bounds[joint]
            if (isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value)
                    or not bound.min_position <= value <= bound.max_position):
                raise ValueError(f'関節目標が不正です: {name}/{joint}')
    paths = data.get('paths', {})
    if not isinstance(paths, dict):
        raise ValueError('paths: 辞書が必要です')
    for name, path in paths.items():
        if not isinstance(name, str) or not isinstance(path, list) or not path:
            raise ValueError('経由点列が不正です')
        if any(not isinstance(pose, str) or pose not in poses for pose in path):
            raise ValueError(f'未知の経由姿勢: {name}')
    items = data.get('tasks')
    if not isinstance(items, list) or not items:
        raise ValueError('空でないtasksが必要です')
    tasks = []
    for item in items:
        try:
            checked_mapping(item, ('kind', 'method', 'target', 'path', 'duration_sec'), 'task')
            kind = task_kind(item['kind'])
            method = item.get('method', defaults[kind.value])
            strategy = methods[(kind, method)]
            targets = strategy.resolve_targets(item, poses, paths)
            duration = positive(item['duration_sec'], 'duration_sec') if kind == task_kind.hold else 0.0
            if duration >= limits['max_task_sec']:
                raise ValueError('保持時間にはタスク期限内の余裕が必要です')
            tasks.append(task_spec(kind, method, targets, strategy.plan, duration))
        except (KeyError, TypeError) as error:
            raise ValueError(f'未対応方式・不足設定・未知参照: {item}') from error
    return task_program(tuple(joint_bounds), tuple(joint_bounds.values()), tuple(tasks), limits, input_sources)


def plan_segment(start, target, bounds, limits):
    """停止姿勢間の5次補間。全区間の解析的な速度・加速度上限。"""
    delta = tuple(end - begin for begin, end in zip(start, target))
    duration = max(0.3, *(max(1.875 * abs(step) / min(limits['max_velocity'], bound.max_velocity),
                              math.sqrt((10 / math.sqrt(3)) * abs(step) / limits['max_acceleration']))
                          for step, bound in zip(delta, bounds)))
    if not math.isfinite(duration) or duration > limits['max_task_sec'] or duration > 3600.0:
        raise ValueError('軌道時間の上限超過')
    num = max(2, math.ceil(duration / 0.05))
    points = []
    for idx in range(num + 1):
        ratio = idx / num
        position = 10 * ratio**3 - 15 * ratio**4 + 6 * ratio**5
        velocity = (30 * ratio**2 - 60 * ratio**3 + 30 * ratio**4) / duration
        acceleration = (60 * ratio - 180 * ratio**2 + 120 * ratio**3) / duration**2
        points.append(motion_point(duration * ratio,
                                  tuple(begin + step * position for begin, step in zip(start, delta)),
                                  tuple(step * velocity for step in delta),
                                  tuple(step * acceleration for step in delta)))
    return tuple(points)


# 方式追加時の変更点は設定解釈・計画関数と登録の1件。
task_methods = {
    (task_kind.move, 'direct'): task_method(direct_targets, plan_segment),
    (task_kind.move, 'waypoints'): task_method(waypoint_targets, plan_segment),
    (task_kind.hold, 'position'): task_method(position_targets, plan_segment),
}


def check_segment(points, start, target, bounds, limits):
    """方式に依存しない出力検査。独自補間の区間内衝突判定は計画側の責務。"""
    if not points or len(points) < 2 or len(points) > 100000:
        raise ValueError('軌道点数不正')
    previous = -1.0
    for point in points:
        if not math.isfinite(point.time_sec) or not previous < point.time_sec <= limits['max_task_sec']:
            raise ValueError('軌道時刻不正')
        previous = point.time_sec
        fields = (point.positions, point.velocities, point.accelerations)
        if any(len(field) != len(bounds) or not all(math.isfinite(value) for value in field) for field in fields):
            raise ValueError('軌道要素数・数値不正')
        for position, velocity, acceleration, bound in zip(*fields, bounds):
            if (not bound.min_position - 1e-9 <= position <= bound.max_position + 1e-9
                    or abs(velocity) > min(limits['max_velocity'], bound.max_velocity) + 1e-9
                    or abs(acceleration) > limits['max_acceleration'] + 1e-9):
                raise ValueError('軌道の位置・速度・加速度制限違反')
    if points[0].time_sec != 0.0:
        raise ValueError('始点時刻は0秒が必要です')
    for point, reference in ((points[0], start), (points[-1], target)):
        if (max(abs(left - right) for left, right in zip(point.positions, reference)) > 1e-9
                or any(abs(value) > 1e-9 for value in point.velocities + point.accelerations)):
            raise ValueError('始終点姿勢・停止条件不正')
    return points


class task_runner:
    """方式選択と切り離した進行管理。backendはbegin/cancel/is_busy/resultを提供。"""

    def __init__(self, program, backend):
        self.program, self.backend = program, backend
        self.state = run_state.idle
        self.reason = ''
        self.task_idx = self.waypoint_idx = 0
        self.has_obstacle = False
        self.target = None
        self.reference_positions = None
        self.last_sec = None
        self.stopped_since = None
        self.active_sec = self.hold_sec = 0.0
        self.stop_started_sec = None
        self.stop_state = run_state.paused

    def feedback_error(self, now, sample):
        if sample is None or not math.isfinite(now) or not math.isfinite(sample.stamp_sec):
            return '関節状態未受信'
        if not 0 <= now - sample.stamp_sec <= self.program.limits['max_feedback_age_sec']:
            return '関節状態の時刻不正・期限切れ'
        if len(sample.positions) != len(self.program.bounds) or len(sample.velocities) != len(self.program.bounds):
            return '関節状態の要素数不正'
        if not all(math.isfinite(value) for value in sample.positions + sample.velocities):
            return '非有限の関節状態'
        margin = self.program.limits['max_position_error_th']
        for position, bound in zip(sample.positions, self.program.bounds):
            if not bound.min_position - margin <= position <= bound.max_position + margin:
                return '実測関節位置の制限違反'
        return ''

    def is_stopped(self, sample):
        return max(abs(value) for value in sample.velocities) <= self.program.limits['max_stopped_velocity_th']

    def can_begin(self, now, sample):
        error = self.feedback_error(now, sample)
        if error:
            raise ValueError(error)
        if (self.has_obstacle or self.backend.is_busy or self.stopped_since is None
                or now - self.stopped_since < self.program.limits['settle_sec'] or not self.is_stopped(sample)):
            raise ValueError('障害物なし・停止継続・指令完了の確認が必要です')

    def start(self, now, sample):
        if self.state not in (run_state.idle, run_state.succeeded, run_state.canceled):
            raise ValueError('startは待機・完了・取消後だけ使用可能です')
        self.can_begin(now, sample)
        if self.reference_positions is None:
            self.reference_positions = sample.positions
        self.task_idx = self.waypoint_idx = 0
        self.active_sec = self.hold_sec = 0.0
        self.target = None
        self.state, self.reason, self.last_sec = run_state.running, '', now

    def resume(self, now, sample):
        if self.state != run_state.paused:
            raise ValueError('resumeはpausedだけ使用可能です')
        self.can_begin(now, sample)
        self.target = None
        self.state, self.reason, self.last_sec = run_state.running, '', now

    def interrupt(self, now, reason='操作による中断', is_cancel=False):
        if self.state not in (run_state.running, run_state.stopping, run_state.paused):
            raise ValueError('中断対象のタスクがありません')
        if self.state != run_state.stopping:
            self.backend.cancel()
            self.stop_started_sec = now
        if self.state != run_state.stopping or is_cancel:
            self.stop_state = run_state.canceled if is_cancel else run_state.paused
        self.state, self.reason = run_state.stopping, reason

    def set_obstacle(self, has_obstacle, now):
        self.has_obstacle = has_obstacle
        if has_obstacle and self.state == run_state.running:
            self.interrupt(now, '障害物入力による中断')

    def fail(self, reason):
        self.backend.cancel()
        self.state, self.reason = run_state.failed, reason

    def tick(self, now, sample):
        error = self.feedback_error(now, sample)
        if self.last_sec is not None and now < self.last_sec:
            error = '時刻の巻戻り'
        elapsed = max(0.0, now - self.last_sec) if self.last_sec is not None else 0.0
        self.last_sec = now
        if error:
            self.stopped_since = None
            if self.state in (run_state.running, run_state.stopping, run_state.paused):
                self.fail(error)
            return
        if not self.is_stopped(sample):
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        if self.state == run_state.stopping:
            self._tick_stopping(now)
        elif self.state == run_state.running:
            self._tick_running(elapsed, sample)

    def _tick_stopping(self, now):
        if now - self.stop_started_sec > self.program.limits['max_stopping_sec']:
            self.fail('停止確認の期限切れ')
        elif (not self.backend.is_busy and self.stopped_since is not None
              and now - self.stopped_since >= self.program.limits['settle_sec']):
            self.state = self.stop_state
            self.target = None

    def _tick_running(self, elapsed, sample):
        self.active_sec += elapsed
        if self.active_sec > self.program.limits['max_task_sec']:
            self.fail('タスク実行期限切れ')
            return
        task = self.program.tasks[self.task_idx]
        if self.target is None:
            pose = task.targets[self.waypoint_idx]
            self.target = tuple(pose.get(name, position) for name, position in zip(self.program.joint_names, self.reference_positions))
            if any(not bound.min_position <= position <= bound.max_position
                   for position, bound in zip(self.target, self.program.bounds)):
                self.fail('再計画始点・目標の位置制限違反')
                return
            try:
                points = task.planner(sample.positions, self.target, self.program.bounds, self.program.limits)
                self.backend.begin(check_segment(points, sample.positions, self.target, self.program.bounds, self.program.limits))
                self.reference_positions = self.target
            except (ValueError, RuntimeError) as error:
                self.fail(str(error))
            return
        if self.backend.is_busy:
            return
        if self.backend.result is not True:
            self.fail('軌道指令の失敗')
            return
        has_arrived = max(abs(actual - target) for actual, target in zip(sample.positions, self.target)) <= self.program.limits['max_position_error_th']
        if not has_arrived or self.stopped_since is None or self.last_sec - self.stopped_since < self.program.limits['settle_sec']:
            return
        if task.kind == task_kind.hold:
            self.hold_sec += elapsed
            if self.hold_sec < task.duration_sec:
                return
        self.waypoint_idx += 1
        self.target = None
        if self.waypoint_idx < len(task.targets):
            return
        self.task_idx += 1
        self.waypoint_idx = 0
        self.active_sec = self.hold_sec = 0.0
        if self.task_idx == len(self.program.tasks):
            self.state = run_state.succeeded

    def status(self):
        task = self.program.tasks[self.task_idx] if self.task_idx < len(self.program.tasks) else None
        return dict(state=self.state.value, task_idx=self.task_idx, waypoint_idx=self.waypoint_idx,
                    kind=task.kind.value if task else None, method=task.method if task else None,
                    has_obstacle=self.has_obstacle, active_sec=self.active_sec, hold_sec=self.hold_sec,
                    reason=self.reason)
