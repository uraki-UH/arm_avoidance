"""平面差動二輪の状態遷移グラフ。衝突判定・実機指令・GNG学習は対象外。"""
from dataclasses import asdict, dataclass
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as et


@dataclass(frozen=True)
class drive_model:
    wheel_radius: float
    wheel_separation: float
    max_wheel_speed: float

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in asdict(self).values()):
            raise ValueError('車輪半径・輪間隔・車輪速度上限の正値が必要です')

    @classmethod
    def from_urdf(cls, text):
        root = et.fromstring(text)
        plugins = [item for item in root.findall('.//gazebo/plugin')
                   if item.get('filename') == 'libgazebo_ros_diff_drive.so']
        if len(plugins) != 1:
            raise ValueError('差動二輪pluginが1個のURDFが必要です')
        plugin = plugins[0]
        joints = {item.get('name'): item for item in root.findall('joint')}
        speeds = [float(joints[plugin.findtext(side+'_joint')].find('limit').get('velocity'))
                  for side in ('left', 'right')]
        return cls(float(plugin.findtext('wheel_diameter'))/2,
                   float(plugin.findtext('wheel_separation')), min(speeds))


@dataclass(frozen=True)
class graph_config:
    # 距離[m]、速度[m/s]、角速度[rad/s]。平坦床・横滑りなしの仮定。
    radius: float = 2.0
    min_speed: float = -0.2
    max_speed: float = 0.4
    max_angular_speed: float = 1.2
    max_acceleration: float = 0.4
    max_angular_acceleration: float = 1.2
    initial_speed: float = 0.0
    initial_angular_speed: float = 0.0
    duration_sec: float = 0.5
    integration_step_sec: float = 0.05
    max_depth: int = 8
    max_nodes: int = 3000
    # 状態代表点の間引き幅。近接状態への端点の付け替えなし。
    position_step: float = 0.1
    yaw_step_deg: float = 15.0
    speed_step: float = 0.1
    angular_speed_step: float = 0.3

    def __post_init__(self):
        values = asdict(self)
        if any(not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in values.values()):
            raise ValueError('全設定に有限の数値が必要です')
        signed = {'min_speed', 'initial_speed', 'initial_angular_speed'}
        if any(value <= 0 for name, value in values.items() if name not in signed):
            raise ValueError('時間・分解能・上限設定の正値が必要です')
        if not self.min_speed <= self.initial_speed <= self.max_speed or self.min_speed > 0:
            raise ValueError('速度範囲と初期速度が不正です')
        if abs(self.initial_angular_speed) > self.max_angular_speed or self.yaw_step_deg > 180:
            raise ValueError('初期角速度または角度分解能が不正です')
        if type(self.max_depth) is not int or type(self.max_nodes) is not int:
            raise ValueError('深さ・ノード数は整数が必要です')
        if self.max_nodes > 65535 or self.integration_step_sec > self.duration_sec:
            raise ValueError('ノード数または積分刻みが不正です')


def wrap_angle(value):
    return (value+math.pi) % (2*math.pi)-math.pi


def has_valid_speed(state, config, model):
    speed, angular = state[3:5]
    wheel_speed = (abs(speed)+model.wheel_separation*abs(angular)/2)/model.wheel_radius
    return (config.min_speed-1e-10 <= speed <= config.max_speed+1e-10
            and abs(angular) <= config.max_angular_speed+1e-10
            and wheel_speed <= model.max_wheel_speed+1e-10)


def rollout(state, acceleration, angular_acceleration, config, model):
    """一定加速度の数値積分。速度端点・途中位置の制限検査付き。"""
    if len(state) != 5 or not all(math.isfinite(value) for value in (*state, acceleration, angular_acceleration)):
        return None
    if abs(acceleration) > config.max_acceleration+1e-10 or abs(angular_acceleration) > config.max_angular_acceleration+1e-10:
        return None
    end = [*state[:3], state[3]+acceleration*config.duration_sec,
           state[4]+angular_acceleration*config.duration_sec]
    # 各車輪速度は区間内で一次式。両端検査による途中の速度上限保証。
    if not has_valid_speed(state, config, model) or not has_valid_speed(end, config, model):
        return None
    if math.hypot(state[0], state[1]) > config.radius+1e-10:
        return None
    num = math.ceil(config.duration_sec/config.integration_step_sec)
    step = config.duration_sec/num
    current = list(state)
    samples = [current]
    for _ in range(num):
        x, y, yaw, speed, angular = current
        def derivative(sec):
            angle = yaw+angular*sec+angular_acceleration*sec*sec/2
            value = speed+acceleration*sec
            return value*math.cos(angle), value*math.sin(angle)
        start, middle, end = derivative(0), derivative(step/2), derivative(step)
        current = [x+step*(start[0]+4*middle[0]+end[0])/6,
                   y+step*(start[1]+4*middle[1]+end[1])/6,
                   wrap_angle(yaw+angular*step+angular_acceleration*step*step/2),
                   speed+acceleration*step, angular+angular_acceleration*step]
        if math.hypot(current[0], current[1]) > config.radius+1e-10:
            return None
        samples.append(current)
    return samples


def state_cell(state, config):
    yaw_bins = max(2, round(360/config.yaw_step_deg))
    return (round(state[0]/config.position_step), round(state[1]/config.position_step),
            round(wrap_angle(state[2])*yaw_bins/(2*math.pi)) % yaw_bins,
            round(state[3]/config.speed_step), round(state[4]/config.angular_speed_step))


def state_error(left, right):
    return max(abs(a-b) if idx != 2 else abs(wrap_angle(a-b))
               for idx, (a, b) in enumerate(zip(left, right)))


def fingerprint(config, model):
    value = json.dumps({'config': asdict(config), 'drive': asdict(model)}, sort_keys=True)
    return hashlib.sha256(value.encode()).hexdigest()


def generate_graph(config, model):
    begin = time.perf_counter()
    root = [0.0, 0.0, 0.0, config.initial_speed, config.initial_angular_speed]
    if not has_valid_speed(root, config, model):
        raise ValueError('初期状態が車輪速度制限を超えています')
    states, depths, edges = [root], [0], []
    cells = {state_cell(root, config): 0}
    controls = list(itertools.product((0.0, config.max_acceleration, -config.max_acceleration),
                                     (0.0, config.max_angular_acceleration, -config.max_angular_acceleration)))
    has_node_limit = False
    source = 0
    while source < len(states):
        if depths[source] < config.max_depth:
            for acceleration, angular_acceleration in controls:
                samples = rollout(states[source], acceleration, angular_acceleration, config, model)
                if samples is None:
                    continue
                end = samples[-1]
                cell = state_cell(end, config)
                target = cells.get(cell)
                if target is None:
                    if len(states) >= config.max_nodes:
                        has_node_limit = True
                        continue
                    target = len(states)
                    cells[cell] = target
                    states.append(end)
                    depths.append(depths[source]+1)
                elif state_error(states[target], end) > 1e-9:
                    # 同じセルでも異なる状態への架空の移動エッジは不採用。
                    continue
                if target != source:
                    edges.append([source, target, acceleration, angular_acceleration, config.duration_sec])
        source += 1
    return {'schema': 'mobile_state_graph_v1', 'fingerprint': fingerprint(config, model),
            'state_fields': ['x', 'y', 'yaw', 'v', 'omega'],
            'state_units': ['m', 'm', 'rad', 'm/s', 'rad/s'],
            'edge_fields': ['source', 'target', 'acceleration', 'angular_acceleration', 'duration_sec'],
            'config': asdict(config), 'drive': asdict(model), 'states': states, 'edges': edges,
            'collision_checked': False, 'has_node_limit': has_node_limit,
            'build_ms': 1000*(time.perf_counter()-begin)}


def load_or_build(path, config, model, enable_rebuild=False):
    path = Path(path)
    if path.exists():
        graph = json.loads(path.read_text())
        if graph.get('schema') != 'mobile_state_graph_v1':
            raise ValueError('保存先は対応する状態グラフではありません')
        if not enable_rebuild:
            if graph.get('fingerprint') != fingerprint(config, model):
                raise ValueError('保存グラフと設定が異なります。enable_rebuild:=trueで再生成してください')
            validate_graph(graph, config, model)
            return graph, False
    graph = generate_graph(config, model)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(graph, stream, allow_nan=False, separators=(',', ':'))
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return graph, True


def validate_graph(graph, config, model):
    states = graph['states']
    if not states or len(states) > config.max_nodes or graph.get('collision_checked') is not False:
        raise ValueError('保存グラフの状態数または検証種別が不正です')
    expected = [0, 0, 0, config.initial_speed, config.initial_angular_speed]
    for state in states:
        if len(state) != 5 or not all(math.isfinite(value) for value in state) or not has_valid_speed(state, config, model):
            raise ValueError('保存状態が不正です')
        if math.hypot(state[0], state[1]) > config.radius+1e-10:
            raise ValueError('保存状態が範囲外です')
    if state_error(states[0], expected) > 1e-9:
        raise ValueError('保存初期状態が不正です')
    for source, target, acceleration, angular_acceleration, duration in graph['edges']:
        if type(source) is not int or type(target) is not int or not (0 <= source < len(states) and 0 <= target < len(states)):
            raise ValueError('保存エッジの添字が不正です')
        samples = rollout(states[source], acceleration, angular_acceleration, config, model)
        if duration != config.duration_sec or samples is None or state_error(samples[-1], states[target]) > 1e-9:
            raise ValueError('保存エッジが運動モデルと不一致です')
