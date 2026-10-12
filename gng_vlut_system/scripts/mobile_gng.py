"""ロボット原点基準の移動状態サンプルと、共通C++ GNGによる状態配置。"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

from mobile_state_graph import drive_model, graph_config, has_valid_speed, rollout


@dataclass(frozen=True)
class training_config:
    max_nodes: int = 1000
    num_samples: int = 60000
    max_iter: int = 200000
    insert_interval: int = 100
    max_edge_age: int = 100
    max_rollout_steps: int = 16
    seed: int = 42
    learn_rate_winner: float = 0.08
    learn_rate_neighbor: float = 0.008
    # 学習距離の単位換算。位置[m]、向きのcos・sin、速度[m/s]、角速度[rad/s]。
    position_scale: float = 0.5
    yaw_scale: float = 1.0
    speed_scale: float = 0.4
    angular_speed_scale: float = 1.2

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError('GNG学習設定の非有限値・負値: '+name)
        for name in ('max_nodes', 'num_samples', 'max_iter', 'insert_interval', 'max_edge_age', 'max_rollout_steps', 'seed'):
            if type(getattr(self, name)) is not int:
                raise ValueError('整数のGNG設定が必要: '+name)
        if not 2 <= self.max_nodes <= 65535 or self.num_samples < self.max_nodes or self.max_iter < 1 or self.max_iter < self.insert_interval*(self.max_nodes-2):
            raise ValueError('ノード予算・サンプル数・成長反復の不足')
        if any(getattr(self, name) <= 0 for name in ('insert_interval', 'max_edge_age', 'max_rollout_steps', 'position_scale', 'yaw_scale', 'speed_scale', 'angular_speed_scale')):
            raise ValueError('GNG学習の間隔・距離換算の設定不正')
        if not 0 < self.learn_rate_neighbor <= self.learn_rate_winner <= 1:
            raise ValueError('GNG学習率の設定不正')


def encode_states(states, config):
    values = np.asarray(states, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 5 or not np.isfinite(values).all():
        raise ValueError('移動状態の次元または非有限値')
    return np.column_stack((values[:, 0]/config.position_scale, values[:, 1]/config.position_scale,
        np.cos(values[:, 2])/config.yaw_scale, np.sin(values[:, 2])/config.yaw_scale,
        values[:, 3]/config.speed_scale, values[:, 4]/config.angular_speed_scale)).astype(np.float32)


def sample_states(config, motion, model, num_samples=None, seed=None):
    """原点・前方・初期速度からの運動積分。軌跡途中の位置・車輪速度制限付き。"""
    generator = np.random.default_rng(config.seed if seed is None else seed)
    num_samples = config.num_samples if num_samples is None else num_samples
    states = []
    num_rollouts = 0
    while len(states) < num_samples:
        num_rollouts += 1
        if num_rollouts > num_samples*20:
            raise ValueError('指定の運動条件で学習サンプルを生成できません')
        state = [0., 0., 0., motion.initial_speed, motion.initial_angular_speed]
        if not has_valid_speed(state, motion, model):
            raise ValueError('初期速度が車輪制約と不一致')
        # 一定目標の円弧・直進と、目標変更を伴う経路の混合。
        enable_constant_control = num_rollouts % 2 == 0
        desired = None
        for step in range(config.max_rollout_steps):
            path = None
            for attempt in range(16):
                if desired is None or not enable_constant_control or attempt > 0:
                    desired = [float(generator.uniform(motion.min_speed, motion.max_speed)),
                               float(generator.uniform(-motion.max_angular_speed, motion.max_angular_speed))]
                acceleration = float(np.clip((desired[0]-state[3])/motion.duration_sec, -motion.max_acceleration, motion.max_acceleration))
                angular_acceleration = float(np.clip((desired[1]-state[4])/motion.duration_sec, -motion.max_angular_acceleration, motion.max_angular_acceleration))
                path = rollout(state, acceleration, angular_acceleration, motion, model)
                if path is not None:
                    break
            if path is None:
                break
            state = path[-1]
            states.append(state)
            if len(states) == num_samples:
                break
    return np.asarray(states, dtype=np.float64)


def fingerprint(config, motion, model):
    data = {'schema': 'mobile_gng_state_v1', 'training': asdict(config), 'motion': asdict(motion), 'drive': asdict(model), 'representative_method': 'nearest_unused_motion_sample'}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, allow_nan=False, separators=(',', ':'))
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def train(config, motion, model, native_path, artifacts_path):
    from scipy.spatial import cKDTree
    began = time.perf_counter()
    artifacts = Path(artifacts_path)
    artifacts.mkdir(parents=True, exist_ok=True)
    samples = sample_states(config, motion, model)
    features = encode_states(samples, config)
    sample_ms = 1000*(time.perf_counter()-began)
    training_input = artifacts/'training_input.json'
    prototypes_path = artifacts/'prototypes.json'
    save_json(training_input, {'schema': 'mobile_gng_training_v1', 'config': asdict(config),
        'features': features.tolist(), 'stats_path': str(artifacts/'training_stats.log')})
    command = [str(native_path), str(training_input), str(prototypes_path)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=300)
    (artifacts/'training.log').write_text(result.stdout+result.stderr)
    if result.returncode != 0:
        raise ValueError('C++ GNG学習失敗: '+result.stderr[-2000:])
    native = json.loads(prototypes_path.read_text())
    prototypes = np.asarray(native['features'], dtype=np.float32)
    if prototypes.ndim != 2 or prototypes.shape[1] != 6 or not np.isfinite(prototypes).all():
        raise ValueError('GNG学習結果の特徴不正')
    # 学習中心に最も近い実サンプルを代表状態に採用。平均による未観測姿勢の作成回避。
    sample_tree = cKDTree(features)
    candidates = sample_tree.query(prototypes, k=min(8, len(samples)))[1]
    used = set()
    sample_ids = []
    for prototype, row in zip(prototypes, candidates):
        selected = next((int(idx) for idx in np.atleast_1d(row) if int(idx) not in used), None)
        if selected is None:
            expanded = sample_tree.query(prototype, k=len(samples))[1]
            selected = next(int(idx) for idx in np.atleast_1d(expanded) if int(idx) not in used)
        sample_ids.append(selected)
        used.add(selected)
    sample_ids = np.asarray(sample_ids, dtype=np.int64)
    states = samples[sample_ids]
    node_ids = cKDTree(prototypes).query(features)[1]
    counts = np.bincount(node_ids, minlength=len(prototypes))
    holdout = sample_states(config, motion, model, num_samples=5000, seed=config.seed+1)
    errors, nearest = cKDTree(encode_states(states, config)).query(encode_states(holdout, config))
    position_errors = np.linalg.norm(holdout[:, :2]-states[nearest, :2], axis=1)
    yaw_errors = np.abs((holdout[:, 2]-states[nearest, 2]+math.pi) % (2*math.pi)-math.pi)
    metrics = {'num_holdout_samples': len(holdout), 'mean_feature_error': float(errors.mean()),
        'p95_feature_error': float(np.quantile(errors, .95)), 'mean_position_error_m': float(position_errors.mean()),
        'p95_position_error_m': float(np.quantile(position_errors, .95)),
        'mean_yaw_error_deg': float(np.degrees(yaw_errors).mean()),
        'p95_yaw_error_deg': float(np.quantile(np.degrees(yaw_errors), .95))}
    normal = np.column_stack((np.cos(states[:, 2]), np.sin(states[:, 2]), np.zeros(len(states))))
    velocity = normal*states[:, 3, None]
    output = {'schema': 'mobile_gng_state_v1', 'fingerprint': fingerprint(config, motion, model),
        'reference': {'origin': [0., 0., 0.], 'x_axis': 'initial_robot_forward', 'yaw': 'relative_to_initial_robot'},
        'initial_state': [0., 0., 0., motion.initial_speed, motion.initial_angular_speed],
        'representative_method': 'nearest_unused_motion_sample',
        'state_fields': ['x', 'y', 'yaw', 'v', 'omega'], 'state_units': ['m', 'm', 'rad', 'm/s', 'rad/s'],
        'feature_fields': ['x_scaled', 'y_scaled', 'cos_yaw_scaled', 'sin_yaw_scaled', 'v_scaled', 'omega_scaled'],
        'edge_semantics': 'gng_state_neighbors', 'has_checked_motion_edges': False, 'collision_checked': False,
        'config': asdict(config), 'motion_config': asdict(motion), 'drive': asdict(model),
        'states': states.tolist(), 'normal': normal.tolist(), 'velocity': velocity.tolist(),
        'prototype_features': prototypes.tolist(), 'sample_ids': sample_ids.tolist(), 'sample_counts': counts.tolist(),
        'edges': native['edges'], 'metrics': metrics, 'sample_ms': sample_ms, 'train_ms': native['train_ms'],
        'total_ms': 1000*(time.perf_counter()-began), 'num_training_samples': len(samples)}
    validate_model(output, config, motion, model)
    save_json(artifacts/'training_samples.json', {'states': samples.tolist()})
    return output


def validate_model(data, config, motion, model):
    if data.get('schema') != 'mobile_gng_state_v1' or data.get('fingerprint') != fingerprint(config, motion, model):
        raise ValueError('状態GNGの形式または設定不一致')
    states = np.asarray(data['states'], dtype=np.float64)
    if states.ndim != 2 or states.shape[1] != 5 or not 2 <= len(states) <= config.max_nodes or not np.isfinite(states).all():
        raise ValueError('状態GNGのノード不正')
    if data.get('edge_semantics') != 'gng_state_neighbors' or data.get('has_checked_motion_edges') is not False or data.get('collision_checked') is not False:
        raise ValueError('状態GNGと走行・衝突検査の意味の不一致')
    if len(data['sample_counts']) != len(states) or any(type(count) is not int or count < 0 for count in data['sample_counts']):
        raise ValueError('状態GNGのサンプル集計不正')
    for state in states:
        if not has_valid_speed(state, motion, model) or math.hypot(state[0], state[1]) > motion.radius+1e-10:
            raise ValueError('代表状態の運動・位置制限逸脱')
    normals = np.asarray(data['normal']);velocity = np.asarray(data['velocity'])
    expected = np.column_stack((np.cos(states[:, 2]), np.sin(states[:, 2]), np.zeros(len(states))))
    if normals.shape != expected.shape or not np.allclose(normals, expected, atol=1e-6) or velocity.shape != expected.shape or not np.allclose(velocity, expected*states[:, 3, None], atol=1e-6):
        raise ValueError('yaw・速度と表示ベクトルの不一致')
    edges = set()
    for edge in data['edges']:
        if len(edge) != 2 or any(type(idx) is not int or not 0 <= idx < len(states) for idx in edge) or edge[0] == edge[1]:
            raise ValueError('状態GNGの辺不正')
        pair = tuple(sorted(edge))
        if pair in edges:
            raise ValueError('状態GNGの辺重複')
        edges.add(pair)
