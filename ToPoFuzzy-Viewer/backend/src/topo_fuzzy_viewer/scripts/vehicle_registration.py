"""選択したGNG観測への車両表面モデル照合。確率・車種確定・損傷判定を含まない幾何適合度。"""

import json
import math
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def yaw_rotation(yaw):
    cos, sin = math.cos(yaw), math.sin(yaw)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def load_models(path=None):
    if path is None:
        path = Path(__file__).resolve().parents[1] / 'config/vehicle_models/models.json'
    models = json.loads(Path(path).read_text())['models']
    for model in models:
        model['points'] = np.asarray(model['points'], dtype=float)
        model['tree'] = cKDTree(model['points'])
    return models


def validate_observation(snapshot):
    if not isinstance(snapshot, dict) or not snapshot.get('frame_id'):
        raise ValueError('座標系が指定されたクラスタの取得が必要です。')
    if snapshot.get('selection', {}).get('kind') not in ('cluster', 'component'):
        raise ValueError('照合対象のクラスタまたは連結成分を選択してください。')
    graph = snapshot.get('graph', {})
    if graph.get('frameId', snapshot['frame_id']) != snapshot['frame_id']:
        raise ValueError('観測グラフと選択範囲の座標系が一致していません。')
    nodes = graph.get('nodes', [])
    if not 12 <= len(nodes) <= 20000:
        raise ValueError('照合対象は12〜20000ノードの単一クラスタです。')
    try:
        points = np.array([[p['x'], p['y'], p['z']] for p in nodes], dtype=float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('観測座標の形式が不正です。') from error
    if not np.isfinite(points).all():
        raise ValueError('観測座標に非有限値が含まれています。')
    if np.max(np.ptp(points, axis=0)) > 30:
        raise ValueError('対象が広すぎます。車両1台に対応するクラスタを選択してください。')
    if np.linalg.matrix_rank(points - points.mean(axis=0), tol=0.02) < 2:
        raise ValueError('位置合わせに必要な面方向の観測が不足しています。')
    return points


def refine_pose(points, model, yaw, translation, dist_th, max_iter):
    tree, surface = model['tree'], model['points']
    for _ in range(max_iter):
        rotation = yaw_rotation(yaw)
        dists, ids = tree.query((points - translation) @ rotation)
        # 外れ値の影響を抑制した対応点重み。観測そのものの削除なし
        weights = 1.0 / (1.0 + (dists / dist_th) ** 2)
        weights[dists > np.quantile(dists, 0.95)] *= 0.1
        weights /= weights.sum()
        matched = surface[ids]
        source_center = weights @ matched
        target_center = weights @ points
        source_xy = matched[:, :2] - source_center[:2]
        target_xy = points[:, :2] - target_center[:2]
        cross = np.sum(weights * (source_xy[:, 0] * target_xy[:, 1] - source_xy[:, 1] * target_xy[:, 0]))
        dot = np.sum(weights * np.sum(source_xy * target_xy, axis=1))
        next_yaw = math.atan2(cross, dot)
        next_translation = target_center - source_center @ yaw_rotation(next_yaw).T
        delta = np.linalg.norm(next_translation - translation) + abs(math.atan2(math.sin(next_yaw-yaw), math.cos(next_yaw-yaw)))
        yaw, translation = next_yaw, next_translation
        if delta < 0.0001:
            break
    dists, _ = tree.query((points - translation) @ yaw_rotation(yaw))
    # 初期姿勢間の比較用の上限付き二乗誤差
    loss = float(np.mean(np.minimum(dists ** 2, (3 * dist_th) ** 2)))
    return loss, yaw, translation


def fit_model(points, model, dist_th, support_dist_th):
    # 最適化だけの決定的な間引き。最終指標は全観測ノードで評価
    fit_points = points[np.linspace(0, len(points) - 1, min(len(points), 500), dtype=int)]
    surface = model['points']
    model_min, model_max = surface.min(axis=0), surface.max(axis=0)
    model_center = (model_min + model_max) / 2
    candidates = []
    for yaw in np.linspace(-math.pi, math.pi, 16, endpoint=False):
        rotation = yaw_rotation(yaw)
        local = fit_points @ rotation
        low, high = np.quantile(local, [0.01, 0.99], axis=0)
        center = (low + high) / 2
        gap = np.maximum(0, model_max - model_min - (high - low)) / 2
        for side in (-1, 0, 1):
            for top in (False, True):
                offset = center - model_center
                offset[1] += side * gap[1]
                offset[2] = (high[2] - model_max[2]) if top else (low[2] - model_min[2])
                candidates.append(refine_pose(fit_points, model, yaw, offset @ rotation.T, dist_th, 8))
    candidates.sort(key=lambda value: value[0])
    refined = [refine_pose(fit_points, model, yaw, translation, dist_th, 40)
               for _, yaw, translation in candidates[:6]]
    _, yaw, translation = min(refined, key=lambda value: value[0])
    rotation = yaw_rotation(yaw)
    dists, _ = model['tree'].query((points - translation) @ rotation)
    transformed = surface @ rotation.T + translation
    support_dists, _ = cKDTree(points).query(transformed)
    is_inlier, has_support = dists <= dist_th, support_dists <= support_dist_th
    match_ratio, support_ratio = float(is_inlier.mean()), float(has_support.mean())
    rms = float(np.sqrt(np.mean(dists[is_inlier] ** 2))) if is_inlier.any() else None
    compatibility = match_ratio * math.exp(-(rms if rms is not None else 3 * dist_th) / dist_th)
    # 未観測面を強い不一致とせず、観測支持を弱く加味した順位付け
    rank_score = compatibility * (0.65 + 0.35 * support_ratio)
    return {
        'model_id': model['id'], 'label': model['label'], 'model_note': model['note'],
        'dimensions_m': model['dimensions_m'], 'yaw_deg': math.degrees(yaw),
        'translation': translation.tolist(), 'match_ratio': match_ratio,
        'support_ratio': support_ratio, 'unmatched_ratio': 1 - support_ratio,
        'inlier_rms_m': rms, 'compatibility': compatibility, 'rank_score': rank_score,
        'num_inliers': int(is_inlier.sum()), 'num_observed': len(points),
        'matched_positions': transformed[has_support].round(5).reshape(-1).tolist(),
        'unmatched_positions': transformed[~has_support].round(5).reshape(-1).tolist(),
        'outlier_positions': points[~is_inlier].round(5).reshape(-1).tolist(),
        'min_position': np.minimum(points.min(axis=0), transformed.min(axis=0)).tolist(),
        'max_position': np.maximum(points.max(axis=0), transformed.max(axis=0)).tolist(),
    }


def register_vehicle(snapshot, models, dist_th=0.25, support_dist_th=0.35, progress=None):
    started = time.perf_counter()
    points = validate_observation(snapshot)
    for value in (dist_th, support_dist_th):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0.05 <= value <= 1.0:
            raise ValueError('対応距離は0.05〜1.0 mの有限値が必要です。')
    if not models:
        raise ValueError('照合モデルがありません。')
    candidates = []
    for idx, model in enumerate(models):
        candidates.append(fit_model(points, model, dist_th, support_dist_th))
        if progress:
            progress((idx + 1) / len(models), model['label'])
    candidates.sort(key=lambda item: item['rank_score'], reverse=True)
    best = candidates[0]
    local = (points - best['translation']) @ yaw_rotation(math.radians(best['yaw_deg']))
    observed_span = np.ptp(local, axis=0)
    has_vehicle_support = best['match_ratio'] >= 0.65 and best['support_ratio'] >= 0.18 and observed_span[0] >= 1.5 and observed_span[2] >= 0.5
    gap = best['rank_score'] - candidates[1]['rank_score'] if len(candidates) > 1 else None
    can_separate_class = has_vehicle_support and best['support_ratio'] >= 0.35 and gap is not None and gap >= 0.08
    return {
        'source_id': snapshot.get('source_id', ''), 'selection': snapshot.get('selection'),
        'frame_id': snapshot['frame_id'], 'timestamp': snapshot['graph'].get('timestamp'),
        'observation_kind': 'gng_nodes', 'dist_th': dist_th, 'support_dist_th': support_dist_th,
        'state': 'class_candidate' if can_separate_class else ('ambiguous' if has_vehicle_support else 'insufficient'),
        'message': ('形状候補: ' + best['label']) if can_separate_class else (
            '車両モデルに適合。分類は複数候補' if has_vehicle_support else '車両の判定保留: 観測不足またはモデルとの不一致'),
        'candidates': candidates, 'elapsed_ms': (time.perf_counter() - started) * 1000,
        'limitations': 'GNGノードへの幾何適合度。認識確率ではありません。未対応部分には遮蔽・疎な観測・車体形状差を含みます。前後方向は未確定です。',
    }
