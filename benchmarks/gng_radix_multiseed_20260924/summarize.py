"""シード単位の集計と、観測代表点変更の独立性確認。"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(mean=float(values.mean()), std=float(values.std(ddof=1)),
                min=float(values.min()), max=float(values.max()), values=values.tolist())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--artifacts', type=Path, default=Path('/ros2_ws/src/artifacts/gng_radix_multiseed_20260924'))
    args = parser.parse_args()
    root = args.artifacts
    seeds = [11, 101, 1009, 10007, 104729, 20260924]
    result = dict(seeds=seeds, frames_per_run=200, warmup=50, conditions={}, isolation=[], baseline_identity=[])
    same_fields = ('kept_input_num', 'voxel_num', 'cell_membership_sha256', 'cell_sequence_sha256', 'ranges_sha256', 'learning_num')
    graph_fields = ('graph_sha256', 'topology_sha256', 'cluster_sha256', 'label_sha256', 'learning_num')
    for voxel in ('0.1', '0.5'):
        samples = {method: {} for method in ('boost', 'radix')}
        num_compared = 0
        for seed in seeds:
            runs = {}
            for method in samples:
                data = json.loads((root / method / f'voxel_{voxel}_seed_{seed}.json').read_text())
                assert data['seed'] == seed and data['has_observation'] and data['warmup'] == 50
                records = data['records']
                assert len(records) == 200 and all(row['learning_num'] == 4000 for row in records)
                runs[method] = records
                metrics = dict(data['mean'])
                snapshot = np.load(root / method / f'voxel_{voxel}_seed_{seed}_frame_199.npz')
                positions = snapshot['positions'].astype(float)
                edges = snapshot['edges'].reshape(-1, 2)
                edge_lengths = np.linalg.norm(positions[edges[:, 0]] - positions[edges[:, 1]], axis=1)
                assert len(edge_lengths) and np.isfinite(edge_lengths).all()
                metrics['last_graph.mean_edge_dist'] = float(edge_lengths.mean())
                metrics['last_graph.p95_edge_dist'] = float(np.percentile(edge_lengths, 95))
                for axis, name in enumerate('xyz'):
                    metrics[f'last_graph.{name}_span'] = float(np.ptp(positions[:, axis]))
                for key in ('nodes', 'edges', 'clusters', 'attention_num'):
                    metrics[key] = float(np.mean([row[key] for row in records[50:]]))
                for category in ('quality', 'observation'):
                    selected = [row[category] for row in records[50:] if category in row]
                    assert len(selected) == 16
                    for key in selected[0]:
                        metrics[f'{category}.{key}'] = float(np.mean([row[key] for row in selected]))
                for key, value in metrics.items():
                    samples[method].setdefault(key, []).append(value)
            for left, right in zip(runs['boost'], runs['radix']):
                assert all(left[key] == right[key] for key in same_fields)
                num_compared += 1
        metrics = {}
        for key in samples['boost']:
            left = np.asarray(samples['boost'][key])
            right = np.asarray(samples['radix'][key])
            delta = right - left
            # 独立単位はシード。フレームを独立標本とみなさない、自由度5の対応差区間。
            half_width = 2.570581835636305 * delta.std(ddof=1) / np.sqrt(len(seeds))
            metrics[key] = dict(boost=stats(left), radix=stats(right), delta=stats(delta),
                paired_mean_ci95=[float(delta.mean() - half_width), float(delta.mean() + half_width)])
        left = np.load(root / 'boost' / f'voxel_{voxel}_seed_11_frame_0.npz')
        right = np.load(root / 'radix' / f'voxel_{voxel}_seed_11_frame_0.npz')
        a = left['representative_points'].astype(float)
        b = right['representative_points'].astype(float)
        assert np.array_equal(left['ranges'], right['ranges'])
        assert np.array_equal(left['indices'][:, 0], right['indices'][:, 0])
        lengths_a, lengths_b = np.linalg.norm(a, axis=1), np.linalg.norm(b, axis=1)
        has_ray = (lengths_a > 0) & (lengths_b > 0)
        angles = np.degrees(np.arccos(np.clip(np.sum(a[has_ray] * b[has_ray], axis=1) /
            (lengths_a[has_ray] * lengths_b[has_ray]), -1, 1)))
        pitch_a = np.arctan2(a[has_ray, 2], np.hypot(a[has_ray, 0], a[has_ray, 1]))
        pitch_b = np.arctan2(b[has_ray, 2], np.hypot(b[has_ray, 0], b[has_ray, 1]))
        yaw_a, yaw_b = np.arctan2(a[has_ray, 1], a[has_ray, 0]), np.arctan2(b[has_ray, 1], b[has_ray, 0])
        assert np.max(np.abs(a-b)) <= float(voxel) + 1e-5
        representatives = dict(voxel_num=len(a), changed_num=int(np.any(a != b, axis=1).sum()),
            max_dist_m=float(np.linalg.norm(a-b, axis=1).max()), mean_angle_deg=float(angles.mean()),
            p95_angle_deg=float(np.percentile(angles, 95)), max_angle_deg=float(angles.max()),
            mean_signed_pitch_delta_deg=float(np.degrees(pitch_b-pitch_a).mean()),
            mean_signed_yaw_delta_deg=float(np.degrees((yaw_b-yaw_a+np.pi) % (2*np.pi) - np.pi).mean()),
            zero_ray_a_num=int((lengths_a == 0).sum()), zero_ray_b_num=int((lengths_b == 0).sum()))
        result['conditions'][voxel] = dict(compared_frames=num_compared, metrics=metrics, first_frame_representatives=representatives)
        for method in samples:
            main_records = json.loads((root / method / f'voxel_{voxel}_seed_20260924.json').read_text())['records']
            old_records = json.loads((root.parent / 'gng_radix_production_20260924' / method / f'voxel_{voxel}_1.json').read_text())['records']
            matches = sum(all(a[key] == b[key] for key in graph_fields) for a,b in zip(main_records,old_records))
            assert matches == 200
            result['baseline_identity'].append(dict(method=method, voxel=voxel, frames=matches))
            for mode in ('off', 'last'):
                records = json.loads((root / method / f'observation_{voxel}_{mode}.json').read_text())['records']
                assert len(records) == 30
                matches = sum(all(a[key] == b[key] for key in graph_fields) for a,b in zip(main_records,records))
                assert matches == 30
                observation_pairs = [(a,b) for a,b in zip(main_records,records) if 'observation_sha256' in a and 'observation_sha256' in b]
                result['isolation'].append(dict(method=method, voxel=voxel, mode=mode, graph_matches=matches,
                    observation_compared=len(observation_pairs), observation_changed=sum(a['observation_sha256'] != b['observation_sha256'] for a,b in observation_pairs)))
    result['executed_frames'] = 6 * 2 * 2 * 200 + 2 * 2 * 2 * 30
    result['library_sha256'] = {method: hashlib.sha256((root/method/'libgng_cpu.so').read_bytes()).hexdigest() for method in ('boost','radix')}
    (root/'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    for voxel, condition in result['conditions'].items():
        print('voxel', voxel)
        for key in ('total_ms','sort_ms','quality.coverage_0_2','quality.coverage_0_4','quality.mean_dist','quality.p95_dist','nodes','observation.supported_ratio','observation.mean_yaw_span_deg','observation.mean_pitch_span_deg'):
            entry=condition['metrics'][key]
            print(key, entry['boost']['mean'], '->',entry['radix']['mean'], 'delta',entry['delta']['mean'], 'baseline_std',entry['boost']['std'], 'CI95',entry['paired_mean_ci95'])
        print('representatives',condition['first_frame_representatives'])
    print('isolation', result['isolation'])


if __name__ == '__main__':
    main()
