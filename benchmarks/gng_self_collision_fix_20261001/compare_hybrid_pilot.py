"""旧ボクセル判定とhybrid判定の有限pilot比較、採否遷移・姿勢不変性・既知干渉の検査。"""
import argparse
import csv
import json
from pathlib import Path


def require(is_valid, message):
    if not is_valid:
        raise ValueError(message)


def read_rows(path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    result = {int(row['id']): row for row in rows}
    require(len(result) == len(rows), 'CSVの重複ID')
    return result


def collect_cases(batch):
    report = json.loads((batch / 'report.json').read_text())
    require(report['status'] == 'completed', '未完了のpilot')
    cases = {}
    for record in report['records']:
        require(record['name'] not in cases and record['returncode'] == 0, '反復または失敗のpilot')
        cases[record['name']] = (batch / Path(record['log']).parent.name / 'result', record)
    return cases, report


def pose_rows(directory, group):
    kept = read_rows(directory / f'{group}_kept.csv')
    rejected = read_rows(directory / f'{group}_rejected.csv')
    require(not kept.keys() & rejected.keys(), '採否両方に含まれるID')
    return kept | rejected


def check_time(metrics):
    output = {key: metrics[key] for key in ('elapsed_sec', 'pose_check_sec', 'safe_pose_check_sec',
                                           'rejected_pose_check_sec', 'collision_pair_collection_sec')}
    for state in ('safe', 'rejected'):
        num = metrics[f'num_{state}']
        output[f'mean_{state}_pose_check_ms'] = 1000 * metrics[f'{state}_pose_check_sec'] / num if num else None
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--old-batch', type=Path, required=True)
    parser.add_argument('--hybrid-batch', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(), '既存出力への上書き拒否')
    old_cases, old_report = collect_cases(args.old_batch)
    hybrid_cases, hybrid_report = collect_cases(args.hybrid_batch)
    require(old_cases.keys() == hybrid_cases.keys(), 'pilotのモデル集合不一致')
    result = {'old_elapsed_sec': old_report['elapsed_sec'], 'hybrid_elapsed_sec': hybrid_report['elapsed_sec'],
              'cases': [], 'is_all_checks_passed': True}
    for name in old_cases:
        old_dir, _ = old_cases[name]
        hybrid_dir, record = hybrid_cases[name]
        old = json.loads((old_dir / 'metrics.json').read_text())
        hybrid = json.loads((hybrid_dir / 'metrics.json').read_text())
        require(hybrid.get('collision_method') == 'mesh_surface_and_component_voxel_containment', '新方式以外の監査結果')
        require(all(old[key] == hybrid[key] for key in ('collision_voxel_size_m', 'manual_exclusions', 'max_joint_step_rad')),
                '比較対象の解像度・明示除外・補間条件の不一致')
        case = {'name': name, 'pid': record['pid'], 'cleanup_ok': record['cleanup_ok'],
                'is_zero_safe': hybrid['zero_pose']['is_safe'], 'zero_collision_pairs': hybrid['zero_pose'].get('collision_pairs', []),
                'groups': {}, 'known_collision_nodes': {}, 'metrics_path': str(hybrid_dir / 'metrics.json')}
        for group in ('nodes', 'references', 'heldout'):
            old_rows = read_rows(old_dir / f'{group}_audit.csv')
            hybrid_rows = read_rows(hybrid_dir / f'{group}_audit.csv')
            require(old_rows.keys() == hybrid_rows.keys(), '比較対象の姿勢ID不一致')
            is_pose_equal = pose_rows(old_dir, group) == pose_rows(hybrid_dir, group)
            old_safe_to_unsafe = [node_id for node_id in old_rows if old_rows[node_id]['is_safe'] == '1' and hybrid_rows[node_id]['is_safe'] != '1']
            old_unsafe_to_safe = [node_id for node_id in old_rows if old_rows[node_id]['is_safe'] != '1' and hybrid_rows[node_id]['is_safe'] == '1']
            values = {'num_checked': hybrid[group]['num_checked'], 'num_old_safe': old[group]['num_safe'],
                      'num_hybrid_safe': hybrid[group]['num_safe'], 'num_hybrid_rejected': hybrid[group]['num_rejected'],
                      'num_limit_rejected': hybrid[group]['num_limit_rejected'], 'is_pose_equal': is_pose_equal,
                      'old_safe_to_unsafe_ids': old_safe_to_unsafe, 'old_unsafe_to_safe_ids': old_unsafe_to_safe,
                      'old_time': check_time(old[group]), 'hybrid_time': check_time(hybrid[group])}
            case['groups'][group] = values
            result['is_all_checks_passed'] &= is_pose_equal and not old_safe_to_unsafe
            if name == 'max' and group == 'nodes':
                for node_id in (10025, 10062):
                    row = hybrid_rows.get(node_id)
                    require(row is not None, '既知干渉IDがpilot範囲外')
                    case['known_collision_nodes'][str(node_id)] = row
                    result['is_all_checks_passed'] &= row['is_safe'] == '0' and row['reason'] == 'self_collision'
        overhead = hybrid['elapsed_sec'] - sum(hybrid[group]['elapsed_sec'] for group in case['groups'])
        case['full_audit_estimate_sec'] = overhead + sum(hybrid[group]['pose_check_sec'] * hybrid[group]['num_input'] / hybrid[group]['num_checked'] +
                                                        hybrid[group]['collision_pair_collection_sec'] for group in case['groups'])
        result['is_all_checks_passed'] &= case['is_zero_safe'] and not case['zero_collision_pairs']
        result['cases'].append(case)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(args.output), 'is_all_checks_passed': result['is_all_checks_passed'],
                      'elapsed_sec': result['hybrid_elapsed_sec']}, ensure_ascii=False))
    require(result['is_all_checks_passed'], 'hybrid pilotの安全性・姿勢照合で不一致。保存結果参照。')


if __name__ == '__main__':
    main()
