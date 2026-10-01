"""同一入力に対する厳密TFキャッシュ導入前後の判定同値性検査。"""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--old-batch', type=Path, required=True)
parser.add_argument('--cached-batch', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
assert not args.output.exists(), 'Output already exists'
result = {'is_equal': True, 'models': {}}
policy_keys = ['collision_method', 'collision_voxel_size_m', 'max_joint_step_rad', 'manual_exclusions']
for model in ('max', 'long'):
    old_dir = args.old_batch / f'001_{model}' / 'result'
    new_dir = args.cached_batch / f'001_{model}' / 'result'
    old = json.loads((old_dir / 'metrics.json').read_text())
    new = json.loads((new_dir / 'metrics.json').read_text())
    model_result = {'files': {}, 'groups': {}, 'is_policy_equal': all(old[key] == new[key] for key in policy_keys)}
    model_result['is_zero_safe'] = old['zero_pose']['is_safe'] and new['zero_pose']['is_safe']
    model_result['is_equal'] = model_result['is_policy_equal'] and model_result['is_zero_safe']
    assert new['collision_method'] == 'mesh_surface_and_component_voxel_containment'
    for group in ('nodes', 'references', 'heldout'):
        for kind in ('audit', 'kept', 'rejected'):
            name = f'{group}_{kind}.csv'
            before = (old_dir / name).read_bytes()
            after = (new_dir / name).read_bytes()
            is_equal = before == after
            model_result['files'][name] = {'is_equal': is_equal, 'sha256': hashlib.sha256(after).hexdigest()}
            model_result['is_equal'] &= is_equal
        group_result = {}
        for label, values in (('before', old), ('after', new)):
            item = values[group]
            group_result[label] = {key: item[key] for key in ('num_checked', 'num_safe', 'num_rejected', 'pose_check_sec', 'safe_pose_check_sec', 'rejected_pose_check_sec')}
            group_result[label]['mean_safe_query_ms'] = 1000 * item['safe_pose_check_sec'] / item['num_safe'] if item['num_safe'] else 0
        model_result['groups'][group] = group_result
    if model == 'max':
        import csv
        rejected_ids = {int(row['id']) for row in csv.DictReader((new_dir / 'nodes_rejected.csv').open())}
        model_result['known_bad_ids_rejected'] = {str(node_id): node_id in rejected_ids for node_id in (10025, 10062)}
        model_result['is_equal'] &= all(model_result['known_bad_ids_rejected'].values())
    model_result['elapsed_sec'] = {'before': old['elapsed_sec'], 'after': new['elapsed_sec']}
    model_result['num_collision_checks'] = {'before': old['num_collision_checks'], 'after': new['num_collision_checks']}
    result['models'][model] = model_result
    result['is_equal'] &= model_result['is_equal']
with args.output.open('x') as output:
    json.dump(result, output, indent=2)
    output.write('\n')
print(json.dumps(result, separators=(',', ':')))
raise SystemExit(0 if result['is_equal'] else 1)
