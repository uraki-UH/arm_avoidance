"""検査済み補完グラフからの別保存モデル・Viewer設定の準備。"""
import argparse
import json
from pathlib import Path
import shutil
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--root', type=Path, required=True)
parser.add_argument('--model', choices=('max', 'long'), required=True)
parser.add_argument('--experiment-root', type=Path, required=True)
args = parser.parse_args()
root = args.root.resolve()
folder = args.experiment_root.resolve() / args.model
source = folder / 'graph_output'
output = folder / 'model'
report = json.loads((source / 'repair_metrics.json').read_text())
assert report['num_original_collision_rejected'] == 0
assert report['num_original_limit_rejected'] == 0
assert report['num_original_collision_free'] == report['num_original_nodes']
assert report['is_binary_roundtrip_verified'] and report['is_existing_gng_loader_verified']
assert report['output_graph'][0]['num_new_nodes_connected_to_original'] == report['num_added_nodes']
assert report['heldout']['num_trials_per_arm'] > 0
assert not (output / 'gng.bin').exists() and not (output / 'vlut.bin').exists()
output.mkdir(exist_ok=True)
for name in ('gng.bin', 'repair_metrics.json', 'heldout.csv', 'added_nodes.csv', 'added_edges.csv', 'original_node_collision.csv'):
    assert not (output / name).exists()
    shutil.copy2(source / name, output / name)
name = 'topo_dual_arm_max' + ('_long' if args.model == 'long' else '')
data = yaml.safe_load((root / f'gng_vlut_system/config/{name}.yaml').read_text())
params = data['/**']['ros__parameters']
params['robot_name'] = f'coverage_{args.model}_2cm'
params['gng']['data_directory'] = str(Path('/ros2_ws/src') / folder.relative_to(root))
params['gng']['experiment_id'] = 'model'
params['gng']['vlut_only'] = True
params['gng']['use_voxel_collision'] = False
params.setdefault('gng_params', {})['max_node_num'] = report['num_output_nodes']
params.setdefault('visualization_gng', {})['enabled'] = False
params.setdefault('self_recognition', {})['enable_self_recognition_viz'] = False
(output / 'preview.yaml').write_text('# 既知手先位置の2 cm被覆を補完した別モデルのViewer設定\n' + yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
print(json.dumps({'model': args.model, 'output': str(output), 'num_nodes': report['num_output_nodes']}))
