"""再検査済みグラフからの通常VLUT生成・Viewer配信用設定。"""
import argparse
import importlib.util
import json
from pathlib import Path

import yaml
from prepare_exclusions import write_atomic_new

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path, default=Path('/home/uraki/uraki_ws'))
parser.add_argument('--model', choices=('max', 'long'), required=True)
parser.add_argument('--model-dir', type=Path, required=True)
args = parser.parse_args()
root = args.root.resolve()
folder = args.model_dir.resolve()
metrics = json.loads((folder / 'metrics.json').read_text())
if metrics.get('collision_method') != 'mesh_surface_and_component_voxel_containment':
    raise ValueError('最終判定方式での再構成記録が必要')
if metrics['mode'] != 'rebuild' or not metrics['rebuild']['is_roundtrip_verified']:
    raise ValueError('保存・再読込検証済みグラフが必要')
if (folder / 'preview.yaml').exists() or (folder / 'expected.json').exists():
    raise ValueError('既存設定への上書き拒否')
name = 'topo_dual_arm_max' + ('_long' if args.model == 'long' else '')
data = yaml.safe_load((root / f'gng_vlut_system/config/{name}.yaml').read_text())
params = data['/**']['ros__parameters']
params['robot_name'] = f'self_collision_{args.model}_2cm'
params['gng']['data_directory'] = str(Path('/ros2_ws/src') / folder.parent.relative_to(root))
params['gng']['experiment_id'] = folder.name
params['gng']['vlut_only'] = True
params['gng']['use_voxel_collision'] = True
params.setdefault('gng_params', {})['max_node_num'] = metrics['rebuild']['num_output_nodes']
params.setdefault('visualization_gng', {})['enabled'] = False
params.setdefault('self_recognition', {})['enable_self_recognition_viz'] = False
path = root / 'benchmarks/voxel_pose_compression_20260930/export_preview.py'
spec = importlib.util.spec_from_file_location('self_collision_export', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
graph = module.read_gng(folder / 'gng.bin')
# Viewerの配信内容との照合用の保存姿勢・手先位置
# バイナリ構造の読込は既存の検証実装を共用
verify_path = root / 'benchmarks/gng_coverage_repair_20261001/verify_repair.py'
spec = importlib.util.spec_from_file_location('self_collision_verify', verify_path)
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)
ids, q, tcp = verify.decode_states(graph)
nodes = {str(int(node_id)): {'angles': q[idx].tolist(), 'tcp': tcp[idx].tolist()}
         for idx, node_id in enumerate(ids)}
if len(ids) != metrics['rebuild']['num_output_nodes']:
    raise ValueError('モデルのノード数と検証記録の不一致')
if params['collision']['voxel_size'] != metrics['collision_voxel_size_m']:
    raise ValueError('検証記録と設定の衝突解像度の不一致')
if params['frame_id'] != 'base_link':
    raise ValueError('未対応の配信座標フレーム')
expected = {'robot_name': params['robot_name'],
            'frame_id': params['robot_name'] + '/base_link', 'nodes': nodes,
            'edge_counts': [len(layer) for layer in graph['edge_layers']],
            'edge_pairs': [
                [sorted((edge['first_id'], edge['second_id'])) for edge in layer]
                for layer in graph['edge_layers']]}
expected_path = folder / 'expected.json'
write_atomic_new(expected_path, (json.dumps(expected, ensure_ascii=False) + '\n').encode('utf-8'))
try:
    config_text = '# 全身自己衝突の再検査済みグラフのViewer設定\n' + yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    write_atomic_new(folder / 'preview.yaml', config_text.encode('utf-8'))
except BaseException:
    expected_path.unlink()
    raise
print(json.dumps({'model': args.model, 'model_dir': str(folder), 'num_nodes': len(ids)}))
