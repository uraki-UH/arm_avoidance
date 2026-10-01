"""元レコード保持による問題辺の単独抽出・除外。"""
import argparse
import importlib.util
import json
from pathlib import Path
import struct

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--first-id', type=int, required=True)
parser.add_argument('--second-id', type=int, required=True)
parser.add_argument('--mode', choices=('fixture', 'remove'), required=True)
args = parser.parse_args()
spec = importlib.util.spec_from_file_location('gng_records',
    Path(__file__).parents[1] / 'voxel_pose_compression_20260930/export_preview.py')
codec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(codec)
graph = codec.read_gng(args.input, expected_num_layers=1, allow_missing_endpoints=True)
pair = {args.first_id, args.second_id}
assert len(pair) == 2 and pair <= graph['node_ids']
nodes = [node for node in graph['nodes'] if args.mode == 'remove' or node['id'] in pair]
layers = []
num_matched_edges = 0
num_missing_endpoints = 0
for edges in graph['edge_layers']:
    layer = []
    for edge in edges:
        if not {edge['first_id'], edge['second_id']} <= graph['node_ids']:
            num_missing_endpoints += 1
            continue
        is_match = {edge['first_id'], edge['second_id']} == pair
        num_matched_edges += int(is_match)
        if is_match == (args.mode == 'fixture'):
            layer.append(edge['record'])
    layers.append(layer)
assert num_matched_edges > 0
with args.output.open('xb') as output:
    output.write(struct.pack('<Iii', 9, graph['num_layers'], len(nodes)))
    for node in nodes:
        output.write(node['record'])
    for edges in layers:
        output.write(struct.pack('<i', len(edges)))
        for edge in edges:
            output.write(edge)
checked = codec.read_gng(args.output, expected_num_layers=1)
assert checked['nodes'] == nodes
print(json.dumps({'output': str(args.output), 'mode': args.mode,
                  'num_nodes': len(nodes), 'num_matched_edges': num_matched_edges,
                  'num_missing_endpoints': num_missing_endpoints,
                  'num_edges': [len(layer) for layer in layers]}))
