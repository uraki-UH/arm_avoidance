import argparse
import csv
import itertools
import json
import math
import random
import struct
import subprocess
from pathlib import Path

# 負座標・格子境界・空集合・同一形状・離れた関節角の組合せ。
parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, default=Path('/tmp/voxel_pose_synthetic'))
parser.add_argument('--executable', type=Path, required=True)
args = parser.parse_args()
base = args.output
base.mkdir(parents=True, exist_ok=False)
rng = random.Random(9001)
nodes = []
for node_idx in range(90):
    group_idx = node_idx % 9
    shift = (group_idx - 4) * 7
    masks = []
    for link_idx in range(3):
        if group_idx == 0 or (group_idx == 1 and link_idx == 2):
            masks.append([])
            continue
        points = set()
        local_shift = rng.choice([-2, -1, 0, 0, 0, 1, 2])
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    points.add((shift + local_shift + x, -shift + y + link_idx, z - 3))
        masks.append(sorted(points))
    nodes.append((1000 + node_idx, [node_idx * 0.01, group_idx * 0.1],
                  [node_idx * 0.001, group_idx * 0.01, 0.0], masks))
# int32 座標上端・下端の算術境界。
for shift in [2147483647, 2147483646, -2147483648, -2147483647]:
    nodes.append((2000 + len(nodes), [0.0, 0.0], [0.0, 0.0, 0.0],
                  [[(shift, 0, 0)], [(0, shift, 0)], [(0, 0, shift)]]))
input_path = base / 'synthetic.voxpose'
with input_path.open('wb') as out:
    out.write(b'VOXPOSE1')
    out.write(struct.pack('<IIIIf', len(nodes), 3, 2, 1, 0.01))
    for node_id, angles, tcp, masks in nodes:
        out.write(struct.pack('<i', node_id))
        out.write(struct.pack('<2f', *angles))
        out.write(struct.pack('<3f', *tcp))
        for mask in masks:
            out.write(struct.pack('<I', len(mask)))
            for point in mask:
                out.write(struct.pack('<iii', *point))

def is_close(first, second, radius_cells):
    for first_cells, second_cells in zip(first[3], second[3]):
        if bool(first_cells) != bool(second_cells):
            return False
        for source, target in [(first_cells, second_cells), (second_cells, first_cells)]:
            for point in source:
                if not any(sum((a - b) ** 2 for a, b in zip(point, other)) <= radius_cells ** 2
                           for other in target):
                    return False
    return True

nodes_by_id = {node[0]: node for node in nodes}
results = []
for radius_cells, seed in itertools.product([0, 1, 2, 4], [0, 17, 991]):
    paths = []
    for is_exhaustive in [False, True]:
        prefix = base / f'synthetic_r{radius_cells}_s{seed}_e{int(is_exhaustive)}'
        command = [str(args.executable.resolve()), str(input_path), str(radius_cells), str(seed), str(prefix)]
        if is_exhaustive:
            command.append('--exhaustive')
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
        paths.append(prefix)
    indexed = Path(str(paths[0]) + '.assignments.csv').read_text()
    exhaustive = Path(str(paths[1]) + '.assignments.csv').read_text()
    assert indexed == exhaustive, (radius_cells, seed)
    rows = list(csv.DictReader(indexed.splitlines()))
    representatives = {int(row['group_idx']): int(row['original_id']) for row in rows if row['is_representative'] == '1'}
    for row in rows:
        node = nodes_by_id[int(row['original_id'])]
        representative = nodes_by_id[int(row['representative_id'])]
        assert is_close(node, representative, radius_cells), row
        for group_idx in range(int(row['group_idx'])):
            assert not is_close(node, nodes_by_id[representatives[group_idx]], radius_cells), row
    metrics = json.loads(Path(str(paths[0]) + '.metrics.json').read_text())
    assert metrics['num_union_false_negative_refs'] == 0
    assert metrics['num_original_link_voxel_refs'] >= metrics['num_group_union_link_voxel_refs']
    assert metrics['num_group_union_link_voxel_refs'] >= metrics['num_representative_link_voxel_refs']
    assert metrics['num_group_union_link_voxel_refs'] - metrics['num_representative_link_voxel_refs'] == metrics['num_additional_union_link_voxel_refs']
    results.append({'radius_cells': radius_cells, 'seed': seed, 'num_representatives': metrics['num_representatives'], 'is_passed': True})
(base / 'synthetic_test_results.json').write_text(json.dumps(results, indent=2) + '\n')
print(f'{len(results)} cases passed; indexed/exhaustive assignments identical; independent Euclidean checks passed')
