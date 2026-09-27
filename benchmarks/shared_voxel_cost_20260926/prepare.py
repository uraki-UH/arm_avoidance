"""実装非変更の計測用コピー生成。置換箇所数の検査付き。"""
from pathlib import Path
import hashlib
import json
import os

root = Path(__file__).resolve().parents[2]
output = root / 'artifacts/shared_voxel_cost_20260926' / os.environ.get('SHARED_COST_VARIANT', '') / 'generated'
output.mkdir(parents=True, exist_ok=True)

def replace(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)

def insert_scope(text, signature, name, extra=''):
    start = text.index(signature)
    pos = text.index('{', start) + 1
    return text[:pos] + f'\n    shared_bench::scope bench_time("{name}");\n' + extra + text[pos:]

world_path = root / 'gng_vlut_system/src/nodes/bridge/world_index_to_voxel_node.cpp'
fvg_path = root / 'fuzzy_voxel_grid/src/voxel_grid_node.cpp'
world = '#include "trace.hpp"\n' + world_path.read_text()
world = insert_scope(world, 'void pointCallback(', 'world_total',
    '    bench_time.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();\n'
    '    bench_time.num = msg->width * msg->height;\n')
world = insert_scope(world, 'void publishWorldBuckets(', 'world_publish')
for begin, end, name in [
    ('    const auto world_index_build_start =', '    const double world_index_build_ms =', 'world_build'),
    ('    const auto primary_query_start =', '    const double primary_query_ms =', 'world_roi')]:
    world = replace(world, begin, f'    shared_bench::scope bench_{name}("{name}");\n' + begin)
    world = replace(world, end, f'    bench_{name}.finish();\n' + end)
world = replace(world, 'int main(int argc, char **argv)', 'int unused_world_main(int argc, char **argv)')
(output / 'world.cpp').write_text(world)

fvg = '#include "trace.hpp"\n' + fvg_path.read_text()
for method, name in [('update_shared_points', 'fvg_points'),
    ('rebuildVoxelsFromLatestMessages', 'fvg_cells'), ('rebuildViewsFromManagedVoxels', 'fvg_labels'),
    ('topologicalMapCallback', 'fvg_tmap'), ('timerCallback', 'fvg_timer'),
    ('publishCombinedMarkerArray', 'fvg_marker')]:
    extra = '    if (!shared_bench::enable_markers()) {return;}\n' if name == 'fvg_marker' else ''
    fvg = insert_scope(fvg, f'void VoxelGridNode::{method}(', name, extra)
fvg = replace(fvg, '    has_pending_update_ = true;\n    if (latest_topological_map_msg_',
    '    shared_bench::record("fvg_frame", shared_bench::steady_ns(), 0, 0, frame->stamp_ns, frame->point_idx->point_num());\n'
    '    has_pending_update_ = true;\n    if (latest_topological_map_msg_')
fvg = replace(fvg, '    printDebugVoxelSummary();',
    '    shared_bench::record("fvg_state", shared_bench::steady_ns(), 0, 0, rclcpp::Time(point_header_.stamp).nanoseconds(), combined_voxels_.size());\n'
    '    printDebugVoxelSummary();')
if 'VoxelGridNode::make_marker_cache(' in fvg:
    fvg = insert_scope(fvg, 'VoxelGridNode::make_marker_cache(', 'fvg_marker_build',
        '    if (!shared_bench::enable_markers()) {return visualization_msgs::msg::MarkerArray();}\n')
(output / 'fvg.cpp').write_text(fvg)
paths = [world_path, fvg_path, root/'fuzzy_voxel_grid/include/fuzzy_voxel_grid/voxel_grid_node.hpp',
         root/'voxel_idx/include/point_cloud_store.hpp', root/'voxel_idx/src/point_cloud_store.cpp']
(output.parent/'source_sha256.json').write_text(json.dumps(
    {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, indent=2))
print(output)
