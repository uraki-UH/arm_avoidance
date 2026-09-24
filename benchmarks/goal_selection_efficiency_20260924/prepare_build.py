"""旧8分木・bsp3d・全走査削減版の同一プロセス比較ビルド。"""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

root = Path('/ros2_ws/src')
trial = root / 'benchmarks/goal_selection_efficiency_20260924'
out = root / 'artifacts/goal_selection_efficiency_20260924'
work = Path('/tmp/goal_selection_efficiency_20260924')
work.mkdir(exist_ok=True)
legacy = out / 'legacy'
# ODR違反を避ける比較専用名前空間。製品ヘッダへの旧依存の復活なし。
for p in (legacy / 'SpatialTree/include').rglob('*.hpp'):
    target = work / 'reference_include' / p.relative_to(legacy / 'SpatialTree/include')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(p.read_text().replace('namespace SpatialTree', 'namespace reference_spatial_tree')
                      .replace('SpatialTree::', 'reference_spatial_tree::')
                      .replace('SPATIAL_TREE', 'REFERENCE_SPATIAL_TREE'))
wrappers = []
for method in ['old_before', 'bsp_before', 'old_cached', 'bsp_cached', 'bsp_incremental']:
    dest = work / method
    dest.mkdir(exist_ok=True)
    is_cached = method.endswith('cached') or method == 'bsp_incremental'
    spatial = (legacy / 'gng_vlut_system/src/nodes/planning/goal_spatial_index.hpp').read_text() if method.startswith('old') else (out / 'goal_spatial_index.hpp').read_text()
    if method == 'bsp_incremental': spatial = (root / 'gng_vlut_system/src/nodes/planning/goal_spatial_index.hpp').read_text()
    spatial = spatial.replace('SpatialTree::', 'reference_spatial_tree::') if method.startswith('old') else spatial
    files = {'goal_spatial_index.hpp': spatial}
    files['goal_node_selection.hpp'] = (root / 'gng_vlut_system/src/nodes/planning/goal_node_selection.hpp').read_text() if is_cached else (out / 'goal_node_selection.hpp').read_text()
    if is_cached:
        files['goal_selection_cache.hpp'] = (root / 'gng_vlut_system/src/nodes/planning/goal_selection_cache.hpp').read_text()
    for name, content in files.items():
        (dest / name).write_text(content.replace('namespace robot_sim::planning', 'namespace compare_' + method))
    wrappers.append('#include "' + method + '/goal_node_selection.hpp"')
    wrappers.append(f'''class {method}_runner final : public runner {{
  compare_{method}::goal_spatial_index spatial_;
  {f'compare_{method}::goal_selection_cache cache_;' if is_cached else ''}
  bool enable_map_ = true, enable_features_ = true;
public:
  {method}_runner(bool enable_map = true, bool enable_features = true)
      : enable_map_(enable_map), enable_features_(enable_features) {{}}
  void update_map(map_type::ConstSharedPtr map) override {{
    spatial_.update(map);
    {('if (enable_map_) cache_.update_map(map);' if is_cached else '')}
  }}
  void update_features(features_type::ConstSharedPtr features) override {{
    {('if (enable_features_) cache_.update_features(features);' if is_cached else '')}
  }}
  result_type select(const map_type *map, const source_type &source, const features_type *features,
      const lookup_type &lookup, bool enable_spatial = true) override {{
    auto result = compare_{method}::select_goal_nodes(map, source, features, {{}}, lookup,
        enable_spatial ? &spatial_ : nullptr{', &cache_' if is_cached else ''});
    return {{std::move(result.map), std::move(result.ids)}};
  }}
}};''')
(work / 'variants.hpp').write_text('\n'.join(wrappers))
flags_path = Path('/ros2_ws/build/gng_vlut_system/CMakeFiles/test_grasp_candidate_reachability.dir/flags.make')
includes = shlex.split(next(x.split(' = ', 1)[1] for x in flags_path.read_text().splitlines() if x.startswith('CXX_INCLUDES = ')))
command = ['c++', '-std=c++17', '-O3', '-DNDEBUG', '-I'+str(work), '-I'+str(work/'reference_include'), '-I'+str(root/'bsp3d/include'), *includes, str(trial/'benchmark.cpp'), '-o', str(work/'benchmark')]
(out/'build_commands.txt').write_text(shlex.join(command)+'\n')
with (out/'benchmark_build.log').open('w') as log:
    result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=240)
if result.returncode:
    raise RuntimeError((out/'benchmark_build.log').read_text()[-6000:])
(out/'benchmark_sha256.json').write_text(json.dumps({'binary':hashlib.sha256((work/'benchmark').read_bytes()).hexdigest()},indent=2)+'\n')
print('Same-process benchmark built', flush=True)
