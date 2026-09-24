from pathlib import Path
import hashlib
import json
import shutil
import argparse

# 本番に持ち込まない、比較用コピーだけの乱数・時間刻み固定と計測。
root = Path('/ros2_ws/src')
output = root / 'artifacts/gng_incremental_updates_20260924'
# 比較用パッケージ名の重複検出防止。
(output / 'COLCON_IGNORE').touch()
parser = argparse.ArgumentParser()
parser.add_argument('--methods', nargs='+', default=['before', 'after'])
args = parser.parse_args()
for method in args.methods:
    source = output / (method + '_deterministic')
    original = output / (method + '_source')
    shutil.copytree(original, source)
    path = source / 'src/cpu/cugng.cpp'
    text = path.read_text()
    text = text.replace('    random_device rnd;  // 非決定的な乱数生成器\n    mt19937 mt(rnd());  // 初期シード値',
                        '    mt19937 mt(20260924U + frame_number); // 比較用の固定シード。')
    assert 'mt19937 mt(20260924U' in text
    text = text.replace('#include <numeric>', '#include <numeric>\nuint64_t gng_efficiency_learning_num = 0;')
    text = text.replace('uint32_t raw_idx, bool enable_statistics) {',
                        'uint32_t raw_idx, bool enable_statistics) {\n    ++gng_efficiency_learning_num;')
    assert '++gng_efficiency_learning_num;' in text
    path.write_text(text)
    path = source / 'src/utils/utils.hpp'
    text = path.read_text().replace('dt = LIMIT(dt, 0.1, 0.5);', 'dt = 0.1f; // 比較用の固定時間刻み。')
    path.write_text(text)
    path = source / 'src/cpu/gng.cpp'
    text = path.read_text().replace('#include "gng.hpp"', '#include "gng.hpp"\ndouble gng_efficiency_ms[6]{};\nextern uint64_t gng_efficiency_learning_num;')
    text = text.replace('    auto t0 = std::chrono::system_clock::now();',
                        '    gng_efficiency_learning_num = 0;\n    auto t0 = std::chrono::system_clock::now();')
    anchor = '    auto t6 = std::chrono::system_clock::now();'
    text = text.replace(anchor, anchor + '\n' + '\n'.join(
        f'    gng_efficiency_ms[{idx}] = std::chrono::duration<double, std::milli>(t{idx+1} - t{idx}).count();'
        for idx in range(6)))
    path.write_text(text)
    path = source / 'src/api.cpp'
    path.write_text(path.read_text() + '''
// 比較コピー限定の中間結果・時間の参照。
extern double gng_efficiency_ms[6];
extern double gng_normal_phase_ms[2];
extern uint64_t gng_efficiency_learning_num;
struct gng_efficiency_view {
    const double *ms;
    const Voxel *indices;
    const VoxelRange *ranges;
    const Vec3f *points;
    uint32_t input_num, voxel_num, attention_num;
    uint64_t learning_num;
};
extern "C" MY_API gng_efficiency_view gng_get_efficiency_view() {
    return {gng_efficiency_ms, gng.vg.voxel_index.data(), gng.vg.voxel_range.data(),
        gng.vg.filtered_pcl.data(), gng.vg.voxel_index_num, gng.vg.filtered_pcl_num,
        static_cast<uint32_t>(gng.attention_pcl_num), gng_efficiency_learning_num};
}
''')
    path = source / 'src/api.cpp'
    path.write_text(path.read_text() + '\nextern "C" MY_API const double *gng_get_normal_phase_ms() {return gng_normal_phase_ms;}\n')
    path = source / 'src/cpu/labelling.cpp'
    text = path.read_text().replace('#include "labelling.hpp"', '#include "labelling.hpp"\ndouble gng_normal_phase_ms[2]{};')
    text = text.replace('    float max_exp;', '    float max_exp;\n    const auto phase_begin = std::chrono::steady_clock::now();')
    marker = '    for (auto &node : gng->nodes) {\n        if (node.id == NODE_NOID)\n            continue;\n        float previous_rho;'
    assert marker in text
    text = text.replace(marker, '    const auto phase_middle = std::chrono::steady_clock::now();\n' + marker, 1)
    marker = '\n}\n\nfloat Labelling::_fuzzy_safe_label'
    assert marker in text
    text = text.replace(marker, '\n    const auto phase_end = std::chrono::steady_clock::now();\n    gng_normal_phase_ms[0] = std::chrono::duration<double, std::milli>(phase_middle - phase_begin).count();\n    gng_normal_phase_ms[1] = std::chrono::duration<double, std::milli>(phase_end - phase_middle).count();' + marker, 1)
    path.write_text(text)
    path = source / 'gng_cpu.v'
    path.write_text(path.read_text().replace('  global:', '  global:\n    gng_get_efficiency_view;\n    gng_get_normal_phase_ms;'))
    manifest = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(source.rglob('*')) if p.is_file()}
    (output / (method + '_source_sha256.json')).write_text(json.dumps(manifest, indent=2) + '\n')
print('comparison copies prepared')
