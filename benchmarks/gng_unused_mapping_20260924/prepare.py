"""比較コピー限定の固定乱数・時間刻み・区間計測。"""
from pathlib import Path
import hashlib
import json
import shutil

root = Path('/ros2_ws/src')
output = root / 'artifacts/gng_unused_mapping_20260924'
shutil.copytree(root / 'ais_gng_cpu/src/gng_cpu', output / 'after_source')
for method in ('before', 'after'):
    original = output / (method + '_source')
    source = output / (method + '_deterministic')
    shutil.copytree(original, source)
    path = source / 'src/cpu/cugng.cpp'
    text = path.read_text().replace('mt19937 mt(rnd());', 'mt19937 mt(20260924U + frame_number);')
    assert 'mt19937 mt(20260924U' in text
    text = text.replace('#include <numeric>', '#include <numeric>\nuint64_t gng_efficiency_learning_num = 0;')
    text = text.replace('uint32_t raw_idx, bool enable_statistics) {',
                        'uint32_t raw_idx, bool enable_statistics) {\n    ++gng_efficiency_learning_num;')
    assert '++gng_efficiency_learning_num;' in text
    path.write_text(text)
    path = source / 'src/utils/utils.hpp'
    text = path.read_text()
    assert 'dt = LIMIT(dt, 0.1, 0.5);' in text
    path.write_text(text.replace('dt = LIMIT(dt, 0.1, 0.5);', 'dt = 0.1f;'))
    path = source / 'src/cpu/gng.cpp'
    text = path.read_text().replace('#include "gng.hpp"',
        '#include "gng.hpp"\ndouble gng_efficiency_ms[6]{};\nextern uint64_t gng_efficiency_learning_num;')
    text = text.replace('    auto t0 = std::chrono::system_clock::now();',
        '    gng_efficiency_learning_num = 0;\n    auto t0 = std::chrono::system_clock::now();')
    anchor = '    auto t6 = std::chrono::system_clock::now();'
    assert anchor in text
    text = text.replace(anchor, anchor + '\n' + '\n'.join(
        f'    gng_efficiency_ms[{idx}] = std::chrono::duration<double, std::milli>(t{idx+1} - t{idx}).count();'
        for idx in range(6)))
    path.write_text(text)
    path = source / 'src/api.cpp'
    path.write_text(path.read_text() + """
// 比較コピー限定の中間結果と区間時間の参照。
extern double gng_efficiency_ms[6];
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
""")
    path = source / 'gng_cpu.v'
    path.write_text(path.read_text().replace('  global:', '  global:\n    gng_get_efficiency_view;'))
    for name, folder in (('source', original), ('deterministic', source)):
        manifest = {str(path.relative_to(folder)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sorted(folder.rglob('*')) if path.is_file()}
        (output / f'{method}_{name}_sha256.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('before/after comparison copies ready')
