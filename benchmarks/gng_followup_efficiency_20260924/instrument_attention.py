"""比較コピー限定の、全点照合・重点候補生成・対応ソートの個別計測。"""
import argparse
import hashlib
import json
from pathlib import Path


def replace(text, before, after):
    assert text.count(before) == 1, (before, text.count(before))
    return text.replace(before, after, 1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    assert source.name.endswith('_deterministic') and 'artifacts' in source.parts, source
    path = source / 'src/cpu/gng.cpp'
    text = path.read_text()
    assert 'double gng_efficiency_ms[6]' in text, 'prepare.py適用後の比較コピーが必要'
    assert 'gng_attention_phase_ms' not in text, '既に計測済み'
    text = replace(text, '#include "gng.hpp"', '#include "gng.hpp"\ndouble gng_attention_phase_ms[3]{};')
    marker = '    n1.getDownSampling(vg.filtered_pcl, vg.filtered_pcl_num, voxel_labels, voxel2node_ids, voxel2node_ids_num);'
    text = replace(text, marker, '''    const auto attention_match_begin = std::chrono::steady_clock::now();
''' + marker + '''
    const auto attention_match_end = std::chrono::steady_clock::now();''')
    marker = '    boost::sort::spreadsort::integer_sort(voxel2node_ids.data(),'
    text = replace(text, marker, '    const auto attention_sort_begin = std::chrono::steady_clock::now();\n' + marker)
    marker = '        [](const Voxel &voxel, unsigned offset) { return voxel.voxel_index >> offset; });'
    text = replace(text, marker, marker + '''
    const auto attention_sort_end = std::chrono::steady_clock::now();
    // 全点照合、重点候補生成、対応ソートの実時間。入力読込・ハッシュ計算は対象外。
    gng_attention_phase_ms[0] = std::chrono::duration<double, std::milli>(attention_match_end - attention_match_begin).count();
    gng_attention_phase_ms[1] = std::chrono::duration<double, std::milli>(attention_sort_begin - attention_match_end).count();
    gng_attention_phase_ms[2] = std::chrono::duration<double, std::milli>(attention_sort_end - attention_sort_begin).count();''')
    path.write_text(text)
    path = source / 'src/api.cpp'
    text = path.read_text()
    assert 'gng_get_attention_phase_ms' not in text
    path.write_text(text + '''
// 比較コピー限定の段階時間と、公開Node出力で未提供の入力voxel対応の参照。
extern double gng_attention_phase_ms[3];
extern "C" MY_API const double *gng_get_attention_phase_ms() {
    return gng_attention_phase_ms;
}
extern "C" MY_API const Voxel *gng_get_attention_mapping(uint32_t *num) {
    *num = gng.voxel2node_ids_num;
    return gng.voxel2node_ids.data();
}
''')
    path = source / 'gng_cpu.v'
    path.write_text(replace(path.read_text(), '  global:',
        '  global:\n    gng_get_attention_phase_ms;\n    gng_get_attention_mapping;'))
    # 追加計測を含めた、最終比較ソースの保存。
    method = source.name.removesuffix('_deterministic')
    manifest = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(source.rglob('*')) if p.is_file()}
    (source.parent / (method + '_source_sha256.json')).write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
