"""実在エッジ配列の単一走査と、学習中に更新済みの座標配列の再利用。"""
from pathlib import Path
import sys
source = Path(sys.argv[1])
def replace(text,before,after):
    assert text.count(before) == 1, (before,text.count(before))
    return text.replace(before,after,1)
path=source/'src/cpu/cugng.hpp'
text=path.read_text()
text=text.replace('GNG::exec内の同期再利用と、変更候補だけの保守。単独呼出しは全走査。',
                  'フレーム内の同期再利用・孤立候補確認・実在エッジ走査。単独呼出しは従来処理。')
text=text.replace('    bool has_untracked_edge_changes = true;\n','')
text=text.replace('    vector<uint64_t> changed_node_words;', '    vector<array<uint32_t, 2>> edge_node_ids;')
start=text.index('    void mark_edge_update(uint32_t node_id) {')
end=text.index('    void begin_search_batch();',start)
text=text[:start]+text[end:]
path.write_text(text)
path=source/'src/cpu/cugng.cpp'
text=path.read_text()
for line in ('    changed_node_words.assign((node_num_max + 63) / 64, 0);\n',
             '    changed_node_words.clear();\n', '    has_untracked_edge_changes = true;\n',
             '    mark_edge_update(node.id);\n','            mark_edge_update(i);\n',
             '    mark_edge_update(idx1);\n'):
    text=text.replace(line,'')
text=replace(text, '    edge_reference_num.assign(1, 0);', '    edge_reference_num.assign(1, 0);\n    edge_node_ids.assign(1, {0, 0});')
text=replace(text, '    edge_reference_num.reserve(max_edge_num);', '    edge_reference_num.reserve(max_edge_num);\n    edge_node_ids.reserve(max_edge_num);')
text=replace(text, '    edge_reference_num.clear();', '    edge_reference_num.clear();\n    edge_node_ids.clear();')
text=replace(text, '            edge_reference_num.push_back(0);', '            edge_reference_num.push_back(0);\n            edge_node_ids.push_back({0, 0});')
text=replace(text, '    edge_reference_num[edge_idx] += 2;', '''    edge_node_ids[edge_idx] = {std::min(idx1, idx2), std::max(idx1, idx2)};
    edge_reference_num[edge_idx] += 2;''')
text=replace(text, '    is_search_batch = false;\n}\nvoid CUGNG::learn_normal', '    is_search_batch = enable_frame_search_reuse;\n}\nvoid CUGNG::learn_normal')
start=text.index('    if (enable_frame_edge_updates && !has_untracked_edge_changes)')
end=text.index('    int i;\n    uint32_t edge_id;',start)
text=text[:start]+'''    if (enable_frame_edge_updates) {
        // 両端の隣接配列を重複走査しない、実在エッジ用プールの単一走査。
        for (uint32_t edge_idx = 1; edge_idx < edge_node_ids.size(); ++edge_idx) {
            if (edge_reference_num[edge_idx] == 0) {continue;}
            const auto &ids = edge_node_ids[edge_idx];
            auto &first = is_search_batch ? search_nodes[ids[0]].pos : nodes[ids[0]].pos;
            auto &second = is_search_batch ? search_nodes[ids[1]].pos : nodes[ids[1]].pos;
            edge_distance[edge_idx] = first.squaredNormXY(second);
        }
        return;
    }
'''+text[end:]
text=text.replace('    std::fill(changed_node_words.begin(), changed_node_words.end(), 0);\n','')
text=text.replace('    has_untracked_edge_changes = false;\n','')
path.write_text(text)
print('single edge pool traversal prepared')
