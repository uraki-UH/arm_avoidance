"""同期の再利用・孤立候補・距離更新候補の独立比較用変換。"""
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
parser.add_argument('--method', choices=('sync', 'orphan', 'edges', 'combined'), required=True)
args = parser.parse_args()

def replace(text, before, after):
    assert text.count(before) == 1, (before, text.count(before))
    return text.replace(before, after, 1)

path = args.source / 'src/cpu/cugng.hpp'
text = path.read_text()
text = replace(text, '    void clear();', '''    void clear();
    // GNG::exec内の同期再利用と、変更候補だけの保守。単独呼出しは全走査。
    void begin_update_frame(bool enable_search_reuse, bool enable_orphan_updates, bool enable_edge_updates);
    void end_update_frame();''')
text = replace(text, '    bool is_search_batch = false;', '''    bool is_search_batch = false;
    bool enable_frame_search_reuse = false;
    bool enable_frame_orphan_updates = false;
    bool enable_frame_edge_updates = false;
    bool has_untracked_edge_changes = true;
    // ノードID順の候補走査用ビット列。削除順・残存ノード数制約の維持。
    vector<uint64_t> orphan_node_words;
    vector<uint64_t> changed_node_words;
    void mark_orphan_node(const Node &node) {
        if (enable_frame_orphan_updates && node.id != NODE_NOID && node.edge_num == 0) {
            orphan_node_words[node.id / 64] |= uint64_t{1} << (node.id % 64);
        }
    }
    void mark_edge_update(uint32_t node_id) {
        if (enable_frame_edge_updates) {
            changed_node_words[node_id / 64] |= uint64_t{1} << (node_id % 64);
        } else {
            has_untracked_edge_changes = true;
        }
    }''')
path.write_text(text)

path = args.source / 'src/cpu/cugng.cpp'
text = path.read_text()
text = replace(text, '    search_nodes.resize(node_num_max);', '''    search_nodes.resize(node_num_max);
    orphan_node_words.assign((node_num_max + 63) / 64, 0);
    changed_node_words.assign((node_num_max + 63) / 64, 0);
    has_untracked_edge_changes = true;
    end_update_frame();''')
text = replace(text, '    search_nodes.clear();', '''    search_nodes.clear();
    orphan_node_words.clear();
    changed_node_words.clear();
    has_untracked_edge_changes = true;
    end_update_frame();''')
text = replace(text, 'void CUGNG::begin_search_batch() {', '''void CUGNG::begin_update_frame(bool enable_search_reuse, bool enable_orphan_updates, bool enable_edge_updates) {
    enable_frame_search_reuse = enable_search_reuse;
    enable_frame_orphan_updates = enable_orphan_updates;
    enable_frame_edge_updates = enable_edge_updates;
    is_search_batch = false;
    std::fill(orphan_node_words.begin(), orphan_node_words.end(), 0);
}

void CUGNG::end_update_frame() {
    is_search_batch = false;
    enable_frame_search_reuse = false;
    enable_frame_orphan_updates = false;
    enable_frame_edge_updates = false;
}

void CUGNG::begin_search_batch() {''')
text = replace(text, '            search_nodes[node.id] = {node.pos, node.label, node.clusted_label};',
    '            search_nodes[node.id] = {node.pos, node.label, node.clusted_label};\n            mark_orphan_node(node);')
text = replace(text, '    voxel2node_ids_num = j;\n    is_search_batch = false;',
    '    voxel2node_ids_num = j;\n    is_search_batch = enable_frame_search_reuse;')
text = replace(text, '    begin_search_batch();\n    uniform_int_distribution<> rA',
    '    if (!is_search_batch) {begin_search_batch();}\n    uniform_int_distribution<> rA')
text = replace(text, '    node.pos.p[0] = new_pos.p[0];',
    '    mark_edge_update(node.id);\n    node.pos.p[0] = new_pos.p[0];')
text = replace(text, '            node.frame = frame_number;',
    '            node.frame = frame_number;\n            mark_orphan_node(node);\n            mark_edge_update(i);')
text = replace(text, '    if (has_edge) {recordEdgeDelta(first, second, GNG_DELTA_REMOVE);}',
    '    if (has_edge) {recordEdgeDelta(first, second, GNG_DELTA_REMOVE);}\n    mark_orphan_node(first);\n    mark_orphan_node(second);')
text = replace(text, '                edge_count[edge_idx] = EDGE_NO_CONNECT;\n                break;',
    '                edge_count[edge_idx] = EDGE_NO_CONNECT;\n                mark_orphan_node(second);\n                break;')
text = replace(text, '    first.edge_num = 0;', '    first.edge_num = 0;\n    mark_orphan_node(first);')
text = replace(text, '    first.edges[first.edge_num++] = idx2;',
    '    mark_edge_update(idx1);\n    first.edges[first.edge_num++] = idx2;')
text = replace(text, 'void CUGNG::check_delete_no_edge_and_decay_eta() {', '''void CUGNG::check_delete_no_edge_and_decay_eta() {
    if (enable_frame_orphan_updates && !(gng_config.eta_decay_rate < 1.f)) {
        for (size_t word_idx = 0; word_idx < orphan_node_words.size(); ++word_idx) {
            auto word = orphan_node_words[word_idx];
            while (word != 0) {
                const uint32_t node_id = word_idx * 64 + __builtin_ctzll(word);
                word &= word - 1;
                auto &node = nodes[node_id];
                if (node.id != NODE_NOID && node.edge_num == 0) {delete_node(node_id);}
            }
        }
        return;
    }''')
text = replace(text, 'void CUGNG::calc_edge_distanceXY(){', '''void CUGNG::calc_edge_distanceXY(){
    if (enable_frame_edge_updates && !has_untracked_edge_changes) {
        for (size_t word_idx = 0; word_idx < changed_node_words.size(); ++word_idx) {
            auto word = changed_node_words[word_idx];
            while (word != 0) {
                const uint32_t node_id = word_idx * 64 + __builtin_ctzll(word);
                word &= word - 1;
                auto &node = nodes[node_id];
                if (node.id == NODE_NOID) {continue;}
                for (uint32_t slot_idx = 0; slot_idx < node.edge_num; ++slot_idx) {
                    const uint32_t other_id = node.edges[slot_idx];
                    const bool is_other_changed = (changed_node_words[other_id / 64] &
                        (uint64_t{1} << (other_id % 64))) != 0;
                    if (node_id < other_id || !is_other_changed) {
                        // 既存実装と同じ小IDから大IDへの差分計算。
                        edge_distance[edge_slots[node_id][slot_idx]] = node_id < other_id
                            ? node.pos.squaredNormXY(nodes[other_id].pos)
                            : nodes[other_id].pos.squaredNormXY(node.pos);
                    }
                }
            }
        }
        std::fill(changed_node_words.begin(), changed_node_words.end(), 0);
        return;
    }''')
assert text.endswith('}\n')
text = text[:-2] + '''    std::fill(changed_node_words.begin(), changed_node_words.end(), 0);
    has_untracked_edge_changes = false;
}
'''
path.write_text(text)

path = args.source / 'src/cpu/gng.cpp'
text = path.read_text()
options = ['true' if args.method in (name, 'combined') else 'false' for name in ('sync', 'orphan', 'edges')]
text = replace(text, '    attention();', '    n1.begin_update_frame(' + ', '.join(options) + ');\n    attention();')
text = replace(text, '    n1.calc_edge_distanceXY();', '    n1.calc_edge_distanceXY();\n    n1.end_update_frame();')
path.write_text(text)
print(args.method, 'prepared')
