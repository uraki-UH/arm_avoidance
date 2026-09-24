"""隣接スロットと共有エッジプールによる、二乗サイズ配列の置換。"""
import argparse
import shutil
from pathlib import Path


def replace(path, before, after):
    source = path.read_text()
    assert source.count(before) == 1, (str(path), before)
    path.write_text(source.replace(before, after, 1))


def function(path, signature, next_signature, body):
    source = path.read_text()
    begin = source.index(signature)
    end = source.index(next_signature, begin)
    path.write_text(source[:begin] + body + '\n\n' + source[end:])


def add_test(source):
    # 疎保存適用前の実装との、同一操作列による差分照合用コピー。
    reference = source / 'test/edge_dense_reference'
    reference.mkdir(parents=True, exist_ok=False)
    for name in ('cugng.cpp', 'cugng.hpp'):
        text = (source / 'src/cpu' / name).read_text()
        for before, after in (('CUGNG', 'dense_cugng'), ('Node_d', 'dense_node_d'),
                              ('observation_attention_span', 'dense_attention_span'),
                              ('node_search_data', 'dense_node_search_data')):
            text = text.replace(before, after)
        text = text.replace('"../utils/', '"../../src/utils/')
        text = text.replace('"voxel_grid.hpp"', '"../../src/cpu/voxel_grid.hpp"')
        text = text.replace('"define.h"', '"../../src/cpu/define.h"')
        # 新規比較コピーに含まれる、旧実装の説明コメントの日本語化。
        for before, after in (('// Voxel Grid', '// 入力ボクセル設定'),
                              ('// copy', '// 設定値のコピー'), ('// clear', '// 既存状態の初期化'),
                              ('// malloc', '// 作業領域の確保'), ('// frame', '// フレーム更新')):
            text = text.replace(before, after)
        (reference / name).write_text(text)
    shutil.copy2(Path(__file__).with_name('sparse_edges_test.cpp'), source / 'test/sparse_edges_test.cpp')
    replace(source / 'CMakeLists.txt', '  add_executable(node_id_reuse_test', '''  add_executable(sparse_edges_test test/sparse_edges_test.cpp test/edge_dense_reference/cugng.cpp
    src/cpu/cugng.cpp src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(sparse_edges_test PRIVATE src include)
  target_compile_definitions(sparse_edges_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME sparse_edges_test COMMAND sparse_edges_test)
  add_executable(node_id_reuse_test''')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    source = args.source
    add_test(source)

    path = source / 'src/cpu/cugng.hpp'
    replace(path, '    vector<uint8_t> edge_count;', '''    vector<uint8_t> edge_count;
    // 隣接配列と同順の共有エッジID。ID 0は未接続の読み出し専用領域。
    vector<array<uint32_t, NODE_MAX_EDGE>> edge_slots;''')
    replace(path, '    vector<float> edge_distance; // エッジの距離',
            '    vector<float> edge_distance; // 共有エッジIDごとのXY距離の二乗')
    replace(path, '    /* エッジIDを検索 */', '    /* 接続中の共有エッジIDの検索。未接続時は予約ID 0。 */')
    replace(path, '   private:\n', '''   private:
    // 寿命のuint8巻戻りによる重複隣接も含めた、プールIDの参照数。
    vector<uint8_t> edge_reference_num;
    vector<uint32_t> free_edge_ids;
    void release_edge_slot(uint32_t edge_idx) {
        if (--edge_reference_num[edge_idx] == 0) {
            free_edge_ids.push_back(edge_idx);
        }
    }
''')

    path = source / 'src/cpu/cugng.cpp'
    replace(path, '''    edge_count.resize(node_num_max * node_num_max);
    memset(edge_count.data(), 0, sizeof(uint8_t) * node_num_max * node_num_max);
    edge_distance.resize(node_num_max * node_num_max);''', '''    // ノード上限と次数上限から決まる、実在エッジ用領域の最大容量。
    const size_t max_edge_num = static_cast<size_t>(node_num_max) * NODE_MAX_EDGE / 2 + 1;
    edge_count.assign(1, EDGE_NO_CONNECT);
    edge_distance.assign(1, 0.f);
    edge_reference_num.assign(1, 0);
    free_edge_ids.clear();
    edge_count.reserve(max_edge_num);
    edge_distance.reserve(max_edge_num);
    edge_reference_num.reserve(max_edge_num);
    free_edge_ids.reserve(max_edge_num);
    edge_slots.resize(node_num_max);''')
    replace(path, '''    tn_id.clear();
    edge_count.clear();''', '''    tn_id.clear();
    edge_count.clear();
    edge_distance.clear();
    edge_slots.clear();
    edge_reference_num.clear();
    free_edge_ids.clear();''')
    replace(path, '        uint32_t edge_index = getEdgeIndex(node0.id, node0.edges[i]);',
            '        const uint32_t edge_idx = edge_slots[node0.id][i];')
    replace(path, '        edge_count[edge_index]++;', '        edge_count[edge_idx]++;')
    replace(path, '        if (edge_count[edge_index] > edge_config->age_max){',
            '        if (edge_count[edge_idx] > edge_config->age_max){')
    function(path, 'void CUGNG::disconnect(uint32_t idx1, uint32_t idx2) {',
             'void CUGNG::disconnect_all(uint32_t idx) {', '''void CUGNG::disconnect(uint32_t idx1, uint32_t idx2) {
    if (idx1 == idx2) {return;}
    auto &first = nodes[idx1];
    auto &second = nodes[idx2];
    const uint32_t edge_idx = getEdgeIndex(idx1, idx2);
    const bool has_edge = edge_count[edge_idx] != EDGE_NO_CONNECT;
    // 従来と同じ末尾交換による削除順序と、共有IDの対応維持。
    for (uint32_t idx = 0; idx < first.edge_num; ++idx) {
        if (first.edges[idx] == idx2) {
            release_edge_slot(edge_slots[idx1][idx]);
            first.edges[idx] = first.edges[--first.edge_num];
            edge_slots[idx1][idx] = edge_slots[idx1][first.edge_num];
            break;
        }
    }
    for (uint32_t idx = 0; idx < second.edge_num; ++idx) {
        if (second.edges[idx] == idx1) {
            release_edge_slot(edge_slots[idx2][idx]);
            second.edges[idx] = second.edges[--second.edge_num];
            edge_slots[idx2][idx] = edge_slots[idx2][second.edge_num];
            break;
        }
    }
    edge_count[edge_idx] = EDGE_NO_CONNECT;
    if (has_edge) {recordEdgeDelta(first, second, GNG_DELTA_REMOVE);}
}''')
    function(path, 'void CUGNG::disconnect_all(uint32_t idx) {',
             'void CUGNG::connect(uint32_t idx1, uint32_t idx2) {', '''void CUGNG::disconnect_all(uint32_t idx) {
    auto &first = nodes[idx];
    // 対象ノードの元の隣接順による削除イベントと、相手側の末尾交換。
    for (uint32_t slot_idx = 0; slot_idx < first.edge_num; ++slot_idx) {
        const uint32_t second_idx = first.edges[slot_idx];
        auto &second = nodes[second_idx];
        const uint32_t edge_idx = edge_slots[idx][slot_idx];
        for (uint32_t other_slot_idx = 0; other_slot_idx < second.edge_num; ++other_slot_idx) {
            if (second.edges[other_slot_idx] == idx) {
                recordEdgeDelta(first, second, GNG_DELTA_REMOVE);
                release_edge_slot(edge_slots[second_idx][other_slot_idx]);
                second.edges[other_slot_idx] = second.edges[--second.edge_num];
                edge_slots[second_idx][other_slot_idx] = edge_slots[second_idx][second.edge_num];
                edge_count[edge_idx] = EDGE_NO_CONNECT;
                break;
            }
        }
        release_edge_slot(edge_idx);
    }
    first.edge_num = 0;
}''')
    function(path, 'void CUGNG::connect(uint32_t idx1, uint32_t idx2) {',
             'void CUGNG::check_delete_no_edge_and_decay_eta() {', '''void CUGNG::connect(uint32_t idx1, uint32_t idx2) {
    if (idx1 == idx2) {return;}
    uint32_t edge_idx = getEdgeIndex(idx1, idx2);
    if (edge_count[edge_idx] != EDGE_NO_CONNECT) {
        edge_count[edge_idx] = EDGE_CONNECT;
        return;
    }
    auto &first = nodes[idx1];
    auto &second = nodes[idx2];
    if (first.edge_num == NODE_MAX_EDGE || second.edge_num == NODE_MAX_EDGE) {return;}
    if (edge_idx == 0) {
        if (!free_edge_ids.empty()) {
            edge_idx = free_edge_ids.back();
            free_edge_ids.pop_back();
            edge_distance[edge_idx] = 0.f;
        } else {
            edge_idx = edge_count.size();
            edge_count.push_back(EDGE_NO_CONNECT);
            edge_distance.push_back(0.f);
            edge_reference_num.push_back(0);
        }
    }
    edge_count[edge_idx] = EDGE_CONNECT;
    edge_reference_num[edge_idx] += 2;
    edge_slots[idx1][first.edge_num] = edge_idx;
    edge_slots[idx2][second.edge_num] = edge_idx;
    first.edges[first.edge_num++] = idx2;
    second.edges[second.edge_num++] = idx1;
    recordEdgeDelta(first, second, GNG_DELTA_ADD);
}''')
    function(path, 'uint32_t CUGNG::getEdgeIndex(uint32_t idx1, uint32_t idx2){',
             'void CUGNG::normal_vector(', '''uint32_t CUGNG::getEdgeIndex(uint32_t idx1, uint32_t idx2) {
    const auto &first = nodes[idx1];
    for (uint32_t slot_idx = 0; slot_idx < first.edge_num; ++slot_idx) {
        if (first.edges[slot_idx] == idx2) {return edge_slots[idx1][slot_idx];}
    }
    return 0;
}''')
    replace(path, 'edge_distance[getEdgeIndex(node.id, edge_id)] =',
            'edge_distance[edge_slots[node.id][i]] =')
    path = source / 'src/cpu/clustering.cpp'
    replace(path, '        uint32_t edge_index = gng->getEdgeIndex(idx, edge_id);',
            '        const uint32_t edge_idx = gng->edge_slots[idx][i];')
    replace(path, 'gng->edge_distance[edge_index]', 'gng->edge_distance[edge_idx]')


if __name__ == '__main__':
    main()
