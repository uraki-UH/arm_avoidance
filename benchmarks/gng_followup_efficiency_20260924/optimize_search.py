"""近傍探索の連続配列参照への変更。比較用コピーだけへの適用。"""
import argparse
import shutil
from pathlib import Path


def replace(path, before, after):
    text = path.read_text()
    assert text.count(before) == 1, (str(path), before, text.count(before))
    path.write_text(text.replace(before, after, 1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    args = parser.parse_args()
    source = args.source
    header = source / 'src/cpu/cugng.hpp'
    cpp = source / 'src/cpu/cugng.cpp'
    replace(header, '   private:\n', '''   private:
    // 探索に必要な座標とラベルだけの連続配置。バッチ外の直接参照は維持。
    struct search_node_state {
        Vec3f pos;
        int label;
        int clusted_label;
    };
    vector<search_node_state> search_nodes;
    vector<uint32_t> point_order;
    bool is_search_batch = false;
    void begin_search_batch();
    template<bool enable_packed_search> bool get_min_grid_impl(Vec3f &point, Node_d &result);
    template<bool enable_packed_search> bool get_down_sampling_grid_impl(Vec3f &point, uint8_t &label, Node_d &result);
''')
    replace(cpp, '    nodes.resize(node_num_max);', '''    nodes.resize(node_num_max);
    search_nodes.resize(node_num_max);
    is_search_batch = false;''')
    replace(cpp, '    nodes.clear();', '''    nodes.clear();
    search_nodes.clear();
    point_order.clear();
    is_search_batch = false;''')
    replace(cpp, 'void CUGNG::getDownSampling(vector<Vec3f>', '''void CUGNG::begin_search_batch() {
    // 前フレームのラベル更新と、バッチ外のノード操作の反映。
    search_nodes.resize(nodes.size());
    for (const auto &node : nodes) {
        if (node.id != NODE_NOID) {
            search_nodes[node.id] = {node.pos, node.label, node.clusted_label};
        }
    }
    is_search_batch = true;
}

void CUGNG::getDownSampling(vector<Vec3f>''')
    replace(cpp, '    std::vector<uint32_t> point_order(input_pcl_num);', '''    begin_search_batch();
    point_order.resize(input_pcl_num);''')
    replace(cpp, '    voxel2node_ids_num = j;', '''    voxel2node_ids_num = j;
    is_search_batch = false;''')
    replace(cpp, '    uniform_int_distribution<> rA(0, input_pcl_num - 1);', '''    begin_search_batch();
    uniform_int_distribution<> rA(0, input_pcl_num - 1);''')
    replace(cpp, '''    }
}
void CUGNG::learn_normal''', '''    }
    is_search_batch = false;
}
void CUGNG::learn_normal''')
    replace(cpp, 'bool CUGNG::getMinGrid(Vec3f& p, Node_d& n){', '''bool CUGNG::getMinGrid(Vec3f &p, Node_d &n) {
    return is_search_batch ? get_min_grid_impl<true>(p, n) : get_min_grid_impl<false>(p, n);
}

template<bool enable_packed_search>
bool CUGNG::get_min_grid_impl(Vec3f& p, Node_d& n){''')
    replace(cpp, 'bool CUGNG::getDownSamplingGrid(Vec3f& p, uint8_t& label, Node_d &n){', '''bool CUGNG::getDownSamplingGrid(Vec3f &p, uint8_t &label, Node_d &n) {
    return is_search_batch ? get_down_sampling_grid_impl<true>(p, label, n)
                           : get_down_sampling_grid_impl<false>(p, label, n);
}

template<bool enable_packed_search>
bool CUGNG::get_down_sampling_grid_impl(Vec3f& p, uint8_t& label, Node_d &n){''')
    text = cpp.read_text()
    begin = text.index('template<bool enable_packed_search>\nbool CUGNG::get_min_grid_impl')
    end = text.index('\nvoid CUGNG::delete_node(', begin)
    body = text[begin:end]
    body = body.replace('                    auto &node = nodes[id];', '''                    const auto &node = [&]() -> const auto & {
                        if constexpr (enable_packed_search) {return search_nodes[id];}
                        else {return nodes[id];}
                    }();''')
    body = body.replace('                        node.age_s1 = 0;', '                        nodes[id].age_s1 = 0;')
    cpp.write_text(text[:begin] + body + text[end:])
    replace(cpp, '    node.pos.p[2] = new_pos.p[2];', '''    node.pos.p[2] = new_pos.p[2];
    if (is_search_batch) {search_nodes[node.id].pos = node.pos;}''')
    replace(cpp, '            node.frame = frame_number;', '''            node.frame = frame_number;
            if (is_search_batch) {
                search_nodes[i] = {node.pos, node.label, node.clusted_label};
            }''')
    shutil.copy2(Path(__file__).with_name('search_cache_test.cpp'), source / 'test/search_cache_test.cpp')
    replace(source / 'CMakeLists.txt', '  add_executable(node_id_reuse_test', '''  add_executable(search_cache_test test/search_cache_test.cpp src/cpu/cugng.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(search_cache_test PRIVATE src include)
  target_compile_definitions(search_cache_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME search_cache_test COMMAND search_cache_test)
  add_executable(node_id_reuse_test''')


if __name__ == '__main__':
    main()
