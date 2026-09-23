"""法線・曲率計算の連続配列参照への変更。"""
import argparse
import shutil
from pathlib import Path


def replace(path, before, after):
    text = path.read_text()
    assert before in text, str(path)
    path.write_text(text.replace(before, after, 1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--variant', choices=['packed', 'reuse'], default='reuse')
    args = parser.parse_args()
    source = args.source
    replace(source/'src/cpu/labelling.hpp', '    Time time;', '''    Time time;
    // 近傍参照用の連続配列。初期化時の確保とフレーム間再利用。
    vector<Vec3f> node_positions;
    vector<Vec3f> node_normals;''')
    replace(source/'src/cpu/labelling.cpp', '    gng = _gng;', '''    gng = _gng;
    node_positions.resize(_gng->nodes.size());
    node_normals.resize(_gng->nodes.size());''')
    replace(source/'src/cpu/labelling.cpp', '    float max_exp;', '''    float max_exp;
    // 座標だけを集約した、隣接ノード参照時のキャッシュ局所性改善。
    for (const auto &node : gng->nodes) {
        if (node.id != NODE_NOID) {node_positions[node.id] = node.pos;}
    }''')
    replace(source/'src/cpu/labelling.cpp', '        gng->normal_vector(node);', '''        gng->normal_vector(node, node_positions.data());
        node_normals[node.id] = node.normal;''')
    replace(source/'src/cpu/labelling.cpp', '            gng->rho(node);', '            gng->rho(node, node_normals.data());')
    replace(source/'src/cpu/cugng.hpp', '    void normal_vector(Node& node);', '    void normal_vector(Node& node, Vec3f *node_positions = nullptr);')
    replace(source/'src/cpu/cugng.hpp', '    void rho(Node& node);', '    void rho(Node& node, Vec3f *node_normals = nullptr);')
    replace(source/'src/cpu/cugng.cpp', 'void CUGNG::normal_vector(Node& node) {', '''void CUGNG::normal_vector(Node& node, Vec3f *node_positions) {
    // 連続座標配列がある場合の直接参照。単独呼出しでは従来のノード配列参照。
    const auto position = [&](uint32_t idx) -> Vec3f & {
        return node_positions ? node_positions[idx] : nodes[idx].pos;
    };''')
    path = source/'src/cpu/cugng.cpp'
    text = path.read_text()
    begin = text.index('void CUGNG::normal_vector(')
    end = text.index('\nvoid CUGNG::rho(', begin)
    body = text[begin:end]
    for before, after in [('nodes[node.edges[0]].pos', 'position(node.edges[0])'),
                          ('nodes[node.id].pos','position(node.id)'),
                          ('nodes[node.edges[1]].pos','position(node.edges[1])'),
                          ('nodes[node.edges[node.edge_num-1]].pos','position(node.edges[node.edge_num-1])'),
                          ('nodes[node.edges[i]].pos','position(node.edges[i])'),
                          ('nodes[node.edges[i+1]].pos','position(node.edges[i+1])')]:
        body = body.replace(before,after)
    path.write_text(text[:begin]+body+text[end:])
    replace(path, 'void CUGNG::rho(Node& node) {','void CUGNG::rho(Node& node, Vec3f *node_normals) {')
    replace(path, '        dot = node.normal.dot(nodes[node.edges[i]].normal);',
            '        dot = node.normal.dot(node_normals ? node_normals[node.edges[i]] : nodes[node.edges[i]].normal);')

    if args.variant == 'reuse':
        path = source/'src/cpu/cugng.cpp'
        text = path.read_text()
        begin = text.index('    int i;\n', text.index('void CUGNG::normal_vector'))
        end = text.index('\n}\n\nvoid CUGNG::rho(', begin)
        body = '''    if (node.edge_num <= 1) {
        node.normal.zero();
        return;
    }
    // 同じ差分ベクトルの再利用。外積・正規化・加算の順序は維持。
    const auto origin = position(node.id);
    auto previous = position(node.edges[0]) - origin;
    if (node.edge_num == 2) {
        node.normal = previous.cross(position(node.edges[1]) - origin).normalized();
        return;
    }
    const auto last = position(node.edges[node.edge_num - 1]) - origin;
    auto reference = previous.cross(last);
    auto normal_sum = reference;
    for (uint32_t idx = 1; idx < node.edge_num; ++idx) {
        auto next = position(node.edges[idx]) - origin;
        auto normal = previous.cross(next).normalized();
        if (reference.dot(normal) < 0) {normal_sum += normal.reverse();}
        else {normal_sum += normal;}
        previous = next;
    }
    node.normal = normal_sum.normalized();'''
        path.write_text(text[:begin] + body + text[end:])
        shutil.copy2(Path(__file__).with_name('normal_cache_test.cpp'), source/'test/normal_cache_test.cpp')
        replace(source/'CMakeLists.txt', '  add_executable(node_id_reuse_test', '''  add_executable(normal_cache_test test/normal_cache_test.cpp src/cpu/cugng.cpp src/cpu/labelling.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(normal_cache_test PRIVATE src include)
  target_compile_definitions(normal_cache_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME normal_cache_test COMMAND normal_cache_test)
  add_executable(node_id_reuse_test''')


if __name__ == '__main__':
    main()
