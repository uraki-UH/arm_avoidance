"""クラスタ管理の作業配列・ID索引・移動演算によるコピー削減。"""
import argparse
import shutil
from pathlib import Path


def replace(path, before, after):
    source = path.read_text()
    assert source.count(before) == 1, (str(path), before)
    path.write_text(source.replace(before, after, 1))


def add_test(source):
    shutil.copy2(Path(__file__).with_name('cluster_bookkeeping_test.cpp'),
                 source / 'test/cluster_bookkeeping_test.cpp')
    replace(source / 'CMakeLists.txt', '  add_executable(node_id_reuse_test', '''  add_executable(cluster_bookkeeping_test test/cluster_bookkeeping_test.cpp
    src/cpu/clustering.cpp src/cpu/cugng.cpp src/utils/cluster.cpp
    src/utils/node.cpp src/utils/param.cpp src/utils/vec3f.cpp src/utils/utils.cpp)
  target_include_directories(cluster_bookkeeping_test PRIVATE src include)
  target_compile_definitions(cluster_bookkeeping_test PRIVATE GNG_VERSION=${GNG_VERSION})
  add_test(NAME cluster_bookkeeping_test COMMAND cluster_bookkeeping_test)
  add_executable(node_id_reuse_test''')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('--tests-only', dest='enable_tests_only', action='store_true')
    args = parser.parse_args()
    source = args.source
    if args.enable_tests_only:
        add_test(source)
        return

    path = source / 'src/cpu/clustering.hpp'
    replace(path, '    vector<Cluster> disable_clusters;', '''    vector<Cluster> disable_clusters;
    // 次フレームの構築用配列と、探索・引継ぎ用作業領域の再利用。
    vector<Cluster> new_clusters;
    vector<int> cluster_tmp;
    vector<pair<uint64_t, size_t>> prev_cluster_order;''')

    path = source / 'src/cpu/clustering.cpp'
    replace(path, '    vector<Cluster> new_clusters;  // 現在のノード情報で新規にクラスタリング', '''    new_clusters.clear();
    prev_cluster_order.clear();
    for (size_t idx = 0; idx < clusters.size(); ++idx) {
        prev_cluster_order.emplace_back(clusters[idx].id, idx);
    }
    // 重複ID時にも従来の線形探索と同じ先頭要素を選択する索引。
    sort(prev_cluster_order.begin(), prev_cluster_order.end());''')
    replace(path, '    vector<int> cluster_tmp;\n', '')
    replace(path, '''                // 新規ID
                if (cluster_id_map.find(cluster_id) == cluster_id_map.end())
                    cluster_id_map.emplace(cluster_id, 1);
                else
                    cluster_id_map.at(cluster_id)++;''', '''                // 単一の木探索による投票集計。同票時の昇順ID選択は維持。
                ++cluster_id_map.try_emplace(cluster_id, 0).first->second;''')
    replace(path, '''            // 見つかった場合
            for (auto &prev_cluster : clusters) {
                if (prev_cluster.id == cluster_id) {
                    _take_over_cluster(prev_cluster, cluster);
                    used_id.insert(cluster_id);
                    break;
                }
            }''', '''            // 旧クラスタのID索引による引継ぎ先の検索。
            const auto previous = lower_bound(prev_cluster_order.begin(), prev_cluster_order.end(),
                pair<uint64_t, size_t>{cluster_id, 0});
            if (previous != prev_cluster_order.end() && previous->first == cluster_id) {
                _take_over_cluster(clusters[previous->second], cluster);
                used_id.insert(cluster_id);
            }''')
    replace(path, '        // 新規クラスタへの処理\n', '''        // 使用済みROSIDの追加だけに対応した、最小空き候補の継続。
        uint32_t min_free_ros_id = 0;
        // 新規クラスタへの処理
''')
    replace(path, 'for (uint32_t ros_id = 0; ros_id < (UINT32_MAX - 1); ++ros_id)',
            'for (uint32_t ros_id = min_free_ros_id; ros_id < (UINT32_MAX - 1); ++ros_id)')
    replace(path, '''                    used_rosid.insert(ros_id);
                    break;''', '''                    used_rosid.insert(ros_id);
                    min_free_ros_id = ros_id + 1;
                    break;''')
    replace(path, '''    clusters.clear();
    clusters = new_clusters;''', '''    // 所属ノード配列の深いコピーを伴わない、構築結果と旧領域の交換。
    clusters.swap(new_clusters);''')
    text = path.read_text()
    assert text.count('\n                new_clusters.emplace_back(c);') == 2
    path.write_text(text.replace('\n                new_clusters.emplace_back(c);', '\n                new_clusters.emplace_back(std::move(c));'))

    path = source / 'src/utils/cluster.hpp'
    replace(path, '    Cluster(const Cluster& cluster);', '''    Cluster(const Cluster& cluster);
    Cluster& operator=(const Cluster&) = default;
    // 所属ノード配列の所有権移動による、ソート・再確保時のコピー削減。
    Cluster(Cluster&&) noexcept = default;
    Cluster& operator=(Cluster&&) noexcept = default;''')
    add_test(source)


if __name__ == '__main__':
    main()
