// 平面照合方式と接触セル抽出の同一グラフ上での比較。
#include "cpu/gng.hpp"
#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "ais_gng/topological_plane/plane_cluster_parameters.hpp"
#include "reference.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <array>
#include <unordered_map>
#include <unordered_set>

extern "C" GNG gng;
#ifdef GNG_ENABLE_CHURN_DIAGNOSTICS
// ノード同一性ではなくセル再訪の計測。再訪は移動体の往復も含み、誤削除の断定ではない。
struct churn_diagnostics {
    bool enable_capture = false;
    bool enable_support_reset = false;
    uint32_t frame = 0;
    std::unordered_map<uint32_t, uint32_t> removed_cells;
    std::unordered_set<uint32_t> matched_ids, covered_ids;
    std::array<uint32_t, 8> counts{};
    const std::array<const char *, 8> names{
        "all_added", "all_removed", "readded_same_frame", "readded_recent",
        "age_removed", "age_removed_occupied", "age_removed_matched", "age_removed_covered"};
    void begin(uint32_t input_frame) {
        frame = input_frame; enable_capture = true;
        counts.fill(0); matched_ids.clear(); covered_ids.clear();
        for (auto it = removed_cells.begin(); it != removed_cells.end();) {
            it = frame - it->second > 3U ? removed_cells.erase(it) : std::next(it);
        }
    }
} churn;

bool gng_churn_support_reset_enabled() {return churn.enable_support_reset;}

// イベント番号は0:追加、1:削除、2:通常寿命削除、3:入力最近傍照合。
void gng_churn_event(uint32_t event, uint32_t id, const Vec3f &point) {
    if (!churn.enable_capture) {return;}
    if (event == 3) {
        churn.matched_ids.insert(id);
        const auto &node = gng.n1.nodes[id];
        auto input = point;
        if (input.squaredNorm(node.pos) < gng.n1.gng_config.vigilance2[node.label]) {
            churn.covered_ids.insert(id);
        }
        return;
    }
    auto position = point;
    const auto key = gng.n1.voxel_config.getIndex(position);
    if (event == 0) {
        ++churn.counts[0];
        churn.matched_ids.erase(id); churn.covered_ids.erase(id);
        const auto found = churn.removed_cells.find(key);
        if (key < gng.n1.voxel_config.maxXYZ && found != churn.removed_cells.end()) {
            ++churn.counts[3];
            if (found->second == churn.frame) {++churn.counts[2];}
        }
    } else if (event == 1) {
        ++churn.counts[1];
        if (key < gng.n1.voxel_config.maxXYZ) {churn.removed_cells[key] = churn.frame;}
    } else if (event == 2) {
        ++churn.counts[4];
        if (key < gng.n1.voxel_config.maxXYZ && gng.vg.has_occupied_cell(key)) {++churn.counts[5];}
        if (churn.matched_ids.count(id)) {++churn.counts[6];}
        if (churn.covered_ids.count(id)) {++churn.counts[7];}
    }
}
#endif
// 最適化前の接触抽出。結果と処理時間の同一フレーム比較用。
uint32_t reference_plane_contact_voxels(const gng_sampling_node_ref *nodes, uint32_t num_nodes,
    const gng_plane_contact_voxel **voxels, float *cell_size) {
    if (voxels) {*voxels = nullptr;}
    if (cell_size) {*cell_size = 0;}
    if (!voxels || !cell_size || !gng.initialized || !gng.has_voxelized_input ||
        !gng.vg.enable_voxel_downsampling || !nodes || num_nodes == 0 || num_nodes > gng.n1.nodes.size()) {return 0;}
    auto &grid = *gng.vg.voxel_config;
    *cell_size = grid.unit;
    // 可視化を要求された時だけの索引。既存入力セル番号と配列容量の再利用。
    static std::vector<uint32_t> keys;
    static std::vector<gng_plane_contact_voxel> result;
    size_t capacity = 16;
    while (capacity < size_t(num_nodes) * 2) {capacity *= 2;}
    keys.resize(capacity); std::fill(keys.begin(), keys.end(), UINT32_MAX);
    result.clear(); result.reserve(gng.vg.filtered_pcl_num);
    const auto find_slot = [&](uint32_t key) {
        size_t slot = (uint64_t(key) * 11400714819323198485ULL >> 32) & (capacity-1);
        while (keys[slot] != UINT32_MAX && keys[slot] != key) {slot = (slot+1) & (capacity-1);}
        return slot;
    };
    for (uint32_t idx = 0; idx < num_nodes; ++idx) {
        const auto &ref = nodes[idx];
        if (ref.id >= gng.n1.nodes.size()) {continue;}
        auto &node = gng.n1.nodes[ref.id];
        if (node.id == NODE_NOID || node.frame != ref.frame) {continue;}
        const auto key = grid.getIndex(node.pos);
        if (key < grid.maxXYZ) {keys[find_slot(key)] = key;}
    }
    const auto has_key = [&](uint32_t key) {return keys[find_slot(key)] != UINT32_MAX;};
    for (uint32_t idx = 0; idx < gng.vg.filtered_pcl_num; ++idx) {
        const auto &range = gng.vg.voxel_range[idx];
        const auto key = gng.vg.voxel_index[range.start].voxel_index;
        if (key >= grid.maxXYZ) {continue;}
        const auto x = key % grid.max[0], y = key / grid.max[0] % grid.max[1], z = key / grid.maxXY;
        const uint8_t contact = has_key(key) ? 1 : (
            (x > 0 && has_key(key-1)) || (x+1 < grid.max[0] && has_key(key+1)) ||
            (y > 0 && has_key(key-grid.max[0])) || (y+1 < grid.max[1] && has_key(key+grid.max[0])) ||
            (z > 0 && has_key(key-grid.maxXY)) || (z+1 < grid.max[2] && has_key(key+grid.maxXY)) ? 2 : 0);
        if (contact) {
            result.push_back({{grid.x_min+(x+.5f)*grid.unit, grid.y_min+(y+.5f)*grid.unit,
                grid.z_min+(z+.5f)*grid.unit}, range.end-range.start, contact});
        }
    }
    *voxels = result.data();
    return result.size();
}
using clock_type = std::chrono::steady_clock;
namespace plane_core = fuzzrobo::topological_plane::incremental;
void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}
double elapsed_ms(clock_type::time_point start) {
    return std::chrono::duration<double, std::milli>(clock_type::now() - start).count();
}
struct parameters {
    std::map<std::string, double> values;
    template<class T> T declare_parameter(const std::string &name, T fallback) {
        const auto found = values.find(name);
        return found == values.end() ? fallback : static_cast<T>(found->second);
    }
};
bool matches_plane(const CUGNG &core, uint32_t idx, const Vec3f &point) {
    const auto &plane = core.insertion_planes[idx];
    double height = 0, u = 0, v = 0;
    for (uint32_t dim = 0; dim < 3; ++dim) {
        const double delta = point.p[dim] - plane.center[dim];
        height += delta * plane.normal[dim];
        u += delta * plane.tangent_u[dim]; v += delta * plane.tangent_v[dim];
    }
    const auto &config = core.insertion_config;
    return std::abs(height) <= config.max_plane_dist_th &&
        u >= plane.min_u - config.plane_margin && u <= plane.max_u + config.plane_margin &&
        v >= plane.min_v - config.plane_margin && v <= plane.max_v + config.plane_margin;
}
bool has_owner(const CUGNG &core, const gng_insertion_owner &owner) {
    return owner.id < core.nodes.size() && core.nodes[owner.id].id != NODE_NOID &&
        core.nodes[owner.id].frame == owner.frame && owner.plane_idx < core.insertion_planes.size();
}
// 既存セル番号をキーとする平坦なハッシュ索引。点群・ボクセル本体のコピーなし。
// 同一セル内の所属列は連続領域の単方向リスト、平面の重複照合は世代印で除外。
struct cell_planes {
    struct entry {uint32_t plane_idx, next;};
    std::vector<uint32_t> keys, heads, seen;
    std::vector<entry> entries;
    uint32_t epoch = 0;
    size_t find(uint32_t key) const {
        size_t slot = (uint64_t(key) * 11400714819323198485ULL >> 32) & (keys.size()-1);
        while (keys[slot] != UINT32_MAX && keys[slot] != key) {slot = (slot+1) & (keys.size()-1);}
        return slot;
    }
    void build(const CUGNG &core, GridConfig &grid) {
        size_t capacity = 16;
        while (capacity < core.insertion_owners.size()*2) {capacity *= 2;}
        keys.resize(capacity); heads.resize(capacity);
        std::fill(keys.begin(), keys.end(), UINT32_MAX);
        entries.clear(); entries.reserve(core.insertion_owners.size());
        seen.assign(core.insertion_planes.size(), 0); epoch = 0;
        for (const auto &owner : core.insertion_owners) {
            if (!has_owner(core, owner)) {continue;}
            auto point = core.nodes[owner.id].pos;
            const auto key = grid.getIndex(point);
            if (key >= grid.maxXYZ) {continue;}
            const auto slot = find(key);
            if (keys[slot] == UINT32_MAX) {keys[slot] = key; heads[slot] = UINT32_MAX;}
            entries.push_back({owner.plane_idx, heads[slot]});
            heads[slot] = entries.size()-1;
        }
    }
    bool lookup(const CUGNG &core, GridConfig &grid, Vec3f point) {
        const uint32_t key = grid.getIndex(point);
        if (key >= grid.maxXYZ) {return false;}
        if (++epoch == 0) {std::fill(seen.begin(), seen.end(), 0); ++epoch;}
        uint32_t candidates[7]{key}; uint32_t num_candidates = 1;
        const auto x = key % grid.max[0], y = key / grid.max[0] % grid.max[1], z = key / grid.maxXY;
        if (x > 0) {candidates[num_candidates++] = key-1;}
        if (x+1 < grid.max[0]) {candidates[num_candidates++] = key+1;}
        if (y > 0) {candidates[num_candidates++] = key-grid.max[0];}
        if (y+1 < grid.max[1]) {candidates[num_candidates++] = key+grid.max[0];}
        if (z > 0) {candidates[num_candidates++] = key-grid.maxXY;}
        if (z+1 < grid.max[2]) {candidates[num_candidates++] = key+grid.maxXY;}
        for (uint32_t idx = 0; idx < num_candidates; ++idx) {
            const auto slot = find(candidates[idx]);
            if (keys[slot] == UINT32_MAX) {continue;}
            for (auto pos = heads[slot]; pos != UINT32_MAX; pos = entries[pos].next) {
                const auto plane_idx = entries[pos].plane_idx;
                if (seen[plane_idx] == epoch) {continue;}
                seen[plane_idx] = epoch;
                if (matches_plane(core, plane_idx, point)) {return true;}
            }
        }
        return false;
    }
    size_t bytes() const {
        return (keys.capacity()+heads.capacity()+seen.capacity()) * sizeof(uint32_t) +
            entries.capacity() * sizeof(entry);
    }
};
// 索引に依存しない6隣接の全所有ノード走査。速度測定外の検証用。
bool oracle(const CUGNG &core, GridConfig &grid, Vec3f point) {
    const auto key = grid.getIndex(point);
    if (key >= grid.maxXYZ) {return false;}
    const auto coord = [&](uint32_t k) {
        return std::array<int, 3>{int(k%grid.max[0]), int(k/grid.max[0]%grid.max[1]), int(k/grid.maxXY)};
    };
    const auto target = coord(key);
    for (const auto &owner : core.insertion_owners) {
        if (!has_owner(core, owner)) {continue;}
        auto pos = core.nodes[owner.id].pos;
        const auto other_key = grid.getIndex(pos);
        if (other_key >= grid.maxXYZ) {continue;}
        const auto other = coord(other_key);
        if (std::abs(target[0]-other[0])+std::abs(target[1]-other[1])+std::abs(target[2]-other[2]) <= 1 &&
            matches_plane(core, owner.plane_idx, point)) {return true;}
    }
    return false;
}
void tests() {
    CUGNG core;
    core.nodes.resize(4); core.insertion_owners.resize(4);
    for (uint32_t idx = 0; idx < 4; ++idx) {
        core.nodes[idx].id = idx; core.nodes[idx].frame = 5;
        core.nodes[idx].pos = Vec3f(.1f, .1f, .01f);
        core.insertion_owners[idx] = {idx, 5, 0};
    }
    gng_insertion_plane plane;
    plane.normal[2] = plane.tangent_u[0] = plane.tangent_v[1] = 1;
    plane.min_u = plane.min_v = -4; plane.max_u = plane.max_v = 4;
    core.insertion_planes = {plane};
    GridConfig config{}; config.unit = .5; config.x_min = config.y_min = config.z_min = -2;
    config.x_max = config.y_max = config.z_max = 2;
    GridConfig grid; require(grid.init(config), "検証グリッド初期化");
    cell_planes index; index.build(core, grid);
    require(index.lookup(core, grid, {.6f,.1f,.02f}), "面共有隣接の候補");
    require(index.lookup(core, grid, {-.1f,.1f,.02f}), "負座標側の隣接");
    require(!index.lookup(core, grid, {.6f,.6f,.02f}), "斜め隣接の非対象");
    require(!index.lookup(core, grid, {.1f,.1f,.2f}), "同一セルでも面外の点の除外");
    require(!index.lookup(core, grid, {1.9f,-.1f,.02f}), "行端の誤った折返しの防止");
    require(!index.lookup(core, grid, {4.f,.1f,.02f}), "範囲外の除外");
    core.insertion_planes[0].max_u = .2;
    require(!index.lookup(core, grid, {.6f,.1f,.02f}), "有限範囲の制約");
    core.insertion_planes[0].max_u = 4;
    for (auto &owner : core.insertion_owners) {owner.frame = 4;}
    index.build(core, grid);
    require(!index.lookup(core, grid, {.1f,.1f,.02f}), "再利用IDへの旧所属の除外");
    core.insertion_owners.clear(); index.build(core, grid);
    require(!index.lookup(core, grid, {.1f,.1f,.02f}), "空の対応表");
    core.insertion_owners = {{0, 5, 0}, {1, 5, 1}};
    plane.center[2] = .2; core.insertion_planes.push_back(plane);
    index.build(core, grid);
    require(index.lookup(core, grid, {.1f,.1f,.2f}), "同一セルの複数候補平面");
    std::cout << "synthetic_tests=10 passed\n";
}
void update_planes(const ais_gng_msgs::msg::PlaneClusterArray &planes, const TopologicalMap &map) {
    auto &core = gng.n1;
    core.insertion_planes.clear();
    core.insertion_owners.assign(core.nodes.size(), {UINT32_MAX, UINT32_MAX, UINT32_MAX});
    for (const auto &cluster : planes.clusters) {
        if (cluster.node_indices.empty()) {continue;}
        gng_insertion_plane plane;
        const double center[] = {cluster.centroid.x, cluster.centroid.y, cluster.centroid.z};
        const double normal[] = {cluster.normal.x, cluster.normal.y, cluster.normal.z};
        const double u[] = {cluster.tangent_u.x, cluster.tangent_u.y, cluster.tangent_u.z};
        const double v[] = {cluster.tangent_v.x, cluster.tangent_v.y, cluster.tangent_v.z};
        for (uint32_t dim = 0; dim < 3; ++dim) {
            plane.center[dim] = center[dim]; plane.normal[dim] = normal[dim];
            plane.tangent_u[dim] = u[dim]; plane.tangent_v[dim] = v[dim];
        }
        plane.min_u = plane.min_v = INFINITY; plane.max_u = plane.max_v = -INFINITY;
        for (const auto idx : cluster.node_indices) {
            require(idx < map.node_num, "平面の所属添字");
            const auto &node = map.nodes[idx];
            const double delta[] = {node.pos.x-center[0], node.pos.y-center[1], node.pos.z-center[2]};
            double pos_u = 0, pos_v = 0;
            for (uint32_t dim = 0; dim < 3; ++dim) {pos_u += delta[dim]*u[dim]; pos_v += delta[dim]*v[dim];}
            plane.min_u = std::min(plane.min_u, pos_u); plane.max_u = std::max(plane.max_u, pos_u);
            plane.min_v = std::min(plane.min_v, pos_v); plane.max_v = std::max(plane.max_v, pos_v);
            core.insertion_owners[node.id] = {node.id, node.frame, uint32_t(core.insertion_planes.size())};
        }
        core.insertion_planes.push_back(plane);
    }
}
struct query {Vec3f point; Node_d nearest;};
struct totals {
    std::map<std::string, std::vector<double>> data;
    void add(const std::string &key, double value) {data[key].push_back(value);}
    void write(const std::string &path) {
        std::ofstream file(path); file << std::setprecision(10) << "{";
        bool is_first = true;
        for (auto &[name, values] : data) {
            std::sort(values.begin(), values.end());
            if (!is_first) {file << ",";} is_first = false;
            file << "\n\"" << name << "\":" << std::accumulate(values.begin(), values.end(), 0.)/values.size();
            if (name.ends_with("_ms")) {
                file << ",\n\"" << name << "_p95\":" << values[size_t((values.size()-1)*.95)];
            }
        }
        file << "\n}\n";
    }
};
int main(int argc, char **argv) {
    try {
        require(argc == 4 || argc == 5, "入力ディレクトリ・種・出力ディレクトリ・省略可能な比較方式");
        const bool enable_occupancy_test = argc == 5;
        const bool enable_occupancy = enable_occupancy_test &&
            (std::string(argv[4]) == "occupancy" || std::string(argv[4]) == "occupancy_supported");
#ifdef GNG_ENABLE_CHURN_DIAGNOSTICS
        churn.enable_support_reset = enable_occupancy_test && std::string(argv[4]) == "occupancy_supported";
#endif
        tests();
        const std::string source = argv[1], output = argv[3];
        const auto seed = uint32_t(std::stoul(argv[2]));
        std::ifstream params(source+"/parameters.txt"); std::string name; uint32_t idx; float value;
        while (params >> name >> idx >> value) {gng_setParameter(name.c_str(), idx, value);}
        require(gng_init() == SUCCESS, "GNG初期化");
        parameters plane_params;
        std::ifstream plane_file(source+"/plane_parameters.txt"); double plane_value;
        while (plane_file >> name >> plane_value) {plane_params.values[name] = plane_value;}
        auto options = plane_core::declareClusterOptions(plane_params);
        plane_core::Clusterizer clusters(options);
        LiDAR_Config lidar; lidar.point_step = sizeof(Vec3);
        std::ifstream input(source+"/points.bin", std::ios::binary);
        std::ofstream detail(output+"/frames.csv");
        detail << "frame,cell_size,nodes,planes,queries,node_ms,build_ms,cell_query_ms,node_only,cell_only,both,neither,bytes\n";
        totals results; uint32_t frame = 0; uint64_t checksum = 0;
        std::ofstream occupancy_detail;
#ifdef GNG_ENABLE_CHURN_DIAGNOSTICS
        std::ofstream churn_detail(output+"/churn_frames.csv");
        churn_detail << "frame";
        for (const auto name : churn.names) {churn_detail << ',' << name;}
        churn_detail << '\n';
#endif
        if (enable_occupancy_test) {
            occupancy_detail.open(output+"/occupancy_frames.csv");
            occupancy_detail << "frame,gng_ms,nodes,added,aged,removed,plane_rejected\n";
        }
        cell_planes index;
        while (input.peek() != EOF) {
            uint32_t num; input.read(reinterpret_cast<char *>(&num), sizeof(num));
            require(num <= 200000, "入力上限");
            std::vector<Vec3> points(num);
            input.read(reinterpret_cast<char *>(points.data()), num*sizeof(Vec3)); require(bool(input), "点群データ終端");
            gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), num, &lidar);
            if (frame > 0 && !enable_occupancy_test) {
                // 同じ前フレームのグラフ・平面と現入力。判定中のグラフ変更なし。
                for (const float size : {.5f, 1.f}) {
                    GridConfig config = gng.n1.voxel_config; config.unit = size;
                    GridConfig grid; require(grid.init(config), "比較用セル初期化");
                    VoxelGrid voxels; voxels.init(&grid, &gng.param.config);
                    std::vector<uint8_t> labels(num);
                    voxels.applyFilter(gng.map.input_pcl, gng.map.input_pcl_num, labels);
                    std::vector<uint32_t> order(voxels.filtered_pcl_num);
                    std::iota(order.begin(), order.end(), 0U);
                    std::mt19937 random(seed+frame); std::shuffle(order.begin(), order.end(), random);
                    std::vector<query> queries;
                    for (const auto cell : order) {
                        const auto range = voxels.voxel_range[cell];
                        if (range.end-range.start < 3) {continue;}
                        Node_d nearest;
                        gng.n1.getMinGrid(voxels.filtered_pcl[cell], nearest);
                        if (nearest.id1 != NODE_NOID && nearest.id1_d2 <= .25*.25) {continue;}
                        auto point = gng.map.input_pcl[voxels.voxel_index[range.start].raw_index];
                        gng.n1.getMinGrid(point, nearest);
                        if (nearest.id1 != NODE_NOID && nearest.id1_d2 <= .25*.25) {continue;}
                        queries.push_back({point, nearest});
                    }
                    // 全候補と128照合を比較。32追加での打切りはなく、照合量を多めに評価。
                    for (const bool enable_all : {false, true}) {
                        const size_t num_queries = enable_all ? queries.size() : std::min<size_t>(128, queries.size());
                        double node_ms = 0, build_ms = 0, query_ms = 0;
                        std::vector<uint8_t> node_result(num_queries), cell_result(num_queries);
                        constexpr uint32_t num_repeats = 6;
                        for (uint32_t repeat = 0; repeat < num_repeats; ++repeat) {
                            const auto run_node = [&] {
                                asm volatile("" ::: "memory");
                                const auto start = clock_type::now();
                                for (size_t q = 0; q < num_queries; ++q) {
                                    node_result[q] = node_lookup(gng.n1, queries[q].point, queries[q].nearest);
                                }
                                node_ms += elapsed_ms(start);
                            };
                            const auto run_cell = [&] {
                                asm volatile("" ::: "memory");
                                auto start = clock_type::now();
                                index.build(gng.n1, grid); build_ms += elapsed_ms(start);
                                start = clock_type::now();
                                for (size_t q = 0; q < num_queries; ++q) {
                                    cell_result[q] = index.lookup(gng.n1, grid, queries[q].point);
                                }
                                query_ms += elapsed_ms(start);
                            };
                            if ((repeat+frame+seed)%2) {run_node(); run_cell();}
                            else {run_cell(); run_node();}
                            checksum += std::accumulate(node_result.begin(), node_result.end(), uint64_t(0)) +
                                std::accumulate(cell_result.begin(), cell_result.end(), uint64_t(0));
                        }
                        node_ms /= num_repeats; build_ms /= num_repeats; query_ms /= num_repeats;
                        uint32_t node_only = 0, cell_only = 0, both = 0, neither = 0;
                        for (size_t q = 0; q < num_queries; ++q) {
                            if (node_result[q] && cell_result[q]) {++both;}
                            else if (node_result[q]) {++node_only;}
                            else if (cell_result[q]) {++cell_only;}
                            else {++neither;}
                            if (q < 16) {
                                require(bool(cell_result[q]) == oracle(gng.n1, grid, queries[q].point), "実bagの索引と全走査の一致");
                            }
                        }
                        if (frame >= 20) {
                            const std::string prefix = std::string(size == .5f ? "s05_" : "s10_") + (enable_all ? "all_" : "128_");
                            results.add(prefix+"node_ms", node_ms);
                            results.add(prefix+"build_ms", build_ms);
                            results.add(prefix+"query_ms", query_ms);
                            results.add(prefix+"total_ms", build_ms+query_ms);
                            results.add(prefix+"delta_ms", build_ms+query_ms-node_ms);
                            results.add(prefix+"queries", num_queries);
                            results.add(prefix+"node_only", node_only); results.add(prefix+"cell_only", cell_only);
                            results.add(prefix+"both", both); results.add(prefix+"neither", neither);
                            results.add(prefix+"bytes", index.bytes());
                        }
                        detail << frame << ',' << size << ',' << gng.n1.node_num << ',' << gng.n1.insertion_planes.size() << ','
                            << num_queries << ',' << node_ms << ',' << build_ms << ',' << query_ms << ','
                            << node_only << ',' << cell_only << ',' << both << ',' << neither << ',' << index.bytes() << '\n';
                    }
                }
            }
            // 照合比較は共通グラフ、占有更新比較は各方式の継続学習。
            gng.n1.enable_node_insertion = enable_occupancy && frame > 0;
#ifdef GNG_ENABLE_CHURN_DIAGNOSTICS
            churn.begin(frame);
#endif
            const auto start = clock_type::now(); gng_exec();
            const double gng_ms = elapsed_ms(start);
#ifdef GNG_ENABLE_CHURN_DIAGNOSTICS
            churn.enable_capture = false;
            require(churn.counts[2] <= churn.counts[3] && churn.counts[3] <= churn.counts[0], "再追加数の包含関係");
            require(churn.counts[5] <= churn.counts[4] && churn.counts[7] <= churn.counts[6] &&
                churn.counts[6] <= churn.counts[4] && churn.counts[4] <= churn.counts[1], "寿命削除数の包含関係");
            churn_detail << frame;
            for (std::size_t idx = 0; idx < churn.counts.size(); ++idx) {
                churn_detail << ',' << churn.counts[idx];
                if (frame >= 20) {results.add(churn.names[idx], churn.counts[idx]);}
            }
            churn_detail << '\n';
#endif
            auto map = gng_getTopologicalMap();
            const auto plane_start = clock_type::now();
            const std_msgs::msg::Header header;
            const auto result = clusters.update(plane_core::make_graph_view(map.nodes, map.node_num, map.edges, map.edge_num), header, frame);
            const double plane_ms = elapsed_ms(plane_start);
            const auto model_start = clock_type::now(); update_planes(result.clusters, map);
            const double model_ms = elapsed_ms(model_start);
            if (enable_occupancy_test) {
                const auto stats = gng_get_node_insertion_stats();
                require(map.node_num <= 20000, "総ノード上限の維持");
                occupancy_detail << frame << ',' << gng_ms << ',' << map.node_num << ',' << stats.num_added_nodes << ','
                    << stats.num_aged_nodes << ',' << stats.num_removed_nodes << ',' << stats.num_plane_rejected_cells << '\n';
                if (frame >= 20) {
                    results.add("gng_ms", gng_ms); results.add("nodes", map.node_num);
                    results.add("added", stats.num_added_nodes); results.add("aged", stats.num_aged_nodes);
                    results.add("removed", stats.num_removed_nodes); results.add("plane_ms", plane_ms);
                    results.add("plane_rejected", stats.num_plane_rejected_cells);
                }
                ++frame;
                continue;
            }
            const auto contact_start = clock_type::now();
            std::vector<gng_sampling_node_ref> contact_nodes;
            for (const auto &plane : result.clusters.clusters) {
                for (const auto node_idx : plane.node_indices) {
                    contact_nodes.push_back({map.nodes[node_idx].id, map.nodes[node_idx].frame});
                }
            }
            const gng_plane_contact_voxel *contact_voxels = nullptr;
            float contact_size = 0;
            const auto num_contacts = gng_get_plane_contact_voxels(contact_nodes.data(), contact_nodes.size(),
                &contact_voxels, &contact_size);
            const auto contact_ms = elapsed_ms(contact_start);
            double old_ms = 0, new_ms = 0;
            for (uint32_t iter = 0; iter < 6; ++iter) {
                const gng_plane_contact_voxel *old_voxels = nullptr, *new_voxels = nullptr;
                uint32_t old_num = 0, new_num = 0;
                float old_size = 0, new_size = 0;
                const auto measure_old = [&] {
                    const auto start = clock_type::now();
                    old_num = reference_plane_contact_voxels(contact_nodes.data(), contact_nodes.size(), &old_voxels, &old_size);
                    old_ms += elapsed_ms(start);
                };
                const auto measure_new = [&] {
                    const auto start = clock_type::now();
                    new_num = gng_get_plane_contact_voxels(contact_nodes.data(), contact_nodes.size(), &new_voxels, &new_size);
                    new_ms += elapsed_ms(start);
                };
                if (iter % 2) {measure_new(); measure_old();} else {measure_old(); measure_new();}
                require(old_num == new_num && old_size == new_size, "接触セル数・幅の旧方式一致");
                for (uint32_t idx = 0; idx < old_num; ++idx) {
                    const auto &a = old_voxels[idx], &b = new_voxels[idx];
                    require(a.center.x == b.center.x && a.center.y == b.center.y && a.center.z == b.center.z &&
                        a.num_points == b.num_points && a.contact == b.contact, "接触セル順序・中心・点数・区分の旧方式一致");
                }
            }
            if (frame >= 20) {
                results.add("contact_old_ms", old_ms / 6); results.add("contact_new_ms", new_ms / 6);
                results.add("contact_extract_ms", contact_ms); results.add("num_contact_voxels", num_contacts);
                results.add("gng_reference_ms", gng_ms); results.add("plane_reference_ms", plane_ms);
                results.add("model_common_ms", model_ms); results.add("nodes", map.node_num);
                results.add("planes", result.clusters.clusters.size());
            }
            if (frame == 30 && !contact_nodes.empty()) {
                const auto check_contacts = [&] {
                    const gng_plane_contact_voxel *a = nullptr, *b = nullptr;
                    float a_size = 0, b_size = 0;
                    const auto a_num = reference_plane_contact_voxels(contact_nodes.data(), contact_nodes.size(), &a, &a_size);
                    const auto b_num = gng_get_plane_contact_voxels(contact_nodes.data(), contact_nodes.size(), &b, &b_size);
                    require(a_num == b_num && a_size == b_size, "索引切替・参照変更後のセル数一致");
                    for (uint32_t idx = 0; idx < a_num; ++idx) {
                        require(a[idx].center.x == b[idx].center.x && a[idx].center.y == b[idx].center.y &&
                            a[idx].center.z == b[idx].center.z && a[idx].num_points == b[idx].num_points &&
                            a[idx].contact == b[idx].contact, "索引切替・参照変更後の全セル一致");
                    }
                };
                auto &grid = *gng.vg.voxel_config;
                const auto saved_max = grid.maxXYZ;
                // 疎な索引への退避、ビット表縮小・復帰、古い所属・重複参照の検証。
                grid.maxXYZ = UINT32_MAX; check_contacts();
                grid.maxXYZ = 64; check_contacts();
                grid.maxXYZ = saved_max; check_contacts();
                const auto saved_refs = contact_nodes;
                for (auto &ref : contact_nodes) {++ref.frame;}
                check_contacts();
                contact_nodes = saved_refs;
                if (contact_nodes.size() > 1) {contact_nodes.back() = contact_nodes.front();}
                check_contacts();
                contact_nodes = saved_refs; check_contacts();
                results.add("contact_state_tests", 6);
            }
            ++frame;
        }
        require(frame > 20, "ウォームアップ後の入力フレーム数");
        results.add("frames", frame); results.add("checksum", checksum); results.add("synthetic_tests", 10);
        results.write(output+"/metrics.json");
        std::cout << "frames=" << frame << " checksum=" << checksum << " oracle=passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
