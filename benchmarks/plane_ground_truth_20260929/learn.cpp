#include "ais_gng/topological_plane/plane_cluster_incremental.hpp"
#include "cpu/cugng.hpp"
#include "cpu/labelling.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace plane = fuzzrobo::topological_plane::incremental;

// GTは学習器の外側で保持。入力点と学習イベントの対応専用。
struct input_point {
    uint32_t frame_idx = 0, point_idx = 0;
    Vec3f pos;
    int gt_label = -1;
};
struct point_vote {int winner_id = -1; int64_t winner_frame = -1;};

std::vector<std::vector<input_point>> read_dataset(const std::string &path) {
    std::ifstream input(path);
    if (!input) {throw std::runtime_error("入力CSVを開けません: " + path);}
    std::string line;
    if (!std::getline(input, line)) {throw std::runtime_error("入力CSVが空です");}
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    if (line != "frame_idx,point_idx,x,y,z,gt_label") {throw std::runtime_error("入力CSVのヘッダーが不正です");}
    std::vector<std::vector<input_point>> frames;
    std::unordered_set<uint32_t> point_ids;
    std::size_t line_num = 1;
    while (std::getline(input, line)) {
        ++line_num;
        if (line.empty()) {continue;}
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        input_point point;
        int64_t frame_idx, point_idx;
        std::string extra;
        if (!(row >> frame_idx >> point_idx >> point.pos[0] >> point.pos[1] >> point.pos[2] >> point.gt_label) ||
            (row >> extra) || frame_idx < 0 || frame_idx > UINT32_MAX || point_idx < 0 || point_idx > UINT32_MAX ||
            !std::isfinite(point.pos[0]) || !std::isfinite(point.pos[1]) || !std::isfinite(point.pos[2]) ||
            point.gt_label < -1) {
            throw std::runtime_error("入力CSVの不正な行: " + std::to_string(line_num));
        }
        point.frame_idx = static_cast<uint32_t>(frame_idx);
        point.point_idx = static_cast<uint32_t>(point_idx);
        if (frames.empty() || frames.back().front().frame_idx != point.frame_idx) {
            if (point.frame_idx != frames.size()) {
                throw std::runtime_error("フレーム番号は0始まりの連続区間が必要です");
            }
            frames.emplace_back(); point_ids.clear();
        }
        if (!point_ids.insert(point.point_idx).second) {throw std::runtime_error("同一フレーム内の点番号が重複しています");}
        frames.back().push_back(point);
    }
    if (frames.empty()) {throw std::runtime_error("入力点がありません");}
    return frames;
}

void write_graph(CUGNG &gng, std::ofstream &output, uint32_t &num_nodes, uint32_t &num_edges) {
    std::vector<plane::node_input> nodes;
    nodes.reserve(gng.node_num);
    for (const auto &node : gng.nodes) {
        if (node.id == NODE_NOID) {continue;}
        gng.tn_id[node.id] = static_cast<uint16_t>(nodes.size());
        nodes.emplace_back();
        auto &result = nodes.back();
        // native構造体のパディングを含む、同一ビルド間のバイト再現性。
        std::memset(&result, 0, sizeof(result));
        result.id = static_cast<uint16_t>(node.id);
        result.label = static_cast<uint8_t>(node.label);
        result.rho = node.rho;
        result.pos = {node.pos.p[0], node.pos.p[1], node.pos.p[2]};
        result.normal = {node.normal.p[0], node.normal.p[1], node.normal.p[2]};
        result.frame = node.frame;
    }
    std::vector<uint16_t> edges;
    for (const auto &node : gng.nodes) {
        if (node.id == NODE_NOID) {continue;}
        for (int edge_idx = 0; edge_idx < node.edge_num; ++edge_idx) {
            const auto other_id = node.edges[edge_idx];
            if (node.id < other_id && other_id < gng.nodes.size() && gng.nodes[other_id].id != NODE_NOID) {
                edges.push_back(gng.tn_id[node.id]); edges.push_back(gng.tn_id[other_id]);
            }
        }
    }
    num_nodes = static_cast<uint32_t>(nodes.size()); num_edges = static_cast<uint32_t>(edges.size() / 2);
    const uint32_t counts[]{num_nodes, static_cast<uint32_t>(edges.size())};
    output.write(reinterpret_cast<const char *>(counts), sizeof(counts));
    output.write(reinterpret_cast<const char *>(nodes.data()), nodes.size() * sizeof(plane::node_input));
    output.write(reinterpret_cast<const char *>(edges.data()), edges.size() * sizeof(uint16_t));
    if (!output) {throw std::runtime_error("グラフの書き込みに失敗しました");}
}

int main(int argc, char **argv) {
    try {
        if (argc < 3 || argc > 4 || (argc == 4 && std::string(argv[3]) != "0" && std::string(argv[3]) != "1")) {
            std::cerr << "usage: learn input_dataset.csv output_dir [capture=1|0]\n"; return 2;
        }
        const bool enable_capture = argc == 3 || std::string(argv[3]) == "1";
        auto frames = read_dataset(argv[1]);
        Param params;
        params.node.num_max = 1000;
        std::size_t max_point_num = 0;
        float min_pos[3]{INFINITY, INFINITY, INFINITY}, max_pos[3]{-INFINITY, -INFINITY, -INFINITY};
        for (const auto &frame : frames) {
            max_point_num = std::max(max_point_num, frame.size());
            for (const auto &point : frame) {
                for (int axis = 0; axis < 3; ++axis) {
                    min_pos[axis] = std::min(min_pos[axis], point.pos.p[axis]);
                    max_pos[axis] = std::max(max_pos[axis], point.pos.p[axis]);
                }
            }
        }
        if (max_point_num > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("1フレームの入力点数が上限を超えています");
        }
        // GTと独立した点座標の外接範囲。グリッド端への一致回避用の余白[m]。
        params.config.x_min = std::floor(min_pos[0]) - 1; params.config.x_max = std::ceil(max_pos[0]) + 1;
        params.config.y_min = std::floor(min_pos[1]) - 1; params.config.y_max = std::ceil(max_pos[1]) + 1;
        params.config.z_min = std::floor(min_pos[2]) - 1; params.config.z_max = std::ceil(max_pos[2]) + 1;
        params.config.point_cloud_num = static_cast<uint32_t>(max_point_num);
        params.node.learning_num = static_cast<int>(max_point_num);
        CUGNG gng;
        if (!gng.init(&params.node, &params.edge, &params.config)) {throw std::runtime_error("GNGの初期化に失敗しました");}
        gng.setTrainingEventMaxWinnerRank(1);
        Labelling labeler;
        labeler.init(&params.node, &params.label, &gng);
        const std::filesystem::path output_dir(argv[2]);
        std::filesystem::create_directories(output_dir);
        std::ofstream graphs(output_dir / "graphs.bin", std::ios::binary);
        std::ofstream votes(output_dir / "votes.csv"), stats(output_dir / "stats.csv");
        std::ofstream metadata(output_dir / "gng_metadata.json");
        if (!graphs || !votes || !stats || !metadata) {throw std::runtime_error("出力ファイルを開けません");}
        metadata << "{\"max_nodes\":1000,\"input_passes_per_frame\":1,\"input_order\":\"csv\","
                 << "\"enable_capture\":" << (enable_capture ? "true" : "false")
                 << ",\"max_winner_rank\":1,\"label_dt_sec\":0.5,\"node_grid_m\":" << params.config.node_grid
                 << ",\"eta_s1\":" << params.node.eta_s1 << ",\"eta_s2\":" << params.node.eta_s2
                 << ",\"eta_decay_rate\":" << params.node.eta_decay_rate << ",\"max_edge_age\":" << params.edge.age_max
                 << ",\"enable_voxel_sampling\":false,\"enable_object_clustering\":false,\"enable_plane_feedback\":false}\n";
        votes << "frame_idx,point_idx,gt_label,winner_id,winner_frame\n";
        stats << "frame_idx,gng_ms,num_nodes,num_edges,num_points,num_votes\n" << std::setprecision(12);
        for (auto &frame : frames) {
            std::vector<point_vote> frame_votes(frame.size());
            uint32_t num_votes = 0;
            const auto start = std::chrono::steady_clock::now();
            ++gng.frame_number;
            gng.setTrainingEventCapture(enable_capture);
            gng.begin_update_frame(true, true, true);
            for (std::size_t point_idx = 0; point_idx < frame.size(); ++point_idx) {
                auto &point = frame[point_idx];
                uint32_t num_before = 0, num_after = 0;
                gng.getTrainingEvents(&num_before);
                // 入力順の通常学習を各点1回。GTの参照・追加の最近傍探索なし。
                gng.learn_normal(point.pos, nullptr, point.point_idx);
                const auto *events = gng.getTrainingEvents(&num_after);
                if (num_after > num_before) {
                    if (num_after != num_before + 1 || events[num_before].winner_rank != 1) {
                        throw std::runtime_error("学習イベントと入力点の対応が不正です");
                    }
                    frame_votes[point_idx] = {events[num_before].winner_node_id, events[num_before].winner_node_frame};
                    ++num_votes;
                }
            }
            // 既存Timeの0.5秒クランプ固定による、実行速度と独立した幾何ラベル。
            labeler.time.ts_prev = std::chrono::system_clock::time_point{};
            labeler.labelling_fuzzy();
            gng.check_edge_distance(); gng.check_age(); gng.check_delete_no_edge_and_decay_eta();
            gng.calc_edge_distanceXY(); gng.end_update_frame();
            const double gng_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            uint32_t num_nodes = 0, num_edges = 0;
            write_graph(gng, graphs, num_nodes, num_edges);
            for (std::size_t point_idx = 0; point_idx < frame.size(); ++point_idx) {
                const auto &point = frame[point_idx]; const auto &vote = frame_votes[point_idx];
                votes << point.frame_idx << ',' << point.point_idx << ',' << point.gt_label << ','
                      << vote.winner_id << ',' << vote.winner_frame << '\n';
            }
            stats << frame.front().frame_idx << ',' << gng_ms << ',' << num_nodes << ',' << num_edges << ','
                  << frame.size() << ',' << num_votes << '\n';
        }
        graphs.close(); votes.close(); stats.close(); metadata.close();
        if (!graphs || !votes || !stats || !metadata) {throw std::runtime_error("出力の完了確認に失敗しました");}
        std::cout << "frames=" << frames.size() << " capture=" << enable_capture << '\n';
        return 0;
    } catch (const std::exception &error) {std::cerr << error.what() << '\n'; return 1;}
}
