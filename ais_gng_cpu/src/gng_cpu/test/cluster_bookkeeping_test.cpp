#include "cpu/clustering.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <set>
#include <utility>

void require(bool is_valid, const char *message) {
    if (!is_valid) {std::cerr << message << '\n'; std::exit(1);}
}

std::vector<int> add_group(CUGNG &core, uint32_t num, float offset) {
    std::vector<int> ids;
    for (uint32_t idx = 0; idx < num; ++idx) {
        Vec3f position(offset + (idx & 1) * 0.3f, (idx % 3) * 0.4f, (idx % 4) * 0.5f);
        const auto node_id = core.add_node(position);
        require(node_id != NODE_NOID, "node_add_failed");
        auto &node = core.nodes[node_id];
        node.label = UNKNOWN_OBJECT;
        node.clusted_label = UNKNOWN_OBJECT;
        node.normal = Vec3f(0.f, 0.f, 1.f);
        node.static_node = false;
        node.frame = 1;
        ids.push_back(node_id);
        if (idx > 0) {core.connect(ids[idx - 1], node_id);}
    }
    return ids;
}

Cluster &find_group(Clustering &clustering, int node_id) {
    for (auto &cluster : clustering.clusters) {
        if (std::find(cluster.nodes_ids.begin(), cluster.nodes_ids.end(), node_id) != cluster.nodes_ids.end()) {
            return cluster;
        }
    }
    require(false, "cluster_missing");
    return clustering.clusters.front();
}

void prepare_frame(CUGNG &core, Clustering &clustering, uint32_t frame) {
    core.frame_number = frame;
    for (auto &node : core.nodes) {node.clustered_flag = false;}
    core.calc_edge_distanceXY();
    clustering.time.ts_prev = std::chrono::system_clock::now();
}

int main() {
    NodeConfig node_config{};
    node_config.num_max = 128;
    EdgeConfig edge_config{};
    OtherConfig input{};
    input.node_grid = input.voxel_grid_unit = 1;
    input.x_min = input.y_min = input.z_min = -10;
    input.x_max = input.y_max = input.z_max = 30;
    CUGNG core;
    require(core.init(&node_config, &edge_config, &input), "core_init_failed");
    ClusterConfig config{};
    config.node_num_min = 3;
    config.plane_volume = 0;
    config.unknown_edge_distance_max2 = config.other_edge_distance_max2 = 100;
    config.lpf_time = 0.5f;
    config.human_hysteresis_age = config.car_hysteresis_age = 5;
    config.human_confirmation_age = config.car_confirmation_age = 2;
    config.human_radius = 0.7f;
    Clustering clustering;
    clustering.init(&core, &config);
    auto group_a = add_group(core, 4, 0.f);
    auto group_b = add_group(core, 6, 4.f);
    auto group_c = add_group(core, 4, 8.f);
    auto group_d = add_group(core, 4, 12.f);
    auto group_e = add_group(core, 4, 16.f);
    for (uint32_t idx = 0; idx < group_a.size(); ++idx) {
        core.nodes[group_a[idx]].clusted_id = idx < 2 ? 80 : 70;
    }
    for (auto node_id : group_b) {core.nodes[node_id].clusted_id = 90;}
    for (auto node_id : group_c) {core.nodes[node_id].clusted_id = 70;}
    for (auto node_id : group_d) {core.nodes[node_id].clusted_id = 999;}
    clustering.clusters.emplace_back(group_b, core.nodes, 1);
    clustering.clusters.back().id = 90;
    clustering.clusters.back().ros_id = 1;
    clustering.clusters.emplace_back(group_a, core.nodes, 1);
    clustering.clusters.back().id = 70;
    clustering.clusters.back().ros_id = 5;
    clustering.clusters.emplace_back(group_d, core.nodes, 1);
    clustering.clusters.back().id = 80;
    clustering.clusters.back().ros_id = 7;
    // 重複IDの旧クラスタが存在する場合の、先頭一致の維持。
    clustering.clusters.emplace_back(group_e, core.nodes, 1);
    clustering.clusters.back().id = 70;
    clustering.clusters.back().ros_id = 9;
    prepare_frame(core, clustering, 10);
    clustering.clustering();
    require(clustering.clusters.size() == 5, "cluster_count_mismatch");
    require(find_group(clustering, group_a.front()).id == 70, "vote_tie_min_id_mismatch");
    require(find_group(clustering, group_a.front()).ros_id == 5, "duplicate_old_id_first_match_mismatch");
    require(find_group(clustering, group_b.front()).id == 90, "largest_cluster_match_mismatch");
    require(find_group(clustering, group_c.front()).id != 70, "used_id_reused");
    require(find_group(clustering, group_d.front()).id != 999, "missing_old_id_inherited");
    std::set<uint32_t> used_ros_ids{1, 5};
    std::set<uint64_t> used_ids;
    for (const auto &cluster : clustering.clusters) {
        require(used_ids.insert(cluster.id).second, "cluster_id_duplicate");
        if (cluster.id != 70 && cluster.id != 90) {
            uint32_t expected_ros_id = 0;
            while (used_ros_ids.count(expected_ros_id)) {++expected_ros_id;}
            require(cluster.ros_id == expected_ros_id, "min_free_ros_id_mismatch");
            used_ros_ids.insert(cluster.ros_id);
        }
        for (auto node_id : cluster.nodes_ids) {
            require(core.nodes[node_id].clusted_id == cluster.id, "node_cluster_id_mismatch");
        }
    }
    // コピーの独立性と、移動後のメンバー順序・属性保持。
    auto copy = clustering.clusters;
    const auto expected_ids = copy.front().nodes_ids;
    const auto expected_id = copy.front().id;
    Cluster moved(std::move(copy.front()));
    require(moved.nodes_ids == expected_ids && moved.id == expected_id, "move_constructor_mismatch");
    copy.back() = std::move(moved);
    require(copy.back().nodes_ids == expected_ids && copy.back().id == expected_id, "move_assignment_mismatch");
    copy.back().nodes_ids.front() = -1;
    require(clustering.clusters.front().nodes_ids == expected_ids, "copy_alias");

    auto &previous_human = find_group(clustering, group_b.front());
    previous_human.label_inferred = HUMAN;
    previous_human.count_inferred = 2;
    previous_human.frame_inferred = 10;
    auto &previous_car = find_group(clustering, group_a.front());
    previous_car.label_inferred = CAR;
    previous_car.count_inferred = 2;
    previous_car.frame_inferred = 10;
    prepare_frame(core, clustering, 12);
    clustering.clustering();
    require(find_group(clustering, group_b.front()).label == HUMAN, "human_inference_lost");
    require(find_group(clustering, group_a.front()).label == CAR, "car_inference_lost");
    require(find_group(clustering, group_b.front()).match == 1.f, "cluster_match_mismatch");
    prepare_frame(core, clustering, 16);
    clustering.clustering();
    require(find_group(clustering, group_b.front()).label_inferred == DEFAULT, "expired_inference_retained");
    require(find_group(clustering, group_a.front()).count_inferred == 0, "expired_confirmation_retained");

    // 全クラスタ消滅後と、ノードID再利用後の作業領域の残留確認。
    for (uint32_t idx = 0; idx < core.nodes.size(); ++idx) {core.delete_node(idx);}
    prepare_frame(core, clustering, 17);
    clustering.clustering();
    require(clustering.clusters.empty(), "empty_frame_stale_clusters");
    auto reused_group = add_group(core, 5, 3.f);
    require(reused_group.front() == 0, "node_id_not_reused");
    prepare_frame(core, clustering, 18);
    clustering.clustering();
    require(clustering.clusters.size() == 1, "reused_frame_count_mismatch");
    require(clustering.clusters.front().ros_id == 0, "reused_frame_ros_id_mismatch");
    require(clustering.clusters.front().id != 70 && clustering.clusters.front().id != 90, "reused_frame_old_id");

    // 別コアへの再初期化後の、旧作業領域と旧ID索引の無効化。
    core.clear();
    node_config.num_max = 64;
    require(core.init(&node_config, &edge_config, &input), "core_reinit_failed");
    clustering.clusters.clear();
    clustering.init(&core, &config);
    auto final_group = add_group(core, 7, 2.f);
    prepare_frame(core, clustering, 2);
    clustering.clustering();
    require(clustering.clusters.size() == 1, "reinit_cluster_count_mismatch");
    require(clustering.clusters.front().ros_id == 0, "reinit_ros_id_mismatch");
    require(clustering.clusters.front().nodes_ids.size() == final_group.size(), "reinit_members_mismatch");
    std::cout << "cluster_bookkeeping_test=passed\n";
}
