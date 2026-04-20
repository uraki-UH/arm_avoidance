#include <ais_gng/cluster_classification.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace {
constexpr float kEpsilon = 1.0e-6f;
constexpr float kPi = 3.14159265358979323846f;
constexpr double kMinYawDtSec = 1.0e-3;

struct ClusterFeature {
    bool valid = false;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::array<float, 3> normal{0.0f, 0.0f, 1.0f};
};

class DisjointSet {
   public:
    explicit DisjointSet(const size_t n) : parent_(n), rank_(n, 0) {
        for (size_t i = 0; i < n; ++i) {
            parent_[i] = static_cast<int>(i);
        }
    }

    int find(const int x) {
        if (parent_[x] == x) {
            return x;
        }
        parent_[x] = find(parent_[x]);
        return parent_[x];
    }

    void unite(const int a, const int b) {
        int root_a = find(a);
        int root_b = find(b);
        if (root_a == root_b) {
            return;
        }
        if (rank_[root_a] < rank_[root_b]) {
            std::swap(root_a, root_b);
        }
        parent_[root_b] = root_a;
        if (rank_[root_a] == rank_[root_b]) {
            ++rank_[root_a];
        }
    }

   private:
    std::vector<int> parent_;
    std::vector<int> rank_;
};

std::array<float, 3> normalize_vector(const std::array<float, 3> &v) {
    const float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (norm < kEpsilon) {
        return {0.0f, 0.0f, 1.0f};
    }
    return {v[0] / norm, v[1] / norm, v[2] / norm};
}

float angle_between_deg(const std::array<float, 3> &a, const std::array<float, 3> &b) {
    const auto na = normalize_vector(a);
    const auto nb = normalize_vector(b);
    float dot = na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2];
    dot = std::clamp(dot, -1.0f, 1.0f);
    return std::acos(dot) * 180.0f / kPi;
}

ClusterFeature make_cluster_feature(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const ais_gng_msgs::msg::TopologicalCluster &cluster) {
    ClusterFeature feature;
    if (cluster.nodes.empty()) {
        return feature;
    }

    float pos_sum_x = 0.0f;
    float pos_sum_y = 0.0f;
    float pos_sum_z = 0.0f;
    float normal_sum_x = 0.0f;
    float normal_sum_y = 0.0f;
    float normal_sum_z = 0.0f;
    size_t used_nodes = 0;

    for (const auto node_id : cluster.nodes) {
        if (node_id >= map.nodes.size()) {
            continue;
        }
        const auto &node = map.nodes[node_id];
        pos_sum_x += node.pos.x;
        pos_sum_y += node.pos.y;
        pos_sum_z += node.pos.z;
        normal_sum_x += node.normal.x;
        normal_sum_y += node.normal.y;
        normal_sum_z += node.normal.z;
        ++used_nodes;
    }

    if (used_nodes == 0) {
        return feature;
    }

    const float inv_count = 1.0f / static_cast<float>(used_nodes);
    feature.x = pos_sum_x * inv_count;
    feature.y = pos_sum_y * inv_count;
    feature.z = pos_sum_z * inv_count;
    feature.normal = normalize_vector({normal_sum_x, normal_sum_y, normal_sum_z});
    feature.valid = true;
    return feature;
}

ais_gng_msgs::msg::TopologicalCluster merge_cluster_group(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const std::vector<int> &indices,
    const uint8_t merged_label) {
    auto merged = map.clusters[indices.front()];
    merged.label = merged_label;

    float vel_sum_x = 0.0f;
    float vel_sum_y = 0.0f;
    float vel_sum_z = 0.0f;
    float match_sum = 0.0f;
    float reliability_sum = 0.0f;
    std::unordered_set<uint16_t> unique_node_ids;

    for (const auto idx : indices) {
        const auto &cluster = map.clusters[idx];
        merged.id = std::min(merged.id, cluster.id);
        merged.age = std::max(merged.age, cluster.age);
        vel_sum_x += cluster.velocity.x;
        vel_sum_y += cluster.velocity.y;
        vel_sum_z += cluster.velocity.z;
        match_sum += cluster.match;
        reliability_sum += cluster.reliability;
        for (const auto node_id : cluster.nodes) {
            unique_node_ids.insert(node_id);
        }
    }

    merged.nodes.assign(unique_node_ids.begin(), unique_node_ids.end());
    std::sort(merged.nodes.begin(), merged.nodes.end());

    const float inv_cluster_count = 1.0f / static_cast<float>(indices.size());
    merged.velocity.x = vel_sum_x * inv_cluster_count;
    merged.velocity.y = vel_sum_y * inv_cluster_count;
    merged.velocity.z = vel_sum_z * inv_cluster_count;
    merged.match = match_sum * inv_cluster_count;
    merged.reliability = reliability_sum * inv_cluster_count;

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    float pos_sum_x = 0.0f;
    float pos_sum_y = 0.0f;
    float pos_sum_z = 0.0f;
    size_t used_nodes = 0;

    for (const auto node_id : merged.nodes) {
        if (node_id >= map.nodes.size()) {
            continue;
        }
        const auto &node = map.nodes[node_id];
        min_x = std::min(min_x, node.pos.x);
        min_y = std::min(min_y, node.pos.y);
        min_z = std::min(min_z, node.pos.z);
        max_x = std::max(max_x, node.pos.x);
        max_y = std::max(max_y, node.pos.y);
        max_z = std::max(max_z, node.pos.z);
        pos_sum_x += node.pos.x;
        pos_sum_y += node.pos.y;
        pos_sum_z += node.pos.z;
        ++used_nodes;
    }

    if (used_nodes > 0) {
        const float inv_node_count = 1.0f / static_cast<float>(used_nodes);
        merged.pos.x = pos_sum_x * inv_node_count;
        merged.pos.y = pos_sum_y * inv_node_count;
        merged.pos.z = pos_sum_z * inv_node_count;
        merged.scale.x = max_x - min_x;
        merged.scale.y = max_y - min_y;
        merged.scale.z = max_z - min_z;
    }

    return merged;
}

uint8_t dominant_non_safe_label(
    const ais_gng_msgs::msg::TopologicalMap &map,
    const std::vector<int> &indices) {
    std::array<uint32_t, 6> node_votes{};
    for (const auto idx : indices) {
        const auto &cluster = map.clusters[idx];
        if (cluster.label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN ||
            cluster.label >= node_votes.size()) {
            continue;
        }
        node_votes[cluster.label] += static_cast<uint32_t>(std::max<size_t>(cluster.nodes.size(), 1));
    }

    uint8_t best_label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
    uint32_t best_vote = 0;
    // SAFE_TERRAINは除外する。ラベル優先順位はWALL→UNKNOWN→HUMAN→CAR→DEFAULT。
    constexpr std::array<uint8_t, 5> kPriority = {
        ais_gng_msgs::msg::TopologicalMap::WALL,
        ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT,
        ais_gng_msgs::msg::TopologicalMap::HUMAN,
        ais_gng_msgs::msg::TopologicalMap::CAR,
        ais_gng_msgs::msg::TopologicalMap::DEFAULT
    };
    for (const auto label : kPriority) {
        if (node_votes[label] > best_vote) {
            best_vote = node_votes[label];
            best_label = label;
        }
    }
    return best_label;
}

double normalize_radian_pi(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double stamp_to_sec(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat_msg) {
    tf2::Quaternion q(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}
}  // namespace

ClusterClassification::ClusterClassification()
    : human_enable(true),
      human_model_name("NN_human.pt"),
      car_enable(false),
      car_model_name("NN_car.pt"),
      threshold(0.8f),
      simple_reliability_human(0.55f),
      simple_reliability_car(0.45f),
      simple_reliability_unknown(0.20f),
      human_yaw_rate_threshold(0.2f),
      human_yaw_hold_seconds(0.5f),
      human_scale_threshold(10.0f),
      safe_terrain_merge_distance_threshold(0.25f),
      safe_terrain_merge_normal_angle_threshold_deg(10.0f),
      non_safe_merge_distance_threshold(0.0f) {}

ClusterClassification::~ClusterClassification() {}

void ClusterClassification::init(const rclcpp::Logger &logger) {
    (void)logger;
}

void ClusterClassification::merge_safe_terrain_clusters(
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map) {
    if (map->clusters.size() < 2) {
        return;
    }
    if (safe_terrain_merge_distance_threshold <= 0.0f || safe_terrain_merge_normal_angle_threshold_deg < 0.0f) {
        // 閾値を負値にした場合は、SAFE_TERRAIN統合を無効化する。
        return;
    }

    std::vector<int> safe_cluster_indices;
    safe_cluster_indices.reserve(map->clusters.size());
    std::vector<ClusterFeature> features(map->clusters.size());
    // Publish対象のクラスタから、SAFE_TERRAINだけを取り出して特徴量を作る。
    for (size_t i = 0; i < map->clusters.size(); ++i) {
        const auto &cluster = map->clusters[i];
        if (cluster.label != ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
            continue;
        }
        safe_cluster_indices.emplace_back(static_cast<int>(i));
        features[i] = make_cluster_feature(*map, cluster);
    }

    if (safe_cluster_indices.size() < 2) {
        return;
    }

    DisjointSet dsu(map->clusters.size());
    // 全SAFE_TERRAINクラスタの組み合わせを見て、
    // 距離・法線角度の両方が閾値以内なら同一グループとして連結する。
    for (size_t i = 0; i < safe_cluster_indices.size(); ++i) {
        for (size_t j = i + 1; j < safe_cluster_indices.size(); ++j) {
            const int idx_i = safe_cluster_indices[i];
            const int idx_j = safe_cluster_indices[j];
            const auto &feature_i = features[idx_i];
            const auto &feature_j = features[idx_j];
            if (!feature_i.valid || !feature_j.valid) {
                continue;
            }

            const float dx = feature_i.x - feature_j.x;
            const float dy = feature_i.y - feature_j.y;
            const float dz = feature_i.z - feature_j.z;
            const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance > safe_terrain_merge_distance_threshold) {
                continue;
            }
            const float normal_angle_deg = angle_between_deg(feature_i.normal, feature_j.normal);
            if (normal_angle_deg > safe_terrain_merge_normal_angle_threshold_deg) {
                continue;
            }
            dsu.unite(idx_i, idx_j);
        }
    }

    std::unordered_map<int, std::vector<int>> group_indices;
    for (const auto idx : safe_cluster_indices) {
        group_indices[dsu.find(idx)].emplace_back(idx);
    }
    if (group_indices.size() == safe_cluster_indices.size()) {
        return;
    }

    std::vector<ais_gng_msgs::msg::TopologicalCluster> merged_clusters;
    merged_clusters.reserve(map->clusters.size());
    std::unordered_set<int> emitted_groups;
    // もとの順序をできるだけ保ちながら、統合後クラスタに置き換える。
    for (size_t i = 0; i < map->clusters.size(); ++i) {
        const auto &cluster = map->clusters[i];
        if (cluster.label != ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
            merged_clusters.emplace_back(cluster);
            continue;
        }

        const int root = dsu.find(static_cast<int>(i));
        if (emitted_groups.find(root) != emitted_groups.end()) {
            continue;
        }
        emitted_groups.insert(root);

        const auto it = group_indices.find(root);
        if (it == group_indices.end() || it->second.size() == 1) {
            merged_clusters.emplace_back(cluster);
            continue;
        }

        // ROS2でPublishする前に、SAFE_TERRAINだけ後段処理で再統合する。
        // これにより「近距離かつ法線方向が近い」複数クラスタを1つとして扱える。
        merged_clusters.emplace_back(
            merge_cluster_group(*map, it->second, ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN));
    }

    map->clusters = std::move(merged_clusters);
}

void ClusterClassification::merge_non_safe_clusters(
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map) {
    if (map->clusters.size() < 2) {
        return;
    }
    if (non_safe_merge_distance_threshold <= 0.0f) {
        // 0以下なら非SAFE_TERRAIN統合は無効。
        return;
    }

    std::vector<int> non_safe_cluster_indices;
    non_safe_cluster_indices.reserve(map->clusters.size());
    for (size_t i = 0; i < map->clusters.size(); ++i) {
        const auto &cluster = map->clusters[i];
        if (cluster.label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
            continue;
        }
        non_safe_cluster_indices.emplace_back(static_cast<int>(i));
    }

    if (non_safe_cluster_indices.size() < 2) {
        return;
    }

    DisjointSet dsu(map->clusters.size());
    for (size_t i = 0; i < non_safe_cluster_indices.size(); ++i) {
        for (size_t j = i + 1; j < non_safe_cluster_indices.size(); ++j) {
            const int idx_i = non_safe_cluster_indices[i];
            const int idx_j = non_safe_cluster_indices[j];
            const auto &cluster_i = map->clusters[idx_i];
            const auto &cluster_j = map->clusters[idx_j];
            // 非SAFE_TERRAIN統合はクラスタ中心同士の距離で評価する。
            // node_idの形式に依存せず、しきい値を上げれば確実にまとまりやすくなる。
            const float dx = cluster_i.pos.x - cluster_j.pos.x;
            const float dy = cluster_i.pos.y - cluster_j.pos.y;
            const float dz = cluster_i.pos.z - cluster_j.pos.z;
            const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance > non_safe_merge_distance_threshold) {
                continue;
            }
            dsu.unite(idx_i, idx_j);
        }
    }

    std::unordered_map<int, std::vector<int>> group_indices;
    for (const auto idx : non_safe_cluster_indices) {
        group_indices[dsu.find(idx)].emplace_back(idx);
    }
    if (group_indices.size() == non_safe_cluster_indices.size()) {
        return;
    }

    std::vector<ais_gng_msgs::msg::TopologicalCluster> merged_clusters;
    merged_clusters.reserve(map->clusters.size());
    std::unordered_set<int> emitted_groups;
    for (size_t i = 0; i < map->clusters.size(); ++i) {
        const auto &cluster = map->clusters[i];
        if (cluster.label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
            merged_clusters.emplace_back(cluster);
            continue;
        }

        const int root = dsu.find(static_cast<int>(i));
        if (emitted_groups.find(root) != emitted_groups.end()) {
            continue;
        }
        emitted_groups.insert(root);

        const auto it = group_indices.find(root);
        if (it == group_indices.end() || it->second.size() == 1) {
            merged_clusters.emplace_back(cluster);
            continue;
        }

        // 距離閾値で非SAFE_TERRAINクラスタを後段統合する。
        // 閾値を十分大きくすると、非SAFE_TERRAINが1クラスタにまとまる。
        const uint8_t merged_label = dominant_non_safe_label(*map, it->second);
        merged_clusters.emplace_back(merge_cluster_group(*map, it->second, merged_label));
    }

    map->clusters = std::move(merged_clusters);
}

void ClusterClassification::classify(
    std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map,
    std::vector<uint32_t> &cluster_ids,
    std::vector<uint32_t> &cluster_ages,
    std::vector<uint8_t> &cluster_labels) {
    merge_safe_terrain_clusters(map);
    merge_non_safe_clusters(map);

    const double current_stamp_sec = stamp_to_sec(map->header.stamp);
    std::unordered_set<uint32_t> active_cluster_ids;
    active_cluster_ids.reserve(map->clusters.size());

    for (auto &c : map->clusters) {
        active_cluster_ids.insert(c.id);

        if (c.nodes.size() < 10) {
            continue;
        }
        if (
            c.label == ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT ||
            c.label == ais_gng_msgs::msg::TopologicalMap::HUMAN ||
            c.label == ais_gng_msgs::msg::TopologicalMap::CAR
        ) {
            const uint8_t original_label = c.label;

            // 前回とのyaw差分から回転速度(rad/s)を求め、閾値超えならHUMAN判定にする。
            const double current_yaw = yaw_from_quaternion(c.quat);
            bool yaw_rate_exceeded = false;

            auto history_it = yaw_histories_.find(c.id);
            if (history_it != yaw_histories_.end()) {
                const double dt = current_stamp_sec - history_it->second.stamp_sec;
                if (dt >= kMinYawDtSec) {
                    const double dyaw = normalize_radian_pi(current_yaw - history_it->second.yaw);
                    const double yaw_rate = std::abs(dyaw) / dt;
                    if (yaw_rate >= static_cast<double>(human_yaw_rate_threshold)) {
                        yaw_rate_exceeded = true;
                    }
                }
            }
            yaw_histories_[c.id] = {current_yaw, current_stamp_sec};

            if (yaw_rate_exceeded) {
                const double hold_sec = std::max(0.0, static_cast<double>(human_yaw_hold_seconds));
                human_hold_until_sec_[c.id] = current_stamp_sec + hold_sec;
            }

            bool is_human = false;
            const auto hold_it = human_hold_until_sec_.find(c.id);
            if (hold_it != human_hold_until_sec_.end() && current_stamp_sec <= hold_it->second) {
                is_human = true;
            }

            // human_scale_threshold は「HUMANにする条件」ではなく、
            // HUMAN判定を通す最大スケール上限として使う。
            if (is_human && human_scale_threshold > 0.0f) {
                const double cluster_min_scale = std::min(
                    {std::abs(static_cast<double>(c.scale.x)),
                     std::abs(static_cast<double>(c.scale.y)),
                     std::abs(static_cast<double>(c.scale.z))});
                if (cluster_min_scale > static_cast<double>(human_scale_threshold)) {
                    is_human = false;
                }
            }
            
            const double pos_threshold = -0.0;
            if (c.pos.y < pos_threshold) {
                is_human = false;
            }

            if (is_human) {
                c.label = ais_gng_msgs::msg::TopologicalMap::HUMAN;
                c.reliability = simple_reliability_human;
            } else if (original_label == ais_gng_msgs::msg::TopologicalMap::CAR) {
                // CARとして入ってきたクラスタは、速度条件を満たさない時も
                // 低信頼度のCARとして残す（UNKNOWNへ落としすぎないため）。
                c.label = ais_gng_msgs::msg::TopologicalMap::CAR;
                c.reliability = simple_reliability_car;
            } else {
                c.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
                c.reliability = simple_reliability_unknown;
            }
            cluster_ids.emplace_back(c.id);
            cluster_ages.emplace_back(c.age);
            cluster_labels.emplace_back(c.label);
        }
    }

    // 今フレームに存在しないクラスタ履歴は削除する（ID再利用時の誤判定防止）。
    for (auto it = yaw_histories_.begin(); it != yaw_histories_.end();) {
        if (active_cluster_ids.find(it->first) == active_cluster_ids.end()) {
            it = yaw_histories_.erase(it);
        } else {
            ++it;
        }
    }
    for (auto it = human_hold_until_sec_.begin(); it != human_hold_until_sec_.end();) {
        const bool inactive = active_cluster_ids.find(it->first) == active_cluster_ids.end();
        const bool expired = current_stamp_sec > it->second;
        if (inactive || expired) {
            it = human_hold_until_sec_.erase(it);
        } else {
            ++it;
        }
    }
}
