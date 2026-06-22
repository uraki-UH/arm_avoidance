#include <ais_gng/ais_gng_component.hpp>
#include <ais_gng/handle_label_utils.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

// #include "core/common/manipulability_serialization.hpp"

using namespace fuzzrobo;
using PC2 = sensor_msgs::msg::PointCloud2;

namespace {
std::array<double, 3> scaleResidualByEta(
    const std::array<double, 3> &residual,
    double eta_s1)
{
    const double eta = std::clamp(eta_s1, 1e-6, 1.0);
    const double inv_eta = 1.0 / eta;
    return {
        residual[0] * inv_eta,
        residual[1] * inv_eta,
        residual[2] * inv_eta,
    };
}

double covarianceScale(const SequentialNodeStats &stats) {
    if (stats.count <= 1.0) {
        return 0.0;
    }

    const double denom = std::max(1.0, stats.count - 1.0);
    const double var_x = std::max(0.0, stats.m2[0] / denom);
    const double var_y = std::max(0.0, stats.m2[4] / denom);
    const double var_z = std::max(0.0, stats.m2[8] / denom);
    return std::sqrt((var_x + var_y + var_z) / 3.0);
}

double movementDecayFactor(
    const std::array<double, 3> &residual,
    const SequentialNodeStats &stats,
    double k_motion,
    double k_cov)
{
    const double norm = std::sqrt(
        residual[0] * residual[0] +
        residual[1] * residual[1] +
        residual[2] * residual[2]);
    const double cov = covarianceScale(stats);
    const double motion_term = std::exp(-std::max(0.0, k_motion) * norm);
    const double cov_term = std::exp(-std::max(0.0, k_cov) * cov);
    return std::clamp(motion_term * cov_term, 0.15, 1.0);
}

bool seedNodeStatsFromNeighbors(
    const TopologicalMap &map,
    std::size_t node_idx,
    SequentialNodeStats &stats)
{
    if (node_idx >= map.node_num || map.edges == nullptr || map.edge_num < 2) {
        return false;
    }

    const auto &node = map.nodes[node_idx];
    std::array<double, 3> sum_abs_diff{0.0, 0.0, 0.0};
    std::size_t neighbor_count = 0;

    for (uint32_t edge_idx = 0; edge_idx + 1 < map.edge_num; edge_idx += 2) {
        const auto lhs = static_cast<std::size_t>(map.edges[edge_idx]);
        const auto rhs = static_cast<std::size_t>(map.edges[edge_idx + 1]);
        std::size_t neighbor_idx = std::numeric_limits<std::size_t>::max();

        if (lhs == node_idx && rhs < map.node_num) {
            neighbor_idx = rhs;
        } else if (rhs == node_idx && lhs < map.node_num) {
            neighbor_idx = lhs;
        }

        if (neighbor_idx == std::numeric_limits<std::size_t>::max()) {
            continue;
        }

        const auto &neighbor = map.nodes[neighbor_idx];
        sum_abs_diff[0] += std::fabs(static_cast<double>(neighbor.pos.x) - static_cast<double>(node.pos.x));
        sum_abs_diff[1] += std::fabs(static_cast<double>(neighbor.pos.y) - static_cast<double>(node.pos.y));
        sum_abs_diff[2] += std::fabs(static_cast<double>(neighbor.pos.z) - static_cast<double>(node.pos.z));
        ++neighbor_count;
    }

    if (neighbor_count == 0) {
        return false;
    }

    const double inv_neighbor_count = 1.0 / static_cast<double>(neighbor_count);
    const double sigma_x = std::max(1e-4, 0.5 * sum_abs_diff[0] * inv_neighbor_count);
    const double sigma_y = std::max(1e-4, 0.5 * sum_abs_diff[1] * inv_neighbor_count);
    const double sigma_z = std::max(1e-4, 0.5 * sum_abs_diff[2] * inv_neighbor_count);

    stats.count = 2.0;
    stats.mean = {0.0, 0.0, 0.0};
    stats.m2 = {
        sigma_x * sigma_x, 0.0, 0.0,
        0.0, sigma_y * sigma_y, 0.0,
        0.0, 0.0, sigma_z * sigma_z,
    };
    return true;
}

void decaySequentialNodeStats(SequentialNodeStats &stats, double decay) {
    const double d = std::clamp(decay, 0.15, 1.0);
    stats.count *= d;
    stats.m2[0] *= d;
    stats.m2[1] *= d;
    stats.m2[2] *= d;
    stats.m2[3] *= d;
    stats.m2[4] *= d;
    stats.m2[5] *= d;
    stats.m2[6] *= d;
    stats.m2[7] *= d;
    stats.m2[8] *= d;
}

void updateSequentialNodeStats(SequentialNodeStats &stats, const std::array<double, 3> &point) {
    stats.count += 1.0;
    const double n = stats.count;

    std::array<double, 3> delta{
        point[0] - stats.mean[0],
        point[1] - stats.mean[1],
        point[2] - stats.mean[2],
    };

    stats.mean[0] += delta[0] / n;
    stats.mean[1] += delta[1] / n;
    stats.mean[2] += delta[2] / n;

    std::array<double, 3> delta2{
        point[0] - stats.mean[0],
        point[1] - stats.mean[1],
        point[2] - stats.mean[2],
    };

    stats.m2[0] += delta[0] * delta2[0];
    stats.m2[1] += delta[0] * delta2[1];
    stats.m2[2] += delta[0] * delta2[2];
    stats.m2[3] += delta[1] * delta2[0];
    stats.m2[4] += delta[1] * delta2[1];
    stats.m2[5] += delta[1] * delta2[2];
    stats.m2[6] += delta[2] * delta2[0];
    stats.m2[7] += delta[2] * delta2[1];
    stats.m2[8] += delta[2] * delta2[2];
}

void accumulateNodeMotionStats(
    const TopologicalMap &map,
    const std::unordered_map<uint16_t, ais_gng_msgs::msg::TopologicalNode> &previous_nodes,
    double eta_s1,
    double cov_decay_k,
    std::unordered_map<uint16_t, SequentialNodeStats> &stats_by_node) {
    if (map.node_num == 0) {
        return;
    }

    std::unordered_map<uint16_t, bool> active_node_ids;
    active_node_ids.reserve(map.node_num);

    // GNG内部の勝者選択後のノード移動量を、ノード座標誤差の逐次統計として蓄積する。
    for (uint32_t node_idx = 0; node_idx < map.node_num; ++node_idx) {
        const auto &node = map.nodes[node_idx];
        active_node_ids[node.id] = true;
        const auto prev_it = previous_nodes.find(node.id);
        if (prev_it == previous_nodes.end()) {
            auto &node_stats = stats_by_node[node.id];
            if (node_stats.count <= 0.0) {
                seedNodeStatsFromNeighbors(map, node_idx, node_stats);
            }
            continue;
        }

        const auto &prev = prev_it->second;
        if (prev.frame != node.frame) {
            stats_by_node.erase(node.id);
            auto &node_stats = stats_by_node[node.id];
            if (seedNodeStatsFromNeighbors(map, node_idx, node_stats)) {
                continue;
            }
            stats_by_node.erase(node.id);
            continue;
        }

        const std::array<double, 3> residual{
            static_cast<double>(node.pos.x) - static_cast<double>(prev.pos.x),
            static_cast<double>(node.pos.y) - static_cast<double>(prev.pos.y),
            static_cast<double>(node.pos.z) - static_cast<double>(prev.pos.z),
        };
        auto &node_stats = stats_by_node[node.id];
        if (node_stats.count <= 0.0) {
            seedNodeStatsFromNeighbors(map, node_idx, node_stats);
        }
        const double decay = movementDecayFactor(residual, node_stats, cov_decay_k, cov_decay_k);
        const auto estimated_input_offset = scaleResidualByEta(residual, eta_s1);
        decaySequentialNodeStats(node_stats, decay);
        updateSequentialNodeStats(node_stats, estimated_input_offset);
    }

    for (auto it = stats_by_node.begin(); it != stats_by_node.end();) {
        if (active_node_ids.find(it->first) == active_node_ids.end()) {
            it = stats_by_node.erase(it);
        } else {
            ++it;
        }
    }
}

void applySequentialWinnerStats(
    ais_gng_msgs::msg::TopologicalMap &map_msg,
    const TopologicalMap &map,
    const std::unordered_map<uint16_t, SequentialNodeStats> &stats_by_node) {
    const std::size_t node_num = std::min(map_msg.nodes.size(), static_cast<std::size_t>(map.node_num));
    for (std::size_t i = 0; i < node_num; ++i) {
        const auto &map_node = map.nodes[i];
        auto &dst = map_msg.nodes[i];
        const auto stats_it = stats_by_node.find(map_node.id);

        if (stats_it == stats_by_node.end()) {
            dst.winner_point_count = 0;
            dst.winner_point_covariance.fill(0.0f);
            continue;
        }

        const auto &stats = stats_it->second;
        const double count = stats.count;
        dst.winner_point_count = static_cast<uint32_t>(std::lround(std::max(0.0, count)));

        const double denom = count > 1 ? static_cast<double>(count - 1) : 1.0;
        for (std::size_t j = 0; j < 9; ++j) {
            dst.winner_point_covariance[j] = static_cast<float>(stats.m2[j] / denom);
        }
    }
}

}  // namespace

AiSGNGComponent::AiSGNGComponent(const rclcpp::NodeOptions & options) : Node("ais_gng_node", options) {
    // Downsampling
    transformed_pcl_pub_ = this->create_publisher<PC2>("scan/transformed", 10);

    topological_map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        "topological_map",
        rclcpp::QoS(1).reliable().transient_local());

    // パラメーターを動的に変える関数をセット
    param_handle_ = this->add_on_set_parameters_callback(std::bind(&AiSGNGComponent::param_cb, this, _1));

    // AiS-GNG関連
    this->declare_parameter("node.num_max", 65534);                                // ノード数の上限(cpu/gpu)
    this->declare_parameter("node.learning_num", 5);                               // 学習回数(cpu/gpu)
    this->declare_parameter("node.unknown_learning_rate", 0.8);                    // 未知物体の学習回数/通常学習回数(cpu)
    this->declare_parameter("node.eta_s1", 0.4);                                   // 第１近傍ノードの学習係数(cpu/gpu)
    this->declare_parameter("node.eta_s2", 0.008);                                 // 近傍ノードの学習係数(cpu)
    this->declare_parameter("node.eta_decay_rate", 1.0);                                 // 学習係数の減衰率(cpu)
    this->declare_parameter("node.cov_decay_k", 1.5);                              // ノード移動量に応じた分散減衰係数
    this->declare_parameter("node.covariance_enabled", true);                      // 共分散楕円の推定を有効にするか
    this->declare_parameter("node.s1_reset_range", 0.1);                           // ノードの選択回数リセット範囲(cpu)
    this->declare_parameter("node.grid", 0.05);      //おそらく0.05ぐらいが限度                              // ノードのグリッドサイズ(m)(cpu/gpu)
    this->declare_parameter("node.s1_age_max", std::vector<int>{6, 6, 6, 3});      // ノードの選択回数に基づく削除(cpu/gpu)
    this->declare_parameter("node.clusted_s1_age", std::vector<int>{20, 20, 6, 3});// クラスタ化されたノードの選択回数に基づく削除(cpu)
    this->declare_parameter("node.interval", std::vector<double>{});               // ノードの間隔(m)(cpu/gpu)
    this->declare_parameter("node.static.age_min", -1);                            // 長期記憶の年齢(cpu)  

    // エッジ関連
    this->declare_parameter("edge.num_max", 300000);                               // エッジ数の上限(cpu/gpu)
    this->declare_parameter("edge.age_max", 100);                                  // エッジの最大年齢(cpu/gpu)

    // ラベリング関連
    this->declare_parameter("label.fuzzy.unknown", 0.6);                            // 未知物体のメンバシップ関数(cpu)
    this->declare_parameter("label.fuzzy.min", 0.5);                                // 未知物体のメンバシップ関数のしきい値(cpu)
    this->declare_parameter("label.fuzzy.lpf_time_constant", 0.5);                  // ラベリングLPF時定数(s)(cpu)

    // ダウンサンプリング関連
    this->declare_parameter("ds.transformed", true);                               // アフィン変換後の点群をPublishするか (cpu/gpu)
    this->declare_parameter("ds.range_max", 0.2);                                  // ダウンサンプリングの範囲(m) (cpu/gpu)
    this->declare_parameter("ds.all.num_max", 4000);                               // 人ノード 最大点群数 (cpu/gpu)
    this->declare_parameter("ds.unknown.num_max", 2000);                           // 未知ノード 最大点群数 (cpu/gpu)
    this->declare_parameter("ds.human.num_max", 3000);                             // 人ノード 最大点群数 (cpu/gpu)

    // クラスタリング関連
    this->declare_parameter("cluster.num_max", 100);                               // クラスタ数の上限 (cpu)
    this->declare_parameter("cluster.node_num_min", 10);                           // 最小構成ノード数 (cpu)
    this->declare_parameter("cluster.velocity.lpf_time_constant", 0.5);            // ラベリングのLPF時定数(s) (cpu)
    this->declare_parameter("cluster.plane.volume", 10.0);                         // 壁と床の平面除去用の最小面積(m^2) (cpu)
    this->declare_parameter("cluster.unknown.edge_distance_max", 0.2);             // 未知物体ノードのXY最大距離(m) (cpu)
    this->declare_parameter("cluster.other.edge_distance_max", 0.4);               // 未知物体ではないノードのXY最大距離(m) (cpu)
    this->declare_parameter("cluster.human.radius", 0.6);                          // 人クラスタの最大半径(m) (cpu)
    this->declare_parameter("cluster.human.hysteresis_age", 5);                    // 人クラスタのヒステリシス年齢 (cpu)
    this->declare_parameter("cluster.human.confirmation_age", 5);                  // 人クラスタと認識する最低フレーム数 (cpu)
    this->declare_parameter("cluster.car.hysteresis_age", 5);                      // 車クラスタのヒステリシス年齢 (cpu)
    this->declare_parameter("cluster.car.confirmation_age", 5);                    // 車クラスタと認識する最低フレーム数 (cpu)

    // 分類器関連
    auto declare_classify_bool = [this](const std::string &name, bool default_value) {
        const bool value = this->declare_parameter<bool>(name, default_value);
        cluster_classification_.setParameter(name, 0, value);
    };
    auto declare_classify_double = [this](const std::string &name, double default_value) {
        const double value = this->declare_parameter<double>(name, default_value);
        cluster_classification_.setParameter(name, 0, value);
    };
    auto declare_classify_string = [this](const std::string &name, const std::string &default_value) {
        const std::string value = this->declare_parameter<std::string>(name, default_value);
        cluster_classification_.setParameter(name, 0, value);
    };
    declare_classify_bool("classify.human", true);                                 // 人の分類の有効化 (cpu)
    declare_classify_string("classify.human.model", "NN_human.pt");               // モデル名 (cpu)
    declare_classify_bool("classify.car", false);                                  // 車の分類を有効にするか (cpu)
    declare_classify_string("classify.car.model", "NN_car.pt");                   // モデル名 (cpu)
    declare_classify_double("classify.threshold", 0.8);                            // 分類のしきい値 (cpu)
    declare_classify_string("classify.device", "cpu");                            // 分類器の推論デバイス(cpu/cuda/auto)

    // 入力点群関連
    this->declare_parameter("input.topic_names", std::vector<std::string>{""});    // 入力点群のtopicの名前 (cpu/gpu)
    this->declare_parameter("input.point_cloud_num", 20000);                       // 入力点群数 (cpu/gpu)
    this->declare_parameter("input.base_frame_id", "map");                         // 入力点群の基準フレームID (cpu/gpu)
    this->declare_parameter("input.voxel_grid_unit", 0.02);                         // ボクセルグリッドのサイズ(m) (cpu/gpu)
    this->declare_parameter("input.visualize", true);                              // 位置フィルタの可視化 (cpu/gpu)
    this->declare_parameter("input.local_coordinates", false);                     // ローカル座標系を使用するか (cpu/gpu)
    this->declare_parameter("input.x_min", -20.);                                  // 位置フィルタの最小 x (cpu/gpu)
    this->declare_parameter("input.x_max", 20.);                                   // 位置フィルタの最大 x (cpu/gpu)
    this->declare_parameter("input.y_min", -20.);                                  // 位置フィルタの最小 y (cpu/gpu)
    this->declare_parameter("input.y_max", 20.);                                   // 位置フィルタの最大 y (cpu/gpu)
    this->declare_parameter("input.z_min", -0.5);                                  // 位置フィルタの最小 z (cpu/gpu)
    this->declare_parameter("input.z_max", 3.0);                                   // 位置フィルタの最大 z (cpu/gpu)
    this->declare_parameter("semantic.handle_label_value", 1);
    this->declare_parameter("semantic.handle_ratio_threshold", 0.5);
    this->declare_parameter("semantic.handle_history_size", 64);
    semantic_handle_label_value_ = static_cast<uint32_t>(
        std::max<int64_t>(0, this->get_parameter("semantic.handle_label_value").as_int()));
    semantic_handle_ratio_threshold_ = this->get_parameter("semantic.handle_ratio_threshold").as_double();
    semantic_handle_history_size_ = static_cast<std::size_t>(
        std::max<int64_t>(1, this->get_parameter("semantic.handle_history_size").as_int()));

    // 初期化
    switch(gng_init()){
        case ERROR_CHECK_FILE:
            RCLCPP_ERROR(this->get_logger(), "Failed to check file");
            return;
        case ERROR_CONNECT_YUBIKEY:
            RCLCPP_ERROR(this->get_logger(), "Failed to connect Yubikey");
            return;
        case ERROR_FAIL_AUTHENTICATION:
            RCLCPP_ERROR(this->get_logger(), "Failed to authenticate");
            return;
        case ERROR_VOXEL_GRID_LEAF_SIZE:
            RCLCPP_ERROR(this->get_logger(), "Leaf size is too small");
            return;
        case SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Initialized successfully");
            break;
        default:
            break;
    }


    filter_.init(this);
    downsampling_.init(this);
    cluster_classification_.init(this);
    
    initialized_ = true;

    // semsegの結果
    semseg_sub_ = this->create_subscription<PC2>(
        "scan/segmented", 10,
        std::bind(&AiSGNGComponent::semseg_cb, this, _1)
    );

    // point cloud subscription
    int num_topics = input_topic_names_.size();
    int queue_size = 10;
    if(num_topics == 0){
        RCLCPP_ERROR(this->get_logger(), "input.topic_names is empty.");
    }else if(num_topics == 1){ // 通常のサブスクライバ
        pcl_sub_ = this->create_subscription<PC2>(
            input_topic_names_[0], 10,
            [this](const PC2::ConstSharedPtr& m1){
                this->process_clouds({m1});
            }
        );
    }else { // メッセージフィルタを使用した同期
        for (const auto& name : input_topic_names_) {
            pcl_subs_.push_back(std::make_shared<message_filters::Subscriber<PC2>>(this, name));
        }
        if (num_topics == 2) {
            using Policy = message_filters::sync_policies::ApproximateTime<PC2, PC2>;
            auto sync = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue_size), *pcl_subs_[0], *pcl_subs_[1]);
            sync->registerCallback(
                std::bind([this](const PC2::ConstSharedPtr& m1, const PC2::ConstSharedPtr& m2){
                    this->process_clouds({m1, m2});
                }, std::placeholders::_1, std::placeholders::_2));
            sync_keeper_ = sync;
        } else if (num_topics == 3) {
            using Policy = message_filters::sync_policies::ApproximateTime<PC2, PC2, PC2>;
            auto sync = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue_size), *pcl_subs_[0], *pcl_subs_[1], *pcl_subs_[2]);
            sync->registerCallback(
                std::bind([this](const PC2::ConstSharedPtr& m1, const PC2::ConstSharedPtr& m2, const PC2::ConstSharedPtr& m3){
                    this->process_clouds({m1, m2, m3});
                }, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            sync_keeper_ = sync;
        } else if (num_topics == 4) {
            using Policy = message_filters::sync_policies::ApproximateTime<PC2, PC2, PC2, PC2>;
            auto sync = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue_size), *pcl_subs_[0], *pcl_subs_[1], *pcl_subs_[2], *pcl_subs_[3]);
            sync->registerCallback(
                std::bind([this](const PC2::ConstSharedPtr& m1, const PC2::ConstSharedPtr& m2, const PC2::ConstSharedPtr& m3, const PC2::ConstSharedPtr& m4){
                    this->process_clouds({m1, m2, m3, m4});
                }, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
            sync_keeper_ = sync;
        }else{
            RCLCPP_ERROR(this->get_logger(), "Only up to 4 topics are supported for synchronization.");
        }
    }
}

AiSGNGComponent::~AiSGNGComponent() {

}

rcl_interfaces::msg::SetParametersResult AiSGNGComponent::param_cb(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.reason = "success";
    result.successful = true;
    for (auto &p : params) {
        float value = 0.0F;
        bool success = false;
        std::vector<float> flt_array;
        auto name = p.get_name();
        auto type = p.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_DOUBLE)
            value = p.as_double();
        else if (type == rclcpp::ParameterType::PARAMETER_INTEGER)
            value = p.as_int();
        else if (type == rclcpp::ParameterType::PARAMETER_BOOL)
            value = p.as_bool();
        else if (type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            auto int_array = p.as_integer_array();
            flt_array.resize(int_array.size());
            for (size_t i = 0; i < flt_array.size(); ++i)
                flt_array[i] = static_cast<float>(int_array[i]);
        }else if(type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            auto double_array = p.as_double_array();
            flt_array.resize(double_array.size());
            for (size_t i = 0; i < flt_array.size(); ++i)
                flt_array[i] = static_cast<float>(double_array[i]);
        }

        // プラグインのパラメータ
        success |= filter_.setParameter(name, 0, value); 
        success |= downsampling_.setParameter(name, 0, value);
        success |= cluster_classification_.setParameter(name, 0, value);
        if(type == rclcpp::ParameterType::PARAMETER_STRING){
            success |= cluster_classification_.setParameter(name, 0, p.as_string());
        }
        // このノードのパラメータ
        if (name == "input.base_frame_id"){
            base_frame_id_ = p.as_string();
            success = true;
        } else if (name == "input.local_coordinates") {
            local_coordinates_ = p.as_bool();
            success = true;
        } else if (name == "semantic.handle_label_value") {
            semantic_handle_label_value_ = static_cast<uint32_t>(std::max<int64_t>(0, p.as_int()));
            success = true;
        } else if (name == "semantic.handle_ratio_threshold") {
            semantic_handle_ratio_threshold_ = std::clamp(p.as_double(), 0.0, 1.0);
            success = true;
        } else if (name == "semantic.handle_history_size") {
            semantic_handle_history_size_ = static_cast<std::size_t>(
                std::max<int64_t>(1, p.as_int()));
            success = true;
        } else if (name == "node.eta_s1") {
            node_eta_s1_ = std::max(1e-6, p.as_double());
            success = true;
        } else if (name == "node.cov_decay_k") {
            node_cov_decay_k_ = std::max(0.0, p.as_double());
            success = true;
        } else if (name == "node.covariance_enabled") {
            node_covariance_enabled_ = p.as_bool();
            success = true;
        } else if (name == "input.topic_names") {
            input_topic_names_.clear();
            auto topic_names = p.as_string_array();
            for (const auto &topic_name : topic_names) {
                input_topic_names_.emplace_back(topic_name);
            }
            success = true;
        }
        
        if(flt_array.size() > 0){
            for(std::size_t i = 0; i < flt_array.size(); ++i){
                success |= gng_setParameter(name.c_str(), static_cast<int>(i), flt_array[i]);
            }
        } else {
            success |= gng_setParameter(name.c_str(), 0, value);
        }

        std::stringstream param_value;
        if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            param_value << std::fixed << std::setprecision(2) << value;
        } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER ||
            type == rclcpp::ParameterType::PARAMETER_BOOL) {
            param_value << static_cast<int>(value);
        } else if (type == rclcpp::ParameterType::PARAMETER_STRING) {
            param_value << p.as_string();
        } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            param_value << "[ ";
            auto double_array = p.as_double_array();
            for (size_t i = 0; i < double_array.size(); ++i) {
                param_value << std::fixed << std::setprecision(2) << double_array[i] << ", ";
            }
            param_value << "]";
        } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            param_value << "[ ";
            auto integer_array = p.as_integer_array();
            for (size_t i = 0; i < integer_array.size(); ++i) {
                param_value << integer_array[i] << ", ";
            }
            param_value << "]";
        } else if (type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
            param_value << "[ ";
            auto string_array = p.as_string_array();
            for (size_t i = 0; i < string_array.size(); ++i) {
                param_value << "\"" << string_array[i] << "\", ";
            }
            param_value << "]";
        } else {
            param_value << "<unsupported>";
        }

        if (success) {
            RCLCPP_INFO(
                this->get_logger(), "param %s %s", name.c_str(), param_value.str().c_str());
        } else {
            RCLCPP_WARN(
                this->get_logger(), "Unknown param: %s %s", name.c_str(), param_value.str().c_str());
        }
    }
    return result;
}

void AiSGNGComponent::process_clouds(const std::vector<PC2::ConstSharedPtr>& clouds) {
    // 初期化されていない
    if(!initialized_){
        return;
    }
    auto start = std::chrono::steady_clock::now();

    // 入力点群のセット
    std::vector<uint8_t> semantic_labels;
    for(auto &msg: clouds){
        auto lidar_config = getBase2LidarFrame(msg);
        gng_setPointCloud(msg->data.data(), msg->width * msg->height, &lidar_config);
        auto cloud_semantics = handle_label::extractSemanticLabels(*msg, semantic_handle_label_value_);
        const std::size_t point_count = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
        if (cloud_semantics.size() != point_count) {
            cloud_semantics.assign(point_count, ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
        }
        semantic_labels.insert(semantic_labels.end(), cloud_semantics.begin(), cloud_semantics.end());
    }
    auto &msg = clouds[0];

    // 出力のヘッダー
    std_msgs::msg::Header header;
    header.frame_id = (local_coordinates_ || base_frame_id_.empty())
        ? msg->header.frame_id
        : base_frame_id_;
    header.stamp = msg->header.stamp;

    // GNGの実行
    gng_exec();

    // GNGの結果を取得
    uint32_t label_num = 0, transformed_pcl_num = 0;
    auto map = gng_getTopologicalMap();// トポロジカルマップ
    auto label = gng_getDownSampling(&label_num); // ダウンサンプリング時のラベル
    auto transformed_pcl = gng_getAffineTransformedInputPointCloud(&transformed_pcl_num); // アフィン変換後の点群

    // トポロジカルマップをROS2メッセージに変換
    auto map_msg = makeTopologicalMapMsg(
        map, header, &semantic_labels);
    
    // アフィン変換後の点群をROS2メッセージに変換
    auto transformed_msg = makePointCloud2Msg(header, transformed_pcl, transformed_pcl_num);

    // クラスタのラベルを分類
    std::vector<uint32_t> cluster_ids, cluster_frames;
    std::vector<uint8_t> cluster_labels;
    cluster_classification_.classify(map_msg, cluster_ids, cluster_frames, cluster_labels);

    // GNGにフィードバック
    gng_setInferredClusterLabels(cluster_ids.data(), cluster_frames.data(), cluster_labels.data(), cluster_ids.size());

    // トポロジカルマップをPublish
    publishTopologicalMapUpdate(*map_msg);
    topological_map_pub_->publish(std::move(map_msg));

    if(downsampling_.isTransformed()){
        downsampling_.publish<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(transformed_msg, label, label_num, header); // 変換後の点群をPublish
    }else{
        downsampling_.publish<sensor_msgs::msg::PointCloud2::ConstSharedPtr>(msg, label, label_num, msg->header); // 変換前の点群をPublish
    }

    // アフィン変換後の点群をPublish
    transformed_pcl_pub_->publish(std::move(transformed_msg));

    // 入力範囲の可視化
    filter_.publish(header);

    // debug log
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Processing time: %ld ms", duration.count());
}

void AiSGNGComponent::semseg_cb(const PC2::SharedPtr msg) {
    if(!initialized_ || ((msg->point_step) < (12 + sizeof(NodeSemSeg)))) {
        return;
    }
    uint32_t point_step = msg->point_step;
    uint32_t node_num = msg->width;
    std::vector<NodeSemSeg> result;
    result.resize(node_num);
    uint8_t *data_ptr = (uint8_t*)msg->data.data();
    uint8_t *result_ptr = (uint8_t*)result.data();
    for (uint32_t i = 0; i < node_num; ++i) {
        uint32_t offset = i * point_step + 12 + 12; 
        std::copy(data_ptr + offset, data_ptr + offset + sizeof(NodeSemSeg), result_ptr + i * sizeof(NodeSemSeg));
    }
    gng_setInferredNodeLabels((NodeSemSeg*)result.data(), (uint32_t)result.size());
    // RCLCPP_INFO(this->get_logger(), "Received semseg result with %u points", (uint32_t)result.size());
}

void AiSGNGComponent::publishTopologicalMapUpdate(const ais_gng_msgs::msg::TopologicalMap &map_msg) {
    std::unordered_map<uint16_t, ais_gng_msgs::msg::TopologicalNode> current_nodes;
    current_nodes.reserve(map_msg.nodes.size());
    for (const auto &node : map_msg.nodes) {
        current_nodes.emplace(node.id, node);
    }

    last_published_nodes_ = std::move(current_nodes);
    has_topological_map_snapshot_ = true;
}

std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> AiSGNGComponent::makeTopologicalMapMsg(
    const TopologicalMap &map,
    const std_msgs::msg::Header &header,
    const std::vector<uint8_t> *semantic_labels) {
    auto topological_map_msg = std::make_unique<ais_gng_msgs::msg::TopologicalMap>();
    topological_map_msg->header = header;
    topological_map_msg->frame_number = map.frame_number;
    topological_map_msg->nodes.reserve(map.node_num);
    for (uint32_t i = 0; i < map.node_num; ++i) {
        ais_gng_msgs::msg::TopologicalNode node_msg;
        auto &node = map.nodes[i];
        node_msg.id = node.id;
        node_msg.pos.x = node.pos.x;
        node_msg.pos.y = node.pos.y;
        node_msg.pos.z = node.pos.z;
        node_msg.normal.x = node.normal.x;
        node_msg.normal.y = node.normal.y;
        node_msg.normal.z = node.normal.z;
        node_msg.rho = node.rho;
        node_msg.label = node.label;
        node_msg.semantic_label = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
        node_msg.semantic_reliability = 0.0f;
        node_msg.frame = node.frame;
        // node_msg.is_goal = false;
        // node_msg.manip_valid = false;
        // node_msg.manip_orientation.w = 1.0;
        if (node.inpcl_num > 0) {
            node_msg.inpcl_ids.resize(node.inpcl_num);
            node_msg.inpcl_ids.assign(node.inpcl_ids, node.inpcl_ids + node.inpcl_num);
        }
        topological_map_msg->nodes.emplace_back(node_msg);
    }
    topological_map_msg->edges.resize(map.edge_num);
    topological_map_msg->edges.assign(map.edges, map.edges + map.edge_num);
    topological_map_msg->clusters.reserve(map.cluster_num);
    for (uint32_t i = 0; i < map.cluster_num; ++i) {
        ais_gng_msgs::msg::TopologicalCluster cluster_msg;
        auto &cluster = map.clusters[i];
        cluster_msg.id = cluster.id;
        cluster_msg.label = cluster.label;
        cluster_msg.label_reliability = cluster.label_reliability;
        cluster_msg.semantic_label = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
        cluster_msg.semantic_reliability = 0.0f;
        cluster_msg.pos.x = cluster.pos.x;
        cluster_msg.pos.y = cluster.pos.y;
        cluster_msg.pos.z = cluster.pos.z;
        cluster_msg.scale.x = cluster.scale.x;
        cluster_msg.scale.y = cluster.scale.y;
        cluster_msg.scale.z = cluster.scale.z;
        cluster_msg.quat.x = cluster.quat.x;
        cluster_msg.quat.y = cluster.quat.y;
        cluster_msg.quat.w = cluster.quat.w;
        cluster_msg.quat.z = cluster.quat.z;
        cluster_msg.frame = cluster.frame;
        cluster_msg.match = cluster.match;
        cluster_msg.velocity.x = cluster.velocity.x;
        cluster_msg.velocity.y = cluster.velocity.y;
        cluster_msg.velocity.z = cluster.velocity.z;
        cluster_msg.nodes.resize(cluster.node_num);
        cluster_msg.nodes.assign(cluster.nodes, cluster.nodes + cluster.node_num);
        topological_map_msg->clusters.emplace_back(cluster_msg);
    }

    if (node_covariance_enabled_) {
        accumulateNodeMotionStats(
            map,
            last_published_nodes_,
            node_eta_s1_,
            node_cov_decay_k_,
            winner_point_stats_);
        applySequentialWinnerStats(
            *topological_map_msg,
            map,
            winner_point_stats_);
    } else {
        winner_point_stats_.clear();
    }

    if (semantic_labels && !semantic_labels->empty()) {
        handle_label::applySemanticLabelsToMap(
            *topological_map_msg,
            *semantic_labels,
            semantic_handle_ratio_threshold_);
    }
    updateSemanticLabelHistory(*topological_map_msg);

    std::size_t nodes_with_winners = 0;
    std::size_t total_samples = 0;
    std::size_t max_samples = 0;
    uint32_t max_winner_count = 0;
    float max_cov_abs = 0.0f;
    for (const auto &node : topological_map_msg->nodes) {
        const auto stats_it = winner_point_stats_.find(node.id);
        if (node_covariance_enabled_ && stats_it != winner_point_stats_.end()) {
            total_samples += static_cast<std::size_t>(std::lround(std::max(0.0, stats_it->second.count)));
            max_samples = std::max<std::size_t>(
                max_samples,
                static_cast<std::size_t>(std::lround(std::max(0.0, stats_it->second.count))));
        }
        if (node.winner_point_count > 0) {
            ++nodes_with_winners;
            max_winner_count = std::max(max_winner_count, node.winner_point_count);
            for (const auto value : node.winner_point_covariance) {
                max_cov_abs = std::max(max_cov_abs, std::fabs(value));
            }
        }
    }
    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "motion stats: nodes=%zu total_samples=%zu max_samples=%zu winners=%zu max_count=%u max_cov_abs=%.6f",
        topological_map_msg->nodes.size(),
        total_samples,
        max_samples,
        nodes_with_winners,
        max_winner_count,
        max_cov_abs);

    return topological_map_msg;
}

void AiSGNGComponent::updateSemanticLabelHistory(ais_gng_msgs::msg::TopologicalMap &map_msg) {
    const std::size_t node_count = map_msg.nodes.size();
    if (semantic_label_history_.size() != node_count) {
        semantic_label_history_.clear();
        semantic_label_history_.resize(node_count);
    }
    if (semantic_handle_history_size_ == 0) {
        semantic_handle_history_size_ = 1;
    }

    std::vector<uint8_t> node_semantic_labels(node_count, ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
    for (std::size_t i = 0; i < node_count; ++i) {
        auto &hist = semantic_label_history_[i];
        const uint8_t current = map_msg.nodes[i].semantic_label;
        hist.push_back(current);
        while (hist.size() > semantic_handle_history_size_) {
            hist.pop_front();
        }

        std::size_t handle_count = 0;
        for (const auto value : hist) {
            if (value == ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE) {
                ++handle_count;
            }
        }
        const double ratio = hist.empty()
            ? 0.0
            : static_cast<double>(handle_count) / static_cast<double>(hist.size());
        const bool is_handle = ratio >= semantic_handle_ratio_threshold_ && handle_count > 0;
        map_msg.nodes[i].semantic_label = is_handle
            ? ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE
            : ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
        map_msg.nodes[i].semantic_reliability = static_cast<float>(ratio);
        node_semantic_labels[i] = map_msg.nodes[i].semantic_label;
    }

    for (auto &cluster : map_msg.clusters) {
        std::size_t total = 0;
        std::size_t handle = 0;
        for (const auto node_id : cluster.nodes) {
            if (node_id >= node_semantic_labels.size()) {
                continue;
            }
            ++total;
            if (node_semantic_labels[node_id] == ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE) {
                ++handle;
            }
        }
        const double ratio = total == 0 ? 0.0 : static_cast<double>(handle) / static_cast<double>(total);
        const bool is_handle = ratio >= semantic_handle_ratio_threshold_ && handle > 0;
        cluster.semantic_label = is_handle
            ? ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE
            : ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
        cluster.semantic_reliability = static_cast<float>(ratio);
    }
}

LiDAR_Config AiSGNGComponent::getBase2LidarFrame(const PC2::ConstSharedPtr msg) {
    static auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    static auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static LiDAR_Config lidar_config;
    lidar_config.pos.x = 0;
    lidar_config.pos.y = 0;
    lidar_config.pos.z = 0;
    lidar_config.quat.x = 0;
    lidar_config.quat.y = 0;
    lidar_config.quat.z = 0;
    lidar_config.quat.w = 1;
    lidar_config.point_step = msg->point_step;
    if(local_coordinates_ || base_frame_id_.empty() || base_frame_id_ == msg->header.frame_id){
        return lidar_config;
    }
    try {
        geometry_msgs::msg::TransformStamped tf_msg =
            tf_buffer_->lookupTransform(base_frame_id_, msg->header.frame_id, tf2::TimePointZero);
        auto q = tf_msg.transform.rotation;
        auto t = tf_msg.transform.translation;
        lidar_config.pos.x = t.x;
        lidar_config.pos.y = t.y;
        lidar_config.pos.z = t.z;
        lidar_config.quat.x = q.x;
        lidar_config.quat.y = q.y;
        lidar_config.quat.z = q.z;
        lidar_config.quat.w = q.w;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Could not transform %s to %s: %s",
            base_frame_id_.c_str(), msg->header.frame_id.c_str(), ex.what());
    }
    return lidar_config;
}

std::unique_ptr<PC2> AiSGNGComponent::mixPointCloud2Msg(
        const std_msgs::msg::Header &header,
        const PC2::SharedPtr &msg,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num) {
    auto pcl2_msg = std::make_unique<PC2>();
    if(msg->width * msg->height != transformed_pcl_num){
        RCLCPP_WARN(this->get_logger(), "The number of points in the original point cloud and the transformed point cloud do not match.");
        return pcl2_msg;
    }
    pcl2_msg->header = header;
    pcl2_msg->height = msg->height;
    pcl2_msg->width = msg->width;
    pcl2_msg->is_dense = msg->is_dense;
    pcl2_msg->is_bigendian = msg->is_bigendian;
    pcl2_msg->point_step = msg->point_step;
    pcl2_msg->row_step = msg->row_step;
    pcl2_msg->fields = msg->fields;
    pcl2_msg->data.resize(msg->data.size());

    std::copy(msg->data.begin(), msg->data.end(), pcl2_msg->data.begin());

    for(uint32_t i = 0; i < transformed_pcl_num; ++i) {
        std::copy(
            (uint8_t*)transformed_pcl + i * 3 * sizeof(float),
            (uint8_t*)transformed_pcl + (i + 1) * 3 * sizeof(float),
            pcl2_msg->data.begin() + i * msg->point_step);
    }

    return pcl2_msg;
}

std::unique_ptr<PC2> AiSGNGComponent::makePointCloud2Msg(
        const std_msgs::msg::Header &header,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num) {
    auto pcl2_msg = std::make_unique<PC2>();
    pcl2_msg->header = header;
    pcl2_msg->height = 1;
    pcl2_msg->width = transformed_pcl_num;
    pcl2_msg->is_dense = false;
    pcl2_msg->is_bigendian = false;
    pcl2_msg->point_step = 3 * sizeof(float);
    pcl2_msg->row_step = pcl2_msg->point_step * pcl2_msg->width;
    pcl2_msg->fields.resize(3);
    pcl2_msg->fields[0].name = "x";
    pcl2_msg->fields[0].offset = 0;
    pcl2_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[0].count = 1;
    pcl2_msg->fields[1].name = "y";
    pcl2_msg->fields[1].offset = 4;
    pcl2_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[1].count = 1;
    pcl2_msg->fields[2].name = "z";
    pcl2_msg->fields[2].offset = 8;
    pcl2_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[2].count = 1;
    pcl2_msg->data.resize(pcl2_msg->row_step * pcl2_msg->height);

    std::copy((uint8_t*)transformed_pcl, (uint8_t*)transformed_pcl + pcl2_msg->data.size(), pcl2_msg->data.begin());

    return pcl2_msg;
}

std::unique_ptr<PC2> AiSGNGComponent::makePointCloud2MsgFromClustedNode(
        const std_msgs::msg::Header &header,
        const ais_gng_msgs::msg::TopologicalMap &map) {
    auto node_is_clusted = std::vector<bool>(map.nodes.size(), false);
    for(const auto &cluster: map.clusters){
        for(const auto &node_id: cluster.nodes){
            node_is_clusted[node_id] = true;
        }
    }
    size_t point_step = 6 * sizeof(float) + 2 * sizeof(uint32_t);// x,y,z,normal + node_id + age
    std::vector<uint8_t> data(point_step * map.nodes.size(), 0);
    uint32_t clusted_node_num = 0;
    for(std::size_t i=0;i<map.nodes.size();++i){
        if(node_is_clusted[i]){
            std::copy(
                (uint8_t*)&map.nodes[i].pos.x,
                (uint8_t*)&map.nodes[i].pos.x + 3 * sizeof(float),
                data.data() + clusted_node_num * point_step
            );
            std::copy(
                (uint8_t*)&map.nodes[i].normal.x,
                (uint8_t*)&map.nodes[i].normal.x + 3 * sizeof(float),
                data.data() + clusted_node_num * point_step + 3 * sizeof(float)
            );
            std::copy(
                (uint8_t*)&map.nodes[i].id,
                (uint8_t*)&map.nodes[i].id + sizeof(uint16_t),
                data.data() + clusted_node_num * point_step + 6 * sizeof(float)
            );
            std::copy(
                (uint8_t*)&map.nodes[i].frame,
                (uint8_t*)&map.nodes[i].frame + sizeof(uint32_t),
                data.data() + clusted_node_num * point_step + 6 * sizeof(float) + sizeof(uint32_t)
            );
            clusted_node_num++;
        }
    }
    data.resize(clusted_node_num * point_step);

    auto pcl2_msg = std::make_unique<PC2>();
    pcl2_msg->header = header;
    pcl2_msg->height = 1;
    pcl2_msg->width = clusted_node_num;
    pcl2_msg->is_dense = false;
    pcl2_msg->is_bigendian = false;
    pcl2_msg->point_step = point_step;
    pcl2_msg->row_step = pcl2_msg->point_step * pcl2_msg->width;

    pcl2_msg->fields.resize(6);

    pcl2_msg->fields[0].name = "x";
    pcl2_msg->fields[0].offset = 0;
    pcl2_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[0].count = 1;

    pcl2_msg->fields[1].name = "y";
    pcl2_msg->fields[1].offset = 4;
    pcl2_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[1].count = 1;

    pcl2_msg->fields[2].name = "z";
    pcl2_msg->fields[2].offset = 8;
    pcl2_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[2].count = 1;

    pcl2_msg->fields[3].name = "normal";
    pcl2_msg->fields[3].offset = 12;
    pcl2_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcl2_msg->fields[3].count = 3;

    pcl2_msg->fields[4].name = "node_id";
    pcl2_msg->fields[4].offset = 24;
    pcl2_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT32;
    pcl2_msg->fields[4].count = 1;

    pcl2_msg->fields[5].name = "frame";
    pcl2_msg->fields[5].offset = 28;
    pcl2_msg->fields[5].datatype = sensor_msgs::msg::PointField::UINT32;
    pcl2_msg->fields[5].count = 1;

    pcl2_msg->data.resize(pcl2_msg->row_step * pcl2_msg->height);
    std::copy(data.begin(), data.end(), pcl2_msg->data.begin());
    return pcl2_msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(fuzzrobo::AiSGNGComponent)
