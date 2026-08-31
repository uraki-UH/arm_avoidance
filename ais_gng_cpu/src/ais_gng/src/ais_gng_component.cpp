#include <ais_gng/ais_gng_component.hpp>
#include <ais_gng/handle_label_utils.hpp>

#if defined(AIS_GNG_BACKEND_CPU)
#include <ais_gng/topological_plane/plane_cluster_parameters.hpp>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <unistd.h>

// #include "core/common/manipulability_serialization.hpp"

using namespace fuzzrobo;
using PC2 = sensor_msgs::msg::PointCloud2;

namespace {
class GngStdoutCapture {
 public:
    GngStdoutCapture() {
        output_file_ = std::tmpfile();
        if (output_file_ == nullptr) {
            return;
        }
        std::fflush(stdout);
        saved_stdout_fd_ = dup(STDOUT_FILENO);
        if (saved_stdout_fd_ < 0) {
            std::fclose(output_file_);
            output_file_ = nullptr;
            return;
        }
        if (dup2(fileno(output_file_), STDOUT_FILENO) < 0) {
            close(saved_stdout_fd_);
            saved_stdout_fd_ = -1;
            std::fclose(output_file_);
            output_file_ = nullptr;
            return;
        }
        is_capturing_ = true;
    }

    ~GngStdoutCapture() {
        restoreStdout();
        if (output_file_ != nullptr) {
            std::fclose(output_file_);
        }
    }

    std::string takeOutput() {
        if (!is_capturing_ || output_file_ == nullptr) {
            return "";
        }
        std::fflush(stdout);
        restoreStdout();
        std::rewind(output_file_);

        std::string output;
        std::array<char, 1024> buffer{};
        while (true) {
            const std::size_t read_size = std::fread(
                buffer.data(), sizeof(char), buffer.size(), output_file_);
            output.append(buffer.data(), read_size);
            if (read_size < buffer.size()) {
                break;
            }
        }
        std::fclose(output_file_);
        output_file_ = nullptr;
        return output;
    }

 private:
    void restoreStdout() {
        if (!is_capturing_) {
            return;
        }
        std::fflush(stdout);
        dup2(saved_stdout_fd_, STDOUT_FILENO);
        close(saved_stdout_fd_);
        saved_stdout_fd_ = -1;
        is_capturing_ = false;
    }

    FILE *output_file_{nullptr};
    int saved_stdout_fd_{-1};
    bool is_capturing_{false};
};

void replayGngSummaryWithTime(
    const std::string &output,
    double gng_ms,
    bool plane_cluster_ran,
    double plane_cluster_ms,
    bool nonplane_ran,
    double nonplane_ms) {
    std::size_t line_start = 0;
    while (line_start < output.size()) {
        const std::size_t line_end = output.find('\n', line_start);
        const bool has_newline = line_end != std::string::npos;
        const std::size_t line_size = has_newline
            ? line_end - line_start
            : output.size() - line_start;
        const std::string line = output.substr(line_start, line_size);

        int input_num = 0;
        int voxel_num = 0;
        int active_num = 0;
        int node_num = 0;
        int cluster_num = 0;
        if (std::sscanf(
                line.c_str(),
                "I: %d, V: %d, A: %d, Nodes: %d, Clusters: %d",
                &input_num,
                &voxel_num,
                &active_num,
                &node_num,
                &cluster_num) == 5) {
            char nonplane_time_text[32]{};
            if (nonplane_ran) {
                std::snprintf(
                    nonplane_time_text, sizeof(nonplane_time_text), "%.2f ms", nonplane_ms);
            } else {
                std::snprintf(nonplane_time_text, sizeof(nonplane_time_text), "off");
            }
            if (plane_cluster_ran) {
                std::fprintf(
                    stdout,
                    "I: %d, V: %d, A: %d, Nodes: %d, Clusters: %d, "
                    "GNG: %.2f ms, Plane: %.2f ms, Nonplane: %s",
                    input_num,
                    voxel_num,
                    active_num,
                    node_num,
                    cluster_num,
                    gng_ms,
                    plane_cluster_ms,
                    nonplane_time_text);
            } else {
                std::fprintf(
                    stdout,
                    "I: %d, V: %d, A: %d, Nodes: %d, Clusters: %d, "
                    "GNG: %.2f ms, Plane: off, Nonplane: %s",
                    input_num,
                    voxel_num,
                    active_num,
                    node_num,
                    cluster_num,
                    gng_ms,
                    nonplane_time_text);
            }
        } else {
            std::fwrite(line.data(), sizeof(char), line.size(), stdout);
        }
        if (has_newline) {
            std::fputc('\n', stdout);
        }
        line_start = has_newline ? line_end + 1 : output.size();
    }
    std::fflush(stdout);
}

bool fillSelectedPointCloud(
    const PC2 &source,
    const std::vector<uint32_t> &source_indices,
    PC2 &selected)
{
    selected.header = source.header;
    selected.height = 1;
    selected.width = static_cast<uint32_t>(source_indices.size());
    selected.fields = source.fields;
    selected.is_bigendian = source.is_bigendian;
    selected.point_step = source.point_step;
    selected.row_step = selected.point_step * selected.width;
    selected.is_dense = source.is_dense;
    selected.data.resize(static_cast<std::size_t>(selected.row_step));

    for (std::size_t i = 0; i < source_indices.size(); ++i) {
        const uint32_t source_index = source_indices[i];
        const uint32_t row = source.width == 0 ? 0 : source_index / source.width;
        const uint32_t column = source.width == 0 ? source_index : source_index % source.width;
        const std::size_t source_offset =
            static_cast<std::size_t>(row) * source.row_step +
            static_cast<std::size_t>(column) * source.point_step;
        const std::size_t destination_offset = i * selected.point_step;
        if (source_offset + source.point_step > source.data.size()) {
            selected.data.clear();
            selected.width = 0;
            selected.row_step = 0;
            return false;
        }
        std::copy_n(
            source.data.begin() + source_offset,
            source.point_step,
            selected.data.begin() + destination_offset);
    }
    return true;
}

#if defined(AIS_GNG_BACKEND_CPU)
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

void accumulateWinnerPointResidualStats(
    const GngTrainingEvent *events,
    uint32_t event_num,
    uint16_t winner_rank_max,
    std::unordered_map<uint16_t, SequentialNodeStats> &stats_by_node) {
    if (events == nullptr || event_num == 0) {
        return;
    }

    for (uint32_t event_idx = 0; event_idx < event_num; ++event_idx) {
        const auto &event = events[event_idx];
        if (event.winner_rank == 0 || event.winner_rank > winner_rank_max) {
            continue;
        }
        auto &stats = stats_by_node[event.winner_node_id];
        if (!stats.has_node_frame || stats.node_frame != event.winner_node_frame) {
            stats = SequentialNodeStats{};
            stats.node_frame = event.winner_node_frame;
            stats.has_node_frame = true;
        }
        updateSequentialNodeStats(stats, {
            static_cast<double>(event.residual.x),
            static_cast<double>(event.residual.y),
            static_cast<double>(event.residual.z),
        });
    }
}

void removeInactiveWinnerPointStats(
    const TopologicalMap &map,
    std::unordered_map<uint16_t, SequentialNodeStats> &stats_by_node) {
    std::unordered_map<uint16_t, uint32_t> active_node_frames;
    active_node_frames.reserve(map.node_num);
    for (uint32_t node_idx = 0; node_idx < map.node_num; ++node_idx) {
        const auto &node = map.nodes[node_idx];
        active_node_frames.emplace(node.id, node.frame);
    }

    for (auto it = stats_by_node.begin(); it != stats_by_node.end();) {
        const auto active_it = active_node_frames.find(it->first);
        if (active_it == active_node_frames.end() ||
            active_it->second != it->second.node_frame) {
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

        if (stats_it == stats_by_node.end() ||
            stats_it->second.node_frame != map_node.frame) {
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
#endif

}  // namespace

AiSGNGComponent::AiSGNGComponent(const rclcpp::NodeOptions & options) : Node("ais_gng_node", options) {
    // Downsampling
    transformed_pcl_pub_ = this->create_publisher<PC2>("scan/transformed", 10);

    topological_map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        "topological_map",
        rclcpp::QoS(1).reliable().transient_local());

#if defined(AIS_GNG_BACKEND_CPU)
    direct_plane_cluster_enabled_ =
        this->declare_parameter<bool>("plane_cluster.direct_enabled", true);
    if (direct_plane_cluster_enabled_) {
        auto options = topological_plane::incremental::declareClusterOptions(
            *this, "plane_cluster.", true);
        direct_plane_clusterizer_ =
        std::make_unique<topological_plane::incremental::Clusterizer>(options);
        const auto output_topic = this->declare_parameter<std::string>(
            "plane_cluster.output_topic", "/plane_clusters");
        direct_plane_cluster_pub_ =
            this->create_publisher<ais_gng_msgs::msg::PlaneClusterArray>(
                output_topic, rclcpp::QoS(1).reliable().transient_local());
        RCLCPP_INFO(
            this->get_logger(), "Direct plane clustering enabled: GNG -> %s",
            output_topic.c_str());
    }
    enable_nonplane_component_timing_ = this->declare_parameter<bool>(
        "enable_nonplane_component_timing", false);
    if (enable_nonplane_component_timing_) {
        const auto timing_topic = this->declare_parameter<std::string>(
            "nonplane_component_timing_topic", "/nonplane_components/timing");
        nonplane_timing_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            timing_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(&AiSGNGComponent::nonplane_timing_cb, this, _1));
    }
#endif

    // パラメーターを動的に変える関数をセット
    param_handle_ = this->add_on_set_parameters_callback(std::bind(&AiSGNGComponent::param_cb, this, _1));

    // AiS-GNG関連
    this->declare_parameter("node.num_max", 65534);                                // ノード数の上限(cpu/gpu)
    this->declare_parameter("node.learning_num", 5);                               // 学習回数(cpu/gpu)
    this->declare_parameter("node.unknown_learning_rate", 0.8);                    // 未知物体の学習回数/通常学習回数(cpu)
    this->declare_parameter("node.eta_s1", 0.4);                                   // 第１近傍ノードの学習係数(cpu/gpu)
    this->declare_parameter("node.eta_s2", 0.008);                                 // 近傍ノードの学習係数(cpu)
    this->declare_parameter("node.eta_decay_rate", 1.0);                                 // 学習係数の減衰率(cpu)
    node_covariance_enabled_ =
        this->declare_parameter<bool>("node.covariance_enabled", false);           // 共分散楕円の推定を有効にするか
    const auto covariance_winner_rank_max =
        this->declare_parameter<int64_t>("node.covariance_winner_rank_max", 1);    // 共分散へ含める勝者rankの最大値
    if (covariance_winner_rank_max < 1 || covariance_winner_rank_max > 2) {
        throw std::invalid_argument(
            "node.covariance_winner_rank_max must be between 1 and 2");
    }
    node_covariance_winner_rank_max_ = static_cast<uint16_t>(covariance_winner_rank_max);
    performance_log_interval_ms_ =
    this->declare_parameter<int64_t>("performance.log_interval_ms", 0);        // 詳細実行周期ログの間隔(ms)。0で無効
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
    const auto input_point_cloud_num =
        this->declare_parameter<int64_t>("input.point_cloud_num", 20000);           // 入力点群数 (cpu/gpu)
    if (input_point_cloud_num <= 0 ||
        input_point_cloud_num > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
        throw std::invalid_argument(
            "input.point_cloud_num must be in uint32 range and greater than zero");
    }
    input_point_cloud_num_ = static_cast<uint32_t>(input_point_cloud_num);
    const auto sampling_mode = this->declare_parameter<std::string>(
        "input.sampling_mode", "head");
    const auto parsed_sampling_mode = parsePointSamplingMode(sampling_mode);
    if (!parsed_sampling_mode) {
        throw std::invalid_argument(
            "input.sampling_mode must be either 'head' or 'uniform'");
    }
    input_sampling_mode_ = *parsed_sampling_mode;
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
#if defined(AIS_GNG_BACKEND_CPU)
            gng_setTrainingEventMaxWinnerRank(node_covariance_winner_rank_max_);
            gng_setTrainingEventCapture(node_covariance_enabled_ ? 1 : 0);
#endif
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

    // 点群入力subscription
    int num_topics = input_topic_names_.size();
    int queue_size = 10;
    const auto pointcloud_qos = rclcpp::SensorDataQoS();
    if(num_topics == 0){
        RCLCPP_ERROR(this->get_logger(), "input.topic_names is empty.");
    }else if(num_topics == 1){ // 通常のサブスクライバ
        pcl_sub_ = this->create_subscription<PC2>(
            input_topic_names_[0], pointcloud_qos,
            [this](const PC2::ConstSharedPtr& m1){
                this->process_clouds({m1});
            }
        );
    }else { // メッセージフィルタを使用した同期
        for (const auto& name : input_topic_names_) {
            pcl_subs_.push_back(std::make_shared<message_filters::Subscriber<PC2>>(
                this, name, pointcloud_qos.get_rmw_qos_profile()));
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

#if defined(AIS_GNG_BACKEND_CPU)
void AiSGNGComponent::nonplane_timing_cb(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
    if (msg->data.size() != 2U || !std::isfinite(msg->data[0]) ||
        !std::isfinite(msg->data[1]) || msg->data[0] < 0.0 || msg->data[1] < 0.0)
    {
        return;
    }
    const auto frame_number = static_cast<uint32_t>(std::llround(msg->data[0]));
    nonplane_summary_entry entry;
    {
        std::lock_guard<std::mutex> lock(nonplane_summary_mutex_);
        const auto entry_it = nonplane_summary_by_frame_.find(frame_number);
        if (entry_it == nonplane_summary_by_frame_.end()) {
            return;
        }
        entry = std::move(entry_it->second);
        nonplane_summary_by_frame_.erase(entry_it);
    }
    replayGngSummaryWithTime(
        entry.output,
        entry.gng_ms,
        entry.plane_cluster_ran,
        entry.plane_cluster_ms,
        true,
        msg->data[1]);
}
#endif

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
        } else if (name == "node.covariance_enabled") {
            node_covariance_enabled_ = p.as_bool();
            if (!node_covariance_enabled_) {
                winner_point_stats_.clear();
            }
#if defined(AIS_GNG_BACKEND_CPU)
            if (initialized_) {
                gng_setTrainingEventMaxWinnerRank(node_covariance_winner_rank_max_);
                gng_setTrainingEventCapture(node_covariance_enabled_ ? 1 : 0);
            }
#endif
            success = true;
        } else if (name == "node.covariance_winner_rank_max") {
            const auto winner_rank_max = p.as_int();
            if (winner_rank_max < 1 || winner_rank_max > 2) {
                result.successful = false;
                result.reason = "node.covariance_winner_rank_max must be between 1 and 2";
                return result;
            }
            node_covariance_winner_rank_max_ = static_cast<uint16_t>(winner_rank_max);
#if defined(AIS_GNG_BACKEND_CPU)
            if (initialized_) {
                gng_setTrainingEventMaxWinnerRank(node_covariance_winner_rank_max_);
            }
#endif
            success = true;
#if defined(AIS_GNG_BACKEND_CPU)
        } else if (name == "plane_cluster.use_node_rho_for_seed_order") {
            if (direct_plane_clusterizer_) {
                direct_plane_clusterizer_->setUseNodeRhoForSeedOrder(p.as_bool());
            }
            success = true;
#endif
        } else if (name == "performance.log_interval_ms") {
            const auto interval_ms = p.as_int();
            if (interval_ms < 0) {
                result.successful = false;
                result.reason = "performance.log_interval_ms must be zero or greater";
                return result;
            }
            performance_log_interval_ms_ = interval_ms;
            success = true;
        } else if (name == "input.topic_names") {
            input_topic_names_.clear();
            auto topic_names = p.as_string_array();
            for (const auto &topic_name : topic_names) {
                input_topic_names_.emplace_back(topic_name);
            }
            success = true;
        } else if (name == "input.point_cloud_num") {
            const auto point_cloud_num = p.as_int();
            if (point_cloud_num <= 0 ||
                point_cloud_num > static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
                result.successful = false;
                result.reason = "input.point_cloud_num must be in uint32 range and greater than zero";
                return result;
            }
            input_point_cloud_num_ = static_cast<uint32_t>(point_cloud_num);
            success = true;
        } else if (name == "input.sampling_mode") {
            const auto mode = parsePointSamplingMode(p.as_string());
            if (!mode) {
                result.successful = false;
                result.reason = "input.sampling_mode must be either 'head' or 'uniform'";
                return result;
            }
            input_sampling_mode_ = *mode;
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
    if(!initialized_ || clouds.empty()){
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    double cycle_period_ms = 0.0;
    if (has_last_process_start_) {
        cycle_period_ms = std::chrono::duration<double, std::milli>(
            start - last_process_start_).count();
    }
    last_process_start_ = start;
    has_last_process_start_ = true;

    // 入力点群のセット
    auto &semantic_labels = semantic_label_buffer_;
    auto &source_point_indices = source_point_index_buffer_;
    semantic_labels.clear();
    source_point_indices.clear();
    PC2::ConstSharedPtr gng_input_msg;
    std::size_t raw_point_count_total = 0;
    std::size_t submitted_point_count_total = 0;
    std::size_t processed_raw_point_count = 0;
    bool has_semantic_labels = false;
    for(auto &msg: clouds){
        const uint64_t raw_point_count =
            static_cast<uint64_t>(msg->width) * static_cast<uint64_t>(msg->height);
        raw_point_count_total += static_cast<std::size_t>(raw_point_count);
        const uint32_t point_count = static_cast<uint32_t>(std::min<uint64_t>(
            raw_point_count, std::numeric_limits<uint32_t>::max()));
        const std::size_t semantic_offset = processed_raw_point_count;
        auto cloud_semantics = handle_label::extractSemanticLabels(*msg, semantic_handle_label_value_);
        if (!cloud_semantics.empty()) {
            if (!has_semantic_labels) {
                semantic_labels.resize(
                    semantic_offset,
                    ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
                has_semantic_labels = true;
            }
            if (cloud_semantics.size() != point_count) {
                cloud_semantics.assign(
                    point_count,
                    ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
            }
            semantic_labels.insert(
                semantic_labels.end(), cloud_semantics.begin(), cloud_semantics.end());
        } else if (has_semantic_labels) {
            semantic_labels.insert(
                semantic_labels.end(), point_count,
                ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
        }
        processed_raw_point_count += point_count;

        const bool requires_repacking =
            input_sampling_mode_ == PointSamplingMode::Uniform &&
            point_count > input_point_cloud_num_;
        if (requires_repacking) {
            if (!sampled_indices_valid_ ||
                sampled_source_point_count_ != point_count ||
                sampled_max_point_count_ != input_point_cloud_num_ ||
                sampled_mode_ != input_sampling_mode_)
            {
                sampled_point_indices_ = selectPointIndices(
                    point_count, input_point_cloud_num_, input_sampling_mode_);
                sampled_source_point_count_ = point_count;
                sampled_max_point_count_ = input_point_cloud_num_;
                sampled_mode_ = input_sampling_mode_;
                sampled_indices_valid_ = true;
            }
            if (!fillSelectedPointCloud(*msg, sampled_point_indices_, *sampled_cloud_buffer_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Failed to repack sampled point cloud; skipping frame");
                return;
            }
            gng_input_msg = sampled_cloud_buffer_;
        } else {
            gng_input_msg = msg;
        }
        submitted_point_count_total +=
            static_cast<std::size_t>(gng_input_msg->width) * gng_input_msg->height;

        source_point_indices.clear();
        if (requires_repacking) {
            source_point_indices.reserve(sampled_point_indices_.size());
            for (const uint32_t index : sampled_point_indices_) {
                source_point_indices.push_back(
                    static_cast<uint32_t>(semantic_offset + index));
            }
        } else if (semantic_offset != 0) {
            const uint32_t selected_count = std::min(point_count, input_point_cloud_num_);
            source_point_indices.reserve(selected_count);
            for (uint32_t index = 0; index < selected_count; ++index) {
                source_point_indices.push_back(
                    static_cast<uint32_t>(semantic_offset + index));
            }
        }

        auto lidar_config = getBase2LidarFrame(gng_input_msg);
        gng_setPointCloud(
            gng_input_msg->data.data(),
            gng_input_msg->width * gng_input_msg->height,
            &lidar_config);
        if (requires_repacking) {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Uniformly sampled GNG input from %u to %u points",
                point_count, gng_input_msg->width * gng_input_msg->height);
        }
    }
    auto &msg = clouds[0];

    // 出力のヘッダー
    std_msgs::msg::Header header;
    header.frame_id = (local_coordinates_ || base_frame_id_.empty())
        ? msg->header.frame_id
        : base_frame_id_;
    header.stamp = msg->header.stamp;
    const auto input_end = std::chrono::steady_clock::now();

    // GNGの実行とライブラリ要約ログへの処理時間付加
    GngStdoutCapture gng_stdout_capture;
    gng_exec();
    const auto gng_end = std::chrono::steady_clock::now();
    const std::string gng_summary_output = gng_stdout_capture.takeOutput();

#if defined(AIS_GNG_BACKEND_CPU)
    if (node_covariance_enabled_) {
        uint32_t event_num = 0;
        const auto *events = gng_getTrainingEvents(&event_num);
        accumulateWinnerPointResidualStats(
            events,
            event_num,
            node_covariance_winner_rank_max_,
            winner_point_stats_);
    }
#endif

    // GNGの結果を取得
    uint32_t label_num = 0, transformed_pcl_num = 0;
    auto map = gng_getTopologicalMap();// トポロジカルマップ
    auto label = gng_getDownSampling(&label_num); // ダウンサンプリング時のラベル
    auto transformed_pcl = gng_getAffineTransformedInputPointCloud(&transformed_pcl_num); // アフィン変換後の点群

    // トポロジカルマップをROS2メッセージに変換
    auto map_msg = makeTopologicalMapMsg(
        map, header,
        has_semantic_labels ? &semantic_labels : nullptr,
        &source_point_indices);
    
    // アフィン変換後の点群をROS2メッセージに変換
    auto transformed_msg = makePointCloud2Msg(header, transformed_pcl, transformed_pcl_num);
    const std::size_t output_node_count = map_msg->nodes.size();
    const std::size_t output_edge_count = map_msg->edges.size() / 2;
    const auto conversion_end = std::chrono::steady_clock::now();

    // クラスタのラベルを分類
    std::vector<uint32_t> cluster_ids, cluster_frames;
    std::vector<uint8_t> cluster_labels;
    cluster_classification_.classify(map_msg, cluster_ids, cluster_frames, cluster_labels);

    // GNGにフィードバック
    gng_setInferredClusterLabels(cluster_ids.data(), cluster_frames.data(), cluster_labels.data(), cluster_ids.size());
    const auto classification_end = std::chrono::steady_clock::now();

#if defined(AIS_GNG_BACKEND_CPU)
    std::unique_ptr<ais_gng_msgs::msg::PlaneClusterArray> direct_plane_clusters;
    bool plane_cluster_ran = false;
    if (direct_plane_clusterizer_) {
        auto result = direct_plane_clusterizer_->update(*map_msg);
        direct_plane_clusters =
            std::make_unique<ais_gng_msgs::msg::PlaneClusterArray>(std::move(result.clusters));
        plane_cluster_ran = true;
    }
#else
    constexpr bool plane_cluster_ran = false;
#endif
    const auto plane_cluster_end = std::chrono::steady_clock::now();
    const double gng_summary_ms = std::chrono::duration<double, std::milli>(
        gng_end - input_end).count();
    const double plane_cluster_summary_ms = std::chrono::duration<double, std::milli>(
        plane_cluster_end - classification_end).count();
#if defined(AIS_GNG_BACKEND_CPU)
    if (enable_nonplane_component_timing_) {
        std::lock_guard<std::mutex> lock(nonplane_summary_mutex_);
        nonplane_summary_by_frame_[map_msg->frame_number] = {
            gng_summary_output,
            gng_summary_ms,
            plane_cluster_ran,
            plane_cluster_summary_ms,
        };
        while (nonplane_summary_by_frame_.size() > 16U) {
            nonplane_summary_by_frame_.erase(nonplane_summary_by_frame_.begin());
        }
    } else {
        replayGngSummaryWithTime(
            gng_summary_output,
            gng_summary_ms,
            plane_cluster_ran,
            plane_cluster_summary_ms,
            false,
            0.0);
    }
#else
    replayGngSummaryWithTime(
        gng_summary_output,
        gng_summary_ms,
        plane_cluster_ran,
        plane_cluster_summary_ms,
        false,
        0.0);
#endif

    // マップを先にPublishし、表示専用ノードが同じフレームのクラスタを描画できるようにする。
    topological_map_pub_->publish(std::move(map_msg));
#if defined(AIS_GNG_BACKEND_CPU)
    if (direct_plane_clusters) {
        direct_plane_cluster_pub_->publish(std::move(direct_plane_clusters));
    }
#endif

    if(downsampling_.isTransformed()){
        downsampling_.publish<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(transformed_msg, label, label_num, header); // 変換後の点群をPublish
    }else{
        downsampling_.publish<sensor_msgs::msg::PointCloud2::ConstSharedPtr>(
            gng_input_msg, label, label_num, gng_input_msg->header); // 変換前の点群をPublish
    }

    // アフィン変換後の点群をPublish
    transformed_pcl_pub_->publish(std::move(transformed_msg));

    // 入力範囲の可視化
    filter_.publish(header);

    const auto end = std::chrono::steady_clock::now();
    if (performance_log_interval_ms_ > 0) {
        const double input_ms =
            std::chrono::duration<double, std::milli>(input_end - start).count();
        const double gng_ms =
            std::chrono::duration<double, std::milli>(gng_end - input_end).count();
        const double conversion_ms =
            std::chrono::duration<double, std::milli>(conversion_end - gng_end).count();
        const double classification_ms =
            std::chrono::duration<double, std::milli>(classification_end - conversion_end).count();
        const double plane_cluster_ms =
            std::chrono::duration<double, std::milli>(
                plane_cluster_end - classification_end).count();
        const double publish_ms =
            std::chrono::duration<double, std::milli>(end - plane_cluster_end).count();
        const double total_ms =
            std::chrono::duration<double, std::milli>(end - start).count();
        const double cycle_hz = cycle_period_ms > 0.0 ? 1000.0 / cycle_period_ms : 0.0;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            static_cast<uint64_t>(performance_log_interval_ms_),
            "cycle period=%.2f ms (%.2f Hz), processing=%.2f ms "
            "[input=%.2f gng=%.2f convert=%.2f classify=%.2f plane=%.2f publish=%.2f], "
            "points=%zu/%zu nodes=%zu edges=%zu covariance=%s",
            cycle_period_ms,
            cycle_hz,
            total_ms,
            input_ms,
            gng_ms,
            conversion_ms,
            classification_ms,
            plane_cluster_ms,
            publish_ms,
            submitted_point_count_total,
            raw_point_count_total,
            output_node_count,
            output_edge_count,
            node_covariance_enabled_ ? "on" : "off");
    }
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

std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> AiSGNGComponent::makeTopologicalMapMsg(
    const TopologicalMap &map,
    const std_msgs::msg::Header &header,
    const std::vector<uint8_t> *semantic_labels,
    const std::vector<uint32_t> *source_point_indices) {
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
            node_msg.inpcl_ids.reserve(node.inpcl_num);
            for (uint32_t point_index = 0; point_index < node.inpcl_num; ++point_index) {
                const uint32_t input_id = node.inpcl_ids[point_index];
                if (source_point_indices && input_id < source_point_indices->size()) {
                    node_msg.inpcl_ids.push_back((*source_point_indices)[input_id]);
                } else {
                    node_msg.inpcl_ids.push_back(input_id);
                }
            }
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

#if defined(AIS_GNG_BACKEND_CPU)
    if (node_covariance_enabled_) {
        removeInactiveWinnerPointStats(map, winner_point_stats_);
        applySequentialWinnerStats(
            *topological_map_msg,
            map,
            winner_point_stats_);
    }
#endif

    if (semantic_labels && !semantic_labels->empty()) {
        handle_label::applySemanticLabelsToMap(
            *topological_map_msg,
            *semantic_labels,
            semantic_handle_ratio_threshold_);
        updateSemanticLabelHistory(*topological_map_msg);
    } else if (!semantic_label_history_.empty()) {
        semantic_label_history_.clear();
    }

    if (!node_covariance_enabled_) {
        return topological_map_msg;
    }

    std::size_t nodes_with_winners = 0;
    std::size_t total_samples = 0;
    std::size_t max_samples = 0;
    uint32_t max_winner_count = 0;
    float max_cov_abs = 0.0f;
    for (const auto &node : topological_map_msg->nodes) {
        const auto stats_it = winner_point_stats_.find(node.id);
        if (stats_it != winner_point_stats_.end()) {
            total_samples += static_cast<std::size_t>(
                std::lround(std::max(0.0, stats_it->second.count)));
            max_samples = std::max<std::size_t>(
                max_samples,
                static_cast<std::size_t>(
                    std::lround(std::max(0.0, stats_it->second.count))));
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
