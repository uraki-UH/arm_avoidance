#include <ais_gng/ais_gng_component.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

using namespace fuzzrobo;
using PC2 = sensor_msgs::msg::PointCloud2;

namespace {

struct SequentialNodeStats {
    uint32_t count = 0;
    std::array<double, 3> mean{0.0, 0.0, 0.0};
    std::array<double, 9> m2{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

bool nearlyEqual(float lhs, float rhs, float eps = 1e-6f) {
    return std::fabs(lhs - rhs) <= eps;
}

bool nodeCoreEquals(
    const ais_gng_msgs::msg::TopologicalNode &lhs,
    const ais_gng_msgs::msg::TopologicalNode &rhs) {
    if (lhs.id != rhs.id || lhs.label != rhs.label || lhs.frame != rhs.frame) {
        return false;
    }
    if (!nearlyEqual(lhs.pos.x, rhs.pos.x) ||
        !nearlyEqual(lhs.pos.y, rhs.pos.y) ||
        !nearlyEqual(lhs.pos.z, rhs.pos.z) ||
        !nearlyEqual(lhs.normal.x, rhs.normal.x) ||
        !nearlyEqual(lhs.normal.y, rhs.normal.y) ||
        !nearlyEqual(lhs.normal.z, rhs.normal.z) ||
        !nearlyEqual(lhs.rho, rhs.rho)) {
        return false;
    }
    return lhs.inpcl_ids == rhs.inpcl_ids;
}

void updateSequentialNodeStats(SequentialNodeStats &stats, const std::array<double, 3> &point) {
    ++stats.count;
    const double n = static_cast<double>(stats.count);

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

std::vector<SequentialNodeStats> computeSequentialWinnerStats(
    const TopologicalMap &map,
    const float *transformed_pcl,
    const uint32_t transformed_pcl_num) {
    // Use the affine-transformed training points that GNG actually saw, and
    // assign each point to the nearest node for an online mean/covariance.
    std::vector<SequentialNodeStats> stats(map.node_num);
    if (map.node_num == 0 || transformed_pcl == nullptr || transformed_pcl_num == 0) {
        return stats;
    }

    for (uint32_t i = 0; i < transformed_pcl_num; ++i) {
        const float *p = transformed_pcl + i * 3;
        const std::array<double, 3> point{
            static_cast<double>(p[0]),
            static_cast<double>(p[1]),
            static_cast<double>(p[2]),
        };

        uint32_t best_idx = 0;
        double best_dist2 = std::numeric_limits<double>::max();
        for (uint32_t node_idx = 0; node_idx < map.node_num; ++node_idx) {
            const auto &node = map.nodes[node_idx];
            const double dx = point[0] - static_cast<double>(node.pos.x);
            const double dy = point[1] - static_cast<double>(node.pos.y);
            const double dz = point[2] - static_cast<double>(node.pos.z);
            const double dist2 = dx * dx + dy * dy + dz * dz;
            if (dist2 < best_dist2) {
                best_dist2 = dist2;
                best_idx = node_idx;
            }
        }

        updateSequentialNodeStats(stats[best_idx], point);
    }

    return stats;
}

void applySequentialWinnerStats(
    ais_gng_msgs::msg::TopologicalMap &map_msg,
    const std::vector<SequentialNodeStats> &stats) {
    const std::size_t node_num = std::min(map_msg.nodes.size(), stats.size());
    for (std::size_t i = 0; i < node_num; ++i) {
        const auto &src = stats[i];
        auto &dst = map_msg.nodes[i];
        dst.winner_point_count = src.count;
        dst.winner_point_mean.x = static_cast<float>(src.mean[0]);
        dst.winner_point_mean.y = static_cast<float>(src.mean[1]);
        dst.winner_point_mean.z = static_cast<float>(src.mean[2]);

        const double denom = src.count > 1 ? static_cast<double>(src.count - 1) : 1.0;
        for (std::size_t j = 0; j < 9; ++j) {
            dst.winner_point_covariance[j] = static_cast<float>(src.m2[j] / denom);
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
    this->declare_parameter("node.s1_reset_range", 0.1);                           // ノードの選択回数リセット範囲(cpu)
    this->declare_parameter("node.grid", 0.1);      //おそらく0.05ぐらいが限度                              // ノードのグリッドサイズ(m)(cpu/gpu)
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
        }else if (name == "input.topic_names") {
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
    for(auto &msg: clouds){
        auto lidar_config = getBase2LidarFrame(msg);
        gng_setPointCloud(msg->data.data(), msg->width * msg->height, &lidar_config);
    }
    auto &msg = clouds[0];

    // 出力のヘッダー
    std_msgs::msg::Header header;
    header.frame_id = base_frame_id_.empty() ? msg->header.frame_id : base_frame_id_;
    header.stamp = msg->header.stamp;

    // GNGの実行
    gng_exec();

    // GNGの結果を取得
    uint32_t label_num = 0, transformed_pcl_num = 0;
    auto map = gng_getTopologicalMap();// トポロジカルマップ
    auto label = gng_getDownSampling(&label_num); // ダウンサンプリング時のラベル
    auto transformed_pcl = gng_getAffineTransformedInputPointCloud(&transformed_pcl_num); // アフィン変換後の点群

    // トポロジカルマップをROS2メッセージに変換
    auto map_msg = makeTopologicalMapMsg(map, header, transformed_pcl, transformed_pcl_num);
    
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
    RCLCPP_INFO(this->get_logger(), "Processing time: %ld ms", duration.count());
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

    if (!has_topological_map_snapshot_) {
        last_published_nodes_ = std::move(current_nodes);
        has_topological_map_snapshot_ = true;
        return;
    }
}

std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> AiSGNGComponent::makeTopologicalMapMsg(
    const TopologicalMap &map,
    const std_msgs::msg::Header &header,
    const float *transformed_pcl,
    const uint32_t transformed_pcl_num) {
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
        node_msg.frame = node.frame;
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

    applySequentialWinnerStats(
        *topological_map_msg,
        computeSequentialWinnerStats(map, transformed_pcl, transformed_pcl_num));

    return topological_map_msg;
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
    if(base_frame_id_.empty() || base_frame_id_ == msg->header.frame_id){
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
        RCLCPP_ERROR(
            this->get_logger(), "Could not transform %s to %s: %s",
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
