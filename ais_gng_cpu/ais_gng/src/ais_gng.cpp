#include <ais_gng/ais_gng.hpp>

using namespace fuzzrobo;

AiSGNG::AiSGNG(const rclcpp::NodeOptions & options) : Node("ais_gng", options) {
    // Downsampling
    unknown_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/unknown", 10);
    human_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/human", 10);
    transformed_pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan/transformed", 10);

    // Filter
    filter_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("filter", 10);

    topological_map_pub_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>("topological_map", 30);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan", qos, std::bind(&AiSGNG::pcl_cb, this, _1));
    //pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan", 10, std::bind(&AiSGNG::pcl_cb, this, _1));

    // パラメーターを動的に変える関数をセット
    param_handle_ = this->add_on_set_parameters_callback(std::bind(&AiSGNG::param_cb, this, _1));

    // AiS-GNG関連
    this->declare_parameter("node.num_max", 20000);                                    // ノード数の上限
    this->declare_parameter("node.learning_num", 4000);                                // 学習回数
    this->declare_parameter("node.unknown_learning_rate", 0.8);                        // 未知物体の学習回数/通常学習回数
    this->declare_parameter("node.eta_s1", 0.08);                                      // 第１近傍ノードの学習係数
    this->declare_parameter("node.eta_s2", 0.008);                                     // 近傍ノードの学習係数
    this->declare_parameter("node.edge_age_max", 100);                                 // エッジの最大年齢
    this->declare_parameter("node.s1_reset_range", 0.1);                               // ノードの選択回数リセット範囲
    this->declare_parameter("node.s1_age", std::vector<int>{6, 6, 6, 3});              // ノードの選択回数に基づく削除
    this->declare_parameter("node.clusted_s1_age", std::vector<int>{20, 20, 6, 3});    // クラスタに属しているデフォルトノード選択回数に基づく削除
    this->declare_parameter("node.interval", std::vector<double>{0.1, 0.4, 0.4, 0.05});// ノードの間隔
    this->declare_parameter("node.static.age_min", -1);                                // ノードを固定する年齢のしきい値．-1だと固定しない．

    // ラベリング関連
    this->declare_parameter("label.fuzzy.unknown", 0.6);            // 未知物体のメンバシップ関数
    this->declare_parameter("label.fuzzy.min", 0.5);                // 未知物体のメンバシップ関数のしきい値
    this->declare_parameter("label.fuzzy.lpf_time_constant", 0.5);  // ラベリングLPF時定数(s)
    this->declare_parameter("label.honda_safe", 30.0);              // Hondaさん用床判定の閾値 [degree]

    // ダウンサンプリング関連
    this->declare_parameter("ds.transformed", true);    // アフィン変換後の点群をPublishするか
    this->declare_parameter("ds.range_max", 0.2);       // ダウンサンプリングの範囲(m)
    this->declare_parameter("ds.all.num_max", 4000);  // 人ノード 最大点群数
    this->declare_parameter("ds.unknown.num_max", 2000);// 未知ノード 最大点群数
    this->declare_parameter("ds.human.num_max", 3000);  // 人ノード 最大点群数

    // クラスタリング関連
    this->declare_parameter("cluster.num_min", 10);                     // 最小構成ノード数
    this->declare_parameter("cluster.velocity.lpf_time_constant", 0.5); // ラベリングのLPF時定数(s)
    this->declare_parameter("cluster.plane.volume", 10.0);              // 壁や床クラスタとして判定する最小面積(m^2)
    this->declare_parameter("cluster.unknown.edge_distance_max", 0.2);  // 未知物体ノードのXY最大距離(m)
    this->declare_parameter("cluster.other.edge_distance_max", 0.4);    // 未知物体ではないノードのXY最大距離(m)
    this->declare_parameter("cluster.human.radius", 0.6);               // 人クラスタの最大半径(m)
    this->declare_parameter("cluster.human.hysteresis_age", 5);         // 人クラスタのヒステリシス年齢
    this->declare_parameter("cluster.human.confirmation_age", 5);       // 人クラスタと認識する最低フレーム数
    this->declare_parameter("cluster.human.yaw_rate_threshold", 0.8);   // yaw変化率でHUMAN判定する閾値(rad/s)
    this->declare_parameter("cluster.human.yaw_hold_seconds", 1.0);     // yaw閾値超え後にHUMAN維持する時間(s)
    this->declare_parameter("cluster.human.scale_threshold", 0.0);      // HUMAN判定を通す最大スケール閾値(m), 0以下で無効
    this->declare_parameter("cluster.car.hysteresis_age", 5);           // 車クラスタのヒステリシス年齢
    this->declare_parameter("cluster.car.confirmation_age", 5);         // 車クラスタと認識する最低フレーム数
    this->declare_parameter("cluster.safe_terrain.merge_distance_max", 0.25);      // SAFE_TERRAIN統合の最大距離(m)
    this->declare_parameter("cluster.safe_terrain.merge_normal_angle_deg", 10.0);  // SAFE_TERRAIN統合の法線角度差(deg)
    this->declare_parameter("cluster.non_safe.merge_distance_max", 0.0);            // 非SAFE_TERRAIN統合の最大距離(m), 0で無効

    // 分類器関連
    this->declare_parameter("classify.human", true);                // 人の分類の有効化
    this->declare_parameter("classify.human.model", "NN_human.pt"); // モデル名
    this->declare_parameter("classify.car", false);                 // 車の分類を有効にするか
    this->declare_parameter("classify.car.model", "NN_car.pt");     // モデル名
    this->declare_parameter("classify.threshold", 0.8);             // 分類のしきい値
    this->declare_parameter("classify.simple_reliability.human", 0.55);   // 簡易再分類(HUMAN)の信頼度
    this->declare_parameter("classify.simple_reliability.car", 0.45);     // 簡易再分類(CAR)の信頼度
    this->declare_parameter("classify.simple_reliability.unknown", 0.20); // 簡易再分類(UNKNOWN)の信頼度

    // 入力点群関連
    this->declare_parameter("input.point_cloud_num", 20000);// 入力点群数
    this->declare_parameter("input.base_frame_id", "map");  // 入力点群の基準フレームID
    this->declare_parameter("input.voxel_grid_unit", 0.1);  // ボクセルグリッドのサイズ(m)
    this->declare_parameter("input.visualize", true);       // 位置フィルタの可視化
    this->declare_parameter("input.local_coordinates", false); // ローカル座標系を使用するか
    this->declare_parameter("input.x_min", -20.);           // 位置フィルタの最小 x
    this->declare_parameter("input.x_max", 20.);            // 位置フィルタの最大 x
    this->declare_parameter("input.y_min", -20.);           // 位置フィルタの最小 y
    this->declare_parameter("input.y_max", 20.);            // 位置フィルタの最大 y
    this->declare_parameter("input.z_min", -0.5);           // 位置フィルタの最小 z
    this->declare_parameter("input.z_max", 3.0);            // 位置フィルタの最大 z
    this->declare_parameter("input.local.x_min", -100.0);
    this->declare_parameter("input.local.x_max", 100.0);
    this->declare_parameter("input.local.y_min", -100.0);
    this->declare_parameter("input.local.y_max", 100.0);
    this->declare_parameter("input.local.z_min", -100.0);
    this->declare_parameter("input.local.z_max", 100.0);

    // ROS2では declare_parameter 直後は param_cb が呼ばれないため、
    // 起動時点で使う値をここでクラスに同期しておく。
    cluster_classification_.human_enable = this->get_parameter("classify.human").as_bool();
    cluster_classification_.human_model_name = this->get_parameter("classify.human.model").as_string();
    cluster_classification_.car_enable = this->get_parameter("classify.car").as_bool();
    cluster_classification_.car_model_name = this->get_parameter("classify.car.model").as_string();
    cluster_classification_.threshold = this->get_parameter("classify.threshold").as_double();
    cluster_classification_.simple_reliability_human =
        this->get_parameter("classify.simple_reliability.human").as_double();
    cluster_classification_.simple_reliability_car =
        this->get_parameter("classify.simple_reliability.car").as_double();
    cluster_classification_.simple_reliability_unknown =
        this->get_parameter("classify.simple_reliability.unknown").as_double();
    cluster_classification_.human_yaw_rate_threshold =
        this->get_parameter("cluster.human.yaw_rate_threshold").as_double();
    cluster_classification_.human_yaw_hold_seconds =
        this->get_parameter("cluster.human.yaw_hold_seconds").as_double();
    cluster_classification_.human_scale_threshold =
        this->get_parameter("cluster.human.scale_threshold").as_double();
    cluster_classification_.safe_terrain_merge_distance_threshold =
        this->get_parameter("cluster.safe_terrain.merge_distance_max").as_double();
    cluster_classification_.safe_terrain_merge_normal_angle_threshold_deg =
        this->get_parameter("cluster.safe_terrain.merge_normal_angle_deg").as_double();
    cluster_classification_.non_safe_merge_distance_threshold =
        this->get_parameter("cluster.non_safe.merge_distance_max").as_double();

    // 重要:
    // ROS2のyamlオーバーライド値はdeclare_parameterで取得できるが、
    // そのままではCライブラリ側(gng_setParameter)へ自動で反映されない。
    // 起動時に現在の全パラメータを一度param_cbへ流し、GNGコア設定を同期する。
    {
        const auto listed_params = this->list_parameters({}, 10);
        std::vector<rclcpp::Parameter> startup_params;
        startup_params.reserve(listed_params.names.size());
        for (const auto &name : listed_params.names) {
            startup_params.emplace_back(this->get_parameter(name));
        }
        (void)param_cb(startup_params);
    }

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

    downsampling_.init(unknown_pcl_pub_, human_pcl_pub_);
    cluster_classification_.init(this->get_logger());
    filter_.init(filter_pub_);
    initialized_ = true;
}

AiSGNG::~AiSGNG() {

}

rcl_interfaces::msg::SetParametersResult AiSGNG::param_cb(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.reason = "success";
    result.successful = true;
    for (auto &p : params) {
        float value = 0.0f;
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

        // 入力範囲の可視化
        if (name == "input.x_min") {
            filter_.x_min = p.as_double();
        } else if (name == "input.x_max") {
            filter_.x_max = p.as_double();
        } else if (name == "input.y_min") {
            filter_.y_min = p.as_double();
        } else if (name == "input.y_max") {
            filter_.y_max = p.as_double();
        } else if (name == "input.z_min") {
            filter_.z_min = p.as_double();
        } else if (name == "input.z_max") {
            filter_.z_max = p.as_double();
        }

        if(name == "classify.human"){
            cluster_classification_.human_enable = p.as_bool();
        } else if(name == "classify.human.model"){
            cluster_classification_.human_model_name = p.as_string();
        } else if(name == "classify.car"){
            cluster_classification_.car_enable = p.as_bool();
        } else if(name == "classify.car.model"){
            cluster_classification_.car_model_name = p.as_string();
        } else if(name == "classify.threshold"){
            cluster_classification_.threshold = p.as_double();
        } else if(name == "classify.simple_reliability.human"){
            cluster_classification_.simple_reliability_human = p.as_double();
        } else if(name == "classify.simple_reliability.car"){
            cluster_classification_.simple_reliability_car = p.as_double();
        } else if(name == "classify.simple_reliability.unknown"){
            cluster_classification_.simple_reliability_unknown = p.as_double();
        } else if(name == "cluster.human.yaw_rate_threshold"){
            cluster_classification_.human_yaw_rate_threshold = p.as_double();
        } else if(name == "cluster.human.yaw_hold_seconds"){
            cluster_classification_.human_yaw_hold_seconds = p.as_double();
        } else if(name == "cluster.human.scale_threshold"){
            cluster_classification_.human_scale_threshold = p.as_double();
        } else if(name == "cluster.safe_terrain.merge_distance_max"){
            cluster_classification_.safe_terrain_merge_distance_threshold = p.as_double();
        } else if(name == "cluster.safe_terrain.merge_normal_angle_deg"){
            cluster_classification_.safe_terrain_merge_normal_angle_threshold_deg = p.as_double();
        } else if(name == "cluster.non_safe.merge_distance_max"){
            cluster_classification_.non_safe_merge_distance_threshold = p.as_double();
        } else if (name == "input.visualize"){
            filter_.enable = p.as_bool();
        } else if (name == "input.base_frame_id"){
            base_frame_id_ = p.as_string();
        } else if (name == "ds.transformed") {
            downsampling_.transformed = p.as_bool();
        } else if (name == "ds.all.num_max" && p.as_int() >= 0){
            downsampling_.all_num_max = p.as_int();
        } else if (name == "ds.unknown.num_max" && p.as_int() >= 0) {
            downsampling_.unknown_num_max = p.as_int();
        } else if (name == "ds.human.num_max" && p.as_int() >= 0) {
            downsampling_.human_num_max = p.as_int();
        } else if(flt_array.size() > 0){
            bool sucess = true;
            for(int i = 0; i < flt_array.size(); ++i){
                if(!gng_setParameter(name.c_str(), i, flt_array[i])){
                    sucess = false;
                }
            }
            if (sucess) {
                RCLCPP_INFO(this->get_logger(), "param %s [%.2f, %.2f, %.2f, %.2f]", name.c_str(), flt_array[0], flt_array[1], flt_array[2], flt_array[3]);
            }
            continue;
        } else if (!gng_setParameter(name.c_str(), 0, value)) {
            // パラメーター名エラー
            continue;
        }

        if (type == rclcpp::ParameterType::PARAMETER_DOUBLE){
            RCLCPP_INFO(this->get_logger(), "param %s %.2f", name.c_str(), value);
        } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER || type == rclcpp::ParameterType::PARAMETER_BOOL){
            RCLCPP_INFO(this->get_logger(), "param %s %d", name.c_str(), (int)value);
        } else if (type == rclcpp::ParameterType::PARAMETER_STRING){
            RCLCPP_INFO(this->get_logger(), "param %s %s", name.c_str(), p.as_string().c_str());
        }
    }
    return result;
}

void AiSGNG::pcl_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 初期化されていない
    if(!initialized_){
        return;
    }

    auto start = std::chrono::steady_clock::now();

    // 出力のヘッダー
    std_msgs::msg::Header header;
    header.frame_id = base_frame_id_.empty() ? msg->header.frame_id : base_frame_id_;
    header.stamp = msg->header.stamp;

    // GNGに点群を入力
    auto lidar_config = getBase2LidarFrame(msg);
    gng_setPointCloud(msg->data.data(), msg->width * msg->height, &lidar_config);

    // GNGの実行
    gng_exec();

    // GNGの結果を取得
    uint32_t label_num = 0, transformed_pcl_num = 0;
    auto map = gng_getTopologicalMap();// トポロジカルマップ
    auto label = gng_getDownSampling(&label_num); // ダウンサンプリング時のラベル
    auto transformed_pcl = gng_getAffineTransformedInputPointCloud(&transformed_pcl_num); // アフィン変換後の点群

    // トポロジカルマップをROS2メッセージに変換
    auto map_msg = makeTopologicalMapMsg(map, header);

    // アフィン変換後の点群をROS2メッセージに変換
    auto transformed_msg = makePointCloud2Msg(msg, transformed_pcl, transformed_pcl_num, header);

    // ここでクラスタ後処理を実施する。
    // 1) SAFE_TERRAIN の幾何統合（距離・法線角度）
    // 2) UNKNOWN/HUMAN/CAR の簡易分類
    std::vector<uint32_t> cluster_ids, cluster_age;
    std::vector<uint8_t> cluster_labels;
    cluster_classification_.classify(map_msg, cluster_ids, cluster_age, cluster_labels);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "clusters before=%u after=%zu non_safe_merge_distance=%.2f safe_merge_distance=%.2f",
        map.cluster_num,
        map_msg->clusters.size(),
        cluster_classification_.non_safe_merge_distance_threshold,
        cluster_classification_.safe_terrain_merge_distance_threshold);
    // GNGにフィードバック
    gng_setInferredClusterLabels(cluster_ids.data(), cluster_age.data(), cluster_labels.data(), cluster_ids.size());

    // トポロジカルマップをPublish
    topological_map_pub_->publish(std::move(map_msg));

    // ダウンサンプリングした点群をPublish
    if(downsampling_.transformed){
        downsampling_.publish<std::unique_ptr<sensor_msgs::msg::PointCloud2>>(transformed_msg, label, label_num, header); // 変換後の点群をPublish
    }else{
        downsampling_.publish<sensor_msgs::msg::PointCloud2::SharedPtr>(msg, label, label_num, msg->header); // 変換前の点群をPublish
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

std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> AiSGNG::makeTopologicalMapMsg(
    const TopologicalMap &map,
    const std_msgs::msg::Header &header) {
    auto topological_map_msg = std::make_unique<ais_gng_msgs::msg::TopologicalMap>();
    topological_map_msg->header = header;
    topological_map_msg->nodes.reserve(map.node_num);
    for (int i = 0; i < map.node_num; ++i) {
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
        node_msg.age = node.age;
        if (node.inpcl_num > 0) {
            node_msg.inpcl_ids.resize(node.inpcl_num);
            node_msg.inpcl_ids.assign(node.inpcl_ids, node.inpcl_ids + node.inpcl_num);
        }
        topological_map_msg->nodes.emplace_back(node_msg);
    }
    topological_map_msg->edges.resize(map.edge_num);
    topological_map_msg->edges.assign(map.edges, map.edges + map.edge_num);
    topological_map_msg->clusters.reserve(map.cluster_num);
    for (int i = 0; i < map.cluster_num; ++i) {
        ais_gng_msgs::msg::TopologicalCluster cluster_msg;
        auto &cluster = map.clusters[i];
        cluster_msg.id = cluster.id;
        cluster_msg.label = cluster.label;
        cluster_msg.label_inferred = cluster.label_inferred;
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
        cluster_msg.age = cluster.age;
        cluster_msg.reliability = -1;
        cluster_msg.match = cluster.match;
        cluster_msg.velocity.x = cluster.velocity.x;
        cluster_msg.velocity.y = cluster.velocity.y;
        cluster_msg.velocity.z = cluster.velocity.z;
        cluster_msg.nodes.resize(cluster.node_num);
        cluster_msg.nodes.assign(cluster.nodes, cluster.nodes + cluster.node_num);
        topological_map_msg->clusters.emplace_back(cluster_msg);
    }

    return topological_map_msg;
}

LiDAR_Config AiSGNG::getBase2LidarFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

std::unique_ptr<sensor_msgs::msg::PointCloud2> AiSGNG::makePointCloud2Msg(
        const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
        const float *transformed_pcl,
        const uint32_t transformed_pcl_num,
        const std_msgs::msg::Header &header) {
    auto pcl2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl2_msg->header = header;
    pcl2_msg->height = 1;
    pcl2_msg->width = transformed_pcl_num;
    pcl2_msg->is_dense = false;
    pcl2_msg->is_bigendian = false;
    pcl2_msg->point_step = msg->point_step;
    pcl2_msg->row_step = pcl2_msg->point_step * pcl2_msg->width;
    pcl2_msg->fields = msg->fields;
    pcl2_msg->data.resize(pcl2_msg->row_step * pcl2_msg->height);

    std::copy(msg->data.begin(), msg->data.begin() + msg->point_step * transformed_pcl_num, pcl2_msg->data.begin());


    for(uint32_t i = 0; i < transformed_pcl_num; ++i) {
        std::copy(
            (uint8_t*)transformed_pcl + i * 3 * sizeof(float),
            (uint8_t*)transformed_pcl + (i + 1) * 3 * sizeof(float),
            pcl2_msg->data.begin() + i * msg->point_step);
    }

    return pcl2_msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(fuzzrobo::AiSGNG)
