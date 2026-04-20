#include <ais_gng/visualizar.hpp>
#include <algorithm>
#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

std::array<float, 3> Visualizar::clusterColorFromId(uint32_t cluster_id) const {
    // クラスタIDから毎フレーム同じ色を得るための簡易ハッシュ
    const uint32_t hash = cluster_id * 2654435761u;
    const float r = 0.3f + 0.8f * static_cast<float>((hash >> 0) & 0xFFu) / 255.0f;
    const float g = 0.3f + 0.8f * static_cast<float>((hash >> 8) & 0xFFu) / 255.0f;
    const float b = 0.3f + 0.8f * static_cast<float>((hash >> 16) & 0xFFu) / 255.0f;
    return {r, g, b};
}

float Visualizar::clusterNodeScaleFromAge(int age) const {
    if (!cluster_node_scale_by_age_) {
        return cluster_node_scale_;
    }
    const int clamped_age = std::max(age, 0);
    const float scaled = cluster_node_scale_ * (1.0f + static_cast<float>(clamped_age) * cluster_node_scale_gain_per_age_);
    const float max_scale = std::max(cluster_node_scale_max_, cluster_node_scale_);
    return std::min(scaled, max_scale);
}

bool Visualizar::isGroundNode(const ais_gng_msgs::msg::TopologicalNode &node) const {
    if (node.label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
        return true;
    }
    const float nx = node.normal.x;
    const float ny = node.normal.y;
    const float nz = node.normal.z;
    const float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm < 1.0e-6f) {
        return false;
    }
    const float cos_threshold = std::cos(ground_normal_angle_deg_ * kPi / 180.0f);
    return std::abs(nz) / norm >= cos_threshold;
}

geometry_msgs::msg::Quaternion quaternionFromXAxisTo(const geometry_msgs::msg::Point32& to)
{
    geometry_msgs::msg::Point32 v_from;
    v_from.x = 1.0; v_from.y = 0.0; v_from.z = 0.0;
    geometry_msgs::msg::Point32 v_to = to;
    geometry_msgs::msg::Quaternion q; // 計算結果のクォータニオン
    // 数値計算用の定数
    const double EPSILON_PURE = 1e-6;

    /* Norm */
    double n = std::sqrt(v_to.x * v_to.x + v_to.y * v_to.y + v_to.z * v_to.z);
    if (n < EPSILON_PURE) {
        q.w = 1.0; // 単位クォータニオンを返す
        return q;
    }
    /* 正規化 */
    v_to.x /= n; v_to.y /= n; v_to.z /= n;
    /* 内積 */
    double cos_theta = v_from.x * v_to.x + v_from.y * v_to.y + v_from.z * v_to.z;
    if (cos_theta > 1.0 - EPSILON_PURE) {
        q.w = 1.0; q.x = 0.0; q.y = 0.0; q.z = 0.0;
    } 
    else if (cos_theta < -1.0 + EPSILON_PURE) {
        q.w = 0.0; q.x = 0.0; q.y = 0.0; q.z = 1.0;
    }
    else {
        geometry_msgs::msg::Point32 axis;
        /* 外積 */
        axis.x = v_from.y * v_to.z - v_from.z * v_to.y;
        axis.y = v_from.z * v_to.x - v_from.x * v_to.z;
        axis.z = v_from.x * v_to.y - v_from.y * v_to.x;
        double s = std::sqrt((1.0 + cos_theta) * 2.0);
        double inv_s = 1.0 / s;
        
        q.w = s * 0.5;
        q.x = axis.x * inv_s;
        q.y = axis.y * inv_s;
        q.z = axis.z * inv_s;
    }
    return q;
}

Visualizar::Visualizar() : Node("visualizar") {
    // RViz描画の調整用パラメータ
    this->declare_parameter("visual.cluster_nodes.enable", true);
    this->declare_parameter("visual.cluster_nodes.scale", 0.08);
    this->declare_parameter("visual.cluster_nodes.alpha", 0.9);
    this->declare_parameter("visual.cluster_nodes.scale_by_age.enable", false);
    this->declare_parameter("visual.cluster_nodes.scale_by_age.gain_per_age", static_cast<double>(cluster_node_scale_gain_per_age_));
    this->declare_parameter("visual.cluster_nodes.scale_by_age.max_scale", static_cast<double>(cluster_node_scale_max_));
    this->declare_parameter("visual.ground_normal_angle_deg", 30.0);
    this->declare_parameter("visual.topological_map.node.width", static_cast<double>(node_width_));
    this->declare_parameter("visual.topological_map.edge.width", static_cast<double>(edge_width_));
    this->declare_parameter("visual.topological_map.node.opacity", static_cast<double>(node_alpha_));
    this->declare_parameter("visual.topological_map.edge.opacity", static_cast<double>(edge_alpha_));
    draw_cluster_nodes_ = this->get_parameter("visual.cluster_nodes.enable").as_bool();
    cluster_node_scale_ = this->get_parameter("visual.cluster_nodes.scale").as_double();
    cluster_node_alpha_ = this->get_parameter("visual.cluster_nodes.alpha").as_double();
    cluster_node_scale_by_age_ = this->get_parameter("visual.cluster_nodes.scale_by_age.enable").as_bool();
    cluster_node_scale_gain_per_age_ = this->get_parameter("visual.cluster_nodes.scale_by_age.gain_per_age").as_double();
    cluster_node_scale_max_ = this->get_parameter("visual.cluster_nodes.scale_by_age.max_scale").as_double();
    ground_normal_angle_deg_ = this->get_parameter("visual.ground_normal_angle_deg").as_double();
    node_width_ = static_cast<float>(this->get_parameter("visual.topological_map.node.width").as_double());
    edge_width_ = static_cast<float>(this->get_parameter("visual.topological_map.edge.width").as_double());
    node_alpha_ = static_cast<float>(this->get_parameter("visual.topological_map.node.opacity").as_double());
    edge_alpha_ = static_cast<float>(this->get_parameter("visual.topological_map.edge.opacity").as_double());

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 30);

    topological_map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        "topological_map", 10, std::bind(&Visualizar::topological_map_cb, this, _1));

    initMarkerArrayMsg(node_width_);
    RCLCPP_INFO(this->get_logger(), "start");
}

Visualizar::~Visualizar() {}

void Visualizar::topological_map_cb(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
    geometry_msgs::msg::Point node_p, p;
    visualization_msgs::msg::Marker cluster_marker, text_marker, path_marker, cluster_node_marker;
    visualization_msgs::msg::Marker vel_marker;
    int offset, i, node_size, color_size;

    node_size = msg->nodes.size();
    std::vector<bool> nodes_is_clusterd(node_size, false);
    for (auto &cluster : msg->clusters) {
        for (auto &node_id : cluster.nodes) {
            if (static_cast<size_t>(node_id) < nodes_is_clusterd.size()) {
                nodes_is_clusterd[node_id] = true;
            }
        }
    }

    ma_.markers.resize(ma_cluster_id_);
    for(auto &m:ma_.markers)
        m.points.clear();
    color_size = label_color_table_.size();
    i = 0;
    for (auto &node : msg->nodes) {
        node_p.x = node.pos.x;
        node_p.y = node.pos.y;
        node_p.z = node.pos.z;
        const bool node_is_clusterd = nodes_is_clusterd[i++];
        const bool is_ground = isGroundNode(node);
        if (is_ground) {
            // 地面ノードはラベルに関係なく常に白で描画する。
            const int marker_id = node_is_clusterd ? ma_ground_clusted_node_id_ : ma_ground_node_id_;
            ma_.markers[marker_id].points.emplace_back(node_p);
        } else {
            offset = node_is_clusterd ? color_size : 0;
            /* ノードの描画 */
            ma_.markers[ma_node_id_ + node.label + offset].points.emplace_back(node_p);
        }
        /* 法線ベクトルの描画 */
        double l = node.rho;
        ma_.markers[ma_normal_id_ + node.label].points.emplace_back(node_p);
        p.x = node_p.x + node.normal.x * l;
        p.y = node_p.y + node.normal.y * l;
        p.z = node_p.z + node.normal.z * l;
        ma_.markers[ma_normal_id_ + node.label].points.emplace_back(p);
    }
    /* エッジの描画 */
    ma_.markers[ma_edge_id_].points.reserve(msg->edges.size());
    for (auto &edge_id : msg->edges) {
        if (node_size <= edge_id)
            continue;
        node_p.x = msg->nodes[edge_id].pos.x;
        node_p.y = msg->nodes[edge_id].pos.y;
        node_p.z = msg->nodes[edge_id].pos.z;
        ma_.markers[ma_edge_id_].points.emplace_back(node_p);
    }
    /*クラスタの描画*/
    ma_.markers.reserve(ma_.markers.size() + msg->clusters.size()*2);
    cluster_marker.action = visualization_msgs::msg::Marker::ADD;
    /*テキストの描画*/
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = text_alpha_;
    text_marker.scale.z = 0.1;
    cluster_node_marker.type = cluster_node_scale_by_age_
        ? visualization_msgs::msg::Marker::SPHERE
        : visualization_msgs::msg::Marker::SPHERE_LIST;
    cluster_node_marker.action = visualization_msgs::msg::Marker::ADD;
    cluster_node_marker.ns = "cluster_nodes";
    cluster_node_marker.pose.orientation.w = 1.0;
    cluster_node_marker.scale.x = clusterNodeScaleFromAge(0);
    cluster_node_marker.scale.y = clusterNodeScaleFromAge(0);
    cluster_node_marker.scale.z = clusterNodeScaleFromAge(0);
    cluster_node_marker.color.a = cluster_node_alpha_;
    i = ma_cluster_id_;
    for (auto &cluster : msg->clusters) {
        uint8_t label = cluster.label;
        if (label != ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN) {
            cluster_marker.id = i++;
            cluster_marker.pose.position.x = cluster.pos.x;
            cluster_marker.pose.position.y = cluster.pos.y;
            cluster_marker.pose.position.z = cluster.pos.z;
            cluster_marker.pose.orientation = cluster.quat;
            cluster_marker.type = visualization_msgs::msg::Marker::CUBE;
            cluster_marker.scale.x = cluster.scale.x;
            cluster_marker.scale.y = cluster.scale.y;
            cluster_marker.scale.z = cluster.scale.z;
            cluster_marker.color.r = label_color_table_[label][0];
            cluster_marker.color.g = label_color_table_[label][1];
            cluster_marker.color.b = label_color_table_[label][2];
            cluster_marker.color.a = (label != ais_gng_msgs::msg::TopologicalMap::HUMAN && label != ais_gng_msgs::msg::TopologicalMap::CAR) ? constant_alpha_ : cluster_alpha_;
            cluster_marker.ns = label_table_[label];
            ma_.markers.emplace_back(cluster_marker);
        }
        text_marker.id = i++;
        text_marker.pose.position.x = cluster.pos.x;
        text_marker.pose.position.y = cluster.pos.y;
        text_marker.pose.position.z = cluster.pos.z + cluster.scale.z / 2 + 0.1;
        text_marker.ns = label_table_[label];
        char buff[100];
        // if (label == ais_gng_msgs::msg::TopologicalMap::HUMAN) {
            float velocity = std::sqrt(cluster.velocity.x*cluster.velocity.x + cluster.velocity.y*cluster.velocity.y) * 3.6;
            snprintf(buff, sizeof(buff), "ID:%d,A:%d,M:%.2f,R:%.2f,V:%.2fkm/h", cluster.id, cluster.age, cluster.match, cluster.reliability, velocity);
        // }
        // else
            // snprintf(buff, sizeof(buff), "ID:%d,A:%d,M:%.2f,R:%.2f", cluster.id, cluster.age, cluster.match, cluster.reliability);
        text_marker.text = buff;
        ma_.markers.emplace_back(text_marker);
        // 実際に同一クラスタとして扱ったノード集合を、クラスタIDごとの色で描画する。
        if (draw_cluster_nodes_) {
            const auto rgb = clusterColorFromId(cluster.id);
            cluster_node_marker.color.r = rgb[0];
            cluster_node_marker.color.g = rgb[1];
            cluster_node_marker.color.b = rgb[2];
            if (cluster_node_scale_by_age_) {
                for (const auto node_id : cluster.nodes) {
                    if (static_cast<size_t>(node_id) >= msg->nodes.size())
                        continue;
                    const auto &cluster_node = msg->nodes[node_id];
                    if (isGroundNode(cluster_node)) {
                        // 地面ノードは白描画に統一するため、クラスタ色マーカーには入れない。
                        continue;
                    }
                    cluster_node_marker.id = i++;
                    cluster_node_marker.points.clear();
                    cluster_node_marker.pose.position.x = cluster_node.pos.x;
                    cluster_node_marker.pose.position.y = cluster_node.pos.y;
                    cluster_node_marker.pose.position.z = cluster_node.pos.z;
                    const float scale = clusterNodeScaleFromAge(cluster_node.age);
                    cluster_node_marker.scale.x = scale;
                    cluster_node_marker.scale.y = scale;
                    cluster_node_marker.scale.z = scale;
                    ma_.markers.emplace_back(cluster_node_marker);
                }
            } else {
                cluster_node_marker.id = i++;
                cluster_node_marker.points.clear();
                cluster_node_marker.points.reserve(cluster.nodes.size());
                for (const auto node_id : cluster.nodes) {
                    if (static_cast<size_t>(node_id) >= msg->nodes.size())
                        continue;
                    if (isGroundNode(msg->nodes[node_id])) {
                        // 地面ノードは白描画に統一するため、クラスタ色マーカーには入れない。
                        continue;
                    }
                    geometry_msgs::msg::Point cluster_node_p;
                    cluster_node_p.x = msg->nodes[node_id].pos.x;
                    cluster_node_p.y = msg->nodes[node_id].pos.y;
                    cluster_node_p.z = msg->nodes[node_id].pos.z;
                    cluster_node_marker.points.emplace_back(cluster_node_p);
                }
                if (!cluster_node_marker.points.empty()) {
                    ma_.markers.emplace_back(cluster_node_marker);
                }
            }
        }
        /* Cluster Velocity */
        if (label == ais_gng_msgs::msg::TopologicalMap::HUMAN && 
            (cluster.velocity.x != 0) &&
            (cluster.velocity.y != 0) && 
            (cluster.velocity.z != 0)) {
            vel_marker.type = visualization_msgs::msg::Marker::ARROW;
            vel_marker.action = visualization_msgs::msg::Marker::ADD;
            vel_marker.ns = "cluster_vel";
            vel_marker.color.r = 1.0;
            vel_marker.color.g = 0.0;
            vel_marker.color.b = 0.0;
            vel_marker.color.a = 0.5;
            vel_marker.pose.position.x = cluster.pos.x;
            vel_marker.pose.position.y = cluster.pos.y;
            vel_marker.pose.position.z = cluster.pos.z;
            vel_marker.pose.orientation = quaternionFromXAxisTo(cluster.velocity);
            vel_marker.scale.x = std::sqrt(cluster.velocity.x*cluster.velocity.x + cluster.velocity.y*cluster.velocity.y + cluster.velocity.z*cluster.velocity.z);
            vel_marker.scale.y = vel_marker.scale.z = 0.1;
            vel_marker.id = i++;
            ma_.markers.emplace_back(vel_marker);
        }
    }
    updateClusterPath(msg);
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.ns = "cluster_path";
    path_marker.scale.x = 0.05;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 0.5;
    for(auto &cluster_path:cluster_paths_){
        path_marker.id = i++;
        path_marker.points = cluster_path.second.poses;
        ma_.markers.emplace_back(path_marker);
    }

    for (auto &m : ma_.markers) {
        m.header = msg->header;
    }
    marker_array_pub_->publish(ma_);
    RCLCPP_INFO(this->get_logger(), "node:%ld, cluster:%ld", msg->nodes.size(), msg->clusters.size());
}

void Visualizar::initMarkerArrayMsg(float scale) {
    int i = 0;
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    m.id = i++;
    ma_.markers.emplace_back(m);
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;

    /* エッジの描画 */
    ma_edge_id_ = i;
    m.id = i++;
    m.ns = "edges";
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.color.g = 1.0;
    m.color.a = edge_alpha_;
    m.scale.x = edge_width_;
    ma_.markers.emplace_back(m);

    /* ノードの描画 */
    ma_node_id_ = i;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.ns = "nodes";
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    for (auto &node_color : label_color_table_) {
        // marker
        m.id = i++;
        m.color.r = node_color[0];
        m.color.g = node_color[1];
        m.color.b = node_color[2];
        m.color.a = node_alpha_;
        ma_.markers.emplace_back(m);
    }
    for (auto &node_color : label_color_table_) {
        // marker
        m.id = i++;
        m.color.r = node_color[0];
        m.color.g = node_color[1];
        m.color.b = node_color[2];
        m.color.a = clusted_node_alpha_;
        ma_.markers.emplace_back(m);
    }

    // 地面ノード専用（常に白）
    ma_ground_node_id_ = i;
    m.id = i++;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = node_alpha_;
    ma_.markers.emplace_back(m);

    ma_ground_clusted_node_id_ = i;
    m.id = i++;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 1.0;
    m.color.a = clusted_node_alpha_;
    ma_.markers.emplace_back(m);

    /* 法線ベクトルの描画 */
    ma_normal_id_ = i;
    m.ns = "normal";
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.scale.x = 0.01;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    for (auto &node_color : label_color_table_) {
        m.id = i++;
        m.color.r = node_color[0];
        m.color.g = node_color[1];
        m.color.b = node_color[2];
        m.color.a = normal_alpha_;
        ma_.markers.emplace_back(m);
    }

    /* クラスタの描画*/
    ma_cluster_id_ = i;
}

void Visualizar::updateClusterPath(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg){
    for(auto &cluster_path:cluster_paths_)
        cluster_path.second.exist = false;
    for(auto &cluster:msg->clusters){
        if(cluster.label != ais_gng_msgs::msg::TopologicalMap::HUMAN)
            continue;
        auto it = cluster_paths_.find(cluster.id);
        geometry_msgs::msg::Point point;
        point.x = cluster.pos.x;
        point.y = cluster.pos.y;
        point.z = 0;
        if(it != cluster_paths_.end()){
            it->second.exist = true;
            if((int)it->second.poses.size() > cluster_path_len_)
                it->second.poses.erase(it->second.poses.begin());
            it->second.poses.emplace_back(point);
        }else{
            ClusterPath c;
            c.exist = true;
            c.poses.emplace_back(point);
            cluster_paths_.insert(std::make_pair(cluster.id, c));
        }
    }
    std::vector<uint32_t> remove_ids;
    for(auto &cluster_path:cluster_paths_){
        if(!cluster_path.second.exist)
            remove_ids.emplace_back(cluster_path.first);
    }
    for(auto &id:remove_ids){
        cluster_paths_.erase(id);
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualizar>());
    rclcpp::shutdown();
    return 0;
}
