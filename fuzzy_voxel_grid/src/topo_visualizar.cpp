#include <fuzzy_voxel_grid/topo_visualizar.hpp>

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
    this->declare_parameter<std::string>("frozen_topological_map_topic", "/frozen_topological_map");
    auto topological_map_topic = this->get_parameter("frozen_topological_map_topic").as_string();
    
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fvg_marker", 30);

    topological_map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        topological_map_topic, 10, std::bind(&Visualizar::topological_map_cb, this, _1));

    initMarkerArrayMsg(0.025);
    RCLCPP_INFO(this->get_logger(), "start");
}

Visualizar::~Visualizar() {}

void Visualizar::topological_map_cb(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
    geometry_msgs::msg::Point node_p, p;
    visualization_msgs::msg::Marker cluster_marker, text_marker, path_marker;
    visualization_msgs::msg::Marker vel_marker;
    int offset, i, node_size, color_size;

    node_size = msg->nodes.size();
    std::vector<bool> nodes_is_clusterd(node_size, false);
    for (auto &cluster : msg->clusters)
        for (auto &node_id : cluster.nodes)
            nodes_is_clusterd[node_id] = true;

    ma_.markers.resize(ma_cluster_id_);
    for(auto &m:ma_.markers)
        m.points.clear();
    color_size = label_color_table_.size();
    i = 0;
    for (auto &node : msg->nodes) {
        node_p.x = node.pos.x;
        node_p.y = node.pos.y;
        node_p.z = node.pos.z;
        offset = nodes_is_clusterd[i++] ? color_size : 0;
        /* ノードの描画 */
        ma_.markers[ma_node_id_ + node.label + offset].points.emplace_back(node_p);
        /* 法線ベクトルの描画 */
        double l = node.rho;
        ma_.markers[ma_normal_id_ + node.label].points.emplace_back(node_p);
        p.x = node_p.x + node.normal.x * l;
        p.y = node_p.y + node.normal.y * l;
        p.z = node_p.z + node.normal.z * l;
        ma_.markers[ma_normal_id_ + node.label].points.emplace_back(p);
    }
    /* エッジの描画 */
    auto &edge_points = ma_.markers[ma_edge_id_].points;
    edge_points.reserve(msg->edges.size());
    for (size_t idx = 0; idx + 1 < msg->edges.size(); idx += 2) {
        const uint16_t a = msg->edges[idx];
        const uint16_t b = msg->edges[idx + 1];
        if (a >= node_size || b >= node_size) {
            continue;
        }
        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;
        p1.x = msg->nodes[a].pos.x;
        p1.y = msg->nodes[a].pos.y;
        p1.z = msg->nodes[a].pos.z;
        p2.x = msg->nodes[b].pos.x;
        p2.y = msg->nodes[b].pos.y;
        p2.z = msg->nodes[b].pos.z;
        edge_points.emplace_back(p1);
        edge_points.emplace_back(p2);
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
    m.scale.x = 0.01;
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

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Visualizar>());
    rclcpp::shutdown();
    return 0;
}
