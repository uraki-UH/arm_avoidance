#include <ais_gng/visualizar.hpp>

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
    this->declare_parameter("marker_size", 0.025);
    this->declare_parameter("node_alpha", 0.5);
    this->declare_parameter("clusted_node_alpha", 1.0);
    this->declare_parameter("edge_alpha", 1.0);
    this->declare_parameter("cluster_alpha", 0.5);
    this->declare_parameter("normal_alpha", 0.5);
    this->declare_parameter("text_alpha", 0.5);
    this->declare_parameter("constant_alpha", 0.1);
    this->declare_parameter("cluster_path_len", 50);
    double marker_size = this->get_parameter("marker_size").as_double();
    node_alpha_ = this->get_parameter("node_alpha").as_double();
    clusted_node_alpha_ = this->get_parameter("clusted_node_alpha").as_double();
    edge_alpha_ = this->get_parameter("edge_alpha").as_double();
    cluster_alpha_ = this->get_parameter("cluster_alpha").as_double();
    normal_alpha_ = this->get_parameter("normal_alpha").as_double();
    text_alpha_ = this->get_parameter("text_alpha").as_double();
    constant_alpha_ = this->get_parameter("constant_alpha").as_double();
    cluster_path_len_ = this->get_parameter("cluster_path_len").as_int();

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 30);

    topological_map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
        "topological_map", 10, std::bind(&Visualizar::topological_map_cb, this, _1));

    initMarkerArrayMsg(marker_size);
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
    ma_.markers[ma_edge_id_].points.reserve(msg->edges.size());
    for (size_t i=0; i < msg->edges.size(); i+=2) {
        uint16_t edge_id1 = msg->edges[i];
        uint16_t edge_id2 = msg->edges[i+1];
        // 範囲外チェック
        if (node_size <= edge_id1 || node_size <= edge_id2)
            continue;
        // クラスタに含まれないノードがある場合は描画しない
        if(node_alpha_ == 0 && 
            (!nodes_is_clusterd[edge_id1] ||
             !nodes_is_clusterd[edge_id2]))
            continue;
        // クラスタに含まれるノードがある場合は描画しない
        if(clusted_node_alpha_ == 0 && 
            (nodes_is_clusterd[edge_id1] ||
             nodes_is_clusterd[edge_id2]))
            continue;
        auto &node1 = msg->nodes[edge_id1];
        auto &node2 = msg->nodes[edge_id2];
        node_p.x = node1.pos.x;
        node_p.y = node1.pos.y;
        node_p.z = node1.pos.z;
        ma_.markers[ma_edge_id_].points.emplace_back(node_p);
        node_p.x = node2.pos.x;
        node_p.y = node2.pos.y;
        node_p.z = node2.pos.z;
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
    i = ma_cluster_id_;
    for (auto &cluster : msg->clusters) {
        cluster_marker.id = i++;
        cluster_marker.pose.position.x = cluster.pos.x;
        cluster_marker.pose.position.y = cluster.pos.y;
        cluster_marker.pose.position.z = cluster.pos.z;
        cluster_marker.pose.orientation = cluster.quat;
        uint8_t label = cluster.label;
        if(label == ais_gng_msgs::msg::TopologicalMap::HUMAN){
            cluster_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            cluster_marker.scale.x = std::max(cluster.scale.x, cluster.scale.y);
            cluster_marker.scale.y = cluster_marker.scale.x;
        }else{
            cluster_marker.type = visualization_msgs::msg::Marker::CUBE;
            cluster_marker.scale.x = cluster.scale.x;
            cluster_marker.scale.y = cluster.scale.y;
        }
        cluster_marker.scale.z = cluster.scale.z;
        cluster_marker.color.r = label_color_table_[label][0];
        cluster_marker.color.g = label_color_table_[label][1];
        cluster_marker.color.b = label_color_table_[label][2];
        cluster_marker.color.a = (label != ais_gng_msgs::msg::TopologicalMap::HUMAN && label != ais_gng_msgs::msg::TopologicalMap::CAR) ? constant_alpha_ : cluster_alpha_;
        cluster_marker.ns = label_table_[label];
        ma_.markers.emplace_back(cluster_marker);
        text_marker.id = i++;
        text_marker.pose.position.x = cluster.pos.x;
        text_marker.pose.position.y = cluster.pos.y;
        text_marker.pose.position.z = cluster.pos.z + cluster.scale.z / 2 + 0.1;
        text_marker.ns = label_table_[label];
        char buff[100];
        uint32_t cluster_age = (msg->frame_number - cluster.frame);
        if (label == ais_gng_msgs::msg::TopologicalMap::HUMAN) {
            float velocity = std::sqrt(cluster.velocity.x*cluster.velocity.x + cluster.velocity.y*cluster.velocity.y + cluster.velocity.z*cluster.velocity.z);
            snprintf(buff, sizeof(buff), "ID:%d,A:%d,M:%.2f,R:%.2f,V:%.2fkm/h", cluster.id, cluster_age, cluster.match, cluster.label_reliability, velocity * 3.6);
        }
        else
            snprintf(buff, sizeof(buff), "ID:%d,A:%d,M:%.2f,R:%.2f", cluster.id, cluster_age, cluster.match, cluster.label_reliability);
        text_marker.text = buff;
        ma_.markers.emplace_back(text_marker);
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