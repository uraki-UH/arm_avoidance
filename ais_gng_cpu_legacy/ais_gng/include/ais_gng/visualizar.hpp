#pragma once

#include <array>
#include <vector>
#include <map>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef struct _cluster_path {
    bool exist;
    std::vector<geometry_msgs::msg::Point> poses;
} ClusterPath;

class Visualizar : public rclcpp::Node {
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_map_sub_;

    // ラベルの色設定（rgb）
    const std::vector<std::array<double, 3>> label_color_table_ = {
        {0.5, 0.5, 0.5}, {1.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0},  {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
    // ラベルの名称
    const std::vector<std::string> label_table_ = {
        "default", "safe", "wall", "unknown", "human", "car"};

    // トポロジカルマップの描画設定
    float node_width_ = 0.05;
    float edge_width_ = 0.01;
    // 透過度
    float node_alpha_ = 0.5;
    float clusted_node_alpha_ = 1.0;
    float edge_alpha_ = 1.0;
    float cluster_alpha_ = 0.5;
    float normal_alpha_ = 0.5;
    float text_alpha_ = 0.5;
    float constant_alpha_ = 0.1;
    // 実際のクラスタ分割結果をノード単位で表示する設定
    float cluster_node_alpha_ = 0.9;
    float cluster_node_scale_ = 0.08;
    bool cluster_node_scale_by_age_ = false;
    float cluster_node_scale_gain_per_age_ = 0.03;
    float cluster_node_scale_max_ = 0.30;
    bool draw_cluster_nodes_ = true;
    float ground_normal_angle_deg_ = 30.0;

    // クラスタの軌跡の数
    int cluster_path_len_ = 50;

    // id
    int ma_edge_id_;
    int ma_node_id_;
    int ma_ground_node_id_;
    int ma_ground_clusted_node_id_;
    int ma_normal_id_;
    int ma_cluster_id_;
    // メッセージ
    visualization_msgs::msg::MarkerArray ma_;

    // 人の追跡
    std::map<uint32_t, ClusterPath> cluster_paths_;

   public:
    Visualizar();
    ~Visualizar();

   private:
    void topological_map_cb(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg);
    void initMarkerArrayMsg(float scale);
    void updateClusterPath(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg);
    std::array<float, 3> clusterColorFromId(uint32_t cluster_id) const;
    float clusterNodeScaleFromAge(int age) const;
    bool isGroundNode(const ais_gng_msgs::msg::TopologicalNode &node) const;
};
