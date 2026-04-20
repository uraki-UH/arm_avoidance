#pragma once

#include <fuzzrobo/libgng/api.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"

#include "ais_gng_msgs/msg/topological_map.hpp"

class ClusterClassification{
   public:
    bool human_enable;
    std::string human_model_name;
    bool car_enable;
    std::string car_model_name;
    float threshold;
    // 簡易再分類で使う信頼度（低めの値を推奨）
    float simple_reliability_human;
    float simple_reliability_car;
    float simple_reliability_unknown;
    // yaw変化率(rad/s)がこの値以上ならHUMAN判定に使う
    float human_yaw_rate_threshold;
    // yaw閾値を超えた後、HUMAN判定を維持する時間[s]
    float human_yaw_hold_seconds;
    // HUMAN判定を通す最大スケール閾値[m]（0以下で無効）
    float human_scale_threshold;
    // SAFE_TERRAIN同士を統合するときの閾値
    float safe_terrain_merge_distance_threshold;
    float safe_terrain_merge_normal_angle_threshold_deg;
    // SAFE_TERRAIN以外を統合するときの距離閾値（0以下で無効）
    float non_safe_merge_distance_threshold;

    ClusterClassification();
    ~ClusterClassification();

    void init(const rclcpp::Logger &logger);

    void classify(std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map,
                                         std::vector<uint32_t> &cluster_ids,
                                         std::vector<uint32_t> &cluster_ages,
                                         std::vector<uint8_t> &cluster_labels);

   private:
    struct YawHistory {
        double yaw = 0.0;
        double stamp_sec = 0.0;
    };
    std::unordered_map<uint32_t, YawHistory> yaw_histories_;
    std::unordered_map<uint32_t, double> human_hold_until_sec_;

    void merge_safe_terrain_clusters(std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map);
    void merge_non_safe_clusters(std::unique_ptr<ais_gng_msgs::msg::TopologicalMap> &map);
};
