#pragma once

#include <algorithm>
#include <random>

#include "geometry_msgs/msg/point.hpp"
#include "ais_gng/plugin/plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"

class VisualizeFilter : public Plugin {
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_pub_;

    visualization_msgs::msg::Marker msg_;

    float x_min_ = -20;
    float x_max_ = 20;
    float y_min_ = -20;
    float y_max_ = 20;
    float z_min_ = -20;
    float z_max_ = 20;
    bool enable_ = false;

   public:
    VisualizeFilter();
    ~VisualizeFilter();
    void init(rclcpp::Node *node) override;
    bool setParameter(const std::string &name, int index, double value) override;
    void publish(const std_msgs::msg::Header &header);

   private:
    void addPos(float x, float y, float z);
};