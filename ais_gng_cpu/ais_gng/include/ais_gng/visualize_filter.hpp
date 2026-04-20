#pragma once

#include <algorithm>
#include <random>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class VisualizeFilter {
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_pub_;

    visualization_msgs::msg::Marker msg_;

   public:
    // param
    float x_min = -20;
    float x_max = 20;
    float y_min = -20;
    float y_max = 20;
    float z_min = -20;
    float z_max = 20;
    bool enable = false;

    VisualizeFilter();
    ~VisualizeFilter();
    void init(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_pub);
    void publish(const std_msgs::msg::Header &header);

   private:
    void addPos(float x, float y, float z);
};