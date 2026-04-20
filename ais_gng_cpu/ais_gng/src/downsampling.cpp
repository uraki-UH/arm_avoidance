#include <ais_gng/downsampling.hpp>

Downsampling::Downsampling() {
}

Downsampling::~Downsampling() {
}

void Downsampling::init(
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub, 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_pub
    ) {
    unknown_pub_ = unknown_pub;
    human_pub_ = human_pub;
}