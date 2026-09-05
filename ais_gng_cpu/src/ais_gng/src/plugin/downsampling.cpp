#include <ais_gng/plugin/downsampling.hpp>

Downsampling::Downsampling() {
}

Downsampling::~Downsampling() {
}

void Downsampling::init(rclcpp::Node *node) {
    unknown_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/unknown", 10);
    human_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/human", 10);
}
bool Downsampling::setParameter(const std::string &name, int, double value) {
    if (name == "ds.transformed") {
        is_transformed_ = static_cast<bool>(value);
    } else if (name == "ds.all.num_max") {
        max_all_num_ = static_cast<int>(value);
    } else if (name == "ds.unknown.num_max") {
        max_unknown_num_ = static_cast<int>(value);
    } else if (name == "ds.human.num_max") {
        max_human_num_ = static_cast<int>(value);
    } else {
        return false;
    }
    return true;
}
