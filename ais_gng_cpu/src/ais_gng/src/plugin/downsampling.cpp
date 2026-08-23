#include <ais_gng/plugin/downsampling.hpp>

Downsampling::Downsampling() {
}

Downsampling::~Downsampling() {
}

void Downsampling::init(rclcpp::Node *node) {
    unknown_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/unknown", 10);
    grasp_support_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/grasp_support", 10);
    human_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("downsampling/human", 10);
}
bool Downsampling::setParameter(const std::string &name, int, double value) {
    if (name == "ds.transformed") {
        transformed_ = static_cast<bool>(value);
    } else if (name == "ds.all.num_max" || name == "ds.grasp_support.num_max") {
        all_num_max_ = static_cast<int>(value);
    } else if (name == "ds.unknown.num_max" || name == "ds.grasp_support.unknown_num_max") {
        unknown_num_max_ = static_cast<int>(value);
    } else if (name == "ds.human.num_max") {
        human_num_max_ = static_cast<int>(value);
    } else {
        return false;
    }
    return true;
}
