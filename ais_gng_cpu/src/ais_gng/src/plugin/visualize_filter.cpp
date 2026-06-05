#include <ais_gng/plugin/visualize_filter.hpp>

VisualizeFilter::VisualizeFilter() {
    msg_.ns = "filter";
    msg_.id = 0;
    msg_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg_.action = visualization_msgs::msg::Marker::MODIFY;
    msg_.scale.x = 0.1;
    msg_.color.r = 1.0;
    msg_.color.g = 0.0;
    msg_.color.b = 0.0;
    msg_.color.a = 1.0;
    msg_.lifetime = rclcpp::Duration(0.0, 0.0);
}

VisualizeFilter::~VisualizeFilter() {
}

void VisualizeFilter::init(rclcpp::Node *node) {
    visualize_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("filter", 10);
}


bool VisualizeFilter::setParameter(const std::string &name, int, double value) {
    if (name == "input.x_min") {
        x_min_ = static_cast<float>(value);
    } else if (name == "input.x_max") {
        x_max_ = static_cast<float>(value);
    } else if (name == "input.y_min") {
        y_min_ = static_cast<float>(value);
    } else if (name == "input.y_max") {
        y_max_ = static_cast<float>(value);
    } else if (name == "input.z_min") {
        z_min_ = static_cast<float>(value);
    } else if (name == "input.z_max") {
        z_max_ = static_cast<float>(value);
    } else if (name == "input.visualize") {
        enable_ = static_cast<bool>(value);
    } else {
        return false;
    }
    return true;
}

void VisualizeFilter::publish(const std_msgs::msg::Header &header) {
    if (!enable_) {
        return;  // 可視化が無効な場合は何もしない
    }

    msg_.header = header;
    msg_.points.clear();
    // 下
    addPos(x_min_, y_max_, z_max_);  // 1
    addPos(x_min_, y_min_, z_max_);  // 2
    addPos(x_min_, y_min_, z_min_);  // 3
    addPos(x_min_, y_max_, z_min_);  // 4
    addPos(x_min_, y_max_, z_max_);  // 5
    addPos(x_max_, y_max_, z_max_);  // 6
    addPos(x_max_, y_max_, z_min_);  // 7
    addPos(x_min_, y_max_, z_min_);  // 8
    addPos(x_min_, y_min_, z_min_);  // 9
    addPos(x_min_, y_min_, z_max_);  // 10
    addPos(x_min_, y_max_, z_max_);  // 11
    addPos(x_min_, y_max_, z_min_);  // 12
    addPos(x_max_, y_max_, z_min_);  // 13
    addPos(x_max_, y_max_, z_max_);  // 14
    addPos(x_max_, y_min_, z_max_);  // 15
    addPos(x_max_, y_min_, z_min_);  // 16
    addPos(x_max_, y_max_, z_min_);  // 17
    addPos(x_max_, y_min_, z_min_);  // 18
    addPos(x_min_, y_min_, z_min_);  // 19
    addPos(x_max_, y_min_, z_min_);  // 20
    addPos(x_max_, y_min_, z_max_);  // 21
    addPos(x_min_, y_min_, z_max_);  // 22
    addPos(x_max_, y_min_, z_max_);  // 23
    addPos(x_max_, y_max_, z_max_);  // 24
    addPos(x_min_, y_max_, z_max_);  // 25
    visualize_pub_->publish(msg_);
}

void VisualizeFilter::addPos(float x, float y, float z) {
    auto p = geometry_msgs::msg::Point();
    p.x = x;
    p.y = y;
    p.z = z;
    msg_.points.emplace_back(p);
}