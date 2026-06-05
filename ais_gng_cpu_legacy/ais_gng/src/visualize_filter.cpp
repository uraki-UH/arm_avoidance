#include <ais_gng/visualize_filter.hpp>

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

void VisualizeFilter::init(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualize_pub) {
    visualize_pub_ = visualize_pub;
}

void VisualizeFilter::publish(const std_msgs::msg::Header &header) {
    if (!enable) {
        return;  // 可視化が無効な場合は何もしない
    }

    msg_.header = header;
    msg_.points.clear();
    // 下
    addPos(x_min, y_max, z_max);  // 1
    addPos(x_min, y_min, z_max);  // 2
    addPos(x_min, y_min, z_min);  // 3
    addPos(x_min, y_max, z_min);  // 4
    addPos(x_min, y_max, z_max);  // 5
    addPos(x_max, y_max, z_max);  // 6
    addPos(x_max, y_max, z_min);  // 7
    addPos(x_min, y_max, z_min);  // 8
    addPos(x_min, y_min, z_min);  // 9
    addPos(x_min, y_min, z_max);  // 10
    addPos(x_min, y_max, z_max);  // 11
    addPos(x_min, y_max, z_min);  // 12
    addPos(x_max, y_max, z_min);  // 13
    addPos(x_max, y_max, z_max);  // 14
    addPos(x_max, y_min, z_max);  // 15
    addPos(x_max, y_min, z_min);  // 16
    addPos(x_max, y_max, z_min);  // 17
    addPos(x_max, y_min, z_min);  // 18
    addPos(x_min, y_min, z_min);  // 19
    addPos(x_max, y_min, z_min);  // 20
    addPos(x_max, y_min, z_max);  // 21
    addPos(x_min, y_min, z_max);  // 22
    addPos(x_max, y_min, z_max);  // 23
    addPos(x_max, y_max, z_max);  // 24
    addPos(x_min, y_max, z_max);  // 25
    visualize_pub_->publish(msg_);
}

void VisualizeFilter::addPos(float x, float y, float z) {
    auto p = geometry_msgs::msg::Point();
    p.x = x;
    p.y = y;
    p.z = z;
    msg_.points.emplace_back(p);
}