#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace robot_sim {
namespace recognition {
class SelfRecognitionManager;
}

namespace self_recognition {

class SelfRecognitionVizNode : public rclcpp::Node {
public:
    explicit SelfRecognitionVizNode(const rclcpp::NodeOptions & options);

private:
    void publishViz();

    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    float voxel_size_f_ = 0.0f;
    std::string frame_id_ = "base_link";
    std::vector<double> current_joints_;
    std::mutex mutex_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace self_recognition
} // namespace robot_sim
