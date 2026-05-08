#ifndef GNG_VLUT_SYSTEM_SELF_RECOGNITION_FILTER_NODE_HPP_
#define GNG_VLUT_SYSTEM_SELF_RECOGNITION_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <memory>
#include <unordered_set>

namespace robot_sim::recognition {
    class SelfRecognitionManager;
}

namespace robot_sim::self_recognition {

class SelfRecognitionFilterNode : public rclcpp::Node {
public:
    explicit SelfRecognitionFilterNode(const rclcpp::NodeOptions & options);
    virtual ~SelfRecognitionFilterNode() = default;

private:
    void pcl_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void joint_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    std::unique_ptr<robot_sim::recognition::SelfRecognitionManager> recognition_manager_;
    
    std::mutex mutex_;
    std::vector<double> current_joints_;
    std::unordered_set<long> current_mask_vids_;
    
    float voxel_size_inv_;
    std::string frame_id_;
};

} // namespace robot_sim::self_recognition

#endif // GNG_VLUT_SYSTEM_SELF_RECOGNITION_FILTER_NODE_HPP_
