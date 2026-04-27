#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Geometry>

/**
 * @brief High-performance GNG Transformer Node.
 * Transforms a TopologicalMap from sensor frame to target frame (e.g.,
 * base_link) using Eigen matrix operations for maximum speed. Supports optional
 * workspace filtering.
 */
class GngTransformerNode : public rclcpp::Node {
public:
  GngTransformerNode() : Node("gng_transformer_node") {
    // Declare and get parameters
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<std::string>("input_topic", "/gng_map");
    this->declare_parameter<std::string>("output_topic",
                                         "/topological_map_transformed");
    this->declare_parameter<double>("filter_radius",
                                    -1.0); // Negative to disable
    this->declare_parameter<double>("filter_z_min", -1.0);
    this->declare_parameter<double>("filter_z_max", 1.0);

    target_frame_ = this->get_parameter("target_frame").as_string();
    filter_radius_ = this->get_parameter("filter_radius").as_double();
    filter_z_min_ = this->get_parameter("filter_z_min").as_double();
    filter_z_max_ = this->get_parameter("filter_z_max").as_double();

    auto input_topic = this->get_parameter("input_topic").as_string();
    auto output_topic = this->get_parameter("output_topic").as_string();

    // TF2 listener and buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscription
    subscription_ =
        this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
            input_topic, 10,
            std::bind(&GngTransformerNode::map_callback, this,
                      std::placeholders::_1));

    // Publisher
    publisher_ = this->create_publisher<ais_gng_msgs::msg::TopologicalMap>(
        output_topic, 10);

    RCLCPP_INFO(this->get_logger(),
                "GNG Transformer started: %s -> %s (Target: %s, Filter: %.2f)",
                input_topic.c_str(), output_topic.c_str(),
                target_frame_.c_str(), filter_radius_);
  }

private:
  void map_callback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg) {
    if (msg->nodes.empty())
      return;

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id,
                                           msg->header.stamp,
                                           rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Could not transform %s to %s: %s",
                           msg->header.frame_id.c_str(), target_frame_.c_str(),
                           ex.what());
      return;
    }

    Eigen::Isometry3d transform = tf2::transformToEigen(tf_msg);
    Eigen::Matrix3d rotation = transform.rotation();

    auto transformed_map =
        std::make_unique<ais_gng_msgs::msg::TopologicalMap>();
    transformed_map->header.stamp = msg->header.stamp;
    transformed_map->header.frame_id = target_frame_;

    std::unordered_map<uint16_t, uint16_t> old_to_new_id;
    const double r2 = filter_radius_ * filter_radius_;

    // 1. Transform and Filter Nodes
    for (const auto &node : msg->nodes) {
      Eigen::Vector3d pos_old(node.pos.x, node.pos.y, node.pos.z);
      Eigen::Vector3d pos_new = transform * pos_old;

      // Optional workspace filtering
      if (filter_radius_ > 0.0) {
        double dist2 = pos_new.x() * pos_new.x() + pos_new.y() * pos_new.y();
        if (dist2 > r2 || pos_new.z() < filter_z_min_ ||
            pos_new.z() > filter_z_max_) {
          continue;
        }
      }

      ais_gng_msgs::msg::TopologicalNode new_node = node;
      new_node.pos.x = pos_new.x();
      new_node.pos.y = pos_new.y();
      new_node.pos.z = pos_new.z();

      // Rotate normal vector
      Eigen::Vector3d normal_old(node.normal.x, node.normal.y, node.normal.z);
      Eigen::Vector3d normal_new = rotation * normal_old;
      new_node.normal.x = normal_new.x();
      new_node.normal.y = normal_new.y();
      new_node.normal.z = normal_new.z();

      old_to_new_id[node.id] = transformed_map->nodes.size();
      transformed_map->nodes.push_back(new_node);
    }

    // 2. Filter Edges
    // Edges are pairs of node indices.
    for (size_t i = 0; i + 1 < msg->edges.size(); i += 2) {
      uint16_t id1 = msg->edges[i];
      uint16_t id2 = msg->edges[i + 1];

      if (old_to_new_id.count(id1) && old_to_new_id.count(id2)) {
        transformed_map->edges.push_back(old_to_new_id[id1]);
        transformed_map->edges.push_back(old_to_new_id[id2]);
      }
    }

    // 3. Keep clusters (optional, depends on implementation)
    transformed_map->clusters = msg->clusters;

    publisher_->publish(std::move(transformed_map));
  }

  std::string target_frame_;
  double filter_radius_;
  double filter_z_min_;
  double filter_z_max_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr
      subscription_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GngTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
