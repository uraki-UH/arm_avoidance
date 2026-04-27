#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace robot_sim {
namespace self_recognition {

class RobotDescriptionPlayer : public rclcpp::Node {
public:
  explicit RobotDescriptionPlayer(const rclcpp::NodeOptions & options);

private:
  bool loadDescription(std::string &out_text) const;
  void tick();
  void republishLastDescription();

  std::string description_file_;
  std::string resource_root_dir_;
  std::string mesh_root_dir_;
  std::string topic_name_;
  int poll_ms_ = 1000;
  int republish_ms_ = 1000;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::filesystem::file_time_type last_mtime_{};
  std::string last_description_;
  bool have_last_mtime_ = false;
  bool have_last_description_ = false;
  bool missing_warned_ = false;
};

} // namespace self_recognition
} // namespace robot_sim
