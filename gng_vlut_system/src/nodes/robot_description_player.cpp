#include "gng_vlut_system/self_recognition/robot_description_player.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "common/resource_utils.hpp"

namespace robot_sim {
namespace self_recognition {

RobotDescriptionPlayer::RobotDescriptionPlayer(const rclcpp::NodeOptions & options)
: Node("robot_description_player", options) {
  const std::string pkg_share = ament_index_cpp::get_package_share_directory("gng_vlut_system");

  description_file_ = declare_parameter<std::string>(
      "robot_description_file", pkg_share + "/urdf/topoarm_robot_model/urdf/topoarm.urdf.xacro");
  resource_root_dir_ = declare_parameter<std::string>("resource_root_dir", "");
  mesh_root_dir_ = declare_parameter<std::string>("mesh_root_dir", "");
  topic_name_ = declare_parameter<std::string>("topic_name", "robot_description");
  poll_ms_ = declare_parameter<int>("poll_ms", 1000);
  republish_ms_ = declare_parameter<int>("republish_ms", 1000);

  if (resource_root_dir_.empty()) {
    resource_root_dir_ =
        robot_sim::common::deriveResourceRootFromMeshRoot(mesh_root_dir_).string();
  }

  description_pub_ = create_publisher<std_msgs::msg::String>(
      topic_name_, rclcpp::QoS(1).transient_local().reliable());

  const auto period = std::chrono::milliseconds(std::max(poll_ms_, 200));
  timer_ = create_wall_timer(period, std::bind(&RobotDescriptionPlayer::tick, this));
  republish_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(republish_ms_, 200)),
      std::bind(&RobotDescriptionPlayer::republishLastDescription, this));

  RCLCPP_INFO(get_logger(), "robot_description_player started: file=%s topic=%s",
              description_file_.c_str(), topic_name_.c_str());
}

bool RobotDescriptionPlayer::loadDescription(std::string &out_text) const {
  if (description_file_.rfind(".xacro") != std::string::npos) {
    const std::string cmd = "xacro " + description_file_;
    std::array<char, 4096> buffer{};
    std::string result;
    if (FILE *pipe = popen(cmd.c_str(), "r")) {
      while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        result += buffer.data();
      }
      const int status = pclose(pipe);
      if (status != 0) {
        return false;
      }
    } else {
      return false;
    }
    out_text = std::move(result);
  } else {
    std::ifstream ifs(description_file_);
    if (!ifs) {
      return false;
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    out_text = oss.str();
  }

  out_text = robot_sim::common::resolvePackageUris(out_text);
  return !out_text.empty();
}

void RobotDescriptionPlayer::tick() {
  const std::filesystem::path path(description_file_);
  if (!std::filesystem::exists(path)) {
    if (!missing_warned_) {
      RCLCPP_WARN(get_logger(), "Robot description file not found: %s",
                  description_file_.c_str());
      missing_warned_ = true;
    }
    return;
  }
  missing_warned_ = false;

  std::error_code ec;
  const auto mtime = std::filesystem::last_write_time(path, ec);
  if (ec) {
    RCLCPP_WARN(get_logger(), "Failed to stat robot description file: %s",
                description_file_.c_str());
    return;
  }

  if (!have_last_mtime_ || mtime != last_mtime_) {
    std::string text;
    if (!loadDescription(text)) {
      RCLCPP_ERROR(get_logger(), "Failed to read robot description file: %s",
                   description_file_.c_str());
      return;
    }

    std_msgs::msg::String msg;
    msg.data = std::move(text);
    last_description_ = msg.data;
    description_pub_->publish(msg);
    last_mtime_ = mtime;
    have_last_mtime_ = true;
    have_last_description_ = true;
    RCLCPP_INFO(get_logger(), "Published robot description from %s",
                description_file_.c_str());
  }
}

void RobotDescriptionPlayer::republishLastDescription() {
  if (!have_last_description_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = last_description_;
  description_pub_->publish(msg);
}

} // namespace self_recognition
} // namespace robot_sim

#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::self_recognition::RobotDescriptionPlayer>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::self_recognition::RobotDescriptionPlayer)
