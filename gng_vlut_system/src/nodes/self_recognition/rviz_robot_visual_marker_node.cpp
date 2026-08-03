#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/resource_utils.hpp"

namespace {

geometry_msgs::msg::Point toPoint(const urdf::Vector3 &v) {
  geometry_msgs::msg::Point out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

geometry_msgs::msg::Quaternion toQuaternion(const urdf::Rotation &r) {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  r.getQuaternion(x, y, z, w);
  geometry_msgs::msg::Quaternion out;
  out.x = x;
  out.y = y;
  out.z = z;
  out.w = w;
  return out;
}

std_msgs::msg::ColorRGBA materialColor(const urdf::MaterialSharedPtr &material) {
  std_msgs::msg::ColorRGBA out;
  out.r = 0.8f;
  out.g = 0.8f;
  out.b = 0.8f;
  out.a = 1.0f;

  if (!material) {
    return out;
  }

  out.r = material->color.r;
  out.g = material->color.g;
  out.b = material->color.b;
  out.a = material->color.a;

  if (material->name == "black" && out.r == 0.0f && out.g == 0.0f && out.b == 0.0f) {
    out.r = 0.24f;
    out.g = 0.24f;
    out.b = 0.24f;
  } else if (material->name == "green" && out.r == 0.0f && out.g == 0.0f && out.b == 0.0f) {
    out.g = 1.0f;
  } else if (material->name == "gray" && out.r == 0.0f && out.g == 0.0f && out.b == 0.0f) {
    out.r = 0.55f;
    out.g = 0.55f;
    out.b = 0.55f;
  } else if (material->name == "silver" && out.r == 0.0f && out.g == 0.0f && out.b == 0.0f) {
    out.r = 0.80f;
    out.g = 0.80f;
    out.b = 0.82f;
  }

  if (out.a <= 0.0f) {
    out.a = 1.0f;
  }
  return out;
}

std::string readDescription(const std::string &path) {
  if (path.find(".xacro") != std::string::npos) {
    const std::string cmd = "xacro " + path;
    std::array<char, 4096> buffer{};
    std::string result;
    if (FILE *pipe = popen(cmd.c_str(), "r")) {
      while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        result += buffer.data();
      }
      const int status = pclose(pipe);
      if (status != 0) {
        return "";
      }
    }
    return result;
  }

  std::ifstream ifs(path);
  if (!ifs) {
    return "";
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

std::string resolveMeshResource(const std::string &filename,
                                const std::string &resource_root_dir,
                                const std::string &mesh_root_dir) {
  if (filename.empty()) {
    return "";
  }
  if (filename.find("file://") == 0) {
    return filename;
  }
  if (filename.find("package://") == 0) {
    return robot_sim::common::resolvePackageUris(filename);
  }

  std::filesystem::path resolved;
  if (filename.rfind("meshes/", 0) == 0) {
    const auto mesh_root =
        robot_sim::common::resolveMeshRootForRelativeUris(resource_root_dir, mesh_root_dir);
    resolved = mesh_root / filename.substr(std::string("meshes/").size());
  } else if (!resource_root_dir.empty()) {
    resolved = std::filesystem::path(robot_sim::common::stripUriScheme(resource_root_dir)) / filename;
  } else {
    resolved = std::filesystem::absolute(filename);
  }

  return "file://" + resolved.string();
}

}  // namespace

class RvizRobotVisualMarkerNode : public rclcpp::Node {
public:
  RvizRobotVisualMarkerNode() : Node("rviz_robot_visual_marker_node") {
    robot_name_ = declare_parameter<std::string>("robot_name", "ToPoDualArm");
    urdf_path_ = declare_parameter<std::string>("urdf_path", "");
    resource_root_dir_ = declare_parameter<std::string>("resource_root_dir", "");
    mesh_root_dir_ = declare_parameter<std::string>("mesh_root_dir", "");
    topic_name_ = declare_parameter<std::string>("topic_name", "rviz_robot_visual_markers");
    robot_description_topic_ =
        declare_parameter<std::string>("robot_description_topic", "rviz_robot_description");
    republish_ms_ = declare_parameter<int>("republish_ms", 1000);

    if (topic_name_.empty()) {
      topic_name_ = "rviz_robot_visual_markers";
    }
    if (topic_name_[0] != '/') {
      topic_name_ = "/" + robot_name_ + "/" + topic_name_;
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        topic_name_, rclcpp::QoS(1).transient_local().reliable());

    if (robot_description_topic_.empty()) {
      robot_description_topic_ = "rviz_robot_description";
    }
    if (robot_description_topic_[0] != '/') {
      robot_description_topic_ = "/" + robot_name_ + "/" + robot_description_topic_;
    }

    if (!urdf_path_.empty()) {
      buildMarkers(readDescription(urdf_path_));
    }
    description_sub_ = create_subscription<std_msgs::msg::String>(
        robot_description_topic_,
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == last_description_) {
            return;
          }
          last_description_ = msg->data;
          buildMarkers(msg->data);
          publishMarkers();
        });

    timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(republish_ms_, 200)),
        [this]() { publishMarkers(); });
  }

private:
  void buildMarkers(const std::string &description) {
    markers_.markers.clear();
    if (description.empty()) {
      RCLCPP_ERROR(get_logger(), "Received empty robot description");
      return;
    }

    auto model = urdf::parseURDF(description);
    if (!model) {
      RCLCPP_ERROR(get_logger(), "Failed to parse robot description: %s", urdf_path_.c_str());
      return;
    }

    int id = 0;
    for (const auto &[link_name, link] : model->links_) {
      for (const auto &visual : link->visual_array) {
        if (!visual || !visual->geometry || visual->geometry->type != urdf::Geometry::MESH) {
          continue;
        }
        const auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(visual->geometry);
        if (!mesh) {
          continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = robot_name_ + "/" + link_name;
        marker.ns = robot_name_ + "_visual_mesh";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = toPoint(visual->origin.position);
        marker.pose.orientation = toQuaternion(visual->origin.rotation);
        marker.scale.x = mesh->scale.x;
        marker.scale.y = mesh->scale.y;
        marker.scale.z = mesh->scale.z;
        marker.color = materialColor(visual->material);
        marker.mesh_resource = resolveMeshResource(mesh->filename, resource_root_dir_, mesh_root_dir_);
        marker.mesh_use_embedded_materials = false;
        marker.frame_locked = true;
        markers_.markers.push_back(marker);
      }
    }

    RCLCPP_INFO(get_logger(), "Prepared %zu RViz robot visual markers on %s",
                markers_.markers.size(), topic_name_.c_str());
  }

  void publishMarkers() {
    const auto now = get_clock()->now();
    for (auto &marker : markers_.markers) {
      marker.header.stamp = now;
    }
    marker_pub_->publish(markers_);
  }

  std::string robot_name_;
  std::string urdf_path_;
  std::string resource_root_dir_;
  std::string mesh_root_dir_;
  std::string topic_name_;
  std::string robot_description_topic_;
  std::string last_description_;
  int republish_ms_ = 1000;
  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizRobotVisualMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
