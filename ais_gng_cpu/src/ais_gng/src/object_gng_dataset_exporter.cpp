#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/srv/save_object_gng_dataset.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

namespace
{

using json = nlohmann::json;
using TopologicalMap = ais_gng_msgs::msg::TopologicalMap;

bool isDatasetId(const std::string &value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_';
  });
}

const char *nodeLabel(std::uint8_t label)
{
  if (label == TopologicalMap::SAFE_TERRAIN) {
    return "safe";
  }
  if (label == TopologicalMap::WALL) {
    return "wall";
  }
  if (label == TopologicalMap::HUMAN) {
    return "human";
  }
  if (label == TopologicalMap::CAR) {
    return "car";
  }
  return "unknown";
}

std::string currentTimestamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm utc_time{};
  gmtime_r(&now, &utc_time);
  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

float finiteOrZero(float value)
{
  return std::isfinite(value) ? value : 0.0F;
}

class ObjectGngDatasetExporterNode : public rclcpp::Node
{
public:
  ObjectGngDatasetExporterNode()
  : Node("object_gng_dataset_exporter_node")
  {
    output_dir_ = declare_parameter<std::string>("output_dir", "/datasets");
    const std::string map_topic = declare_parameter<std::string>("map_topic", "topological_map");
    const std::string save_service =
      declare_parameter<std::string>("save_service", "save_object_gng_dataset");
    if (output_dir_.empty()) {
      throw std::runtime_error("output_dir の指定が必要です。");
    }

    map_sub_ = create_subscription<TopologicalMap>(
      map_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ObjectGngDatasetExporterNode::onMap, this, std::placeholders::_1));
    save_service_ = create_service<ais_gng_msgs::srv::SaveObjectGngDataset>(
      save_service,
      std::bind(
        &ObjectGngDatasetExporterNode::saveDataset, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "物体GNG保存待機: map=%s service=%s output_dir=%s",
      map_topic.c_str(), save_service.c_str(), output_dir_.c_str());
  }

private:
  void onMap(const TopologicalMap::SharedPtr map)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = *map;
  }

  void saveDataset(
    const std::shared_ptr<ais_gng_msgs::srv::SaveObjectGngDataset::Request> request,
    std::shared_ptr<ais_gng_msgs::srv::SaveObjectGngDataset::Response> response)
  {
    TopologicalMap map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!latest_map_) {
        response->success = false;
        response->message = "topological_map をまだ受信していません。";
        return;
      }
      map = *latest_map_;
    }

    try {
      if (!isDatasetId(request->dataset_id)) {
        throw std::runtime_error(
                "dataset_id は英字開始の英数字または_だけで指定してください: " + request->dataset_id);
      }
      const std::string output_dir = request->output_dir.empty() ? output_dir_ : request->output_dir;
      if (output_dir.empty()) {
        throw std::runtime_error("output_dir の指定が必要です。");
      }
      const std::filesystem::path output_path =
        std::filesystem::path(output_dir) /
        (request->dataset_id + "_object_surface_dataset_v1.json");
      std::filesystem::create_directories(output_path.parent_path());
      std::ofstream stream(output_path);
      if (!stream) {
        throw std::runtime_error("保存先を開けません: " + output_path.string());
      }
      stream << buildDataset(map, request->dataset_id).dump(2) << '\n';
      if (!stream) {
        throw std::runtime_error("データセット書込失敗: " + output_path.string());
      }
      response->success = true;
      response->message = "保存完了: " + output_path.string() +
        " nodes=" + std::to_string(map.nodes.size()) +
        " edges=" + std::to_string(map.edges.size() / 2U) +
        " clusters=" + std::to_string(map.clusters.size());
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "物体GNGデータセット保存失敗: %s", error.what());
    }
  }

  json buildDataset(const TopologicalMap &map, const std::string &dataset_id) const
  {
    json nodes = json::array();
    std::unordered_map<std::uint16_t, std::size_t> node_indices;
    for (std::size_t index = 0; index < map.nodes.size(); ++index) {
      const auto &source = map.nodes.at(index);
      json covariance = json::array();
      for (const float value : source.winner_point_covariance) {
        covariance.push_back(finiteOrZero(value));
      }
      nodes.push_back({
        {"id", source.id},
        {"x", finiteOrZero(source.pos.x)},
        {"y", finiteOrZero(source.pos.y)},
        {"z", finiteOrZero(source.pos.z)},
        {"nx", finiteOrZero(source.normal.x)},
        {"ny", finiteOrZero(source.normal.y)},
        {"nz", finiteOrZero(source.normal.z)},
        {"rho", finiteOrZero(source.rho)},
        {"label", nodeLabel(source.label)},
        {"frame", source.frame},
        {"inpcl_ids", source.inpcl_ids},
        {"winner_point_count", source.winner_point_count},
        {"winner_point_covariance", covariance},
      });
      node_indices.emplace(source.id, index);
    }

    const auto resolve_index = [&node_indices, &map](std::uint16_t value) {
        const auto found = node_indices.find(value);
        if (found != node_indices.end()) {
          return found->second;
        }
        return static_cast<std::size_t>(value) < map.nodes.size() ?
               static_cast<std::size_t>(value) : map.nodes.size();
      };

    json edges = json::array();
    for (std::size_t index = 0; index + 1U < map.edges.size(); index += 2U) {
      const std::size_t first = resolve_index(map.edges.at(index));
      const std::size_t second = resolve_index(map.edges.at(index + 1U));
      if (first == map.nodes.size() || second == map.nodes.size() || first == second) {
        continue;
      }
      edges.push_back({{"a", first}, {"b", second}});
    }

    json clusters = json::array();
    for (const auto &source : map.clusters) {
      json indices = json::array();
      for (const std::uint16_t node_id : source.nodes) {
        const std::size_t index = resolve_index(node_id);
        if (index != map.nodes.size()) {
          indices.push_back(index);
        }
      }
      if (!indices.empty()) {
        clusters.push_back({{"id", source.id}, {"idx", indices}});
      }
    }

    const std::string timestamp = currentTimestamp();
    const json source = {
      {"sample", "ros2_ais_gng"},
      {"point_cloud_source", "topological_map"},
      {"gng_algorithm", "ais_gng"},
      {"surface_points_status", "not_captured"},
    };
    const json graph = {
      {"nodes", nodes},
      {"edges", edges},
      {"node_attributes", json::array()},
      {"node_clusters", clusters},
      {"node_features", json::array()},
    };
    const json gng_template = {
      {"schema_version", 1},
      {"kind", "object_template"},
      {"template_id", dataset_id},
      {"display_name", dataset_id},
      {"canonical_yaw_deg", 0.0},
      {"created_at", timestamp},
      {"source", source},
      {"gng", graph},
    };
    return {
      {"schema_version", 1},
      {"kind", "object_surface_dataset"},
      {"dataset_id", dataset_id},
      {"display_name", dataset_id},
      {"canonical_yaw_deg", 0.0},
      {"created_at", timestamp},
      {"source", source},
      {"surface_points", json::array()},
      {"gng_template", gng_template},
    };
  }

  std::string output_dir_;
  std::mutex map_mutex_;
  std::optional<TopologicalMap> latest_map_;
  rclcpp::Subscription<TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Service<ais_gng_msgs::srv::SaveObjectGngDataset>::SharedPtr save_service_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObjectGngDatasetExporterNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_gng_dataset_exporter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
