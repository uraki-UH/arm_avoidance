#include <ais_gng_msgs/msg/topological_cluster.hpp>
#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace robot_sim::bridge
{
namespace
{

using json = nlohmann::json;

float readFiniteFloat(const json &value, const char *key, float fallback)
{
  if (!value.contains(key) || !value.at(key).is_number()) {
    return fallback;
  }
  const float number = value.at(key).get<float>();
  return std::isfinite(number) ? number : fallback;
}

float readRequiredFiniteFloat(const json &value, const char *key)
{
  if (!value.contains(key) || !value.at(key).is_number()) {
    throw std::runtime_error(std::string("GNG nodeの必須座標がありません: ") + key);
  }
  const float number = value.at(key).get<float>();
  if (!std::isfinite(number)) {
    throw std::runtime_error(std::string("GNG nodeの座標が有限値ではありません: ") + key);
  }
  return number;
}

std::optional<std::size_t> readIndex(const json &value)
{
  if (!value.is_number_integer() && !value.is_number_unsigned()) {
    return std::nullopt;
  }
  const auto index = value.get<std::int64_t>();
  if (index < 0) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(index);
}

bool isTopicToken(const std::string &value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_';
  });
}

std::uint8_t nodeLabel(const json &node)
{
  const std::string label = node.value("label", "");
  if (label == "safe") {
    return ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN;
  }
  if (label == "wall") {
    return ais_gng_msgs::msg::TopologicalMap::WALL;
  }
  return ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
}

std::uint16_t sourceNodeId(const json &node, std::size_t fallback)
{
  if (node.contains("id")) {
    const auto id = readIndex(node.at("id"));
    if (id && *id <= std::numeric_limits<std::uint16_t>::max()) {
      return static_cast<std::uint16_t>(*id);
    }
  }
  return static_cast<std::uint16_t>(fallback);
}

}

class ObjectTemplateMapPublisherNode : public rclcpp::Node
{
public:
  explicit ObjectTemplateMapPublisherNode(const rclcpp::NodeOptions &options)
  : Node("object_template_map_publisher_node", options)
  {
    declare_parameter<std::string>("dataset_path", "");
    declare_parameter<std::string>("template_id", "");
    declare_parameter<std::string>("frame_id", "object_template");
    declare_parameter<double>("publish_hz", 1.0);

    const std::string dataset_path = get_parameter("dataset_path").as_string();
    if (dataset_path.empty()) {
      throw std::runtime_error("dataset_path の指定が必要です。");
    }

    const json root = readDataset(dataset_path);
    const json &template_root = selectTemplate(root);
    template_id_ = get_parameter("template_id").as_string();
    if (template_id_.empty()) {
      template_id_ = root.value("dataset_id", template_root.value("template_id", ""));
    }
    if (!isTopicToken(template_id_)) {
      throw std::runtime_error(
              "template_id は英字開始の英数字または_だけで指定してください: " + template_id_);
    }

    message_ = buildMessage(template_root, get_parameter("frame_id").as_string());
    const std::string topic_name = "/" + template_id_ + "/topological_map_static";
    publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
      topic_name, rclcpp::QoS(1).reliable().transient_local());

    publishMessage();
    const double publish_hz = std::max(0.1, get_parameter("publish_hz").as_double());
    const auto period = std::chrono::duration<double>(1.0 / publish_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObjectTemplateMapPublisherNode::publishMessage, this));

    RCLCPP_INFO(
      get_logger(),
      "物体GNGテンプレート配信開始: topic=%s nodes=%zu edges=%zu clusters=%zu frame=%s",
      topic_name.c_str(), message_.nodes.size(), message_.edges.size() / 2,
      message_.clusters.size(), message_.header.frame_id.c_str());
  }

private:
  static json readDataset(const std::string &dataset_path)
  {
    if (dataset_path.size() >= 3U && dataset_path.substr(dataset_path.size() - 3U) == ".gz") {
      gzFile file = gzopen(dataset_path.c_str(), "rb");
      if (file == nullptr) {
        throw std::runtime_error("データセットgzipを開けません: " + dataset_path);
      }
      std::string text;
      std::array<char, 64U * 1024U> buffer{};
      int read_byte_num = 0;
      while ((read_byte_num = gzread(file, buffer.data(), buffer.size())) > 0) {
        text.append(buffer.data(), static_cast<std::size_t>(read_byte_num));
      }
      const bool has_read_error = read_byte_num < 0;
      const int close_result = gzclose(file);
      if (has_read_error || close_result != Z_OK) {
        throw std::runtime_error("データセットgzipの読込失敗: " + dataset_path);
      }
      try {
        return json::parse(text);
      } catch (const json::exception &error) {
        throw std::runtime_error(
                "データセットJSONの解析失敗: " + std::string(error.what()));
      }
    }
    std::ifstream stream(dataset_path);
    if (!stream) {
      throw std::runtime_error("データセットJSONを開けません: " + dataset_path);
    }
    try {
      return json::parse(stream);
    } catch (const json::exception &error) {
      throw std::runtime_error("データセットJSONの解析失敗: " + std::string(error.what()));
    }
  }

  static const json &selectTemplate(const json &root)
  {
    const std::string kind = root.value("kind", "");
    if (kind == "object_template") {
      return root;
    }
    if (kind == "object_surface_dataset" && root.contains("gng_template") &&
      root.at("gng_template").is_object())
    {
      return root.at("gng_template");
    }
    throw std::runtime_error(
            "object_template またはGNG同梱済みobject_surface_datasetが必要です。");
  }

  ais_gng_msgs::msg::TopologicalMap buildMessage(
    const json &template_root, const std::string &frame_id)
  {
    if (!template_root.contains("gng") || !template_root.at("gng").is_object()) {
      throw std::runtime_error("テンプレートにgngがありません。");
    }
    const json &graph = template_root.at("gng");
    if (!graph.contains("nodes") || !graph.at("nodes").is_array()) {
      throw std::runtime_error("テンプレートにGNG node配列がありません。");
    }

    const json &nodes = graph.at("nodes");
    if (nodes.empty()) {
      throw std::runtime_error("テンプレートのGNG nodeが空です。");
    }
    if (nodes.size() > std::numeric_limits<std::uint16_t>::max()) {
      throw std::runtime_error("TopologicalMapのuint16上限を超えるGNG node数です。");
    }

    ais_gng_msgs::msg::TopologicalMap message;
    message.header.frame_id = frame_id;
    message.frame_number = 1;
    message.nodes.reserve(nodes.size());
    std::unordered_map<std::uint16_t, std::uint16_t> source_id_to_message_id;

    for (std::size_t index = 0; index < nodes.size(); ++index) {
      const json &source = nodes.at(index);
      if (!source.is_object()) {
        throw std::runtime_error("GNG nodeがobjectではありません。");
      }

      ais_gng_msgs::msg::TopologicalNode node;
      node.id = static_cast<std::uint16_t>(index);
      node.pos.x = readRequiredFiniteFloat(source, "x");
      node.pos.y = readRequiredFiniteFloat(source, "y");
      node.pos.z = readRequiredFiniteFloat(source, "z");
      node.normal.x = readFiniteFloat(source, "nx", 0.0F);
      node.normal.y = readFiniteFloat(source, "ny", 0.0F);
      node.normal.z = readFiniteFloat(source, "nz", 1.0F);
      const float normal_length = std::sqrt(
        node.normal.x * node.normal.x + node.normal.y * node.normal.y + node.normal.z * node.normal.z);
      if (normal_length > 1e-6F) {
        node.normal.x /= normal_length;
        node.normal.y /= normal_length;
        node.normal.z /= normal_length;
      } else {
        node.normal.x = 0.0F;
        node.normal.y = 0.0F;
        node.normal.z = 1.0F;
      }
      node.rho = readFiniteFloat(source, "rho", 0.0F);
      node.label = nodeLabel(source);
      node.frame = static_cast<std::uint32_t>(std::max(0.0F, readFiniteFloat(source, "frame", 0.0F)));
      node.winner_point_count = static_cast<std::uint32_t>(
        std::max(0.0F, readFiniteFloat(source, "winner_point_count", 0.0F)));
      node.winner_point_covariance.fill(0.0F);
      if (source.contains("winner_point_covariance") &&
        source.at("winner_point_covariance").is_array())
      {
        const json &covariance = source.at("winner_point_covariance");
        for (std::size_t covariance_index = 0;
          covariance_index < node.winner_point_covariance.size() && covariance_index < covariance.size();
          ++covariance_index)
        {
          if (covariance.at(covariance_index).is_number()) {
            const float value = covariance.at(covariance_index).get<float>();
            node.winner_point_covariance[covariance_index] = std::isfinite(value) ? value : 0.0F;
          }
        }
      }
      if (source.contains("inpcl_ids") && source.at("inpcl_ids").is_array()) {
        for (const json &point_id : source.at("inpcl_ids")) {
          const auto index_value = readIndex(point_id);
          if (index_value && *index_value <= std::numeric_limits<std::uint32_t>::max()) {
            node.inpcl_ids.push_back(static_cast<std::uint32_t>(*index_value));
          }
        }
      }
      source_id_to_message_id.emplace(sourceNodeId(source, index), node.id);
      message.nodes.push_back(std::move(node));
    }

    appendEdges(graph, message);
    appendClusters(graph, source_id_to_message_id, message);
    return message;
  }

  static void appendEdges(const json &graph, ais_gng_msgs::msg::TopologicalMap &message)
  {
    if (!graph.contains("edges") || !graph.at("edges").is_array()) {
      return;
    }
    std::unordered_set<std::uint32_t> seen;
    for (const json &edge : graph.at("edges")) {
      std::optional<std::size_t> first;
      std::optional<std::size_t> second;
      if (edge.is_array() && edge.size() >= 2) {
        first = readIndex(edge.at(0));
        second = readIndex(edge.at(1));
      } else if (edge.is_object() && edge.contains("a") && edge.contains("b")) {
        first = readIndex(edge.at("a"));
        second = readIndex(edge.at("b"));
      }
      if (!first || !second || *first == *second || *first >= message.nodes.size() ||
        *second >= message.nodes.size())
      {
        continue;
      }
      const std::uint16_t source = static_cast<std::uint16_t>(std::min(*first, *second));
      const std::uint16_t target = static_cast<std::uint16_t>(std::max(*first, *second));
      const std::uint32_t edge_key =
        (static_cast<std::uint32_t>(source) << 16U) | static_cast<std::uint32_t>(target);
      if (!seen.insert(edge_key).second) {
        continue;
      }
      message.edges.push_back(source);
      message.edges.push_back(target);
    }
  }

  static void appendClusters(
    const json &graph,
    const std::unordered_map<std::uint16_t, std::uint16_t> &source_id_to_message_id,
    ais_gng_msgs::msg::TopologicalMap &message)
  {
    if (!graph.contains("node_clusters") || !graph.at("node_clusters").is_array()) {
      return;
    }
    for (const json &source : graph.at("node_clusters")) {
      if (!source.is_object()) {
        continue;
      }
      const json *indices = nullptr;
      bool has_index_values = false;
      if (source.contains("idx") && source.at("idx").is_array()) {
        indices = &source.at("idx");
        has_index_values = true;
      } else if (source.contains("nodes") && source.at("nodes").is_array()) {
        indices = &source.at("nodes");
      }
      if (!indices) {
        continue;
      }

      ais_gng_msgs::msg::TopologicalCluster cluster;
      cluster.id = static_cast<std::uint32_t>(message.clusters.size());
      cluster.label = ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
      for (const json &value : *indices) {
        const auto source_index = readIndex(value);
        if (!source_index) {
          continue;
        }
        std::optional<std::uint16_t> message_id;
        if (has_index_values && *source_index < message.nodes.size()) {
          message_id = static_cast<std::uint16_t>(*source_index);
        } else if (*source_index <= std::numeric_limits<std::uint16_t>::max()) {
          const auto found = source_id_to_message_id.find(static_cast<std::uint16_t>(*source_index));
          if (found != source_id_to_message_id.end()) {
            message_id = found->second;
          }
        }
        if (message_id) {
          cluster.nodes.push_back(*message_id);
        }
      }
      if (cluster.nodes.empty()) {
        continue;
      }
      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float min_z = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float max_y = std::numeric_limits<float>::lowest();
      float max_z = std::numeric_limits<float>::lowest();
      for (const std::uint16_t node_id : cluster.nodes) {
        const auto &node = message.nodes.at(node_id);
        min_x = std::min(min_x, node.pos.x);
        min_y = std::min(min_y, node.pos.y);
        min_z = std::min(min_z, node.pos.z);
        max_x = std::max(max_x, node.pos.x);
        max_y = std::max(max_y, node.pos.y);
        max_z = std::max(max_z, node.pos.z);
      }
      cluster.pos.x = (min_x + max_x) * 0.5F;
      cluster.pos.y = (min_y + max_y) * 0.5F;
      cluster.pos.z = (min_z + max_z) * 0.5F;
      cluster.scale.x = max_x - min_x;
      cluster.scale.y = max_y - min_y;
      cluster.scale.z = max_z - min_z;
      message.clusters.push_back(std::move(cluster));
    }
  }

  void publishMessage()
  {
    message_.header.stamp = now();
    publisher_->publish(message_);
  }

  std::string template_id_;
  ais_gng_msgs::msg::TopologicalMap message_;
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_sim::bridge::ObjectTemplateMapPublisherNode>(
      rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_template_map_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
