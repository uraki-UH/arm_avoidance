#include <ais_gng/handle_label_utils.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>

namespace fuzzrobo::handle_label
{
namespace
{

using PC2 = sensor_msgs::msg::PointCloud2;

struct SemanticPointField
{
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 0;
};

struct SemanticStats
{
  uint32_t total = 0;
  uint32_t handle = 0;

  double ratio() const noexcept
  {
    return total == 0 ? 0.0 : static_cast<double>(handle) / static_cast<double>(total);
  }
};

std::optional<SemanticPointField> findSemanticField(const PC2 &msg)
{
  static const std::array<const char *, 4> kFieldNames{
    "handle", "semantic_label", "label", "semantic"
  };
  for (const auto *name : kFieldNames) {
    const auto it = std::find_if(msg.fields.begin(), msg.fields.end(), [name](const auto &field) {
      return field.name == name;
    });
    if (it != msg.fields.end()) {
      return SemanticPointField{static_cast<uint32_t>(it->offset), it->datatype, it->count};
    }
  }
  return std::nullopt;
}

std::optional<uint32_t> readSemanticValue(
  const PC2 &msg, std::size_t index, const SemanticPointField &field)
{
  const std::size_t offset = index * msg.point_step + field.offset;
  const std::size_t value_size = [&field]() -> std::size_t {
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::UINT8:
      case sensor_msgs::msg::PointField::INT8:
        return sizeof(uint8_t);
      case sensor_msgs::msg::PointField::UINT16:
      case sensor_msgs::msg::PointField::INT16:
        return sizeof(uint16_t);
      case sensor_msgs::msg::PointField::UINT32:
      case sensor_msgs::msg::PointField::INT32:
      case sensor_msgs::msg::PointField::FLOAT32:
        return sizeof(uint32_t);
      default:
        return 0;
    }
  }();
  if (value_size == 0 || offset + value_size > msg.data.size()) {
    return std::nullopt;
  }

  const auto *ptr = reinterpret_cast<const uint8_t *>(msg.data.data() + offset);
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::UINT8: {
      uint8_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(value);
    }
    case sensor_msgs::msg::PointField::INT8: {
      int8_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(value);
    }
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(value);
    }
    case sensor_msgs::msg::PointField::INT16: {
      int16_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(value);
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t value = 0;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(value);
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      float value = 0.0f;
      std::memcpy(&value, ptr, sizeof(value));
      return static_cast<uint32_t>(std::lround(value));
    }
    default:
      return std::nullopt;
  }
}

SemanticStats collectSemanticStatsFromNodePoints(
  const ais_gng_msgs::msg::TopologicalNode &node,
  const std::vector<uint8_t> &semantic_labels)
{
  SemanticStats stats;
  for (const auto point_id : node.inpcl_ids) {
    if (point_id >= semantic_labels.size()) {
      continue;
    }
    ++stats.total;
    if (semantic_labels[point_id] == ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE) {
      ++stats.handle;
    }
  }
  return stats;
}

std::vector<SemanticStats> collectSemanticStatsFromTransformedPoints(
  const ais_gng_msgs::msg::TopologicalMap &map_msg,
  const float *transformed_pcl,
  uint32_t transformed_pcl_num,
  const std::vector<uint8_t> &semantic_labels)
{
  std::vector<SemanticStats> stats(map_msg.nodes.size());
  if (map_msg.nodes.empty() || transformed_pcl == nullptr || transformed_pcl_num == 0) {
    return stats;
  }

  const std::size_t point_limit = std::min<std::size_t>(
    static_cast<std::size_t>(transformed_pcl_num),
    semantic_labels.size());
  for (std::size_t i = 0; i < point_limit; ++i) {
    const float *p = transformed_pcl + i * 3;
    std::size_t best_idx = 0;
    double best_dist2 = std::numeric_limits<double>::max();
    for (std::size_t node_idx = 0; node_idx < map_msg.nodes.size(); ++node_idx) {
      const auto &node = map_msg.nodes[node_idx];
      const double dx = static_cast<double>(p[0]) - static_cast<double>(node.pos.x);
      const double dy = static_cast<double>(p[1]) - static_cast<double>(node.pos.y);
      const double dz = static_cast<double>(p[2]) - static_cast<double>(node.pos.z);
      const double dist2 = dx * dx + dy * dy + dz * dz;
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best_idx = node_idx;
      }
    }

    ++stats[best_idx].total;
    if (semantic_labels[i] == ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE) {
      ++stats[best_idx].handle;
    }
  }

  return stats;
}

}  // namespace

std::vector<uint8_t> extractSemanticLabels(
  const sensor_msgs::msg::PointCloud2 &msg,
  uint32_t handle_label_value)
{
  std::vector<uint8_t> semantic_labels;
  semantic_labels.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));

  const auto semantic_field = findSemanticField(msg);
  if (!semantic_field) {
    return semantic_labels;
  }

  const std::size_t point_count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  semantic_labels.resize(point_count, ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
  for (std::size_t i = 0; i < point_count; ++i) {
    const auto value = readSemanticValue(msg, i, *semantic_field);
    if (value && *value == handle_label_value) {
      semantic_labels[i] = ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE;
    }
  }
  return semantic_labels;
}

void applySemanticLabelsToMap(
  ais_gng_msgs::msg::TopologicalMap &map_msg,
  const float *transformed_pcl,
  uint32_t transformed_pcl_num,
  const std::vector<uint8_t> &semantic_labels,
  double handle_ratio_threshold)
{
  std::vector<uint8_t> node_semantic_labels(
    map_msg.nodes.size(), ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT);
  const auto point_stats = collectSemanticStatsFromTransformedPoints(
    map_msg, transformed_pcl, transformed_pcl_num, semantic_labels);

  for (std::size_t i = 0; i < map_msg.nodes.size(); ++i) {
    SemanticStats stats;
    if (i < point_stats.size() && point_stats[i].total > 0) {
      stats = point_stats[i];
    } else {
      stats = collectSemanticStatsFromNodePoints(map_msg.nodes[i], semantic_labels);
    }
    const bool is_handle = stats.total > 0 &&
      stats.ratio() >= handle_ratio_threshold &&
      stats.handle > 0;
    map_msg.nodes[i].semantic_label = is_handle
      ? ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE
      : ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
    map_msg.nodes[i].semantic_reliability = static_cast<float>(stats.ratio());
    node_semantic_labels[i] = map_msg.nodes[i].semantic_label;
  }

  for (auto &cluster : map_msg.clusters) {
    SemanticStats stats;
    for (const auto node_id : cluster.nodes) {
      if (node_id >= node_semantic_labels.size()) {
        continue;
      }
      ++stats.total;
      if (node_semantic_labels[node_id] == ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE) {
        ++stats.handle;
      }
    }
    const bool is_handle = stats.total > 0 &&
      stats.ratio() >= handle_ratio_threshold &&
      stats.handle > 0;
    cluster.semantic_label = is_handle
      ? ais_gng_msgs::msg::TopologicalMap::SEMANTIC_HANDLE
      : ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT;
    cluster.semantic_reliability = static_cast<float>(stats.ratio());
  }
}

}  // namespace fuzzrobo::handle_label
