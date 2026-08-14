#include <ais_gng/topological_grid/topological_grid_node.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace
{

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::uint8_t parseLabel(const std::string &raw_token)
{
  std::string token = trim(raw_token);
  std::transform(
    token.begin(), token.end(), token.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});

  static const std::unordered_map<std::string, std::uint8_t> kLabels{
    {"DEFAULT", ais_gng_msgs::msg::TopologicalMap::DEFAULT},
    {"SAFE_TERRAIN", ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN},
    {"WALL", ais_gng_msgs::msg::TopologicalMap::WALL},
    {"UNKNOWN_OBJECT", ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT},
    {"HUMAN", ais_gng_msgs::msg::TopologicalMap::HUMAN},
    {"CAR", ais_gng_msgs::msg::TopologicalMap::CAR},
  };
  const auto named = kLabels.find(token);
  if (named != kLabels.end()) {
    return named->second;
  }

  std::size_t parsed_length = 0;
  int value = -1;
  try {
    value = std::stoi(token, &parsed_length);
  } catch (const std::exception &) {
    throw std::invalid_argument("unknown topological label: " + raw_token);
  }
  if (parsed_length != token.size() || value < 0 || value > 255) {
    throw std::invalid_argument("topological label must be in [0, 255]: " + raw_token);
  }
  return static_cast<std::uint8_t>(value);
}

std::unordered_set<std::uint8_t> parseExcludedLabels(const std::string &csv)
{
  std::unordered_set<std::uint8_t> labels;
  std::stringstream stream(csv);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (!trim(token).empty()) {
      labels.insert(parseLabel(token));
    }
  }
  return labels;
}

std::string labelSetToString(const std::unordered_set<std::uint8_t> &labels)
{
  std::vector<int> sorted_labels;
  sorted_labels.reserve(labels.size());
  for (const auto label : labels) {
    sorted_labels.push_back(static_cast<int>(label));
  }
  std::sort(sorted_labels.begin(), sorted_labels.end());

  std::ostringstream stream;
  for (std::size_t i = 0; i < sorted_labels.size(); ++i) {
    if (i > 0) {
      stream << ",";
    }
    stream << sorted_labels[i];
  }
  return stream.str();
}

}  // namespace

namespace fuzzrobo::topological_grid
{

TopologicalGridNode::TopologicalGridNode(const rclcpp::NodeOptions &options)
: Node("topological_grid_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map/merged");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/topological_grid_assignments");
  summary_topic_ = this->declare_parameter<std::string>("summary_topic", "/topological_grid_assignments/summary");
  grid_spec_.cell_size = this->declare_parameter<double>("grid_size", 0.02);
  grid_spec_.origin_x = this->declare_parameter<double>("origin_x", 0.0);
  grid_spec_.origin_y = this->declare_parameter<double>("origin_y", 0.0);
  grid_spec_.origin_z = this->declare_parameter<double>("origin_z", 0.0);
  origin_shift_half_ = this->declare_parameter<bool>("origin_shift_half", false);
  x_shift_ = this->declare_parameter<int>("x_shift", 42);
  y_shift_ = this->declare_parameter<int>("y_shift", 21);
  z_shift_ = this->declare_parameter<int>("z_shift", 0);
  offset_ = this->declare_parameter<long>("offset", 1000000L);
  excluded_labels_ = parseExcludedLabels(this->declare_parameter<std::string>(
    "excluded_labels", "SAFE_TERRAIN,HUMAN,CAR"));
  const int minimum_observations = this->declare_parameter<int>("minimum_observations", 1);
  const int maximum_missed_updates = this->declare_parameter<int>("maximum_missed_updates", 0);

  if (grid_spec_.cell_size <= 0.0) {
    throw std::invalid_argument("grid_size must be positive");
  }
  if (minimum_observations <= 0 || maximum_missed_updates < 0) {
    throw std::invalid_argument(
            "minimum_observations must be positive and maximum_missed_updates non-negative");
  }
  minimum_observations_ = static_cast<std::size_t>(minimum_observations);
  maximum_missed_updates_ = static_cast<std::size_t>(maximum_missed_updates);
  temporal_filter_ = std::make_unique<TemporalVoxelFilter>(
    minimum_observations_, maximum_missed_updates_);

  if (origin_shift_half_) {
    const double half = grid_spec_.cell_size * 0.5;
    grid_spec_.origin_x = half;
    grid_spec_.origin_y = half;
    grid_spec_.origin_z = half;
  }

  voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    output_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic_, 10);
  map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
    input_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TopologicalGridNode::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "TopologicalGridNode ready: input=%s output=%s grid_size=%.3f origin=(%.3f, %.3f, %.3f) shifted=%s excluded_labels=[%s] stability=%zu observations/%zu missed",
    input_topic_.c_str(),
    output_topic_.c_str(),
    grid_spec_.cell_size,
    grid_spec_.origin_x,
    grid_spec_.origin_y,
    grid_spec_.origin_z,
    origin_shift_half_ ? "true" : "false",
    labelSetToString(excluded_labels_).c_str(),
    minimum_observations_,
    maximum_missed_updates_);
}

void TopologicalGridNode::mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  publishResult(*msg);
}

void TopologicalGridNode::publishResult(const ais_gng_msgs::msg::TopologicalMap &map)
{
  const auto result = voxelizeNodes(map, grid_spec_, excluded_labels_);
  const auto stable_voxels = temporal_filter_->update(result.voxels);

  voxel_msgs::msg::Voxel voxel_msg;
  voxel_msg.header.frame_id = map.header.frame_id;
  voxel_msg.header.stamp = map.header.stamp;
  voxel_msg.voxel_size = static_cast<float>(grid_spec_.cell_size);
  voxel_msg.origin_x = static_cast<float>(grid_spec_.origin_x);
  voxel_msg.origin_y = static_cast<float>(grid_spec_.origin_y);
  voxel_msg.origin_z = static_cast<float>(grid_spec_.origin_z);
  voxel_msg.x_shift = x_shift_;
  voxel_msg.y_shift = y_shift_;
  voxel_msg.z_shift = z_shift_;
  voxel_msg.offset = offset_;
  voxel_msg.data.reserve(stable_voxels.size());
  voxel_msg.labels.reserve(stable_voxels.size());
  for (const auto &voxel : stable_voxels) {
    const std::uint64_t flat_id =
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.x) + offset_) << x_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.y) + offset_) << y_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(voxel.cell.z) + offset_) << z_shift_);
    voxel_msg.data.push_back(flat_id);
    voxel_msg.labels.push_back(voxel.label);
  }
  voxel_pub_->publish(voxel_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"node_count\":" << result.included_node_count << ",";
  oss << "\"input_node_count\":" << map.nodes.size() << ",";
  oss << "\"included_node_count\":" << result.included_node_count << ",";
  oss << "\"excluded_node_count\":" << result.excluded_node_count << ",";
  oss << "\"voxel_count\":" << stable_voxels.size() << ",";
  oss << "\"observed_voxel_count\":" << result.voxels.size() << ",";
  oss << "\"tracked_voxel_count\":" << temporal_filter_->trackedVoxelCount() << ",";
  oss << "\"minimum_observations\":" << minimum_observations_ << ",";
  oss << "\"maximum_missed_updates\":" << maximum_missed_updates_ << ",";
  oss << "\"grid_size\":" << grid_spec_.cell_size << ",";
  oss << "\"voxel_size\":" << voxel_msg.voxel_size << ",";
  oss << "\"origin_x\":" << voxel_msg.origin_x << ",";
  oss << "\"origin_y\":" << voxel_msg.origin_y << ",";
  oss << "\"origin_z\":" << voxel_msg.origin_z << ",";
  oss << "\"origin\":[" << grid_spec_.origin_x << "," << grid_spec_.origin_y << "," << grid_spec_.origin_z << "],";
  oss << "\"x_shift\":" << x_shift_ << ",";
  oss << "\"y_shift\":" << y_shift_ << ",";
  oss << "\"z_shift\":" << z_shift_ << ",";
  oss << "\"offset\":" << offset_ << ",";
  oss << "\"excluded_labels\":[";
  std::vector<int> excluded_labels;
  excluded_labels.reserve(excluded_labels_.size());
  for (const auto label : excluded_labels_) {
    excluded_labels.push_back(static_cast<int>(label));
  }
  std::sort(excluded_labels.begin(), excluded_labels.end());
  for (std::size_t i = 0; i < excluded_labels.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << excluded_labels[i];
  }
  oss << "],";
  std::unordered_map<std::uint8_t, std::size_t> label_counts;
  for (const auto &voxel : stable_voxels) {
    ++label_counts[voxel.label];
  }
  std::vector<std::pair<int, std::size_t>> sorted_label_counts;
  sorted_label_counts.reserve(label_counts.size());
  for (const auto &[label, count] : label_counts) {
    sorted_label_counts.emplace_back(static_cast<int>(label), count);
  }
  std::sort(sorted_label_counts.begin(), sorted_label_counts.end());
  oss << "\"label_counts\":{";
  for (std::size_t i = 0; i < sorted_label_counts.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << "\"" << sorted_label_counts[i].first << "\":" << sorted_label_counts[i].second;
  }
  oss << "},";
  oss << "\"assignments\":[";
  for (std::size_t i = 0; i < stable_voxels.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    const auto &voxel = stable_voxels[i];
    oss << "{\"cell\":[" << voxel.cell.x << "," << voxel.cell.y << "," << voxel.cell.z << "]"
        << ",\"voxel_id\":" << voxel_msg.data[i]
        << ",\"label\":" << static_cast<int>(voxel.label)
        << ",\"node_count\":" << voxel.node_count << "}";
  }
  oss << "]";
  oss << "}";
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Topological grid: input_nodes=%zu included=%zu excluded=%zu observed_voxels=%zu stable_voxels=%zu",
    map.nodes.size(), result.included_node_count, result.excluded_node_count,
    result.voxels.size(), stable_voxels.size());
}

}  // namespace fuzzrobo::topological_grid

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fuzzrobo::topological_grid::TopologicalGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
