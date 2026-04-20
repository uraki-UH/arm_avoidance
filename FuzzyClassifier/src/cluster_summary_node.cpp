#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "ais_gng_msgs/msg/topological_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
double safe_double(const double value)
{
  return std::isfinite(value) ? value : 0.0;
}

double mean(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

std::string label_name(const uint8_t label)
{
  switch (label) {
    case ais_gng_msgs::msg::TopologicalMap::DEFAULT:
      return "DEFAULT";
    case ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN:
      return "SAFE_TERRAIN";
    case ais_gng_msgs::msg::TopologicalMap::WALL:
      return "WALL";
    case ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT:
      return "UNKNOWN_OBJECT";
    case ais_gng_msgs::msg::TopologicalMap::HUMAN:
      return "HUMAN";
    case ais_gng_msgs::msg::TopologicalMap::CAR:
      return "CAR";
    default:
      return "UNDEFINED_" + std::to_string(label);
  }
}

std::array<float, 3> label_color(const uint8_t label)
{
  switch (label) {
    case ais_gng_msgs::msg::TopologicalMap::DEFAULT:
      return {0.5F, 0.5F, 0.5F};
    case ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN:
      return {1.0F, 0.0F, 1.0F};
    case ais_gng_msgs::msg::TopologicalMap::WALL:
      return {0.0F, 0.0F, 1.0F};
    case ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT:
      return {1.0F, 1.0F, 0.0F};
    case ais_gng_msgs::msg::TopologicalMap::HUMAN:
      return {1.0F, 0.0F, 0.0F};
    case ais_gng_msgs::msg::TopologicalMap::CAR:
      return {0.0F, 1.0F, 0.0F};
    default:
      return {1.0F, 1.0F, 0.0F};
  }
}

bool is_known_label(const uint8_t label)
{
  switch (label) {
    case ais_gng_msgs::msg::TopologicalMap::DEFAULT:
    case ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN:
    case ais_gng_msgs::msg::TopologicalMap::WALL:
    case ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT:
    case ais_gng_msgs::msg::TopologicalMap::HUMAN:
    case ais_gng_msgs::msg::TopologicalMap::CAR:
      return true;
    default:
      return false;
  }
}

std::string escape_json(const std::string & src)
{
  std::ostringstream oss;
  for (const unsigned char ch : src) {
    switch (ch) {
      case '\"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (ch < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          oss << ch;
        }
        break;
    }
  }
  return oss.str();
}

class JsonWriter
{
public:
  explicit JsonWriter(const bool pretty) : pretty_(pretty) {}

  void begin_object()
  {
    prepare_value();
    oss_ << "{";
    stack_.push_back(Context{ContextType::Object});
  }

  void end_object()
  {
    if (stack_.empty() || stack_.back().type != ContextType::Object) {
      return;
    }
    const Context ctx = stack_.back();
    const std::size_t depth = stack_.size() - 1;
    if (pretty_ && !ctx.first) {
      oss_ << "\n" << std::string(depth * 2, ' ');
    }
    stack_.pop_back();
    oss_ << "}";
  }

  void begin_array()
  {
    prepare_value();
    oss_ << "[";
    stack_.push_back(Context{ContextType::Array});
  }

  void end_array()
  {
    if (stack_.empty() || stack_.back().type != ContextType::Array) {
      return;
    }
    const Context ctx = stack_.back();
    const std::size_t depth = stack_.size() - 1;
    if (pretty_ && !ctx.first) {
      oss_ << "\n" << std::string(depth * 2, ' ');
    }
    stack_.pop_back();
    oss_ << "]";
  }

  void key(const std::string & key_name)
  {
    if (stack_.empty() || stack_.back().type != ContextType::Object) {
      return;
    }
    auto & ctx = stack_.back();
    if (!ctx.first) {
      oss_ << ",";
    }
    if (pretty_) {
      oss_ << "\n" << std::string(stack_.size() * 2, ' ');
    }
    ctx.first = false;
    ctx.after_key = true;

    oss_ << "\"" << escape_json(key_name) << "\"";
    oss_ << (pretty_ ? ": " : ":");
  }

  void value_string(const std::string & value)
  {
    prepare_value();
    oss_ << "\"" << escape_json(value) << "\"";
  }

  void value_bool(const bool value)
  {
    prepare_value();
    oss_ << (value ? "true" : "false");
  }

  void value_int(const std::int64_t value)
  {
    prepare_value();
    oss_ << value;
  }

  void value_double(const double value)
  {
    prepare_value();
    oss_ << std::setprecision(10) << safe_double(value);
  }

  std::string str() const
  {
    return oss_.str();
  }

private:
  enum class ContextType
  {
    Object,
    Array
  };

  struct Context
  {
    ContextType type;
    bool first = true;
    bool after_key = false;
  };

  void prepare_value()
  {
    if (stack_.empty()) {
      return;
    }

    auto & ctx = stack_.back();
    if (ctx.type == ContextType::Object) {
      if (ctx.after_key) {
        ctx.after_key = false;
      }
      return;
    }

    if (!ctx.first) {
      oss_ << ",";
    }
    if (pretty_) {
      oss_ << "\n" << std::string(stack_.size() * 2, ' ');
    }
    ctx.first = false;
  }

  bool pretty_;
  std::vector<Context> stack_;
  std::ostringstream oss_;
};

void write_vec3_array(JsonWriter & writer, const std::array<double, 3> & values)
{
  writer.begin_array();
  writer.value_double(values[0]);
  writer.value_double(values[1]);
  writer.value_double(values[2]);
  writer.end_array();
}

std::vector<std::pair<std::string, std::int64_t>> sorted_label_counts(
  const std::map<int, std::int64_t> & counts)
{
  const std::vector<int> ordered_labels = {
    ais_gng_msgs::msg::TopologicalMap::DEFAULT,
    ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN,
    ais_gng_msgs::msg::TopologicalMap::WALL,
    ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT,
    ais_gng_msgs::msg::TopologicalMap::HUMAN,
    ais_gng_msgs::msg::TopologicalMap::CAR};
  const std::set<int> ordered_set(ordered_labels.begin(), ordered_labels.end());

  std::vector<std::pair<std::string, std::int64_t>> out;
  for (const auto label : ordered_labels) {
    const auto it = counts.find(label);
    if (it != counts.end() && it->second > 0) {
      out.emplace_back(label_name(static_cast<uint8_t>(label)), it->second);
    }
  }
  for (const auto & [label, value] : counts) {
    if (value <= 0 || ordered_set.count(label) > 0) {
      continue;
    }
    out.emplace_back(label_name(static_cast<uint8_t>(label)), value);
  }
  return out;
}

void write_label_count_object(
  JsonWriter & writer,
  const std::vector<std::pair<std::string, std::int64_t>> & items)
{
  writer.begin_object();
  for (const auto & [label, value] : items) {
    writer.key(label);
    writer.value_int(value);
  }
  writer.end_object();
}

std::string format_label_count_log(
  const std::vector<std::pair<std::string, std::int64_t>> & items)
{
  std::ostringstream oss;
  oss << "{";
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << items[i].first << ":" << items[i].second;
  }
  oss << "}";
  return oss.str();
}

struct ClusterDetail
{
  std::uint32_t id = 0;
  std::uint8_t label_raw = 0;
  std::uint8_t label_inferred_raw = 0;
  std::string label;
  std::string label_inferred;
  std::int64_t node_count = 0;
  std::uint32_t frame = 0;
  std::uint32_t age = 0;
  double reliability = 0.0;
  double match = 0.0;
  double speed_mps = 0.0;
  bool has_velocity_observation = false;
  double vel_cov_xx = 0.0;
  double vel_cov_xy = 0.0;
  double vel_cov_yy = 0.0;
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 3> scale{0.0, 0.0, 0.0};
  std::array<double, 3> velocity{0.0, 0.0, 0.0};
};
}  // namespace

class ClusterSummaryNode : public rclcpp::Node
{
public:
  ClusterSummaryNode()
  : Node("fuzzy_cluster_summary")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map");
    output_topic_ =
      this->declare_parameter<std::string>("output_topic", "/fuzzy_classifier/cluster_summary");
    box_topic_ =
      this->declare_parameter<std::string>("box_topic", "/fuzzy_classifier/cluster_boxes");
    node_topic_ =
      this->declare_parameter<std::string>("node_topic", "/fuzzy_classifier/cluster_nodes");
    include_cluster_details_ = this->declare_parameter<bool>("include_cluster_details", true);
    max_detail_clusters_ = static_cast<int>(
      this->declare_parameter<std::int64_t>("max_detail_clusters", 200));
    pretty_json_ = this->declare_parameter<bool>("pretty_json", false);
    moving_speed_threshold_mps_ =
      this->declare_parameter<double>("moving_speed_threshold_mps", 0.2);
    log_interval_sec_ = this->declare_parameter<double>("log_interval_sec", 1.0);
    publish_cluster_boxes_ = this->declare_parameter<bool>("publish_cluster_boxes", true);
    publish_cluster_nodes_ = this->declare_parameter<bool>("publish_cluster_nodes", false);
    render_safe_terrain_box_ = this->declare_parameter<bool>("render_safe_terrain_box", false);
    render_safe_terrain_node_ = this->declare_parameter<bool>("render_safe_terrain_node", false);
    box_alpha_ = this->declare_parameter<double>("box_alpha", 0.35);
    node_alpha_ = this->declare_parameter<double>("node_alpha", 0.9);
    node_scale_ = this->declare_parameter<double>("node_scale", 0.08);
    color_label_source_ =
      this->declare_parameter<std::string>("color_label_source", "label");

    if (max_detail_clusters_ < 0) {
      max_detail_clusters_ = 0;
    }
    if (log_interval_sec_ < 0.0) {
      log_interval_sec_ = 0.0;
    }
    box_alpha_ = std::clamp(box_alpha_, 0.0, 1.0);
    node_alpha_ = std::clamp(node_alpha_, 0.0, 1.0);
    node_scale_ = std::max(node_scale_, 0.001);
    if (color_label_source_ == "label_inferred") {
      RCLCPP_WARN(
        this->get_logger(),
        "color_label_source='label_inferred' is treated as 'label' for ais_gng_gpu messages.");
      color_label_source_ = "label";
    }
    if (color_label_source_ != "label" && color_label_source_ != "label_inferred") {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid color_label_source='%s'. Fallback to 'label'.",
        color_label_source_.c_str());
      color_label_source_ = "label";
    }

    summary_pub_ = this->create_publisher<std_msgs::msg::String>(output_topic_, 10);
    if (publish_cluster_boxes_) {
      box_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(box_topic_, 10);
    }
    if (publish_cluster_nodes_) {
      node_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(node_topic_, 10);
    }
    topological_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
      input_topic_,
      10,
      std::bind(&ClusterSummaryNode::topological_map_cb, this, std::placeholders::_1));

    last_log_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Cluster summary node started: subscribe=%s summary_publish=%s box_publish=%s node_publish=%s color_source=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      publish_cluster_boxes_ ? box_topic_.c_str() : "disabled",
      publish_cluster_nodes_ ? node_topic_.c_str() : "disabled",
      color_label_source_.c_str());
  }

private:
  std::uint8_t color_label(const ais_gng_msgs::msg::TopologicalCluster & cluster) const
  {
    const auto normalize_label = [](const std::uint8_t label_value) {
        return is_known_label(label_value) ?
               label_value : ais_gng_msgs::msg::TopologicalMap::UNKNOWN_OBJECT;
      };
    (void)color_label_source_;
    return normalize_label(cluster.label);
  }

  void publish_cluster_boxes(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
  {
    if (!publish_cluster_boxes_ || !box_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(clear_marker);

    int marker_id = 0;
    for (const auto & cluster : msg->clusters) {
      const std::uint8_t marker_label = color_label(cluster);
      if (!render_safe_terrain_box_ &&
        marker_label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN)
      {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "cluster_box";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = cluster.pos.x;
      marker.pose.position.y = cluster.pos.y;
      marker.pose.position.z = cluster.pos.z;
      marker.pose.orientation = cluster.quat;
      marker.scale.x = std::max(cluster.scale.x, 0.01F);
      marker.scale.y = std::max(cluster.scale.y, 0.01F);
      marker.scale.z = std::max(cluster.scale.z, 0.01F);

      const auto color = label_color(marker_label);
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = static_cast<float>(box_alpha_);
      marker.lifetime = rclcpp::Duration::from_seconds(0.0);
      marker.frame_locked = false;
      marker_array.markers.emplace_back(marker);
    }

    box_pub_->publish(marker_array);
  }

  void publish_cluster_nodes(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
  {
    if (!publish_cluster_nodes_ || !node_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(clear_marker);

    int marker_id = 0;
    for (const auto & cluster : msg->clusters) {
      const std::uint8_t marker_label = color_label(cluster);
      if (!render_safe_terrain_node_ &&
        marker_label == ais_gng_msgs::msg::TopologicalMap::SAFE_TERRAIN)
      {
        continue;
      }

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "cluster_nodes";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = static_cast<float>(node_scale_);
      marker.scale.y = static_cast<float>(node_scale_);
      marker.scale.z = static_cast<float>(node_scale_);

      const auto color = label_color(marker_label);
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = static_cast<float>(node_alpha_);
      marker.lifetime = rclcpp::Duration::from_seconds(0.0);
      marker.frame_locked = false;
      marker.points.reserve(cluster.nodes.size());

      for (const auto node_index : cluster.nodes) {
        if (static_cast<std::size_t>(node_index) >= msg->nodes.size()) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = msg->nodes[node_index].pos.x;
        point.y = msg->nodes[node_index].pos.y;
        point.z = msg->nodes[node_index].pos.z;
        marker.points.emplace_back(point);
      }

      if (!marker.points.empty()) {
        marker_array.markers.emplace_back(marker);
      }
    }

    node_pub_->publish(marker_array);
  }

  void topological_map_cb(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
  {
    std::map<int, std::int64_t> label_counts;
    std::map<int, std::int64_t> inferred_counts;

    std::vector<double> reliabilities;
    std::vector<double> matches;
    std::vector<double> speeds;
    std::vector<double> volumes;
    std::vector<double> node_counts_for_mean;
    std::vector<std::int64_t> node_counts;
    std::vector<ClusterDetail> detail_rows;

    reliabilities.reserve(msg->clusters.size());
    matches.reserve(msg->clusters.size());
    speeds.reserve(msg->clusters.size());
    volumes.reserve(msg->clusters.size());
    node_counts_for_mean.reserve(msg->clusters.size());
    node_counts.reserve(msg->clusters.size());
    if (include_cluster_details_) {
      detail_rows.reserve(msg->clusters.size());
    }

    for (const auto & cluster : msg->clusters) {
      ++label_counts[cluster.label];
      ++inferred_counts[cluster.label];

      const auto node_count = static_cast<std::int64_t>(cluster.nodes.size());
      const double speed = cluster.has_velocity_observation ?
        std::sqrt(
        cluster.velocity.x * cluster.velocity.x + cluster.velocity.y * cluster.velocity.y +
        cluster.velocity.z * cluster.velocity.z) : 0.0;
      const double volume = std::max(cluster.scale.x, 0.0F) * std::max(cluster.scale.y, 0.0F) *
        std::max(cluster.scale.z, 0.0F);
      const std::uint32_t age = msg->frame_number >= cluster.frame ?
        msg->frame_number - cluster.frame : 0U;

      reliabilities.push_back(safe_double(cluster.label_reliability));
      matches.push_back(safe_double(cluster.match));
      node_counts.push_back(node_count);
      node_counts_for_mean.push_back(static_cast<double>(node_count));
      speeds.push_back(safe_double(speed));
      volumes.push_back(safe_double(volume));

      if (include_cluster_details_) {
        ClusterDetail row;
        row.id = cluster.id;
        row.label_raw = cluster.label;
        row.label_inferred_raw = cluster.label;
        row.label = label_name(cluster.label);
        row.label_inferred = label_name(cluster.label);
        row.node_count = node_count;
        row.frame = cluster.frame;
        row.age = age;
        row.reliability = safe_double(cluster.label_reliability);
        row.match = safe_double(cluster.match);
        row.speed_mps = safe_double(speed);
        row.has_velocity_observation = cluster.has_velocity_observation;
        row.vel_cov_xx = safe_double(cluster.vel_cov_xx);
        row.vel_cov_xy = safe_double(cluster.vel_cov_xy);
        row.vel_cov_yy = safe_double(cluster.vel_cov_yy);
        row.position = {
          safe_double(cluster.pos.x),
          safe_double(cluster.pos.y),
          safe_double(cluster.pos.z)};
        row.scale = {
          safe_double(cluster.scale.x),
          safe_double(cluster.scale.y),
          safe_double(cluster.scale.z)};
        row.velocity = {
          safe_double(cluster.velocity.x),
          safe_double(cluster.velocity.y),
          safe_double(cluster.velocity.z)};
        detail_rows.emplace_back(std::move(row));
      }
    }

    const std::size_t cluster_count = msg->clusters.size();
    const std::size_t moving_cluster_count = static_cast<std::size_t>(std::count_if(
      speeds.begin(),
      speeds.end(),
      [this](const double speed) {return speed >= moving_speed_threshold_mps_;}));

    bool detail_truncated = false;
    if (include_cluster_details_ && static_cast<std::size_t>(max_detail_clusters_) < detail_rows.size()) {
      std::sort(
        detail_rows.begin(),
        detail_rows.end(),
        [](const ClusterDetail & lhs, const ClusterDetail & rhs) {
          return lhs.reliability > rhs.reliability;
        });
      detail_rows.resize(static_cast<std::size_t>(max_detail_clusters_));
      detail_truncated = true;
    }

    const auto label_summary = sorted_label_counts(label_counts);
    const auto label_inferred_summary = sorted_label_counts(inferred_counts);

    const double reliability_avg = mean(reliabilities);
    const double reliability_min = reliabilities.empty() ?
      0.0 : *std::min_element(reliabilities.begin(), reliabilities.end());
    const double reliability_max = reliabilities.empty() ?
      0.0 : *std::max_element(reliabilities.begin(), reliabilities.end());
    const double match_avg = mean(matches);
    const double node_count_avg = mean(node_counts_for_mean);
    const std::int64_t node_count_min = node_counts.empty() ?
      0 : *std::min_element(node_counts.begin(), node_counts.end());
    const std::int64_t node_count_max = node_counts.empty() ?
      0 : *std::max_element(node_counts.begin(), node_counts.end());
    const double speed_avg_mps = mean(speeds);
    const double volume_avg_m3 = mean(volumes);

    JsonWriter writer(pretty_json_);
    writer.begin_object();

    writer.key("header");
    writer.begin_object();
    writer.key("stamp_sec");
    writer.value_int(msg->header.stamp.sec);
    writer.key("stamp_nanosec");
    writer.value_int(msg->header.stamp.nanosec);
    writer.key("frame_id");
    writer.value_string(msg->header.frame_id);
    writer.end_object();

    writer.key("topological_map");
    writer.begin_object();
    writer.key("node_count");
    writer.value_int(static_cast<std::int64_t>(msg->nodes.size()));
    writer.key("edge_index_count");
    writer.value_int(static_cast<std::int64_t>(msg->edges.size()));
    writer.key("cluster_count");
    writer.value_int(static_cast<std::int64_t>(cluster_count));
    writer.key("frame_number");
    writer.value_int(msg->frame_number);
    writer.end_object();

    writer.key("cluster_counts");
    writer.begin_object();
    writer.key("label");
    write_label_count_object(writer, label_summary);
    writer.key("label_inferred");
    write_label_count_object(writer, label_inferred_summary);
    writer.end_object();

    writer.key("cluster_stats");
    writer.begin_object();
    writer.key("reliability_avg");
    writer.value_double(reliability_avg);
    writer.key("reliability_min");
    writer.value_double(reliability_min);
    writer.key("reliability_max");
    writer.value_double(reliability_max);
    writer.key("match_avg");
    writer.value_double(match_avg);
    writer.key("node_count_avg");
    writer.value_double(node_count_avg);
    writer.key("node_count_min");
    writer.value_int(node_count_min);
    writer.key("node_count_max");
    writer.value_int(node_count_max);
    writer.key("speed_avg_mps");
    writer.value_double(speed_avg_mps);
    writer.key("moving_cluster_count");
    writer.value_int(static_cast<std::int64_t>(moving_cluster_count));
    writer.key("moving_speed_threshold_mps");
    writer.value_double(moving_speed_threshold_mps_);
    writer.key("volume_avg_m3");
    writer.value_double(volume_avg_m3);
    writer.end_object();

    writer.key("cluster_details_count");
    writer.value_int(include_cluster_details_ ? static_cast<std::int64_t>(detail_rows.size()) : 0);
    writer.key("cluster_details_truncated");
    writer.value_bool(detail_truncated);

    if (include_cluster_details_) {
      writer.key("cluster_details");
      writer.begin_array();
      for (const auto & row : detail_rows) {
        writer.begin_object();
        writer.key("id");
        writer.value_int(row.id);
        writer.key("label");
        writer.value_string(row.label);
        writer.key("label_raw");
        writer.value_int(row.label_raw);
        writer.key("label_inferred");
        writer.value_string(row.label_inferred);
        writer.key("label_inferred_raw");
        writer.value_int(row.label_inferred_raw);
        writer.key("node_count");
        writer.value_int(row.node_count);
        writer.key("frame");
        writer.value_int(row.frame);
        writer.key("age");
        writer.value_int(row.age);
        writer.key("reliability");
        writer.value_double(row.reliability);
        writer.key("match");
        writer.value_double(row.match);
        writer.key("speed_mps");
        writer.value_double(row.speed_mps);
        writer.key("has_velocity_observation");
        writer.value_bool(row.has_velocity_observation);
        writer.key("velocity_covariance");
        writer.begin_array();
        writer.value_double(row.vel_cov_xx);
        writer.value_double(row.vel_cov_xy);
        writer.value_double(row.vel_cov_yy);
        writer.end_array();
        writer.key("position");
        write_vec3_array(writer, row.position);
        writer.key("scale");
        write_vec3_array(writer, row.scale);
        writer.key("velocity");
        write_vec3_array(writer, row.velocity);
        writer.end_object();
      }
      writer.end_array();
    }

    writer.end_object();

    std_msgs::msg::String out_msg;
    out_msg.data = writer.str();
    summary_pub_->publish(out_msg);
    publish_cluster_boxes(msg);
    publish_cluster_nodes(msg);

    const auto now = this->now();
    if (log_interval_sec_ == 0.0 || (now - last_log_time_).seconds() >= log_interval_sec_) {
      last_log_time_ = now;
      const std::string label_log = format_label_count_log(label_summary);
      RCLCPP_INFO(
        this->get_logger(),
        "clusters=%zu labels=%s moving=%zu avg_rel=%.3f",
        cluster_count,
        label_log.c_str(),
        moving_cluster_count,
        reliability_avg);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string box_topic_;
  std::string node_topic_;
  bool include_cluster_details_{true};
  int max_detail_clusters_{200};
  bool pretty_json_{false};
  double moving_speed_threshold_mps_{0.2};
  double log_interval_sec_{1.0};
  bool publish_cluster_boxes_{true};
  bool publish_cluster_nodes_{false};
  bool render_safe_terrain_box_{false};
  bool render_safe_terrain_node_{false};
  double box_alpha_{0.35};
  double node_alpha_{0.9};
  double node_scale_{0.08};
  std::string color_label_source_{"label"};
  rclcpp::Time last_log_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr node_pub_;
  rclcpp::Subscription<ais_gng_msgs::msg::TopologicalMap>::SharedPtr topological_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusterSummaryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
