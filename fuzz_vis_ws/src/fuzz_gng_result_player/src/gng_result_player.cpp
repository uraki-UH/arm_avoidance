#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/msg/topological_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using TopologicalMapMsg = ais_gng_msgs::msg::TopologicalMap;
using TopologicalNodeMsg = ais_gng_msgs::msg::TopologicalNode;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
using MarkerMsg = visualization_msgs::msg::Marker;

constexpr const char *kDefaultFrameId = "camera_color_optical_frame";
constexpr const char *kDefaultDataDirectory = "gng_results";
constexpr const char *kDefaultPhase2Suffix = "_phase2";
constexpr const char *kDefaultExperimentId = "topoarm_real_v1";

std::string trim(const std::string &value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::map<std::string, std::string> loadKeyValueFile(const std::filesystem::path &path) {
  std::map<std::string, std::string> result;
  std::ifstream ifs(path);
  if (!ifs) {
    return result;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const auto key = trim(line.substr(0, eq));
    const auto val = trim(line.substr(eq + 1));
    if (!key.empty()) {
      result[key] = val;
    }
  }

  return result;
}

template <typename T>
bool readBinary(std::istream &in, T &value) {
  in.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(in);
}

bool readEigenVector(std::istream &in, std::vector<float> &out) {
  Eigen::Index rows = 0;
  Eigen::Index cols = 0;
  if (!readBinary(in, rows) || !readBinary(in, cols)) {
    return false;
  }
  if (rows < 0 || cols < 0) {
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  out.resize(count);
  if (count == 0) {
    return true;
  }
  in.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(count * sizeof(float)));
  return static_cast<bool>(in);
}

struct LoadedNode {
  int original_id = -1;
  bool active = false;
  bool valid = true;
  bool is_surface = false;
  bool is_active_surface = false;
  bool is_boundary = false;
  bool is_mainland = true;
  bool is_colliding = false;
  bool is_danger = false;
  int topology_group_id = -1;
  int level = 0;
  float rho = 1.0f;
  std::array<float, 3> pos{0.0f, 0.0f, 0.0f};
  std::array<float, 3> normal{1.0f, 0.0f, 0.0f};
};

struct LoadedGraph {
  std::vector<LoadedNode> nodes;
  std::vector<uint16_t> edges;
};

std::array<float, 3> labelColor(uint8_t label) {
  switch (label) {
    case TopologicalMapMsg::SAFE_TERRAIN:
      return {1.0f, 1.0f, 1.0f};
    case TopologicalMapMsg::WALL:
      return {0.1f, 0.4f, 1.0f};
    case TopologicalMapMsg::UNKNOWN_OBJECT:
      return {1.0f, 1.0f, 0.1f};
    case TopologicalMapMsg::HUMAN:
      return {1.0f, 0.1f, 0.1f};
    case TopologicalMapMsg::CAR:
      return {0.1f, 1.0f, 0.1f};
    case TopologicalMapMsg::DEFAULT:
    default:
      return {0.75f, 0.75f, 0.75f};
  }
}

uint8_t deriveLabel(const LoadedNode &node) {
  if (!node.active || !node.valid) {
    return TopologicalMapMsg::DEFAULT;
  }
  if (node.is_colliding) {
    return TopologicalMapMsg::HUMAN;
  }
  if (node.is_danger) {
    return TopologicalMapMsg::UNKNOWN_OBJECT;
  }
  if (node.is_boundary) {
    return TopologicalMapMsg::WALL;
  }
  if (node.is_mainland) {
    return TopologicalMapMsg::CAR;
  }
  if (node.is_active_surface || node.is_surface) {
    return TopologicalMapMsg::SAFE_TERRAIN;
  }
  return TopologicalMapMsg::DEFAULT;
}

bool loadGngResultFile(const std::filesystem::path &file_path, LoadedGraph &graph) {
  std::ifstream ifs(file_path, std::ios::binary);
  if (!ifs) {
    return false;
  }

  graph.nodes.clear();
  graph.edges.clear();

  uint32_t version = 0;
  if (!readBinary(ifs, version)) {
    return false;
  }
  if (version < 1 || version > 6) {
    return false;
  }

  int node_count = 0;
  if (!readBinary(ifs, node_count) || node_count < 0) {
    return false;
  }

  graph.nodes.reserve(static_cast<std::size_t>(node_count));
  std::unordered_map<int, std::size_t> id_to_index;
  id_to_index.reserve(static_cast<std::size_t>(node_count));

  for (int i = 0; i < node_count; ++i) {
    LoadedNode node;
    if (!readBinary(ifs, node.original_id)) {
      return false;
    }

    float dummy_float = 0.0f;
    if (!readBinary(ifs, dummy_float) || !readBinary(ifs, dummy_float)) {
      return false;
    }

    std::vector<float> weight_angle;
    std::vector<float> weight_coord;
    if (!readEigenVector(ifs, weight_angle) || !readEigenVector(ifs, weight_coord)) {
      return false;
    }
    (void)weight_angle;
    if (weight_coord.size() >= 3) {
      node.pos = {weight_coord[0], weight_coord[1], weight_coord[2]};
    }

    if (!readBinary(ifs, node.level)) {
      return false;
    }
    bool flag = false;
    if (!readBinary(ifs, flag)) {
      return false;
    }
    node.is_surface = flag;
    if (!readBinary(ifs, flag)) {
      return false;
    }
    node.is_active_surface = flag;
    if (!readBinary(ifs, flag)) {
      return false;
    }
    node.valid = flag;
    if (!readBinary(ifs, flag)) {
      return false;
    }
    node.active = flag;
    if (version >= 5) {
      if (!readBinary(ifs, flag)) {
        return false;
      }
      node.is_boundary = flag;
    }

    std::vector<float> ee_direction;
    if (!readEigenVector(ifs, ee_direction)) {
      return false;
    }
    if (ee_direction.size() >= 3) {
      node.normal = {ee_direction[0], ee_direction[1], ee_direction[2]};
    }

    if (version >= 4) {
      float manip = 0.0f;
      float min_singular = 0.0f;
      float joint_limit_score = 0.0f;
      float combined_score = 0.0f;
      float dynamic_manip = 0.0f;
      bool manip_valid = false;
      if (!readBinary(ifs, manip) || !readBinary(ifs, min_singular) ||
          !readBinary(ifs, joint_limit_score) || !readBinary(ifs, combined_score) ||
          !readBinary(ifs, manip_valid) || !readBinary(ifs, dynamic_manip)) {
        return false;
      }
      (void)manip;
      (void)min_singular;
      (void)joint_limit_score;
      (void)combined_score;
      (void)manip_valid;
      (void)dynamic_manip;
    }

    int jp_size = 0;
    if (!readBinary(ifs, jp_size) || jp_size < 0) {
      return false;
    }
    for (int j = 0; j < jp_size; ++j) {
      std::vector<float> joint_pos;
      if (!readEigenVector(ifs, joint_pos)) {
        return false;
      }
    }

    graph.nodes.push_back(node);
    id_to_index[node.original_id] = graph.nodes.size() - 1;
  }

  int angle_edge_count = 0;
  if (!readBinary(ifs, angle_edge_count) || angle_edge_count < 0) {
    return false;
  }

  std::set<std::pair<uint16_t, uint16_t>> unique_edges;
  for (int i = 0; i < angle_edge_count; ++i) {
    int n1 = 0;
    int n2 = 0;
    int age = 0;
    bool active = true;
    if (!readBinary(ifs, n1) || !readBinary(ifs, n2) || !readBinary(ifs, age)) {
      return false;
    }
    if (version >= 3 && !readBinary(ifs, active)) {
      return false;
    }
    (void)age;
    if (!active) {
      continue;
    }
    const auto it1 = id_to_index.find(n1);
    const auto it2 = id_to_index.find(n2);
    if (it1 == id_to_index.end() || it2 == id_to_index.end()) {
      continue;
    }
    const uint16_t a = static_cast<uint16_t>(it1->second);
    const uint16_t b = static_cast<uint16_t>(it2->second);
    if (a != b) {
      unique_edges.emplace(std::min(a, b), std::max(a, b));
    }
  }

  int coord_edge_count = 0;
  if (!readBinary(ifs, coord_edge_count) || coord_edge_count < 0) {
    return false;
  }
  for (int i = 0; i < coord_edge_count; ++i) {
    int n1 = 0;
    int n2 = 0;
    int age = 0;
    bool active = true;
    if (!readBinary(ifs, n1) || !readBinary(ifs, n2) || !readBinary(ifs, age)) {
      return false;
    }
    if (version >= 3 && !readBinary(ifs, active)) {
      return false;
    }
    (void)n1;
    (void)n2;
    (void)age;
    (void)active;
  }

  graph.edges.reserve(unique_edges.size() * 2);
  for (const auto &edge : unique_edges) {
    graph.edges.push_back(edge.first);
    graph.edges.push_back(edge.second);
  }

  return true;
}

TopologicalMapMsg buildMessage(const LoadedGraph &graph, const std::string &frame_id) {
  TopologicalMapMsg msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.nodes.reserve(graph.nodes.size());

  for (const auto &node : graph.nodes) {
    TopologicalNodeMsg node_msg;
    node_msg.id = static_cast<uint16_t>(std::max(node.original_id, 0));
    node_msg.pos.x = node.pos[0];
    node_msg.pos.y = node.pos[1];
    node_msg.pos.z = node.pos[2];
    node_msg.normal.x = node.normal[0];
    node_msg.normal.y = node.normal[1];
    node_msg.normal.z = node.normal[2];
    node_msg.rho = node.rho;
    node_msg.label = deriveLabel(node);
    node_msg.age = 0;
    msg.nodes.emplace_back(node_msg);
  }

  msg.edges = graph.edges;
  msg.clusters.clear();
  return msg;
}

MarkerArrayMsg buildMarkerArray(const LoadedGraph &graph, const std::string &frame_id,
                                bool publish_edges) {
  MarkerArrayMsg array;
  const auto add_node_marker = [&](uint8_t label, int id) {
    MarkerMsg marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    marker.ns = "nodes";
    marker.id = id;
    marker.type = MarkerMsg::SPHERE_LIST;
    marker.action = MarkerMsg::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    const auto rgb = labelColor(label);
    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
    marker.color.a = 0.9f;
    return marker;
  };

  std::array<MarkerMsg, 6> node_markers = {
      add_node_marker(TopologicalMapMsg::DEFAULT, 0),
      add_node_marker(TopologicalMapMsg::SAFE_TERRAIN, 1),
      add_node_marker(TopologicalMapMsg::WALL, 2),
      add_node_marker(TopologicalMapMsg::UNKNOWN_OBJECT, 3),
      add_node_marker(TopologicalMapMsg::HUMAN, 4),
      add_node_marker(TopologicalMapMsg::CAR, 5),
  };

  std::array<std::vector<geometry_msgs::msg::Point>, 6> points_by_label;
  const auto make_point = [](float x, float y, float z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };
  for (const auto &node : graph.nodes) {
    if (!node.active || !node.valid) {
      points_by_label[TopologicalMapMsg::DEFAULT].push_back(
          make_point(node.pos[0], node.pos[1], node.pos[2]));
      continue;
    }
    const uint8_t label = deriveLabel(node);
    points_by_label[label].push_back(
        make_point(node.pos[0], node.pos[1], node.pos[2]));
  }

  for (std::size_t i = 0; i < node_markers.size(); ++i) {
    node_markers[i].points = std::move(points_by_label[i]);
    array.markers.emplace_back(node_markers[i]);
  }

  if (publish_edges) {
    MarkerMsg edges;
    edges.header.frame_id = frame_id;
    edges.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    edges.ns = "edges";
    edges.id = 100;
    edges.type = MarkerMsg::LINE_LIST;
    edges.action = MarkerMsg::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.01;
    edges.color.r = 0.8f;
    edges.color.g = 0.8f;
    edges.color.b = 0.8f;
    edges.color.a = 0.6f;

    for (std::size_t i = 0; i + 1 < graph.edges.size(); i += 2) {
      const auto a = graph.edges[i];
      const auto b = graph.edges[i + 1];
      if (a >= graph.nodes.size() || b >= graph.nodes.size()) {
        continue;
      }
      geometry_msgs::msg::Point p1;
      geometry_msgs::msg::Point p2;
      p1.x = graph.nodes[a].pos[0];
      p1.y = graph.nodes[a].pos[1];
      p1.z = graph.nodes[a].pos[2];
      p2.x = graph.nodes[b].pos[0];
      p2.y = graph.nodes[b].pos[1];
      p2.z = graph.nodes[b].pos[2];
      edges.points.emplace_back(p1);
      edges.points.emplace_back(p2);
    }
    array.markers.emplace_back(edges);
  }

  return array;
}

struct PlayerConfig {
  std::string experiment_id = kDefaultExperimentId;
  std::string data_directory = kDefaultDataDirectory;
  std::string phase2_suffix = kDefaultPhase2Suffix;
  std::string config_file;
  std::string frame_id = kDefaultFrameId;
  int poll_ms = 1000;
};

} // namespace

class GngResultPlayer : public rclcpp::Node {
public:
  GngResultPlayer()
  : Node("gng_result_player") {
    config_.experiment_id = declare_parameter<std::string>("experiment_id", kDefaultExperimentId);
    config_.data_directory = declare_parameter<std::string>("data_directory", kDefaultDataDirectory);
    config_.phase2_suffix = declare_parameter<std::string>("phase2_suffix", kDefaultPhase2Suffix);
    config_.config_file = declare_parameter<std::string>("config_file", "");
    config_.frame_id = declare_parameter<std::string>("frame_id", kDefaultFrameId);
    config_.poll_ms = declare_parameter<int>("poll_ms", 1000);
    publish_edges_ = declare_parameter<bool>("publish_edges", false);
    republish_ms_ = declare_parameter<int>("republish_ms", 1000);

    applyConfigFileOverrides();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    topological_map_pub_ =
        create_publisher<TopologicalMapMsg>("topological_map", latched_qos);
    marker_pub_ =
        create_publisher<MarkerArrayMsg>("marker", latched_qos);

    const auto period = std::chrono::milliseconds(std::max(config_.poll_ms, 100));
    timer_ = create_wall_timer(period, std::bind(&GngResultPlayer::tick, this));

    RCLCPP_INFO(get_logger(),
                "gng_result_player started: id=%s data_dir=%s suffix=%s frame=%s",
                config_.experiment_id.c_str(), config_.data_directory.c_str(),
                config_.phase2_suffix.c_str(), config_.frame_id.c_str());
  }

private:
  void applyConfigFileOverrides() {
    if (config_.config_file.empty()) {
      return;
    }

    const std::filesystem::path config_path(config_.config_file);
    if (!std::filesystem::exists(config_path)) {
      RCLCPP_WARN(get_logger(), "Config file not found: %s", config_.config_file.c_str());
      return;
    }

    const auto values = loadKeyValueFile(config_path);
    const auto get_value = [&](const std::string &key) -> std::string {
      const auto it = values.find(key);
      return it == values.end() ? std::string() : it->second;
    };

    if (config_.experiment_id == kDefaultExperimentId) {
      const auto v = get_value("experiment_id");
      if (!v.empty()) {
        config_.experiment_id = v;
      }
    }
    if (config_.data_directory == kDefaultDataDirectory) {
      const auto v = get_value("data_directory");
      if (!v.empty()) {
        config_.data_directory = v;
      }
    }
    if (config_.phase2_suffix == kDefaultPhase2Suffix) {
      const auto v = get_value("phase2_output_suffix");
      if (!v.empty()) {
        config_.phase2_suffix = v;
      } else {
        const auto legacy = get_value("online_input_suffix");
        if (!legacy.empty()) {
          config_.phase2_suffix = legacy;
        }
      }
    }
  }

  std::filesystem::path resultFilePath() const {
    std::filesystem::path dir(config_.data_directory);
    const auto nested = dir / config_.experiment_id /
                        (config_.experiment_id + config_.phase2_suffix + ".bin");
    if (std::filesystem::exists(nested)) {
      return nested;
    }

    const auto flat = dir / (config_.experiment_id + config_.phase2_suffix + ".bin");
    if (std::filesystem::exists(flat)) {
      return flat;
    }

    return nested;
  }

  void tick() {
    const auto path = resultFilePath();
    if (!std::filesystem::exists(path)) {
      if (!missing_file_warned_) {
        RCLCPP_WARN(get_logger(), "Result file not found: %s", path.c_str());
        missing_file_warned_ = true;
      }
      return;
    }
    missing_file_warned_ = false;

    std::error_code ec;
    const auto mtime = std::filesystem::last_write_time(path, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Failed to stat result file: %s", path.c_str());
      return;
    }

    if (!have_last_mtime_ || mtime != last_mtime_) {
      LoadedGraph graph;
      if (!loadGngResultFile(path, graph)) {
        RCLCPP_ERROR(get_logger(), "Failed to load result file: %s", path.c_str());
        return;
      }

      last_mtime_ = mtime;
      have_last_mtime_ = true;
      last_graph_ = std::move(graph);
      publishGraph();
      RCLCPP_INFO(get_logger(), "Reloaded %zu nodes / %zu edges from %s",
                  last_graph_.nodes.size(), last_graph_.edges.size() / 2,
                  path.c_str());
      return;
    }

    if (have_last_mtime_ && !last_graph_.nodes.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (!have_last_publish_ ||
          now - last_publish_time_ >= std::chrono::milliseconds(std::max(republish_ms_, 200))) {
        publishGraph();
      }
    }
  }

  void publishGraph() {
    if (last_graph_.nodes.empty()) {
      return;
    }

    float min_x = last_graph_.nodes.front().pos[0];
    float min_y = last_graph_.nodes.front().pos[1];
    float min_z = last_graph_.nodes.front().pos[2];
    float max_x = min_x;
    float max_y = min_y;
    float max_z = min_z;
    for (const auto &node : last_graph_.nodes) {
      min_x = std::min(min_x, node.pos[0]);
      min_y = std::min(min_y, node.pos[1]);
      min_z = std::min(min_z, node.pos[2]);
      max_x = std::max(max_x, node.pos[0]);
      max_y = std::max(max_y, node.pos[1]);
      max_z = std::max(max_z, node.pos[2]);
    }

    RCLCPP_INFO(get_logger(),
                "Publishing MarkerArray: nodes=%zu edges=%zu bbox=[(%.3f, %.3f, %.3f) - (%.3f, %.3f, %.3f)]",
                last_graph_.nodes.size(), last_graph_.edges.size() / 2, min_x, min_y, min_z,
                max_x, max_y, max_z);

    const auto msg = buildMessage(last_graph_, config_.frame_id);
    topological_map_pub_->publish(msg);
    marker_pub_->publish(buildMarkerArray(last_graph_, config_.frame_id, publish_edges_));
    last_publish_time_ = std::chrono::steady_clock::now();
    have_last_publish_ = true;
  }

private:
  PlayerConfig config_;
  rclcpp::Publisher<TopologicalMapMsg>::SharedPtr topological_map_pub_;
  rclcpp::Publisher<MarkerArrayMsg>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  LoadedGraph last_graph_;
  bool publish_edges_ = false;
  int republish_ms_ = 1000;
  std::filesystem::file_time_type last_mtime_{};
  std::chrono::steady_clock::time_point last_publish_time_{};
  bool have_last_mtime_ = false;
  bool have_last_publish_ = false;
  bool missing_file_warned_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GngResultPlayer>());
  rclcpp::shutdown();
  return 0;
}
