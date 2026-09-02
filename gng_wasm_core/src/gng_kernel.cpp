#include "gng_wasm_core/gng_kernel.hpp"

#include "cugng.hpp"
#include "param.hpp"
#include "vec3f.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

namespace gng_wasm_core {

class GngKernel::GngCpuState {
public:
  Param param;
  CUGNG core;
  std::vector<Vec3f> input_points;
  std::vector<Vec3f> attention_points;
};

namespace {

constexpr std::uint32_t kMinNodeNum = 2;
constexpr std::uint32_t kMaxNodeNum = 4096;
constexpr float kRangeMargin = 1.0f;

float finite_or(float value, float fallback) {
  return std::isfinite(value) ? value : fallback;
}

}  // namespace

GngKernel::GngKernel() = default;

GngKernel::~GngKernel() = default;

void GngKernel::reset() {
  cpu_state_.reset();
  points_.clear();
  point_labels_.clear();
  nodes_.clear();
  edges_.clear();
  clusters_.clear();
  iter_ = 0;
}

void GngKernel::set_config(const Config &config) {
  config_ = config;
  cpu_state_.reset();
  nodes_.clear();
  edges_.clear();
  clusters_.clear();
  iter_ = 0;
}

bool GngKernel::set_parameter(const char *name, std::uint32_t, float value) {
  if (name == nullptr || !std::isfinite(value)) {
    return false;
  }

  const std::string parameter_name(name);
  if (parameter_name == "node.num_max" && value >= static_cast<float>(kMinNodeNum)) {
    config_.max_nodes = static_cast<std::uint32_t>(value);
  } else if (parameter_name == "node.learning_num" && value >= 1.0f) {
    config_.learning_num = static_cast<std::uint32_t>(value);
  } else if (parameter_name == "node.eta_s1" && value >= 0.0f && value <= 1.0f) {
    config_.eb = value;
  } else if (parameter_name == "node.eta_s2" && value >= 0.0f && value <= 1.0f) {
    config_.en = value;
  } else if (parameter_name == "edge.age_max" && value >= 0.0f) {
    config_.max_age = static_cast<std::uint32_t>(value);
  } else {
    return false;
  }

  cpu_state_.reset();
  return true;
}

void GngKernel::set_points(const std::vector<Point3f> &points) {
  points_ = points;
  cpu_state_.reset();
  nodes_.clear();
  edges_.clear();
  clusters_.clear();
  iter_ = 0;
}

void GngKernel::update_points(const std::vector<Point3f> &points) {
  points_ = points;
  if (!cpu_state_) {
    return;
  }

  auto &input_points = cpu_state_->input_points;
  input_points.clear();
  input_points.reserve(points_.size());
  for (const auto &point : points_) {
    input_points.emplace_back(
        finite_or(point.x, 0.0f),
        finite_or(point.y, 0.0f),
        finite_or(point.z, 0.0f));
  }
}

void GngKernel::set_point_labels(const std::vector<std::uint8_t> &labels) {
  point_labels_ = labels;
}

bool GngKernel::initialize_cpu_core() {
  if (points_.size() < 2) {
    return false;
  }

  auto state = std::make_unique<GngCpuState>();
  float x_min = std::numeric_limits<float>::infinity();
  float y_min = std::numeric_limits<float>::infinity();
  float z_min = std::numeric_limits<float>::infinity();
  float x_max = -std::numeric_limits<float>::infinity();
  float y_max = -std::numeric_limits<float>::infinity();
  float z_max = -std::numeric_limits<float>::infinity();

  state->input_points.reserve(points_.size());
  for (const auto &point : points_) {
    const float x = finite_or(point.x, 0.0f);
    const float y = finite_or(point.y, 0.0f);
    const float z = finite_or(point.z, 0.0f);
    state->input_points.emplace_back(x, y, z);
    x_min = std::min(x_min, x);
    y_min = std::min(y_min, y);
    z_min = std::min(z_min, z);
    x_max = std::max(x_max, x);
    y_max = std::max(y_max, y);
    z_max = std::max(z_max, z);
  }

  state->param.init();
  state->param.node.num_max = static_cast<int>(std::clamp(
      config_.max_nodes == 0 ? 1200U : config_.max_nodes,
      kMinNodeNum,
      kMaxNodeNum));
  state->param.node.learning_num = static_cast<int>(std::max(
      1U,
      config_.learning_num == 0 ? config_.iterations : config_.learning_num));
  if (config_.grid > 0.0f) {
    state->param.config.node_grid = finite_or(config_.grid, state->param.config.node_grid);
  }
  if (config_.interval[0] > 0.0f) {
    for (std::uint32_t idx = 0; idx < LABEL_NUM; ++idx) {
      state->param.setParameter("node.interval", idx, config_.interval[0]);
    }
  }
  if (config_.eb > 0.0f) {
    state->param.node.eta_s1 = finite_or(config_.eb, state->param.node.eta_s1);
  }
  if (config_.en > 0.0f) {
    state->param.node.eta_s2 = finite_or(config_.en, state->param.node.eta_s2);
  }
  state->param.edge.age_max = config_.max_age;

  state->param.config.x_min = x_min - kRangeMargin;
  state->param.config.x_max = x_max + kRangeMargin;
  state->param.config.y_min = y_min - kRangeMargin;
  state->param.config.y_max = y_max + kRangeMargin;
  state->param.config.z_min = z_min - kRangeMargin;
  state->param.config.z_max = z_max + kRangeMargin;

  if (!state->core.init(&state->param.node, &state->param.edge, &state->param.config)) {
    return false;
  }

  cpu_state_ = std::move(state);
  return true;
}

bool GngKernel::run() {
  if (!initialize_cpu_core()) {
    return false;
  }
  return exec(std::max(1U, config_.iterations));
}

bool GngKernel::exec(std::uint32_t steps) {
  if (steps == 0 || points_.size() < 2) {
    return false;
  }
  if (!cpu_state_ && !initialize_cpu_core()) {
    return false;
  }

  auto &state = *cpu_state_;
  state.core.gng_config.learning_num = static_cast<int>(steps);
  state.core.learn(
      state.input_points,
      static_cast<int>(state.input_points.size()),
      state.attention_points,
      0);
  state.core.check_age();
  state.core.check_delete_no_edge_and_decay_eta();
  state.core.calc_edge_distanceXY();
  update_node_geometry();
  sync_graph();
  iter_ += steps;
  return true;
}

void GngKernel::prune_isolated_nodes_public() {
  if (!cpu_state_) {
    return;
  }
  cpu_state_->core.check_delete_no_edge_and_decay_eta();
  sync_graph();
}

void GngKernel::update_normals_public() {
  update_node_geometry();
  sync_graph();
}

void GngKernel::assign_fuzzy_labels_public() {
}

void GngKernel::build_clusters_public() {
  clusters_.clear();
}

void GngKernel::update_node_geometry() {
  if (!cpu_state_) {
    return;
  }
  for (auto &node : cpu_state_->core.nodes) {
    if (node.id != NODE_NOID) {
      cpu_state_->core.normal_vector(node);
    }
  }
  for (auto &node : cpu_state_->core.nodes) {
    if (node.id != NODE_NOID) {
      cpu_state_->core.rho(node);
    }
  }
}

void GngKernel::sync_graph() {
  nodes_.clear();
  edges_.clear();
  if (!cpu_state_) {
    return;
  }

  const auto &core_nodes = cpu_state_->core.nodes;
  std::unordered_map<std::uint32_t, std::uint32_t> compact_index;
  compact_index.reserve(core_nodes.size());
  for (const auto &core_node : core_nodes) {
    if (core_node.id == NODE_NOID) {
      continue;
    }
    const auto index = static_cast<std::uint32_t>(nodes_.size());
    compact_index.emplace(core_node.id, index);
    Node node;
    node.id = core_node.id;
    node.pos = Point3f{core_node.pos.p[0], core_node.pos.p[1], core_node.pos.p[2]};
    node.normal = Point3f{core_node.normal.p[0], core_node.normal.p[1], core_node.normal.p[2]};
    node.rho = core_node.rho;
    node.label = static_cast<std::uint8_t>(core_node.label);
    node.frame = core_node.frame;
    node.frames_since_win = core_node.age_s1;
    nodes_.push_back(node);
  }

  for (const auto &core_node : core_nodes) {
    if (core_node.id == NODE_NOID) {
      continue;
    }
    const auto node_it = compact_index.find(core_node.id);
    if (node_it == compact_index.end()) {
      continue;
    }
    for (std::uint32_t edge_idx = 0; edge_idx < core_node.edge_num; ++edge_idx) {
      const auto neighbor_id = core_node.edges[edge_idx];
      if (core_node.id >= neighbor_id) {
        continue;
      }
      const auto neighbor_it = compact_index.find(neighbor_id);
      if (neighbor_it == compact_index.end()) {
        continue;
      }
      Edge edge;
      edge.a = node_it->second;
      edge.b = neighbor_it->second;
      edge.age = cpu_state_->core.edge_count[
          cpu_state_->core.getEdgeIndex(core_node.id, neighbor_id)];
      edges_.push_back(edge);
    }
  }
}

const std::vector<Node> &GngKernel::nodes() const {
  return nodes_;
}

const std::vector<Edge> &GngKernel::edges() const {
  return edges_;
}

const std::vector<Cluster> &GngKernel::clusters() const {
  return clusters_;
}

std::uint32_t GngKernel::iteration() const {
  return iter_;
}

std::string GngKernel::to_json() const {
  std::string json = "{\"iterations\":" + std::to_string(iter_) + ",\"nodes\":[";
  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    const auto &node = nodes_[index];
    if (index != 0) {
      json += ',';
    }
    json += "{\"id\":" + std::to_string(node.id) +
            ",\"x\":" + std::to_string(node.pos.x) +
            ",\"y\":" + std::to_string(node.pos.y) +
            ",\"z\":" + std::to_string(node.pos.z) +
            ",\"nx\":" + std::to_string(node.normal.x) +
            ",\"ny\":" + std::to_string(node.normal.y) +
            ",\"nz\":" + std::to_string(node.normal.z) +
            ",\"rho\":" + std::to_string(node.rho) +
            ",\"label\":" + std::to_string(node.label) +
            ",\"frame\":" + std::to_string(node.frame) + "}";
  }
  json += "],\"edges\":[";
  for (std::size_t index = 0; index < edges_.size(); ++index) {
    const auto &edge = edges_[index];
    if (index != 0) {
      json += ',';
    }
    json += "{\"a\":" + std::to_string(edge.a) +
            ",\"b\":" + std::to_string(edge.b) +
            ",\"age\":" + std::to_string(edge.age) + "}";
  }
  return json + "],\"clusters\":[]}";
}

}  // namespace gng_wasm_core
