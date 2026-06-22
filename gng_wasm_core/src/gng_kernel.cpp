#include "gng_wasm_core/gng_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace gng_wasm_core {

namespace {
constexpr float kEps = 1e-6f;
}

void GngKernel::reset() {
  nodes_.clear();
  edges_.clear();
  iter_ = 0;
  next_node_id_ = 0;
}

void GngKernel::set_config(const Config &config) {
  config_ = config;
}

void GngKernel::set_points(const std::vector<Point3f> &points) {
  points_ = points;
}

const std::vector<Node> &GngKernel::nodes() const {
  return nodes_;
}

const std::vector<Edge> &GngKernel::edges() const {
  return edges_;
}

std::uint32_t GngKernel::iteration() const {
  return iter_;
}

std::uint32_t GngKernel::random_u32() {
  config_.seed = config_.seed * 1664525u + 1013904223u;
  return config_.seed;
}

float GngKernel::random_unit() {
  return static_cast<float>((random_u32() >> 8) & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
}

std::size_t GngKernel::pick_point_index() {
  if (points_.empty()) {
    return 0;
  }
  return static_cast<std::size_t>(random_u32() % points_.size());
}

float GngKernel::distance2(const Point3f &a, const Point3f &b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  const float dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

std::pair<int, int> GngKernel::nearest_nodes(const Point3f &p) const {
  int first = -1;
  int second = -1;
  float first_d = std::numeric_limits<float>::infinity();
  float second_d = std::numeric_limits<float>::infinity();

  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const float d = distance2(p, nodes_[i].pos);
    if (d < first_d) {
      second_d = first_d;
      second = first;
      first_d = d;
      first = static_cast<int>(i);
    } else if (d < second_d) {
      second_d = d;
      second = static_cast<int>(i);
    }
  }

  return {first, second};
}

void GngKernel::add_edge(std::size_t a, std::size_t b) {
  if (a == b || a >= nodes_.size() || b >= nodes_.size()) {
    return;
  }
  const std::size_t lo = std::min(a, b);
  const std::size_t hi = std::max(a, b);
  for (auto &edge : edges_) {
    if (edge.a == lo && edge.b == hi) {
      edge.age = 0;
      return;
    }
  }
  edges_.push_back(Edge{static_cast<std::uint32_t>(lo), static_cast<std::uint32_t>(hi), 0});
}

void GngKernel::age_edges_from(std::size_t node_index) {
  for (auto &edge : edges_) {
    if (edge.a == node_index || edge.b == node_index) {
      ++edge.age;
    }
  }
}

void GngKernel::prune_old_edges() {
  const auto max_age = config_.max_age;
  edges_.erase(
      std::remove_if(edges_.begin(), edges_.end(),
          [max_age](const Edge &edge) { return edge.age > max_age; }),
      edges_.end());
}

void GngKernel::insert_node() {
  if (nodes_.size() < 2) {
    return;
  }

  std::size_t q = 0;
  for (std::size_t i = 1; i < nodes_.size(); ++i) {
    if (nodes_[i].error > nodes_[q].error) {
      q = i;
    }
  }

  std::size_t f = q;
  float best_neighbor_error = -1.0f;
  for (const auto &edge : edges_) {
    const std::size_t other = edge.a == q ? edge.b : (edge.b == q ? edge.a : nodes_.size());
    if (other >= nodes_.size()) {
      continue;
    }
    if (nodes_[other].error > best_neighbor_error) {
      best_neighbor_error = nodes_[other].error;
      f = other;
    }
  }

  if (f == q) {
    return;
  }

  Node r;
  r.id = next_node_id_++;
  r.pos = midpoint(nodes_[q].pos, nodes_[f].pos);
  r.error = 0.5f * (nodes_[q].error + nodes_[f].error);
  r.utility = 0.5f * (nodes_[q].utility + nodes_[f].utility);
  r.win_count = 0;

  nodes_[q].error *= 0.5f;
  nodes_[f].error *= 0.5f;

  edges_.erase(
      std::remove_if(edges_.begin(), edges_.end(),
          [q, f](const Edge &edge) {
            return (edge.a == q && edge.b == f) || (edge.a == f && edge.b == q);
          }),
      edges_.end());

  nodes_.push_back(r);
  const std::size_t r_index = nodes_.size() - 1;
  add_edge(q, r_index);
  add_edge(f, r_index);
}

void GngKernel::decay_errors() {
  for (auto &node : nodes_) {
    node.error *= 0.995f;
    node.utility *= 0.999f;
  }
}

Point3f GngKernel::midpoint(const Point3f &a, const Point3f &b) {
  return {
      0.5f * (a.x + b.x),
      0.5f * (a.y + b.y),
      0.5f * (a.z + b.z),
  };
}

bool GngKernel::run() {
  if (points_.size() < 2) {
    return false;
  }

  if (nodes_.size() < 2) {
    const std::size_t a = pick_point_index();
    std::size_t b = pick_point_index();
    if (a == b) {
      b = (b + 1) % points_.size();
    }
    nodes_.clear();
    edges_.clear();
    nodes_.push_back(Node{next_node_id_++, points_[a], 0.0f, 0.0f, 0});
    nodes_.push_back(Node{next_node_id_++, points_[b], 0.0f, 0.0f, 0});
    add_edge(0, 1);
  }

  for (iter_ = 0; iter_ < config_.iterations; ++iter_) {
    const Point3f &p = points_[pick_point_index()];
    const auto [s1, s2] = nearest_nodes(p);
    if (s1 < 0 || s2 < 0) {
      continue;
    }

    auto &winner = nodes_[static_cast<std::size_t>(s1)];
    auto &runner_up = nodes_[static_cast<std::size_t>(s2)];

    const float dx = p.x - winner.pos.x;
    const float dy = p.y - winner.pos.y;
    const float dz = p.z - winner.pos.z;
    const float dist = std::sqrt(std::max(kEps, dx * dx + dy * dy + dz * dz));

    winner.error += dist;
    winner.win_count += 1;
    const float runner_up_dist = std::sqrt(std::max(kEps, distance2(p, runner_up.pos)));
    winner.utility += std::max(0.0f, runner_up_dist - dist);

    winner.pos.x += config_.eb * dx;
    winner.pos.y += config_.eb * dy;
    winner.pos.z += config_.eb * dz;

    age_edges_from(static_cast<std::size_t>(s1));

    auto &neighbor = runner_up;
    neighbor.pos.x += config_.en * (p.x - neighbor.pos.x);
    neighbor.pos.y += config_.en * (p.y - neighbor.pos.y);
    neighbor.pos.z += config_.en * (p.z - neighbor.pos.z);

    add_edge(static_cast<std::size_t>(s1), static_cast<std::size_t>(s2));
    prune_old_edges();

    if (config_.lambda > 0 && (iter_ + 1) % config_.lambda == 0 && nodes_.size() < config_.max_nodes) {
      insert_node();
    }

    decay_errors();
  }

  return nodes_.size() >= 2;
}

std::string GngKernel::to_json() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"iterations\":" << iter_ << ",";
  oss << "\"nodes\":[";
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const auto &n = nodes_[i];
    if (i > 0) {
      oss << ",";
    }
    oss << "{";
    oss << "\"id\":" << n.id << ",";
    oss << "\"x\":" << n.pos.x << ",";
    oss << "\"y\":" << n.pos.y << ",";
    oss << "\"z\":" << n.pos.z << ",";
    oss << "\"error\":" << n.error << ",";
    oss << "\"utility\":" << n.utility << ",";
    oss << "\"wins\":" << n.win_count;
    oss << "}";
  }
  oss << "],";
  oss << "\"edges\":[";
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    const auto &e = edges_[i];
    if (i > 0) {
      oss << ",";
    }
    oss << "{";
    oss << "\"a\":" << e.a << ",";
    oss << "\"b\":" << e.b << ",";
    oss << "\"age\":" << e.age;
    oss << "}";
  }
  oss << "]";
  oss << "}";
  return oss.str();
}

}  // namespace gng_wasm_core
