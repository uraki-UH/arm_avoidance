#include "gng_wasm_core/gng_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <sstream>

namespace gng_wasm_core {

namespace {
constexpr float kEps = 1e-6f;
}

void GngKernel::reset() {
  nodes_.clear();
  edges_.clear();
  clusters_.clear();
  iter_ = 0;
  frame_ = 0;
  next_node_id_ = 0;
}

void GngKernel::set_config(const Config &config) {
  config_ = config;
}

bool GngKernel::set_parameter(const char *name, std::uint32_t index, float value) {
  if (!name) return false;

  if (std::strcmp(name, "node.num_max") == 0) { config_.max_nodes = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "node.learning_num") == 0) { config_.learning_num = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "iterations") == 0) { config_.iterations = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "lambda") == 0) { config_.lambda = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "seed") == 0) { config_.seed = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "node.unknown_learning_rate") == 0) { config_.unknown_learning_rate = value; return true; }
  if (std::strcmp(name, "node.eta_s1") == 0) { config_.eb = value; return true; }
  if (std::strcmp(name, "node.eta_s2") == 0) { config_.en = value; return true; }
  if (std::strcmp(name, "node.eta_decay_rate") == 0) { config_.eta_decay_rate = value; return true; }
  if (std::strcmp(name, "node.s1_reset_range") == 0) { config_.s1_reset_range = value; return true; }
  if (std::strcmp(name, "node.grid") == 0) { config_.grid = value; return true; }
  if (std::strcmp(name, "node.static.age_min") == 0) { config_.static_age_min = static_cast<int>(value); return true; }
  if (std::strcmp(name, "node.covariance_enabled") == 0) { config_.covariance_enabled = (value != 0.0f); return true; }

  if (std::strcmp(name, "node.s1_age_max") == 0 || std::strcmp(name, "node.s1_age") == 0) {
    if (index < LABEL_NUM) { config_.s1_age_max[index] = static_cast<int>(value); return true; }
  }
  if (std::strcmp(name, "node.clusted_s1_age") == 0) {
    if (index < LABEL_NUM) { config_.clustered_s1_age[index] = static_cast<int>(value); return true; }
  }
  if (std::strcmp(name, "node.interval") == 0) {
    if (index < LABEL_NUM) { config_.interval[index] = value; return true; }
  }

  if (std::strcmp(name, "edge.num_max") == 0) { config_.max_edges = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "edge.age_max") == 0) { config_.max_age = static_cast<std::uint32_t>(value); return true; }

  if (std::strcmp(name, "label.fuzzy.unknown") == 0) { config_.fuzzy_unknown = value; return true; }
  if (std::strcmp(name, "label.fuzzy.min") == 0) { config_.fuzzy_min = value; return true; }
  if (std::strcmp(name, "label.fuzzy.lpf_time_constant") == 0) { config_.fuzzy_lpf_time_constant = value; return true; }

  if (std::strcmp(name, "cluster.num_max") == 0) { config_.cluster_num_max = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "cluster.node_num_min") == 0 || std::strcmp(name, "cluster.num_min") == 0) { config_.cluster_node_num_min = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "cluster.velocity.lpf_time_constant") == 0) { config_.cluster_velocity_lpf = value; return true; }
  if (std::strcmp(name, "cluster.plane.volume") == 0) { config_.cluster_plane_volume = value; return true; }
  if (std::strcmp(name, "cluster.unknown.edge_distance_max") == 0) { config_.cluster_unknown_edge_dist_max = value; return true; }
  if (std::strcmp(name, "cluster.other.edge_distance_max") == 0) { config_.cluster_other_edge_dist_max = value; return true; }
  if (std::strcmp(name, "cluster.human.radius") == 0) { config_.cluster_human_radius = value; return true; }
  if (std::strcmp(name, "cluster.human.hysteresis_age") == 0) { config_.cluster_human_hysteresis_age = static_cast<int>(value); return true; }
  if (std::strcmp(name, "cluster.human.confirmation_age") == 0) { config_.cluster_human_confirmation_age = static_cast<int>(value); return true; }

  if (std::strcmp(name, "ds.range_max") == 0) { config_.ds_range_max = value; return true; }
  if (std::strcmp(name, "ds.all.num_max") == 0) { config_.ds_all_num_max = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "ds.unknown.num_max") == 0) { config_.ds_unknown_num_max = static_cast<std::uint32_t>(value); return true; }
  if (std::strcmp(name, "ds.human.num_max") == 0) { config_.ds_human_num_max = static_cast<std::uint32_t>(value); return true; }

  return false;
}

void GngKernel::set_points(const std::vector<Point3f> &points) {
  points_ = points;
}

void GngKernel::set_point_labels(const std::vector<std::uint8_t> &labels) {
  point_labels_ = labels;
}

const std::vector<Node> &GngKernel::nodes() const { return nodes_; }
const std::vector<Edge> &GngKernel::edges() const { return edges_; }
const std::vector<Cluster> &GngKernel::clusters() const { return clusters_; }
std::uint32_t GngKernel::iteration() const { return iter_; }

std::uint32_t GngKernel::random_u32() {
  config_.seed = config_.seed * 1664525u + 1013904223u;
  return config_.seed;
}

float GngKernel::random_unit() {
  return static_cast<float>((random_u32() >> 8) & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
}

std::size_t GngKernel::pick_point_index() {
  if (points_.empty()) return 0;
  return static_cast<std::size_t>(random_u32() % points_.size());
}

float GngKernel::distance2(const Point3f &a, const Point3f &b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  const float dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

float GngKernel::distance_xy2(const Point3f &a, const Point3f &b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
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
  if (a == b || a >= nodes_.size() || b >= nodes_.size()) return;
  if (edges_.size() >= config_.max_edges) return;

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

void GngKernel::prune_isolated_nodes() {
  if (nodes_.size() <= 2) return;

  std::vector<bool> connected(nodes_.size(), false);
  for (const auto &e : edges_) {
    if (e.a < connected.size()) connected[e.a] = true;
    if (e.b < connected.size()) connected[e.b] = true;
  }

  for (std::size_t i = nodes_.size(); i-- > 0;) {
    if (connected[i]) continue;
    if (config_.static_age_min >= 0 &&
        static_cast<int>(frame_ - nodes_[i].frame) >= config_.static_age_min) {
      continue;
    }

    const std::size_t last = nodes_.size() - 1;
    if (i != last) {
      nodes_[i] = nodes_[last];
      for (auto &e : edges_) {
        if (e.a == static_cast<std::uint32_t>(last)) e.a = static_cast<std::uint32_t>(i);
        if (e.b == static_cast<std::uint32_t>(last)) e.b = static_cast<std::uint32_t>(i);
        if (e.a > e.b) std::swap(e.a, e.b);
      }
    }
    nodes_.pop_back();
    connected.pop_back();
  }
}

void GngKernel::insert_node() {
  if (nodes_.size() < 2) return;

  std::size_t q = 0;
  for (std::size_t i = 1; i < nodes_.size(); ++i) {
    if (nodes_[i].error > nodes_[q].error) q = i;
  }

  std::size_t f = q;
  float best_neighbor_error = -1.0f;
  for (const auto &edge : edges_) {
    const std::size_t other = edge.a == q ? edge.b : (edge.b == q ? edge.a : nodes_.size());
    if (other >= nodes_.size()) continue;
    if (nodes_[other].error > best_neighbor_error) {
      best_neighbor_error = nodes_[other].error;
      f = other;
    }
  }
  if (f == q) return;

  Node r;
  r.id = next_node_id_++;
  r.pos = midpoint(nodes_[q].pos, nodes_[f].pos);

  // interval check: skip if new node is too close to any existing node
  const auto new_label = nodes_[q].label;
  const float min_interval = new_label < LABEL_NUM ? config_.interval[new_label] : 0.0f;
  if (min_interval > 0.0f) {
    for (const auto &nd : nodes_) {
      if (distance2(r.pos, nd.pos) < min_interval * min_interval) return;
    }
  }
  r.error = 0.5f * (nodes_[q].error + nodes_[f].error);
  r.utility = 0.5f * (nodes_[q].utility + nodes_[f].utility);
  r.win_count = 0;
  r.label = nodes_[q].label;
  r.frame = frame_;

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

void GngKernel::update_normals() {
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    std::vector<std::size_t> neighbors;
    for (const auto &e : edges_) {
      if (e.a == i) neighbors.push_back(e.b);
      else if (e.b == i) neighbors.push_back(e.a);
    }
    if (neighbors.size() < 2) {
      nodes_[i].normal = {0.0f, 0.0f, 1.0f};
      nodes_[i].rho = 1.0f;
      continue;
    }

    float cx = 0, cy = 0, cz = 0;
    for (auto ni : neighbors) {
      cx += nodes_[ni].pos.x - nodes_[i].pos.x;
      cy += nodes_[ni].pos.y - nodes_[i].pos.y;
      cz += nodes_[ni].pos.z - nodes_[i].pos.z;
    }
    const float n_inv = 1.0f / static_cast<float>(neighbors.size());
    cx *= n_inv; cy *= n_inv; cz *= n_inv;

    float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (auto ni : neighbors) {
      const float dx = nodes_[ni].pos.x - nodes_[i].pos.x - cx;
      const float dy = nodes_[ni].pos.y - nodes_[i].pos.y - cy;
      const float dz = nodes_[ni].pos.z - nodes_[i].pos.z - cz;
      xx += dx * dx; xy += dx * dy; xz += dx * dz;
      yy += dy * dy; yz += dy * dz; zz += dz * dz;
    }

    float nx = xy * yz - xz * yy;
    float ny = xz * xy - xx * yz;
    float nz = xx * yy - xy * xy;
    float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < kEps) {
      nodes_[i].normal = {0.0f, 0.0f, 1.0f};
    } else {
      len = 1.0f / len;
      nodes_[i].normal = {nx * len, ny * len, nz * len};
    }

    float curvature = 0.0f;
    for (auto ni : neighbors) {
      curvature += std::sqrt(distance2(nodes_[i].pos, nodes_[ni].pos));
    }
    nodes_[i].rho = (neighbors.size() > 0) ? curvature / static_cast<float>(neighbors.size()) : 1.0f;
  }
}

void GngKernel::assign_fuzzy_labels() {
  if (point_labels_.empty()) return;

  const float alpha = config_.fuzzy_lpf_time_constant > 0.0f
      ? 1.0f / (1.0f + config_.fuzzy_lpf_time_constant * 30.0f)
      : 1.0f;

  for (auto &node : nodes_) {
    float counts[LABEL_NUM] = {};
    float total = 0.0f;

    for (std::size_t pi = 0; pi < points_.size(); ++pi) {
      const float d2 = distance2(points_[pi], node.pos);
      const float range = config_.s1_reset_range;
      if (d2 > range * range) continue;
      const std::uint8_t pl = pi < point_labels_.size() ? point_labels_[pi] : LABEL_DEFAULT;
      if (pl < LABEL_NUM) {
        counts[pl] += 1.0f;
        total += 1.0f;
      }
    }

    if (total < 1.0f) continue;

    float memberships[LABEL_NUM];
    for (int l = 0; l < LABEL_NUM; ++l) {
      memberships[l] = counts[l] / total;
    }

    for (int l = 0; l < LABEL_NUM; ++l) {
      node.fuzzy_memberships[l] = (1.0f - alpha) * node.fuzzy_memberships[l] + alpha * memberships[l];
    }

    int best_label = LABEL_DEFAULT;
    float best_val = 0.0f;
    for (int l = 0; l < LABEL_NUM; ++l) {
      if (node.fuzzy_memberships[l] > best_val) {
        best_val = node.fuzzy_memberships[l];
        best_label = l;
      }
    }

    if (best_label == LABEL_UNKNOWN && best_val < config_.fuzzy_unknown) {
      best_label = LABEL_DEFAULT;
    }
    if (best_val < config_.fuzzy_min) {
      best_label = LABEL_DEFAULT;
    }

    node.label = static_cast<std::uint8_t>(best_label);
  }
}

void GngKernel::build_clusters() {
  clusters_.clear();
  if (nodes_.empty()) return;

  const std::size_t n = nodes_.size();
  std::vector<int> component(n, -1);
  int next_cluster = 0;

  for (const auto &e : edges_) {
    if (e.a >= n || e.b >= n) continue;

    const auto la = nodes_[e.a].label;
    const auto lb = nodes_[e.b].label;
    const float max_dist2 = (la == LABEL_UNKNOWN || lb == LABEL_UNKNOWN)
        ? config_.cluster_unknown_edge_dist_max * config_.cluster_unknown_edge_dist_max
        : config_.cluster_other_edge_dist_max * config_.cluster_other_edge_dist_max;

    if (distance_xy2(nodes_[e.a].pos, nodes_[e.b].pos) > max_dist2) continue;

    int ca = component[e.a], cb = component[e.b];
    if (ca < 0 && cb < 0) {
      component[e.a] = component[e.b] = next_cluster++;
    } else if (ca < 0) {
      component[e.a] = cb;
    } else if (cb < 0) {
      component[e.b] = ca;
    } else if (ca != cb) {
      const int keep = std::min(ca, cb);
      const int replace = std::max(ca, cb);
      for (auto &c : component) {
        if (c == replace) c = keep;
      }
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (component[i] < 0) component[i] = next_cluster++;
  }

  std::vector<std::vector<std::uint32_t>> groups(next_cluster);
  for (std::size_t i = 0; i < n; ++i) {
    groups[component[i]].push_back(static_cast<std::uint32_t>(i));
  }

  std::uint32_t cid = 0;
  for (auto &group : groups) {
    if (group.size() < config_.cluster_node_num_min) continue;
    if (cid >= config_.cluster_num_max) break;

    Cluster cl;
    cl.id = cid++;
    cl.node_indices = std::move(group);

    int label_counts[LABEL_NUM] = {};
    Point3f min_p = {1e9f, 1e9f, 1e9f};
    Point3f max_p = {-1e9f, -1e9f, -1e9f};
    float cx = 0, cy = 0, cz = 0;

    for (auto ni : cl.node_indices) {
      const auto &nd = nodes_[ni];
      label_counts[nd.label]++;
      cx += nd.pos.x; cy += nd.pos.y; cz += nd.pos.z;
      min_p.x = std::min(min_p.x, nd.pos.x);
      min_p.y = std::min(min_p.y, nd.pos.y);
      min_p.z = std::min(min_p.z, nd.pos.z);
      max_p.x = std::max(max_p.x, nd.pos.x);
      max_p.y = std::max(max_p.y, nd.pos.y);
      max_p.z = std::max(max_p.z, nd.pos.z);
    }

    const float inv_n = 1.0f / static_cast<float>(cl.node_indices.size());
    cl.pos = {cx * inv_n, cy * inv_n, cz * inv_n};
    cl.scale = {max_p.x - min_p.x, max_p.y - min_p.y, max_p.z - min_p.z};

    int best_l = 0;
    for (int l = 1; l < LABEL_NUM; ++l) {
      if (label_counts[l] > label_counts[best_l]) best_l = l;
    }
    cl.label = static_cast<std::uint8_t>(best_l);
    cl.label_reliability = static_cast<float>(label_counts[best_l]) * inv_n;
    cl.frame = frame_;

    clusters_.push_back(std::move(cl));
  }
}

Point3f GngKernel::midpoint(const Point3f &a, const Point3f &b) {
  return {0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.5f * (a.z + b.z)};
}

bool GngKernel::run() {
  if (points_.size() < 2) return false;

  if (nodes_.size() < 2) {
    const std::size_t a = pick_point_index();
    std::size_t b = pick_point_index();
    if (a == b) b = (b + 1) % points_.size();
    nodes_.clear();
    edges_.clear();
    Node na; na.id = next_node_id_++; na.pos = points_[a]; na.frame = frame_;
    Node nb; nb.id = next_node_id_++; nb.pos = points_[b]; nb.frame = frame_;
    if (!point_labels_.empty()) {
      na.label = a < point_labels_.size() ? point_labels_[a] : LABEL_DEFAULT;
      nb.label = b < point_labels_.size() ? point_labels_[b] : LABEL_DEFAULT;
    }
    nodes_.push_back(na);
    nodes_.push_back(nb);
    add_edge(0, 1);
  }

  const std::uint32_t total = config_.iterations;
  for (iter_ = 0; iter_ < total; ++iter_) {
    const std::size_t pi = pick_point_index();
    const Point3f &p = points_[pi];
    const std::uint8_t point_label = (!point_labels_.empty() && pi < point_labels_.size())
        ? point_labels_[pi] : LABEL_DEFAULT;

    const auto [s1, s2] = nearest_nodes(p);
    if (s1 < 0 || s2 < 0) continue;

    auto &winner = nodes_[static_cast<std::size_t>(s1)];

    const float dx = p.x - winner.pos.x;
    const float dy = p.y - winner.pos.y;
    const float dz = p.z - winner.pos.z;
    const float dist = std::sqrt(std::max(kEps, dx * dx + dy * dy + dz * dz));

    winner.error += dist;
    winner.win_count += 1;

    const auto &runner_up = nodes_[static_cast<std::size_t>(s2)];
    const float runner_up_dist = std::sqrt(std::max(kEps, distance2(p, runner_up.pos)));
    winner.utility += std::max(0.0f, runner_up_dist - dist);

    float lr_scale = 1.0f;
    if (point_label == LABEL_UNKNOWN) {
      lr_scale = config_.unknown_learning_rate;
    }

    winner.pos.x += config_.eb * lr_scale * dx;
    winner.pos.y += config_.eb * lr_scale * dy;
    winner.pos.z += config_.eb * lr_scale * dz;

    age_edges_from(static_cast<std::size_t>(s1));

    for (const auto &edge : edges_) {
      std::size_t neighbor_idx = nodes_.size();
      if (edge.a == static_cast<std::size_t>(s1)) neighbor_idx = edge.b;
      else if (edge.b == static_cast<std::size_t>(s1)) neighbor_idx = edge.a;
      if (neighbor_idx >= nodes_.size()) continue;

      auto &nb = nodes_[neighbor_idx];
      nb.pos.x += config_.en * lr_scale * (p.x - nb.pos.x);
      nb.pos.y += config_.en * lr_scale * (p.y - nb.pos.y);
      nb.pos.z += config_.en * lr_scale * (p.z - nb.pos.z);
    }

    add_edge(static_cast<std::size_t>(s1), static_cast<std::size_t>(s2));
    prune_old_edges();

    if (config_.lambda > 0 && (iter_ + 1) % config_.lambda == 0 && nodes_.size() < config_.max_nodes) {
      insert_node();
    }

    decay_errors();
  }

  frame_++;
  prune_isolated_nodes();
  update_normals();
  assign_fuzzy_labels();
  build_clusters();

  return nodes_.size() >= 2;
}

bool GngKernel::exec(std::uint32_t steps) {
  if (points_.size() < 2) return false;

  if (nodes_.size() < 2) {
    const std::size_t a = pick_point_index();
    std::size_t b = pick_point_index();
    if (a == b) b = (b + 1) % points_.size();
    nodes_.clear(); edges_.clear();
    Node na; na.id = next_node_id_++; na.pos = points_[a]; na.frame = frame_;
    Node nb; nb.id = next_node_id_++; nb.pos = points_[b]; nb.frame = frame_;
    if (!point_labels_.empty()) {
      na.label = a < point_labels_.size() ? point_labels_[a] : LABEL_DEFAULT;
      nb.label = b < point_labels_.size() ? point_labels_[b] : LABEL_DEFAULT;
    }
    nodes_.push_back(na);
    nodes_.push_back(nb);
    add_edge(0, 1);
  }

  for (std::uint32_t k = 0; k < steps; ++k, ++iter_) {
    const std::size_t pi = pick_point_index();
    const Point3f &p = points_[pi];
    const std::uint8_t point_label = (!point_labels_.empty() && pi < point_labels_.size())
        ? point_labels_[pi] : LABEL_DEFAULT;

    const auto [s1, s2] = nearest_nodes(p);
    if (s1 < 0 || s2 < 0) continue;

    auto &winner = nodes_[static_cast<std::size_t>(s1)];
    const float dx = p.x - winner.pos.x;
    const float dy = p.y - winner.pos.y;
    const float dz = p.z - winner.pos.z;
    const float dist = std::sqrt(std::max(kEps, dx*dx + dy*dy + dz*dz));

    winner.error += dist;
    winner.win_count += 1;
    winner.frames_since_win = 0;  // reset s1_age counter on win
    const auto &runner_up = nodes_[static_cast<std::size_t>(s2)];
    winner.utility += std::max(0.0f, std::sqrt(std::max(kEps, distance2(p, runner_up.pos))) - dist);

    float lr_scale = (point_label == LABEL_UNKNOWN) ? config_.unknown_learning_rate : 1.0f;
    winner.pos.x += config_.eb * lr_scale * dx;
    winner.pos.y += config_.eb * lr_scale * dy;
    winner.pos.z += config_.eb * lr_scale * dz;

    age_edges_from(static_cast<std::size_t>(s1));
    for (const auto &edge : edges_) {
      std::size_t ni = nodes_.size();
      if (edge.a == static_cast<std::size_t>(s1)) ni = edge.b;
      else if (edge.b == static_cast<std::size_t>(s1)) ni = edge.a;
      if (ni >= nodes_.size()) continue;
      nodes_[ni].pos.x += config_.en * lr_scale * (p.x - nodes_[ni].pos.x);
      nodes_[ni].pos.y += config_.en * lr_scale * (p.y - nodes_[ni].pos.y);
      nodes_[ni].pos.z += config_.en * lr_scale * (p.z - nodes_[ni].pos.z);
    }

    add_edge(static_cast<std::size_t>(s1), static_cast<std::size_t>(s2));
    prune_old_edges();

    if (config_.lambda > 0 && (iter_ + 1) % config_.lambda == 0 && nodes_.size() < config_.max_nodes) {
      insert_node();
    }
    decay_errors();
  }

  frame_++;

  // s1_age pruning: remove nodes that haven't won in s1_age exec() calls
  {
    std::vector<bool> won(nodes_.size(), false);
    // We already ran the inner loop above; we need to know who won.
    // Re-scan: instead, use frames_since_win counter incremented here.
    for (std::size_t i = nodes_.size(); i-- > 0;) {
      auto &nd = nodes_[i];
      if (nd.win_count > 0 && nd.frames_since_win == 0) {
        // already reset inside inner loop — do nothing
      }
      // Increment frames_since_win for nodes that didn't win this exec()
      // (win_count is total, we track via frames_since_win)
      nd.frames_since_win++;
      const int age_max = nd.label < LABEL_NUM ? config_.s1_age_max[nd.label] : config_.s1_age_max[0];
      if (age_max > 0 && static_cast<int>(nd.frames_since_win) > age_max && nodes_.size() > 2) {
        // static memory: keep old nodes
        if (config_.static_age_min < 0 || static_cast<int>(frame_ - nd.frame) < config_.static_age_min) {
          const std::size_t last = nodes_.size() - 1;
          if (i != last) {
            nodes_[i] = nodes_[last];
            for (auto &e : edges_) {
              if (e.a == static_cast<std::uint32_t>(last)) e.a = static_cast<std::uint32_t>(i);
              if (e.b == static_cast<std::uint32_t>(last)) e.b = static_cast<std::uint32_t>(i);
              if (e.a > e.b) std::swap(e.a, e.b);
            }
          }
          nodes_.pop_back();
          // clean up edges referencing removed index
          edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
            [i](const Edge &e){ return e.a == static_cast<std::uint32_t>(i) || e.b == static_cast<std::uint32_t>(i); }),
            edges_.end());
        }
      }
    }
  }

  return nodes_.size() >= 2;
}

std::string GngKernel::to_json() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"iterations\":" << iter_ << ",\"frame\":" << frame_ << ",";

  oss << "\"nodes\":[";
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const auto &n = nodes_[i];
    if (i > 0) oss << ",";
    oss << "{\"id\":" << n.id
        << ",\"x\":" << n.pos.x << ",\"y\":" << n.pos.y << ",\"z\":" << n.pos.z
        << ",\"nx\":" << n.normal.x << ",\"ny\":" << n.normal.y << ",\"nz\":" << n.normal.z
        << ",\"rho\":" << n.rho
        << ",\"error\":" << n.error << ",\"utility\":" << n.utility
        << ",\"wins\":" << n.win_count
        << ",\"label\":" << static_cast<int>(n.label)
        << ",\"frame\":" << n.frame
        << ",\"fuzzy\":[";
    for (int l = 0; l < LABEL_NUM; ++l) {
      if (l > 0) oss << ",";
      oss << n.fuzzy_memberships[l];
    }
    oss << "]}";
  }

  oss << "],\"edges\":[";
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    const auto &e = edges_[i];
    if (i > 0) oss << ",";
    oss << "{\"a\":" << e.a << ",\"b\":" << e.b << ",\"age\":" << e.age << "}";
  }

  oss << "],\"clusters\":[";
  for (std::size_t i = 0; i < clusters_.size(); ++i) {
    const auto &c = clusters_[i];
    if (i > 0) oss << ",";
    oss << "{\"id\":" << c.id
        << ",\"label\":" << static_cast<int>(c.label)
        << ",\"reliability\":" << c.label_reliability
        << ",\"x\":" << c.pos.x << ",\"y\":" << c.pos.y << ",\"z\":" << c.pos.z
        << ",\"sx\":" << c.scale.x << ",\"sy\":" << c.scale.y << ",\"sz\":" << c.scale.z
        << ",\"node_count\":" << c.node_indices.size()
        << ",\"nodes\":[";
    for (std::size_t j = 0; j < c.node_indices.size(); ++j) {
      if (j > 0) oss << ",";
      oss << c.node_indices[j];
    }
    oss << "]}";
  }

  oss << "]}";
  return oss.str();
}

}  // namespace gng_wasm_core
