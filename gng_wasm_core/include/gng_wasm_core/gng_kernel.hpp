#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace gng_wasm_core {

enum Label : std::uint8_t {
  LABEL_DEFAULT = 0,
  LABEL_SAFE = 1,
  LABEL_WALL = 2,
  LABEL_UNKNOWN = 3,
  LABEL_HUMAN = 4,
  LABEL_CAR = 5,
  LABEL_NUM = 6
};

struct Point3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Node {
  std::uint32_t id = 0;
  Point3f pos{};
  Point3f normal{};
  float rho = 1.0f;
  float error = 0.0f;
  float utility = 0.0f;
  std::uint32_t win_count = 0;
  std::uint8_t label = LABEL_DEFAULT;
  std::uint32_t frame = 0;
  float fuzzy_memberships[LABEL_NUM] = {};
};

struct Edge {
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  std::uint32_t age = 0;
};

struct Cluster {
  std::uint32_t id = 0;
  std::uint8_t label = LABEL_DEFAULT;
  float label_reliability = 0.0f;
  Point3f pos{};
  Point3f scale{};
  std::uint32_t frame = 0;
  std::vector<std::uint32_t> node_indices;
};

struct Config {
  // node.*
  std::uint32_t max_nodes = 400;
  std::uint32_t iterations = 12000;
  std::uint32_t learning_num = 5;
  float unknown_learning_rate = 0.8f;
  float eb = 0.05f;            // node.eta_s1
  float en = 0.006f;           // node.eta_s2
  float eta_decay_rate = 1.0f;
  float s1_reset_range = 0.1f;
  float grid = 0.05f;
  int s1_age_max[LABEL_NUM] = {6, 6, 6, 3, 6, 6};
  int clustered_s1_age[LABEL_NUM] = {20, 20, 6, 3, 20, 20};
  float interval[LABEL_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  int static_age_min = -1;
  bool covariance_enabled = false;

  // edge.*
  std::uint32_t max_edges = 300000;
  std::uint32_t max_age = 80;

  // label.fuzzy.*
  float fuzzy_unknown = 0.6f;
  float fuzzy_min = 0.5f;
  float fuzzy_lpf_time_constant = 0.5f;

  // cluster.*
  std::uint32_t cluster_num_max = 100;
  std::uint32_t cluster_node_num_min = 10;
  float cluster_velocity_lpf = 0.5f;
  float cluster_plane_volume = 10.0f;
  float cluster_unknown_edge_dist_max = 0.2f;
  float cluster_other_edge_dist_max = 0.4f;
  float cluster_human_radius = 0.6f;
  int cluster_human_hysteresis_age = 5;
  int cluster_human_confirmation_age = 5;

  // ds.*
  float ds_range_max = 0.2f;
  std::uint32_t ds_all_num_max = 4000;
  std::uint32_t ds_unknown_num_max = 2000;
  std::uint32_t ds_human_num_max = 3000;

  // seed
  std::uint32_t seed = 1;
  // lambda (node insertion interval)
  std::uint32_t lambda = 100;
};

class GngKernel {
public:
  void reset();
  void set_config(const Config &config);
  bool set_parameter(const char *name, std::uint32_t index, float value);
  void set_points(const std::vector<Point3f> &points);
  void set_point_labels(const std::vector<std::uint8_t> &labels);
  bool run();

  const std::vector<Node> &nodes() const;
  const std::vector<Edge> &edges() const;
  const std::vector<Cluster> &clusters() const;
  std::uint32_t iteration() const;
  const Config &config() const { return config_; }

  std::string to_json() const;

private:
  std::vector<Point3f> points_;
  std::vector<std::uint8_t> point_labels_;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::vector<Cluster> clusters_;
  Config config_{};
  std::uint32_t iter_ = 0;
  std::uint32_t frame_ = 0;
  std::uint32_t next_node_id_ = 0;

  std::uint32_t random_u32();
  float random_unit();
  std::size_t pick_point_index();
  std::pair<int, int> nearest_nodes(const Point3f &p) const;
  void add_edge(std::size_t a, std::size_t b);
  void age_edges_from(std::size_t node_index);
  void prune_old_edges();
  void prune_isolated_nodes();
  void insert_node();
  void decay_errors();
  void update_normals();
  void assign_fuzzy_labels();
  void build_clusters();

  static float distance2(const Point3f &a, const Point3f &b);
  static float distance_xy2(const Point3f &a, const Point3f &b);
  static Point3f midpoint(const Point3f &a, const Point3f &b);
};

}  // namespace gng_wasm_core
