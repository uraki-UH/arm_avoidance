#pragma once

#include <cstdint>
#include <memory>
#include <string>
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
  std::uint32_t frames_since_win = 0;  // s1_age カウンタ
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
  std::uint32_t max_nodes = 0;
  std::uint32_t iterations = 0;
  std::uint32_t learning_num = 0;
  float unknown_learning_rate = 1.0f;
  float eb = 0;
  float en = 0;
  float eta_decay_rate = 1.0f;
  float s1_reset_range = 0;
  float grid = 0;
  int s1_age_max[LABEL_NUM] = {};
  int clustered_s1_age[LABEL_NUM] = {};
  float interval[LABEL_NUM] = {};
  int static_age_min = -1;
  bool covariance_enabled = false;
  std::uint32_t max_edges = 300000;
  std::uint32_t max_age = 0;
  float fuzzy_unknown = 0;
  float fuzzy_min = 0;
  float fuzzy_lpf_time_constant = 0;
  std::uint32_t cluster_num_max = 100;
  std::uint32_t cluster_node_num_min = 1;
  float cluster_velocity_lpf = 0;
  float cluster_plane_volume = 0;
  float cluster_unknown_edge_dist_max = 1e9f;
  float cluster_other_edge_dist_max = 1e9f;
  float cluster_human_radius = 0;
  int cluster_human_hysteresis_age = 0;
  int cluster_human_confirmation_age = 0;
  float ds_range_max = 0;
  std::uint32_t ds_all_num_max = 0;
  std::uint32_t ds_unknown_num_max = 0;
  std::uint32_t ds_human_num_max = 0;
  std::uint32_t seed = 1;
  std::uint32_t lambda = 0;
};

class GngKernel {
public:
  GngKernel();
  ~GngKernel();

  void reset();
  void set_config(const Config &config);
  bool set_parameter(const char *name, std::uint32_t index, float value);
  void set_points(const std::vector<Point3f> &points);
  void set_point_labels(const std::vector<std::uint8_t> &labels);
  bool run();
  bool exec(std::uint32_t steps);
  void prune_isolated_nodes_public();
  void update_normals_public();
  void assign_fuzzy_labels_public();
  void build_clusters_public();

  const std::vector<Node> &nodes() const;
  const std::vector<Edge> &edges() const;
  const std::vector<Cluster> &clusters() const;
  std::uint32_t iteration() const;
  const Config &config() const { return config_; }

  std::string to_json() const;

private:
  class GngCpuState;

  std::vector<Point3f> points_;
  std::vector<std::uint8_t> point_labels_;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::vector<Cluster> clusters_;
  Config config_{};
  std::unique_ptr<GngCpuState> cpu_state_;
  std::uint32_t iter_ = 0;

  bool initialize_cpu_core();
  void sync_graph();
  void update_node_geometry();
};

}  // namespace gng_wasm_core
