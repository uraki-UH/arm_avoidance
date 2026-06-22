#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace gng_wasm_core {

struct Point3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Node {
  std::uint32_t id = 0;
  Point3f pos{};
  float error = 0.0f;
  float utility = 0.0f;
  std::uint32_t win_count = 0;
};

struct Edge {
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  std::uint32_t age = 0;
};

struct Config {
  std::uint32_t max_nodes = 400;
  std::uint32_t iterations = 12000;
  std::uint32_t lambda = 100;
  std::uint32_t max_age = 80;
  float eb = 0.05f;
  float en = 0.006f;
  std::uint32_t seed = 1;
};

class GngKernel {
public:
  void reset();
  void set_config(const Config &config);
  void set_points(const std::vector<Point3f> &points);
  bool run();

  const std::vector<Node> &nodes() const;
  const std::vector<Edge> &edges() const;
  std::uint32_t iteration() const;

  std::string to_json() const;

private:
  std::vector<Point3f> points_;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  Config config_{};
  std::uint32_t iter_ = 0;
  std::uint32_t next_node_id_ = 0;

  std::uint32_t random_u32();
  float random_unit();
  std::size_t pick_point_index();
  std::pair<int, int> nearest_nodes(const Point3f &p) const;
  void add_edge(std::size_t a, std::size_t b);
  void age_edges_from(std::size_t node_index);
  void prune_old_edges();
  void insert_node();
  void decay_errors();
  static float distance2(const Point3f &a, const Point3f &b);
  static Point3f midpoint(const Point3f &a, const Point3f &b);
};

}  // namespace gng_wasm_core
