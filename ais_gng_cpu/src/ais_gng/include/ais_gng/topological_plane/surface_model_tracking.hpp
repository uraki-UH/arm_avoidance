#pragma once

#include "ais_gng/topological_plane/surface_model.hpp"

namespace fuzzrobo::surface_model
{
struct retention_options
{
  bool enable_retention = true;
  double max_point_residual = 0.012;
  double max_normal_deg = 45.0;
  double max_rms = 0.006;
  double min_inlier_ratio = 0.6;
  double max_node_displacement = 0.05;
};

class tracker
{
public:
  result update(const ais_gng_msgs::msg::TopologicalMap &map,
    const ais_gng_msgs::msg::PlaneClusterArray &planes, const options &config = {},
    const retention_options &retention = {}, std::size_t min_seed_plane_patches = 2);
  void clear();
private:
  struct reference_node { std::uint16_t id; Eigen::Vector3d position; };
  struct track { region surface; std::vector<reference_node> reference; };
  std::vector<track> tracks_;
  std::string frame_id_;
  std::uint32_t frame_number_ = 0;
  std::int64_t stamp_ = 0;
  // 元ノード添字由来の新規候補IDと重ならない追跡ID。
  std::uint32_t next_id_ = 1U << 30;
};
}  // namespace fuzzrobo::surface_model
