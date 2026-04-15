#pragma once

#include "simulation/viz/gng_visualization_interface.hpp"
#include <drawstuff/drawstuff.h>
#include <mutex>
#include <ode/ode.h>
#include <unordered_set>

namespace simulation {

/**
 * @brief ODE/drawstuff を用いたGNG可視化器の実装
 */
class OdeGngVisualizer : public IGngVisualizer {
public:
  OdeGngVisualizer();
  virtual ~OdeGngVisualizer() = default;

  void update(const std::vector<VisualNode> &nodes,
              const std::vector<VisualEdge> &edges) override;

  void draw(const Eigen::Vector3d &offset = Eigen::Vector3d::Zero()) override;

  void setHighlightedPath(const std::vector<int> &path_node_ids) override;

  void setFilter(const FilterSettings &filter) override;

  // New: Set trajectory to visualize
  void setTrajectory(const std::vector<Eigen::Vector3d> &trajectory);

private:
  std::vector<VisualNode> nodes_;
  std::vector<VisualEdge> edges_;
  std::unordered_set<int> highlighted_path_nodes_;
  std::vector<Eigen::Vector3d> trajectory_; // New: trajectory points
  FilterSettings filter_;
  std::mutex data_mutex_;

  static void setDrawColor(float r, float g, float b, float a);
};

} // namespace simulation
