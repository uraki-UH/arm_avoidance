#include "simulation/robot/ode/ode_gng_visualizer.hpp"
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#ifdef dDOUBLE
#define dsDrawSphere dsDrawSphereD
#define dsDrawLine dsDrawLineD
#endif

namespace simulation {

OdeGngVisualizer::OdeGngVisualizer() {}

void OdeGngVisualizer::update(const std::vector<VisualNode> &nodes,
                              const std::vector<VisualEdge> &edges) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  nodes_ = nodes;
  edges_ = edges;
}

void OdeGngVisualizer::draw(const Eigen::Vector3d &offset) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  float ox = (float)offset.x();
  float oy = (float)offset.y();
  float oz = (float)offset.z();

  // ノードの描画
  for (const auto &node : nodes_) {
    // フィルタリング
    if (node.level > filter_.max_level)
      continue;
    if (filter_.show_only_active && !node.active)
      continue;
    if (filter_.show_only_surface && !node.is_surface)
      continue;
    if (filter_.show_only_active_surface && !node.is_active_surface)
      continue;

    // New: Path Only Filter
    bool is_in_path = highlighted_path_nodes_.count(node.id);
    if (filter_.show_only_path && !is_in_path)
      continue;

    // 色の設定
    float r = node.color.r;
    float g = node.color.g;
    float b = node.color.b;

    if (is_in_path && !filter_.show_only_path) {
      r = 1.0f;
      g = 1.0f;
      b = 1.0f;
    }

    setDrawColor(r, g, b, node.color.a);

    dVector3 pos;
    pos[0] = node.position.x() + ox;
    pos[1] = node.position.y() + oy;
    pos[2] = node.position.z() + oz;

    dMatrix3 RI;
    dRSetIdentity(RI);

    dsDrawSphere(pos, RI, 0.01f);
  }
  // エッジの描画
  for (const auto &edge : edges_) {
    // フィルタリング
    if (edge.level > filter_.max_level)
      continue;

    const VisualNode *n1 = nullptr;
    const VisualNode *n2 = nullptr;
    for (const auto &node : nodes_) {
      if (node.id == edge.node1_id)
        n1 = &node;
      if (node.id == edge.node2_id)
        n2 = &node;
    }
    if (!n1 || !n2)
      continue;
    if (filter_.show_only_active && (!n1->active || !n2->active))
      continue;
    if (filter_.show_only_surface && (!n1->is_surface || !n2->is_surface))
      continue;

    bool edge_in_path = highlighted_path_nodes_.count(edge.node1_id) &&
                        highlighted_path_nodes_.count(edge.node2_id);

    if (filter_.show_only_path && !edge_in_path)
      continue;

    // 色の設定
    float r = edge.color.r;
    float g = edge.color.g;
    float b = edge.color.b;

    if (edge_in_path) {
      r = 1.0f;
      g = 1.0f;
      b = 0.0f; // Yellow for danger
    }

    setDrawColor(r, g, b, edge.color.a);

    dVector3 p1 = {n1->position.x() + ox, n1->position.y() + oy,
                   n1->position.z() + oz};
    dVector3 p2 = {n2->position.x() + ox, n2->position.y() + oy,
                   n2->position.z() + oz};

    dsDrawLine(p1, p2);
  }

  // Draw Trajectory (Green Lines)
  if (!trajectory_.empty()) {
    setDrawColor(0.0f, 1.0f, 0.0f, 1.0f); // Green
    for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
      dVector3 p1 = {trajectory_[i].x() + ox, trajectory_[i].y() + oy,
                     trajectory_[i].z() + oz};
      dVector3 p2 = {trajectory_[i + 1].x() + ox, trajectory_[i + 1].y() + oy,
                     trajectory_[i + 1].z() + oz};
      dsDrawLine(p1, p2);
    }
  }
}

void OdeGngVisualizer::setTrajectory(
    const std::vector<Eigen::Vector3d> &trajectory) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  trajectory_ = trajectory;
}

void OdeGngVisualizer::setHighlightedPath(
    const std::vector<int> &path_node_ids) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  highlighted_path_nodes_.clear();
  for (int id : path_node_ids) {
    highlighted_path_nodes_.insert(id);
  }
}

void OdeGngVisualizer::setFilter(const FilterSettings &filter) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  filter_ = filter;
}

void OdeGngVisualizer::setDrawColor(float r, float g, float b, float a) {
  dsSetColorAlpha(r, g, b, a);
}

} // namespace simulation
