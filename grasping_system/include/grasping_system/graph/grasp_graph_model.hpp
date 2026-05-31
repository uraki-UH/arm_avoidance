#pragma once

#include <grasping_system/core/grasp_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace grasping_system::graph
{

struct GraspGraphNode
{
  int id{-1};
  geometry_msgs::msg::Pose pose_in_object{};
  double weight{0.0};
  bool active{true};
};

struct GraspGraphEdge
{
  int from{-1};
  int to{-1};
  double cost{1.0};
  bool active{true};
};

class GraspGraphModel : public core::GraspGeometryModel
{
public:
  core::GraspGeometryKind kind() const noexcept override
  {
    return core::GraspGeometryKind::kGraph;
  }
  std::string typeName() const override { return "graph"; }

  std::unique_ptr<core::GraspGeometryModel> clone() const override
  {
    return std::make_unique<GraspGraphModel>(*this);
  }

  void clear()
  {
    nodes_.clear();
    edges_.clear();
    adjacency_.clear();
  }

  void addNode(const GraspGraphNode &node)
  {
    nodes_.push_back(node);
    adjacency_[node.id];
  }

  void addEdge(const GraspGraphEdge &edge)
  {
    edges_.push_back(edge);
    adjacency_[edge.from].push_back(edge.to);
    adjacency_[edge.to].push_back(edge.from);
  }

  const std::vector<GraspGraphNode> &nodes() const noexcept { return nodes_; }
  const std::vector<GraspGraphEdge> &edges() const noexcept { return edges_; }

  const std::vector<int> &neighbors(int node_id) const
  {
    static const std::vector<int> kEmpty;
    const auto it = adjacency_.find(node_id);
    return it == adjacency_.end() ? kEmpty : it->second;
  }

  const GraspGraphNode *findNode(int node_id) const noexcept
  {
    for (const auto &node : nodes_) {
      if (node.id == node_id) {
        return &node;
      }
    }
    return nullptr;
  }

private:
  std::vector<GraspGraphNode> nodes_;
  std::vector<GraspGraphEdge> edges_;
  std::unordered_map<int, std::vector<int>> adjacency_;
};

}  // namespace grasping_system::graph
