#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <voxel_indexing_common/voxel_indexing.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace grasping_system::rigid
{

enum class RigidGraspNodeContactClass
{
  kNormal = 0,
  kAllowedContact = 1,
  kHardCollision = 2
};

struct RigidGraspVlutEntry
{
  int gng_node_id{-1};
  geometry_msgs::msg::Pose eef_pose_in_world{};
  geometry_msgs::msg::Pose object_pose_in_world{};
  geometry_msgs::msg::Pose object_pose_in_eef{};
  double traversal_cost{0.0};
  bool active{true};
  RigidGraspNodeContactClass contact_class{RigidGraspNodeContactClass::kNormal};
  std::vector<long> related_voxel_ids;
};

class RigidGraspVlut
{
public:
  void clear()
  {
    entries_.clear();
    voxel_to_node_ids_.clear();
  }
  bool empty() const noexcept { return entries_.empty(); }
  std::size_t size() const noexcept { return entries_.size(); }

  void addEntry(const RigidGraspVlutEntry &entry)
  {
    entries_.push_back(entry);
    for (long voxel_id : entry.related_voxel_ids) {
      voxel_to_node_ids_[voxel_id].push_back(entry.gng_node_id);
    }
  }

  const std::vector<RigidGraspVlutEntry> &entries() const noexcept { return entries_; }
  std::vector<RigidGraspVlutEntry> &entries() noexcept { return entries_; }

  const RigidGraspVlutEntry *findByNodeId(int gng_node_id) const noexcept
  {
    for (const auto &entry : entries_) {
      if (entry.gng_node_id == gng_node_id) {
        return &entry;
      }
    }
    return nullptr;
  }

  voxel_indexing_common::VoxelIndexingSchema &indexing() noexcept { return indexing_; }
  const voxel_indexing_common::VoxelIndexingSchema &indexing() const noexcept { return indexing_; }

  const std::vector<int> *findNodesByVoxel(long voxel_id) const noexcept
  {
    const auto it = voxel_to_node_ids_.find(voxel_id);
    return it == voxel_to_node_ids_.end() ? nullptr : &it->second;
  }

  bool setNodeActive(int gng_node_id, bool active)
  {
    auto *entry = findMutableByNodeId(gng_node_id);
    if (entry == nullptr) {
      return false;
    }
    entry->active = active;
    return true;
  }

  bool setNodeContactClass(int gng_node_id, RigidGraspNodeContactClass contact_class)
  {
    auto *entry = findMutableByNodeId(gng_node_id);
    if (entry == nullptr) {
      return false;
    }
    entry->contact_class = contact_class;
    return true;
  }

  RigidGraspNodeContactClass nodeContactClass(int gng_node_id) const noexcept
  {
    const auto *entry = findByNodeId(gng_node_id);
    return entry ? entry->contact_class : RigidGraspNodeContactClass::kNormal;
  }

  bool isNodeActive(int gng_node_id) const noexcept
  {
    const auto *entry = findByNodeId(gng_node_id);
    return entry ? entry->active : false;
  }

  std::vector<int> nodeIdsByContactClass(RigidGraspNodeContactClass contact_class) const
  {
    std::vector<int> node_ids;
    for (const auto &entry : entries_) {
      if (entry.contact_class == contact_class) {
        node_ids.push_back(entry.gng_node_id);
      }
    }
    return node_ids;
  }

private:
  RigidGraspVlutEntry *findMutableByNodeId(int gng_node_id)
  {
    for (auto &entry : entries_) {
      if (entry.gng_node_id == gng_node_id) {
        return &entry;
      }
    }
    return nullptr;
  }

  voxel_indexing_common::VoxelIndexingSchema indexing_;
  std::vector<RigidGraspVlutEntry> entries_;
  std::unordered_map<long, std::vector<int>> voxel_to_node_ids_;
};

}  // namespace grasping_system::rigid
