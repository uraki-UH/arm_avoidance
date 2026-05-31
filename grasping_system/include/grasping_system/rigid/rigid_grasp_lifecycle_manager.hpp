#pragma once

#include <grasping_system/core/grasp_types.hpp>
#include <grasping_system/rigid/rigid_grasp_vlut_builder.hpp>

#include <cmath>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace grasping_system::rigid
{

enum class GraspLifecycleState
{
  kIdle = 0,
  kTargetIdentified = 1,
  kAttemptingGrasp = 2,
  kGrasped = 3,
  kReleasedCached = 4
};

struct RigidGraspSession
{
  core::GraspObject object;
  voxel::GraspVoxelModel voxel_model;
  core::GraspCollisionPolicy collision_policy;
  std::vector<int> allowed_contact_node_ids;
  std::vector<int> hard_collision_node_ids;
  std::vector<RigidGraspVlutSeed> seeds;
  RigidGraspVlut cached_vlut;
  geometry_msgs::msg::Pose released_object_pose_in_world{};
  GraspLifecycleState state{GraspLifecycleState::kIdle};
  bool object_ready{false};
  bool seeds_ready{false};
  bool lut_ready{false};
};

class RigidGraspLifecycleManager
{
public:
  bool upsertObject(core::GraspObject object, voxel::GraspVoxelModel voxel_model)
  {
    if (!object.valid()) {
      return false;
    }
    auto &session = sessions_[object.object_id];
    session.object = std::move(object);
    session.voxel_model = std::move(voxel_model);
    session.object_ready = true;
    session.lut_ready = false;
    session.state = GraspLifecycleState::kIdle;
    return true;
  }

  bool setSeeds(const std::string &object_id, std::vector<RigidGraspVlutSeed> seeds)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->object_ready) {
      return false;
    }
    session->seeds = std::move(seeds);
    session->seeds_ready = !session->seeds.empty();
    session->lut_ready = false;
    session->state = GraspLifecycleState::kIdle;
    return true;
  }

  bool setCollisionPolicy(const std::string &object_id, core::GraspCollisionPolicy policy)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->object_ready) {
      return false;
    }
    session->collision_policy = std::move(policy);
    return true;
  }

  bool setCollisionNodeIds(const std::string &object_id, std::vector<int> allowed_contact_node_ids,
                           std::vector<int> hard_collision_node_ids)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->object_ready) {
      return false;
    }
    session->allowed_contact_node_ids = std::move(allowed_contact_node_ids);
    session->hard_collision_node_ids = std::move(hard_collision_node_ids);
    return true;
  }

  bool buildVlut(const std::string &object_id)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->object_ready || !session->seeds_ready) {
      return false;
    }
    session->cached_vlut = builder_.build(session->voxel_model, session->seeds);
    session->lut_ready = !session->cached_vlut.empty();
    if (!session->lut_ready) {
      session->state = GraspLifecycleState::kIdle;
      return false;
    }
    applyCollisionClassification(*session);
    session->state = (active_object_id_ == object_id) ? GraspLifecycleState::kGrasped
                                                       : GraspLifecycleState::kTargetIdentified;
    return true;
  }

  bool beginGraspAttempt(const std::string &object_id)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->object_ready || !session->lut_ready) {
      return false;
    }
    session->state = GraspLifecycleState::kAttemptingGrasp;
    active_object_id_ = object_id;
    return true;
  }

  const RigidGraspVlut *activate(const std::string &object_id)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || !session->lut_ready) {
      return nullptr;
    }
    active_object_id_ = object_id;
    session->state = GraspLifecycleState::kGrasped;
    return &session->cached_vlut;
  }

  const RigidGraspVlut *deactivateActive()
  {
    auto *session = activeSession();
    if (session == nullptr) {
      return nullptr;
    }
    const std::string released_id = active_object_id_;
    session->released_object_pose_in_world = session->object.pose_in_world;
    active_object_id_.clear();
    session->state = session->lut_ready ? GraspLifecycleState::kReleasedCached
                                        : GraspLifecycleState::kIdle;
    last_released_object_id_ = released_id;
    return &session->cached_vlut;
  }

  bool updateReleasedDistance(const std::string &object_id, const geometry_msgs::msg::Pose &eef_pose_in_world,
                              double idle_distance_threshold)
  {
    auto *session = findSession(object_id);
    if (session == nullptr || session->state != GraspLifecycleState::kReleasedCached) {
      return false;
    }
    if (distance3(session->released_object_pose_in_world, eef_pose_in_world) < idle_distance_threshold) {
      return false;
    }
    session->state = session->seeds_ready ? GraspLifecycleState::kTargetIdentified
                                          : GraspLifecycleState::kIdle;
    return true;
  }

  bool destroyCachedVlut(const std::string &object_id)
  {
    auto *session = findSession(object_id);
    if (session == nullptr) {
      return false;
    }
    if (!session->lut_ready && session->cached_vlut.empty()) {
      return false;
    }

    if (active_object_id_ == object_id) {
      active_object_id_.clear();
      last_released_object_id_ = object_id;
    }

    session->cached_vlut.clear();
    session->lut_ready = false;
    session->state = session->seeds_ready ? GraspLifecycleState::kTargetIdentified
                                          : GraspLifecycleState::kIdle;
    return true;
  }

  bool destroyActiveCachedVlut()
  {
    if (active_object_id_.empty()) {
      return false;
    }
    return destroyCachedVlut(active_object_id_);
  }

  void clear()
  {
    sessions_.clear();
    active_object_id_.clear();
    last_released_object_id_.clear();
  }

  bool hasSession(const std::string &object_id) const
  {
    return sessions_.find(object_id) != sessions_.end();
  }

  const RigidGraspVlut *cachedVlut(const std::string &object_id) const
  {
    const auto *session = findSession(object_id);
    return (session != nullptr && session->lut_ready) ? &session->cached_vlut : nullptr;
  }

  const RigidGraspVlut *activeVlut() const
  {
    const auto *session = activeSession();
    return (session != nullptr && session->lut_ready) ? &session->cached_vlut : nullptr;
  }

  const core::GraspObject *activeObject() const
  {
    const auto *session = activeSession();
    return session ? &session->object : nullptr;
  }

  const core::GraspCollisionPolicy *collisionPolicy(const std::string &object_id) const
  {
    const auto *session = findSession(object_id);
    return session ? &session->collision_policy : nullptr;
  }

  const core::GraspCollisionPolicy *activeCollisionPolicy() const
  {
    const auto *session = activeSession();
    return session ? &session->collision_policy : nullptr;
  }

  GraspLifecycleState state(const std::string &object_id) const
  {
    const auto *session = findSession(object_id);
    return session ? session->state : GraspLifecycleState::kIdle;
  }

  const std::string &activeObjectId() const noexcept { return active_object_id_; }
  const std::string &lastReleasedObjectId() const noexcept { return last_released_object_id_; }

private:
  static void applyCollisionClassification(RigidGraspSession &session)
  {
    for (auto &entry : session.cached_vlut.entries()) {
      entry.active = true;
      entry.contact_class = RigidGraspNodeContactClass::kNormal;
    }

    for (int node_id : session.allowed_contact_node_ids) {
      session.cached_vlut.setNodeContactClass(node_id, RigidGraspNodeContactClass::kAllowedContact);
      session.cached_vlut.setNodeActive(node_id, true);
    }
    for (int node_id : session.hard_collision_node_ids) {
      session.cached_vlut.setNodeContactClass(node_id, RigidGraspNodeContactClass::kHardCollision);
      session.cached_vlut.setNodeActive(node_id, false);
    }
  }

  static double distance3(const geometry_msgs::msg::Pose &lhs, const geometry_msgs::msg::Pose &rhs) noexcept
  {
    const double dx = lhs.position.x - rhs.position.x;
    const double dy = lhs.position.y - rhs.position.y;
    const double dz = lhs.position.z - rhs.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  RigidGraspSession *findSession(const std::string &object_id)
  {
    const auto it = sessions_.find(object_id);
    return it == sessions_.end() ? nullptr : &it->second;
  }

  const RigidGraspSession *findSession(const std::string &object_id) const
  {
    const auto it = sessions_.find(object_id);
    return it == sessions_.end() ? nullptr : &it->second;
  }

  RigidGraspSession *activeSession()
  {
    return active_object_id_.empty() ? nullptr : findSession(active_object_id_);
  }

  const RigidGraspSession *activeSession() const
  {
    return active_object_id_.empty() ? nullptr : findSession(active_object_id_);
  }

  std::unordered_map<std::string, RigidGraspSession> sessions_;
  std::string active_object_id_;
  std::string last_released_object_id_;
  RigidGraspVlutBuilder builder_;
};

}  // namespace grasping_system::rigid
