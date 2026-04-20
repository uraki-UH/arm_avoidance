/**
 * @file safety_management.hpp
 * @brief Safety state management and strategies for GNG nodes.
 *
 * This module consolidates safety parameters, strategy interfaces, and the
 * state manager into a single cohesive unit. It is designed to be easily
 * wrapped into a ROS 2 Node or Component.
 */

#pragma once

#include "core_safety/gng/node_status.hpp"
#include "core_safety/spatial/dense_spatial_index.hpp"
#include "core_safety/analysis/safety_vlut_mapper.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <vector>

namespace GNG {

struct SafetyLevels {
  static constexpr float RED = 1.0f;
  static constexpr float YELLOW = 0.5f;
  static constexpr float SAFE = 0.0f;
};

class SafetyRules {
public:
  static float deriveLevel(int collision_count, int danger_count) {
    if (collision_count > 0)
      return SafetyLevels::RED;
    if (danger_count > 0)
      return SafetyLevels::YELLOW;
    return SafetyLevels::SAFE;
  }
  static void getLevelColor(float level, float &r, float &g, float &b) {
    if (level >= SafetyLevels::RED) {
      r = 1.f;
      g = 0.f;
      b = 0.f;
    } else if (level >= SafetyLevels::YELLOW) {
      r = 1.f;
      g = 1.f;
      b = 0.f;
    } else {
      r = 0.f;
      g = 0.8f;
      b = 1.f;
    }
  }
};

class CompactIndexSet {
public:
  CompactIndexSet() = default;
  CompactIndexSet(size_t n) { resize(n); }
  void resize(size_t n) {
    mask_.assign(n, false);
    indices_.clear();
    indices_.reserve(n / 4);
  }
  bool add(int i) {
    if (i < 0 || i >= (int)mask_.size() || mask_[i])
      return false;
    mask_[i] = true;
    indices_.push_back(i);
    return true;
  }
  void clear() {
    for (int i : indices_)
      mask_[i] = false;
    indices_.clear();
  }
  void synchronize() {
    indices_.clear();
    for (size_t i = 0; i < mask_.size(); ++i)
      if (mask_[i])
        indices_.push_back((int)i);
  }
  const std::vector<int> &indices() const { return indices_; }
  const std::vector<bool> &mask() const { return mask_; }
  std::vector<bool> &mask() { return mask_; }
  size_t size() const { return indices_.size(); }
  bool empty() const { return indices_.empty(); }
  bool contains(int i) const {
    return i >= 0 && i < (int)mask_.size() && mask_[i];
  }

private:
  std::vector<bool> mask_;
  std::vector<int> indices_;
};

} // namespace GNG

namespace robot_sim {
namespace simulation {

class SafetyStateManager;

/**
 * @brief Parameters for Safety Management.
 */
struct SafetyParameters {
  // Mode selection
  bool instantaneous_mode = true;

  // General parameters
  bool use_point_cloud_dilation = true;
  float danger_voxel_dilation = 0.025f;

  // Time-decay (Field mode) specific parameters
  float T_persist = 1.0f;
  float kernel_sigma = 0.02f;

  // Prediction parameters
  bool enable_prediction_field = false;
  float T_pred = 0.2f;
};

/**
 * @brief Safety Strategy Interface.
 * Handles the logic for updating danger levels and danger fields.
 */
class ISafetyStrategy {
public:
  virtual ~ISafetyStrategy() = default;

  virtual void update(SafetyStateManager &manager, float dt,
                      float max_velocity) = 0;

  virtual void
  updateDangerField(SafetyStateManager &manager,
                    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
                    const Eigen::Vector3d &obstacle_center,
                    double obstacle_radius,
                    const Eigen::Vector3d &velocity) = 0;
};

/**
 * @brief Safety State Manager.
 * Manages danger levels, velocities, and active states for all nodes.
 */
class SafetyStateManager {
public:
  SafetyStateManager();
  ~SafetyStateManager();

  struct VoxelVizData {
    Eigen::Vector3d center;
    Eigen::Vector3d size;
    float danger;
  };

  void resize(size_t num_nodes);
  void reset();

  // Configuration
  void setParameters(const SafetyParameters &params);
  const SafetyParameters &getParameters() const { return params_; }
  void setInstantaneousMode(bool enabled);
  bool isInstantaneousMode() const { return instantaneous_mode_; }

  // State Access
  float getDangerLevel(int node_id) const;
  float getDangerVelocity(int node_id) const;
  size_t getActiveNodeCount() const;
  size_t getTouchedNodeCount() const;
  const std::vector<int> &getTouchedIndices() const { return touched_indices_; }
  const std::vector<float> &getDangerLevels() const { return danger_levels_; }
  const std::vector<float> &getDangerVelocities() const {
    return danger_velocities_;
  }

  // Update Cycle
  void prepareUpdate();
  void update(float dt, float max_velocity = 0.0f);
  void finalizeUpdate(float dt);

  // Field Updates
  void
  updateDangerField(std::shared_ptr<analysis::ISpatialIndex> spatial_index,
                    const Eigen::Vector3d &obstacle_center,
                    double obstacle_radius, const Eigen::Vector3d &velocity);

  void updateDangerFieldFromVoxels(
      std::shared_ptr<analysis::ISpatialIndex> spatial_index,
      const std::vector<long> &occupied_voxels,
      const std::vector<long> &danger_voxels);

  void updateDangerFieldFromPoints(
      std::shared_ptr<analysis::ISpatialIndex> spatial_index,
      const std::vector<Eigen::Vector3d> &points, float dilation = 0.05f,
      bool use_dilation = true);

  // New: Point-based queries for arbitrary positions in space
  float getDangerAt(const analysis::ISpatialIndex *spatial_index,
                    const Eigen::Vector3d &pos) const;
  bool isCollidingAt(const analysis::ISpatialIndex *spatial_index,
                     const Eigen::Vector3d &pos) const;



  // Visualization Helpers
  const std::vector<VoxelVizData> &getDebugVoxels() const {
    return debug_voxels_;
  }
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getLatestAABB() const;

  const std::vector<int>& getCollisionCounts() const { return vlut_mapper_->getCollisionCounts(); }
  const std::vector<int>& getDangerCounts() const { return vlut_mapper_->getDangerCounts(); }

  // Node Manipulation
  void setDangerLevel(int node_id, float level);

private:
  // Data (SoA)
  std::vector<float> danger_levels_;
  std::vector<float> old_danger_levels_;
  std::vector<float> danger_velocities_;
  std::vector<bool> touched_this_frame_;

  ::GNG::CompactIndexSet active_set_;
  size_t last_touched_count_ = 0;

  // Counters for Instantaneous Mode
  // ROS2 Extracted Safety Mapper
  std::shared_ptr<analysis::SafetyVlutMapper> vlut_mapper_;

  // Debug / Visualization
  std::vector<VoxelVizData> debug_voxels_;
  Eigen::Vector3d latest_aabb_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_aabb_max_ = Eigen::Vector3d::Zero();

  // Dilation Cache
  std::vector<Eigen::Vector3i> kernel_offsets_;
  float kernel_threshold_ = -1.0f;
  double kernel_voxel_size_ = -1.0f;

  // Strategy & Parameters
  std::unique_ptr<ISafetyStrategy> strategy_;
  SafetyParameters params_;
  bool instantaneous_mode_ = false;

  // Sparse update tracking
  std::vector<int> touched_indices_;
  std::vector<uint32_t> node_generations_;
  uint32_t current_generation_ = 0;
  float danger_threshold_cache_ = 0.05f;

  // Friendship for standard strategies defined in implementation
  friend class InstantaneousSafetyStrategy;
  friend class TimeDecaySafetyStrategy;
};

// Standard Strategy Definitions (internal to this module)
class InstantaneousSafetyStrategy : public ISafetyStrategy {
public:
  void update(SafetyStateManager &manager, float dt,
              float max_velocity) override;
  void
  updateDangerField(SafetyStateManager &manager,
                    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
                    const Eigen::Vector3d &obstacle_center,
                    double obstacle_radius,
                    const Eigen::Vector3d &velocity) override;
};

class TimeDecaySafetyStrategy : public ISafetyStrategy {
public:
  void update(SafetyStateManager &manager, float dt,
              float max_velocity) override;
  void
  updateDangerField(SafetyStateManager &manager,
                    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
                    const Eigen::Vector3d &obstacle_center,
                    double obstacle_radius,
                    const Eigen::Vector3d &velocity) override;
};

} // namespace simulation
} // namespace robot_sim
