#include "core_safety/management/safety_management.hpp"
#include "common/geometry_utils.hpp"
#include "core_safety/spatial/index_voxel_grid.hpp"
#include <chrono>
#include <iostream>
#include <cmath>
#include <unordered_set>

namespace robot_sim {
namespace simulation {

// =========================================================
// SafetyStateManager Implementation
// =========================================================

SafetyStateManager::SafetyStateManager() {
  strategy_ = std::make_unique<InstantaneousSafetyStrategy>();
  vlut_mapper_ = std::make_shared<analysis::SafetyVlutMapper>();
}

SafetyStateManager::~SafetyStateManager() {}

void SafetyStateManager::resize(size_t num_nodes) {
  danger_levels_.assign(num_nodes, 0.0f);
  old_danger_levels_.assign(num_nodes, 0.0f);
  danger_velocities_.assign(num_nodes, 0.0f);
  vlut_mapper_->ensureCapacity(num_nodes);
  active_set_.resize(num_nodes);
  node_generations_.assign(num_nodes, 0);
}

void SafetyStateManager::reset() {
  std::fill(danger_levels_.begin(), danger_levels_.end(), 0.0f);
  std::fill(old_danger_levels_.begin(), old_danger_levels_.end(), 0.0f);
  std::fill(danger_velocities_.begin(), danger_velocities_.end(), 0.0f);
  vlut_mapper_->resetState();
  active_set_.clear();
  debug_voxels_.clear();
}

void SafetyStateManager::setParameters(const SafetyParameters &params) {
  params_ = params;
  setInstantaneousMode(params.instantaneous_mode);
}

void SafetyStateManager::setInstantaneousMode(bool enabled) {
  if (instantaneous_mode_ == enabled && strategy_)
    return;

  instantaneous_mode_ = enabled;
  if (enabled) {
    strategy_ = std::make_unique<InstantaneousSafetyStrategy>();
  } else {
    strategy_ = std::make_unique<TimeDecaySafetyStrategy>();
  }
}

void SafetyStateManager::prepareUpdate() {
  old_danger_levels_ = danger_levels_;
  
  if (instantaneous_mode_) {
    // In instantaneous mode, we clear levels that are NOT backed by counters.
    // However, for point-cloud/field updates, we might want some persistence.
    // For now, we rely on the strategy to derive the base level from counters.
    std::fill(danger_levels_.begin(), danger_levels_.end(), 0.0f);
    // active_set_ is NOT cleared here because it's incrementally updated by applyOccupancyDeltas
  }
  debug_voxels_.clear();
}

void SafetyStateManager::update(float dt, float max_velocity) {
  if (strategy_) {
    strategy_->update(*this, dt, max_velocity);
  }
}

void SafetyStateManager::finalizeUpdate(float dt) {
  if (dt <= 0)
    return;
  
  // Ensure the active set and touched indices reflect all updates from this frame
  active_set_.synchronize();
  touched_indices_ = active_set_.indices();

  for (int node_id : touched_indices_) {
    danger_velocities_[node_id] =
        (danger_levels_[node_id] - old_danger_levels_[node_id]) / dt;
  }
}


float SafetyStateManager::getDangerLevel(int node_id) const {
  if (node_id < 0 || (size_t)node_id >= danger_levels_.size())
    return 0.0f;
  return danger_levels_[node_id];
}

float SafetyStateManager::getDangerVelocity(int node_id) const {
  if (node_id < 0 || (size_t)node_id >= danger_velocities_.size())
    return 0.0f;
  return danger_velocities_[node_id];
}

size_t SafetyStateManager::getActiveNodeCount() const {
  return active_set_.size();
}
size_t SafetyStateManager::getTouchedNodeCount() const {
  return touched_indices_.size();
}

void SafetyStateManager::updateDangerField(
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const Eigen::Vector3d &obstacle_center, double obstacle_radius,
    const Eigen::Vector3d &velocity) {
  if (strategy_) {
    strategy_->updateDangerField(*this, spatial_index, obstacle_center,
                                 obstacle_radius, velocity);
  }
}

void SafetyStateManager::updateDangerFieldFromVoxels(
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const std::vector<long> &occupied_voxels,
    const std::vector<long> &danger_voxels) {
  if (!spatial_index)
    return;

  vlut_mapper_->initialize(danger_levels_.size(), spatial_index);
  vlut_mapper_->updateFromVoxels(occupied_voxels, danger_voxels);

  const auto& coll_counts = vlut_mapper_->getCollisionCounts();
  const auto& dan_counts = vlut_mapper_->getDangerCounts();

  // 2. Refresh sparsely node states
  auto refresh = [&](const std::vector<long> &vids) {
    for (long vid : vids) {
      for (int nid : spatial_index->getNodesInVoxel(vid)) {
        if (nid >= 0 && (size_t)nid < danger_levels_.size()) {
          danger_levels_[nid] = ::GNG::SafetyRules::deriveLevel(
              coll_counts[nid], dan_counts[nid]);
          if (danger_levels_[nid] > 0.0f)
            active_set_.add(nid);
        }
      }
    }
  };

  refresh(vlut_mapper_->getAddedOccupied());
  refresh(vlut_mapper_->getRemovedOccupied());
  refresh(vlut_mapper_->getAddedDanger());
  refresh(vlut_mapper_->getRemovedDanger());

  auto add_viz = [&](const std::vector<long> &voxels, float level) {
    for (long vid : voxels) {
      Eigen::Vector3i g_idx =
          GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
      VoxelVizData vvd;
      vvd.center = spatial_index->getWorldMin() +
                   (g_idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
                       spatial_index->getVoxelSize();
      vvd.size = Eigen::Vector3d::Constant(spatial_index->getVoxelSize());
      vvd.danger = level;
      debug_voxels_.push_back(vvd);
    }
  };
  add_viz(vlut_mapper_->getPrevDangerVoxels(), 0.5f);
  add_viz(vlut_mapper_->getPrevOccupiedVoxels(), 1.0f);
}

void SafetyStateManager::updateDangerFieldFromPoints(
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const std::vector<Eigen::Vector3d> &points, float dilation,
    bool use_dilation) {
  if (!spatial_index || points.empty())
    return;

  // 2. Voxelized Dilation Lookup
  double res = spatial_index->getVoxelSize();
  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  int K = use_dilation ? static_cast<int>(std::ceil(dilation / res)) : 0;

  std::unordered_set<long> processed_voxels;
  processed_voxels.reserve(points.size());

  for (const auto &p : points) {
    long vid = spatial_index->getVoxelId(p);
    if (vid == -1 || processed_voxels.count(vid))
      continue;
    processed_voxels.insert(vid);

    // Get absolute voxel index from the ID
    Eigen::Vector3i v_idx =
        GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);

    // Dilation Loop (Cube search + Sphere distance filter)
    for (int dz = -K; dz <= K; ++dz) {
      for (int dy = -K; dy <= K; ++dy) {
        for (int dx = -K; dx <= K; ++dx) {
          // Sphere filter
          double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * res;
          if (dist > dilation + 1e-6)
            continue;

          // Points are in dx=0, dy=0, dz=0 voxel.
          // Everything else within dilation radius is danger (0.5).
          float level = (dx == 0 && dy == 0 && dz == 0) ? 1.0f : 0.5f;

          Eigen::Vector3i n_idx = v_idx + Eigen::Vector3i(dx, dy, dz);
          long neighbor_vid = GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(n_idx);
          std::vector<int> nodes = spatial_index->getNodesInVoxel(neighbor_vid);
          for (int nid : nodes) {
              if (nid < 0 || nid >= (int)danger_levels_.size())
                continue;

              if (level > danger_levels_[nid]) {
                danger_levels_[nid] = level;
                active_set_.add(nid);
              }
          }
        }
      }
    }
  }
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
SafetyStateManager::getLatestAABB() const {
  return {latest_aabb_min_, latest_aabb_max_};
}

float SafetyStateManager::getDangerAt(const analysis::ISpatialIndex *spatial_index,
                                       const Eigen::Vector3d &pos) const {
  if (!spatial_index)
    return 0.0f;
  long voxel_id = spatial_index->getVoxelId(pos);
  std::vector<int> nodes_in = spatial_index->getNodesInVoxel(voxel_id);
  float max_danger = 0.0f;
  for (int id : nodes_in) {
    if (id >= 0 && id < (int)danger_levels_.size()) {
      max_danger = std::max(max_danger, danger_levels_[id]);
    }
  }
  return max_danger;
}

bool SafetyStateManager::isCollidingAt(
    const analysis::ISpatialIndex *spatial_index,
    const Eigen::Vector3d &pos) const {
  if (!spatial_index)
    return false;
  long voxel_id = spatial_index->getVoxelId(pos);
  std::vector<int> nodes_in = spatial_index->getNodesInVoxel(voxel_id);
  for (int id : nodes_in) {
    if (id >= 0 && id < (int)danger_levels_.size()) {
      if (danger_levels_[id] >= 1.0f) {
        return true;
      }
    }
  }
  return false;
}

void SafetyStateManager::setDangerLevel(int node_id, float level) {
    if (node_id >= 0 && (size_t)node_id < danger_levels_.size()) {
        danger_levels_[node_id] = level;
        if (level > 0.0f) active_set_.add(node_id);
    }
}

// =========================================================
// InstantaneousSafetyStrategy Implementation
// =========================================================

void InstantaneousSafetyStrategy::update(SafetyStateManager &manager, float dt,
                                         float max_velocity) {
  (void)dt;
  (void)max_velocity;

  auto &active_set = manager.active_set_;
  auto &danger_levels = manager.danger_levels_;
  const auto &coll_counters = manager.getCollisionCounts();
  const auto &dang_counters = manager.getDangerCounts();

  // Filter indices based on active status after update
  std::vector<int> next_indices;
  next_indices.reserve(active_set.size());

  for (int node_id : active_set.indices()) {
    float derived_level = ::GNG::SafetyRules::deriveLevel(
        coll_counters[node_id], dang_counters[node_id]);
    float target_level = std::max(danger_levels[node_id], derived_level);
    danger_levels[node_id] = target_level;

    if (target_level <= 0.0f) {
      active_set.mask()[node_id] = false;
    } else {
      next_indices.push_back(node_id);
    }
  }
  active_set.synchronize();
}

void InstantaneousSafetyStrategy::updateDangerField(
    SafetyStateManager &manager,
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const Eigen::Vector3d &obstacle_center, double obstacle_radius,
    const Eigen::Vector3d &velocity) {
  (void)velocity;
  if (!spatial_index)
    return;

  double res = spatial_index->getVoxelSize();
  double margin = manager.params_.danger_voxel_dilation;
  Eigen::Vector3d min_aabb =
      obstacle_center - Eigen::Vector3d::Constant(obstacle_radius + margin);
  Eigen::Vector3d max_aabb =
      obstacle_center + Eigen::Vector3d::Constant(obstacle_radius + margin);

  manager.latest_aabb_min_ = min_aabb;
  manager.latest_aabb_max_ = max_aabb;

  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  Eigen::Vector3i start_idx =
      ((min_aabb - world_min).array() / res).floor().cast<int>();
  Eigen::Vector3i end_idx = ((max_aabb - world_min).array() / res).ceil().cast<int>();

  manager.debug_voxels_.clear();

  for (int z = start_idx.z(); z <= end_idx.z(); ++z) {
    double pz = world_min.z() + (z + 0.5) * res;
    for (int y = start_idx.y(); y <= end_idx.y(); ++y) {
      double py = world_min.y() + (y + 0.5) * res;
      for (int x = start_idx.x(); x <= end_idx.x(); ++x) {
        double px = world_min.x() + (x + 0.5) * res;
        Eigen::Vector3d p_voxel(px, py, pz);
        double dist_sq = (p_voxel - obstacle_center).squaredNorm();

        float danger_val = 0.0f;
        if (x == start_idx.x() + (end_idx.x()-start_idx.x())/2 && 
            y == start_idx.y() + (end_idx.y()-start_idx.y())/2 && 
            z == start_idx.z() + (end_idx.z()-start_idx.z())/2) {
            // Simplified check for center voxel
            danger_val = 1.0f;
        } else if (dist_sq <= obstacle_radius * obstacle_radius) {
            danger_val = 1.0f;
        } else if (dist_sq <= (obstacle_radius + margin) * (obstacle_radius + margin)) {
            danger_val = 0.5f;
        }

        if (danger_val > 0.0f) {
          SafetyStateManager::VoxelVizData vvd;
          vvd.center = p_voxel;
          vvd.size = Eigen::Vector3d::Constant(res);
          vvd.danger = danger_val;
          manager.debug_voxels_.push_back(vvd);

          std::vector<int> nodes = spatial_index->getNodesInVoxel(GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(Eigen::Vector3i(x,y,z)));
          for (int nid : nodes) {
              if (danger_val > manager.getDangerLevel(nid))
                manager.setDangerLevel(nid, danger_val);
          }
        }
      }
    }
  }
}

// =========================================================
// TimeDecaySafetyStrategy Implementation
// =========================================================

void TimeDecaySafetyStrategy::update(SafetyStateManager &manager, float dt,
                                     float max_velocity) {
  const auto &params = manager.getParameters();
  float base_rate = std::exp(std::log(0.01f) * dt / params.T_persist);
  float velocity_decay_mult = 1.0f / (1.0f + 0.2f * max_velocity);

  auto &active_set = manager.active_set_;
  auto &danger_levels = manager.danger_levels_;
  auto &danger_velocities = manager.danger_velocities_;
  const auto &coll_counters = manager.getCollisionCounts();
  const auto &dang_counters = manager.getDangerCounts();

  for (int node_id : active_set.indices()) {
    float &level = danger_levels[node_id];
    float &vel = danger_velocities[node_id];

    if (coll_counters[node_id] > 0) {
      level = ::GNG::SafetyLevels::RED;
    } else {
      if (level > 0.0f) {
        float intensity_factor = 0.4f;
        float effective_rate =
            base_rate * (1.0f - intensity_factor * level) * velocity_decay_mult;
        if (effective_rate < 0.001f)
          effective_rate = 0.001f;
        level *= effective_rate;

        if (dang_counters[node_id] > 0 && level < ::GNG::SafetyLevels::YELLOW) {
          level = ::GNG::SafetyLevels::YELLOW;
        }
        if (level < 0.01f)
          level = 0.0f;
      } else if (dang_counters[node_id] > 0) {
        level = ::GNG::SafetyLevels::YELLOW;
      } else {
        level = 0.0f;
        vel = 0.0f;
      }
    }

    if (level < 1e-3f && std::abs(vel) < 1e-3f) {
      active_set.mask()[node_id] = false;
    }
  }
  active_set.synchronize();
}

void TimeDecaySafetyStrategy::updateDangerField(
    SafetyStateManager &manager,
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const Eigen::Vector3d &obstacle_center, double obstacle_radius,
    const Eigen::Vector3d &velocity) {
  const auto &params = manager.getParameters();
  if (!spatial_index || obstacle_radius <= 0)
    return;

  double v_mag = velocity.norm();
  double char_length = (obstacle_radius > 0.01) ? obstacle_radius : 0.05;
  double k = std::min(3.0, 1.0 + (v_mag * params.T_pred) / char_length);
  if (k < 1.0)
    k = 1.0;

  Eigen::Vector3d dir =
      (v_mag > 0.02) ? velocity.normalized() : Eigen::Vector3d::UnitX();
  if (v_mag < 0.02)
    k = 1.0;

  double base_margin = 3.0 * params.kernel_sigma;
  double para_margin = base_margin * k;
  double max_expansion = std::max(base_margin, para_margin);

  Eigen::Vector3d min_aabb =
      obstacle_center -
      Eigen::Vector3d::Constant(obstacle_radius + max_expansion);
  Eigen::Vector3d max_aabb =
      obstacle_center +
      Eigen::Vector3d::Constant(obstacle_radius + max_expansion);

  manager.latest_aabb_min_ = min_aabb;
  manager.latest_aabb_max_ = max_aabb;

  double res = spatial_index->getVoxelSize();
  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  Eigen::Vector3i start_idx =
      ((min_aabb - world_min).array() / res).floor().cast<int>();
  Eigen::Vector3i end_idx = ((max_aabb - world_min).array() / res).ceil().cast<int>();

  manager.debug_voxels_.clear();

  for (int z = start_idx.z(); z <= end_idx.z(); ++z) {
    double pz = world_min.z() + (z + 0.5) * res;
    for (int y = start_idx.y(); y <= end_idx.y(); ++y) {
      double py = world_min.y() + (y + 0.5) * res;
      for (int x = start_idx.x(); x <= end_idx.x(); ++x) {
        double px = world_min.x() + (x + 0.5) * res;
        Eigen::Vector3d p_voxel(px, py, pz);
        double dist_phys = (p_voxel - obstacle_center).norm();
        double d_surf_phys = std::max(0.0, dist_phys - obstacle_radius);
        double dist_eff = d_surf_phys;

        if (d_surf_phys > 0) {
          Eigen::Vector3d d_vec = p_voxel - obstacle_center;
          double d_para = d_vec.dot(dir);
          if (d_para > 0) {
            double cos_theta = d_para / dist_phys;
            double stretch_factor = 1.0 + (k - 1.0) * cos_theta;
            dist_eff = d_surf_phys / stretch_factor;
            dist_eff /= (1.0 + 3.0 * d_para);
          }
        }

        if (dist_eff > 3.0 * params.kernel_sigma)
          continue;

        float val =
            (float)std::exp(-(dist_eff * dist_eff) /
                            (2.0 * params.kernel_sigma * params.kernel_sigma));

        if (val > 0.05f) {
          SafetyStateManager::VoxelVizData vvd;
          vvd.center = p_voxel;
          vvd.size = Eigen::Vector3d::Constant(res);
          vvd.danger = val;
          manager.debug_voxels_.push_back(vvd);
        }

        if (val > 0.001f) {
          for (int dz2 = -1; dz2 <= 1; ++dz2) {
            for (int dy2 = -1; dy2 <= 1; ++dy2) {
              for (int dx2 = -1; dx2 <= 1; ++dx2) {
                std::vector<int> nodes = spatial_index->getNodesInVoxel(GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(Eigen::Vector3i(x+dx2, y+dy2, z+dz2)));
                float dist_decay =
                    (dx2 == 0 && dy2 == 0 && dz2 == 0) ? 1.0f : 0.8f;
                float effective_val = val * dist_decay;
                for (int nid : nodes) {
                    if (nid >= 0 &&
                        (size_t)nid < manager.danger_levels_.size()) {
                      if (effective_val > manager.danger_levels_[nid])
                        manager.setDangerLevel(nid, effective_val);
                    }
                }
              }
            }
          }
        }
      }
    }
  }
}

} // namespace simulation
} // namespace robot_sim
