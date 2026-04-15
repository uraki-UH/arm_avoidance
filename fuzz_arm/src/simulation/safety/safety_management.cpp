/**
 * @file safety_management.cpp
 * @brief Implementation of safety state management and strategies.
 */

#include "simulation/safety/safety_management.hpp"
#include "common/node_status.hpp"
#include "spatial/index_voxel_grid.hpp"

namespace robot_sim {
namespace simulation {

// =========================================================
// SafetyStateManager Implementation
// =========================================================

SafetyStateManager::SafetyStateManager() { setParameters(SafetyParameters()); }

SafetyStateManager::~SafetyStateManager() = default;

void SafetyStateManager::resize(size_t num_nodes) {
  danger_levels_.resize(num_nodes, 0.0f);
  old_danger_levels_.resize(num_nodes, 0.0f);
  danger_velocities_.resize(num_nodes, 0.0f);
  touched_this_frame_.resize(num_nodes, false);

  node_collision_counters_.resize(num_nodes, 0);
  node_danger_counters_.resize(num_nodes, 0);
  node_generations_.assign(num_nodes, 0);
  touched_indices_.reserve(num_nodes);

  active_set_.resize(num_nodes);
}

void SafetyStateManager::reset() {
  std::fill(danger_levels_.begin(), danger_levels_.end(), 0.0f);
  std::fill(old_danger_levels_.begin(), old_danger_levels_.end(), 0.0f);
  std::fill(danger_velocities_.begin(), danger_velocities_.end(), 0.0f);
  std::fill(touched_this_frame_.begin(), touched_this_frame_.end(), false);

  std::fill(node_collision_counters_.begin(), node_collision_counters_.end(),
            0);
  std::fill(node_danger_counters_.begin(), node_danger_counters_.end(), 0);
  prev_occupied_voxels_.clear();
  prev_danger_voxels_.clear();

  active_set_.clear();
  last_touched_count_ = 0;
}

void SafetyStateManager::setParameters(const SafetyParameters &params) {
  params_ = params;
  setInstantaneousMode(params_.instantaneous_mode);

  std::cout << "[SafetyManagement] Parameters applied. Mode: "
            << (params_.instantaneous_mode ? "Instantaneous" : "Time-Decay")
            << std::endl;
}

void SafetyStateManager::setInstantaneousMode(bool enabled) {
  instantaneous_mode_ = enabled;
  if (enabled) {
    strategy_ = std::make_unique<InstantaneousSafetyStrategy>();
  } else {
    strategy_ = std::make_unique<TimeDecaySafetyStrategy>();
  }
}

void SafetyStateManager::setDangerLevel(int node_id, float level) {
  if (node_id < 0 || (size_t)node_id >= danger_levels_.size())
    return;

  if (!touched_this_frame_[node_id]) {
    touched_this_frame_[node_id] = true;
    last_touched_count_++;
  }

  if (!active_set_.contains(node_id)) {
    active_set_.add(node_id);
  }

  if (level > danger_levels_[node_id]) {
    danger_levels_[node_id] = level;
  }
}

void SafetyStateManager::prepareUpdate() {
  old_danger_levels_ = danger_levels_;
  std::fill(touched_this_frame_.begin(), touched_this_frame_.end(), false);
  last_touched_count_ = 0;
  if (instantaneous_mode_) {
    for (int nid : active_set_.indices()) {
      danger_levels_[nid] = 0.0f;
    }
  }
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
  return last_touched_count_;
}

void SafetyStateManager::updateDangerField(
    std::shared_ptr<analysis::DenseSpatialIndex> spatial_index,
    const Eigen::Vector3d &obstacle_center, double obstacle_radius,
    const Eigen::Vector3d &velocity) {
  if (strategy_) {
    strategy_->updateDangerField(*this, spatial_index, obstacle_center,
                                 obstacle_radius, velocity);
  }
}

void SafetyStateManager::updateDangerFieldFromVoxels(
    std::shared_ptr<analysis::DenseSpatialIndex> spatial_index,
    const std::vector<long> &occupied_voxels,
    const std::vector<long> &danger_voxels) {
  if (!spatial_index)
    return;

  std::vector<long> added_occ, removed_occ, added_dan, removed_dan;
  ::common::geometry::VoxelUtils::computeDiff(
      prev_occupied_voxels_, occupied_voxels, added_occ, removed_occ);
  ::common::geometry::VoxelUtils::computeDiff(
      prev_danger_voxels_, danger_voxels, added_dan, removed_dan);

  // 1. Update counters using the fast SpatialIndex API
  spatial_index->updateCounts(added_occ, node_collision_counters_, 1);
  spatial_index->updateCounts(removed_occ, node_collision_counters_, -1);
  spatial_index->updateCounts(added_dan, node_danger_counters_, 1);
  spatial_index->updateCounts(removed_dan, node_danger_counters_, -1);

  // 2. Refresh node states
  auto refresh = [&](const std::vector<long> &vids) {
    for (long vid : vids) {
      for (int nid : spatial_index->getNodesInVoxel(vid)) {
        if (nid >= 0 && (size_t)nid < danger_levels_.size()) {
          danger_levels_[nid] = ::GNG::SafetyRules::deriveLevel(
              node_collision_counters_[nid], node_danger_counters_[nid]);
          if (danger_levels_[nid] > 0.0f)
            active_set_.add(nid);
        }
      }
    }
  };

  refresh(added_occ);
  refresh(removed_occ);
  refresh(added_dan);
  refresh(removed_dan);

  prev_occupied_voxels_ = occupied_voxels;
  prev_danger_voxels_ = danger_voxels;

  auto add_viz = [&](const std::vector<long> &voxels, float level) {
    for (long vid : voxels) {
      Eigen::Vector3i g_idx =
          GNG::Analysis::IndexVoxelGrid::getIndexFromFlatId(vid);
      VoxelVizData vvd;
      vvd.center = spatial_index->getWorldMin() +
                   (g_idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
                       spatial_index->getResolution();
      vvd.size = Eigen::Vector3d::Constant(spatial_index->getResolution());
      vvd.danger = level;
      debug_voxels_.push_back(vvd);
    }
  };
  add_viz(prev_danger_voxels_, 0.5f);
  add_viz(prev_occupied_voxels_, 1.0f);
}

void SafetyStateManager::applyOccupancyDeltas(
    std::shared_ptr<analysis::ISpatialIndex> spatial_index,
    const GlobalVoxelDelta &delta, double voxel_size) {
  (void)voxel_size;
  if (!spatial_index)
    return;

  // 1. Update counters using the fast SpatialIndex API
  spatial_index->updateCounts(delta.occupation.added, node_collision_counters_,
                              +1);
  spatial_index->updateCounts(delta.occupation.removed,
                              node_collision_counters_, -1);
  spatial_index->updateCounts(delta.danger.added, node_danger_counters_, +1);
  spatial_index->updateCounts(delta.danger.removed, node_danger_counters_, -1);

  // 2. Activate added nodes to ensure they are processed by strategies
  auto activate_nodes = [&](const std::vector<long> &voxels) {
    auto dense_idx =
        std::dynamic_pointer_cast<analysis::DenseSpatialIndex>(spatial_index);
    if (dense_idx) {
      for (long vid : voxels) {
        auto rels = dense_idx->getRelationsInVoxelRaw(vid);
        for (int i = 0; i < rels.second; ++i) {
          int nid = rels.first[i].node_id;
          if (nid >= 0 && (size_t)nid < active_set_.mask().size()) {
            if (!active_set_.contains(nid)) {
              active_set_.add(nid);
            }
          }
        }
      }
    } else {
      // Fallback for non-dense spatial index
      for (long vid : voxels) {
        auto nodes = spatial_index->getNodesInVoxel(vid);
        for (int nid : nodes) {
          if (nid >= 0 && (size_t)nid < active_set_.mask().size()) {
            if (!active_set_.contains(nid)) {
              active_set_.add(nid);
            }
          }
        }
      }
    }
  };

  activate_nodes(delta.occupation.added);
  activate_nodes(delta.danger.added);
}

void SafetyStateManager::updateDangerFieldFromPoints(
    std::shared_ptr<analysis::DenseSpatialIndex> spatial_index,
    const std::vector<Eigen::Vector3d> &points, float dilation,
    bool use_dilation) {
  if (!spatial_index || points.empty())
    return;

  // 1. Efficient Reset (Handled centrally in prepareUpdate for instantaneous mode)
  // Removed std::fill and active_set_.clear() to prevent overwriting dynamic obstacle fields.

  // 2. Voxelized Dilation Lookup
  double res = spatial_index->getResolution();
  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  Eigen::Vector3i dims = spatial_index->getDims();
  int K = use_dilation ? static_cast<int>(std::ceil(dilation / res)) : 0;

  std::unordered_set<long> processed_voxels;
  processed_voxels.reserve(points.size());

  for (const auto &p : points) {
    long vid = spatial_index->getVoxelId(p);
    if (vid == -1 || processed_voxels.count(vid))
      continue;
    processed_voxels.insert(vid);

    Eigen::Vector3i v_idx =
        ((p - world_min).array() / res).floor().cast<int>();

    // Dilation Loop (Cube search + Sphere distance filter)
    for (int dz = -K; dz <= K; ++dz) {
      int z = v_idx.z() + dz;
      if (z < 0 || z >= dims.z())
        continue;
      for (int dy = -K; dy <= K; ++dy) {
        int y = v_idx.y() + dy;
        if (y < 0 || y >= dims.y())
          continue;
        for (int dx = -K; dx <= K; ++dx) {
          int x = v_idx.x() + dx;
          if (x < 0 || x >= dims.x())
            continue;

          // Sphere filter
          double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * res;
          if (dist > dilation + 1e-6)
            continue;

          float level = (dist < 0.005f) ? 1.0f : 0.5f;

          auto rel_data = spatial_index->getRelationsInVoxelRawByIndex(
              Eigen::Vector3i(x, y, z));
          if (rel_data.first && rel_data.second > 0) {
            for (int i = 0; i < rel_data.second; ++i) {
              int nid = rel_data.first[i].node_id;
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

// =========================================================
// InstantaneousSafetyStrategy Implementation
// =========================================================

void InstantaneousSafetyStrategy::update(SafetyStateManager &manager, float dt,
                                         float max_velocity) {
  (void)dt;
  (void)max_velocity;

  auto &active_set = manager.active_set_;
  auto &danger_levels = manager.danger_levels_;
  const auto &coll_counters = manager.node_collision_counters_;
  const auto &dang_counters = manager.node_danger_counters_;

  manager.touched_indices_ = manager.active_set_.indices();

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
    std::shared_ptr<analysis::DenseSpatialIndex> spatial_index,
    const Eigen::Vector3d &obstacle_center, double obstacle_radius,
    const Eigen::Vector3d &velocity) {
  (void)velocity;
  if (!spatial_index)
    return;

  double res = spatial_index->getResolution();
  double margin = manager.params_.danger_voxel_dilation;
  Eigen::Vector3d min_aabb =
      obstacle_center - Eigen::Vector3d::Constant(obstacle_radius + margin);
  Eigen::Vector3d max_aabb =
      obstacle_center + Eigen::Vector3d::Constant(obstacle_radius + margin);

  manager.latest_aabb_min_ = min_aabb;
  manager.latest_aabb_max_ = max_aabb;

  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  Eigen::Vector3i dims = spatial_index->getDims();
  Eigen::Vector3i start_idx =
      ((min_aabb - world_min).array() / res).floor().cast<int>().cwiseMax(0);
  Eigen::Vector3i end_idx = ((max_aabb - world_min).array() / res)
                                .ceil()
                                .cast<int>()
                                .cwiseMin(dims.array() - 1);

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
        if (dist_sq <= obstacle_radius * obstacle_radius)
          danger_val = 1.0f;
        else if (dist_sq <=
                 (obstacle_radius + margin) * (obstacle_radius + margin))
          danger_val = 0.5f;

        if (danger_val > 0.0f) {
          SafetyStateManager::VoxelVizData vvd;
          vvd.center = p_voxel;
          vvd.size = Eigen::Vector3d::Constant(res);
          vvd.danger = danger_val;
          manager.debug_voxels_.push_back(vvd);

          auto result = spatial_index->getRelationsInVoxelRawByIndex(
              Eigen::Vector3i(x, y, z));
          const auto *rel_ptr = result.first;
          int count = result.second;
          if (rel_ptr && count > 0) {
            for (int i = 0; i < count; ++i) {
              int nid = rel_ptr[i].node_id;
              if (danger_val > manager.getDangerLevel(nid))
                manager.setDangerLevel(nid, danger_val);
            }
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
  const auto &coll_counters = manager.node_collision_counters_;
  const auto &dang_counters = manager.node_danger_counters_;
  manager.touched_indices_ = manager.active_set_.indices();

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
    std::shared_ptr<analysis::DenseSpatialIndex> spatial_index,
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

  double res = spatial_index->getResolution();
  Eigen::Vector3d world_min = spatial_index->getWorldMin();
  Eigen::Vector3i dims = spatial_index->getDims();
  Eigen::Vector3i start_idx =
      ((min_aabb - world_min).array() / res).floor().cast<int>().cwiseMax(0);
  Eigen::Vector3i end_idx = ((max_aabb - world_min).array() / res)
                                .ceil()
                                .cast<int>()
                                .cwiseMin(dims.array() - 1);

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
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                auto result = spatial_index->getRelationsInVoxelRawByIndex(
                    Eigen::Vector3i(x + dx, y + dy, z + dz));
                const auto *rel_ptr = result.first;
                int count = result.second;
                if (rel_ptr && count > 0) {
                  float dist_decay =
                      (dx == 0 && dy == 0 && dz == 0) ? 1.0f : 0.8f;
                  float effective_val = val * dist_decay;
                  for (int i = 0; i < count; ++i) {
                    int nid = rel_ptr[i].node_id;
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
}

} // namespace simulation
} // namespace robot_sim
