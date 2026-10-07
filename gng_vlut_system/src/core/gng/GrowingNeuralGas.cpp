#include "GrowingNeuralGas.hpp"
#include "nearest_node_index.hpp"
#include "collision/joint_segment_collision.hpp"
#include <map>
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/voxel_collision_checker.hpp"
#include "safety_engine/vlut/iself_collision_checker.hpp"
#include "common/resource_utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

using namespace GNG;

namespace {

// 符号付きゼロを含む保存角のビット一致。量子化・許容誤差なし。
template <typename angle_type>
bool has_same_angles(const angle_type &first, const angle_type &second) {
  return first.size() == second.size() &&
      std::memcmp(first.data(), second.data(),
                  first.size() * sizeof(typename angle_type::Scalar)) == 0;
}

static std::string makePairKey(std::string a, std::string b) {
  if (a > b) {
    std::swap(a, b);
  }
  return a + "|" + b;
}

static std::vector<std::pair<std::string, std::string>>
collectCollisionPairs(simulation::ISelfCollisionChecker *checker) {
  if (!checker) {
    return {};
  }
  if (auto *voxel =
          dynamic_cast<simulation::VoxelCollisionChecker *>(checker)) {
    return voxel->collectSelfCollisionPairs();
  }
  if (auto *geometric =
          dynamic_cast<simulation::GeometricSelfCollisionChecker *>(checker)) {
    return geometric->collectSelfCollisionPairs();
  }
  return {};
}

static void printTopCollisionPairs(
    const std::unordered_map<std::string, int> &pair_counts,
    std::size_t max_items = 8) {
  if (pair_counts.empty()) {
    return;
  }
  std::vector<std::pair<std::string, int>> sorted(pair_counts.begin(),
                                                  pair_counts.end());
  std::sort(sorted.begin(), sorted.end(),
            [](const auto &a, const auto &b) {
              if (a.second != b.second) {
                return a.second > b.second;
              }
              return a.first < b.first;
            });

  std::cout << "[StrictFilter] Top collision pairs:" << std::endl;
  for (std::size_t i = 0; i < std::min(max_items, sorted.size()); ++i) {
    std::cout << "  " << sorted[i].first << " -> " << sorted[i].second
              << std::endl;
  }
}

} // namespace

namespace GrowingNeuralGas_Internal {
template <typename T> void write_eigen(std::ofstream &out, const T &matrix) {
  typename T::Index rows = matrix.rows(), cols = matrix.cols();
  out.write((char *)&rows, sizeof(typename T::Index));
  out.write((char *)&cols, sizeof(typename T::Index));
  out.write((char *)matrix.data(), rows * cols * sizeof(typename T::Scalar));
}

template <typename T> void read_eigen(std::ifstream &in, T &matrix) {
  typename T::Index rows = 0, cols = 0;
  in.read((char *)&rows, sizeof(typename T::Index));
  in.read((char *)&cols, sizeof(typename T::Index));
  if (T::RowsAtCompileTime == Eigen::Dynamic ||
      T::ColsAtCompileTime == Eigen::Dynamic) {
    matrix.resize(rows, cols);
  }
  in.read((char *)matrix.data(), rows * cols * sizeof(typename T::Scalar));
}
} // namespace GrowingNeuralGas_Internal

template <typename T_angle, typename T_coord>
GrowingNeuralGas<T_angle, T_coord>::GrowingNeuralGas(
    int angle_dim, int coord_dim, kinematics::KinematicChain *chain)
    : kinematic_chain_(chain), angle_dimension(angle_dim),
      coord_dimension(coord_dim) {
  nodes.resize(params_.max_node_num);
  for (int i = 0; i < (int)nodes.size(); ++i)
    addable_node_indicies.push(i);

  edges_angle.resize(nodes.size());
  edges_coord.resize(nodes.size());
  edges_angle_per_node.resize(nodes.size());
  edges_coord_per_node.resize(nodes.size());
  edges_coord_per_layer_.assign(coord_layer_count_,
                                std::vector<std::unordered_map<int, EdgeInfo>>(nodes.size()));
  edges_coord_per_layer_nodes_.assign(
      coord_layer_count_, std::vector<std::vector<int>>(nodes.size()));
  random_joint_buffer_.reserve(static_cast<std::size_t>(angle_dimension));
  if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic) {
    random_angle_buffer_.resize(angle_dimension);
  }


  if (!kinematic_chain_) {
    for (int i = 0; i < (int)nodes.size(); ++i) {
      nodes[i].id = -1;
      nodes[i].status.active = false;
    }
    return;
  }

  // Initialize nodes
  for (int i = 0; i < params_.start_node_num; ++i) {
    T_angle wa;
    if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
      wa.setZero(angle_dimension);
    else
      wa.setZero();

    kinematic_chain_->sampleRandomJointValues(random_joint_buffer_);
    for (int j = 0; j < angle_dimension; ++j) {
      wa(j) = static_cast<typename T_angle::Scalar>(random_joint_buffer_[j]);
    }
    add_node(wa);
  }
}

template <typename T_angle, typename T_coord>
GrowingNeuralGas<T_angle, T_coord>::~GrowingNeuralGas() {}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::setCoordLayerCount(int layer_count) {
  coord_nearest_indexes_.clear();
  coord_layer_count_ = std::max(1, layer_count);
  edges_coord_per_layer_.assign(
      coord_layer_count_,
      std::vector<std::unordered_map<int, EdgeInfo>>(nodes.size()));
  edges_coord_per_layer_nodes_.assign(
      coord_layer_count_, std::vector<std::vector<int>>(nodes.size()));
  for (int layer = 0; layer < coord_layer_count_; ++layer) {
    for (size_t i = 0; i < nodes.size(); ++i) {
      edges_coord_per_layer_[layer][i].clear();
      edges_coord_per_layer_nodes_[layer][i].clear();
    }
  }
  for (auto &node : nodes) {
    if (node.id >= 0) {
      if (node.weight_coords.size() < static_cast<size_t>(coord_layer_count_)) {
        T_coord zero_coord;
        if constexpr (T_coord::RowsAtCompileTime == Eigen::Dynamic ||
                      T_coord::ColsAtCompileTime == Eigen::Dynamic) {
          zero_coord = T_coord::Zero(coord_dimension);
        } else {
          zero_coord = T_coord::Zero();
        }
        node.weight_coords.resize(static_cast<size_t>(coord_layer_count_),
                                  zero_coord);
      }
    }
  }
}

template <typename T_angle, typename T_coord>
T_coord
GrowingNeuralGas<T_angle, T_coord>::calculateFK(const T_angle &angle_values) {
  return calculateFK(angle_values, 0);
}

template <typename T_angle, typename T_coord>
T_coord GrowingNeuralGas<T_angle, T_coord>::calculateFK(
    const T_angle &angle_values, int coord_layer_index) {
  if (!kinematic_chain_) {
    if constexpr (T_coord::RowsAtCompileTime == Eigen::Dynamic)
      return T_coord::Zero(coord_dimension);
    else
      return T_coord::Zero();
  }
  int dof = kinematic_chain_->getTotalDOF();
  kinematic_chain_->updateKinematics(
      angle_values.head(std::min((int)angle_values.size(), dof)));
  if (coord_layer_index < 0) {
    coord_layer_index = 0;
  }
  const std::size_t arm_index =
      static_cast<std::size_t>(coord_layer_index);
  Eigen::Vector3d eef_position_double =
      kinematic_chain_->getEEFPosition(arm_index);
  return eef_position_double.cast<typename T_coord::Scalar>();
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::runStatusProviders(
    int node_id, UpdateTrigger trigger) {
  if (node_id < 0 || (size_t)node_id >= nodes.size() || nodes[node_id].id == -1)
    return;
  auto &node = nodes[node_id];
  bool has_position_writer = false;
  for (auto &provider : providers_) {
    auto triggers = provider->getTriggers();
    if (std::find(triggers.begin(), triggers.end(), trigger) !=
        triggers.end()) {
      if (provider->shouldUpdate(node, trigger)) {
        has_position_writer = has_position_writer || provider->can_modify_node_positions();
        provider->update(node, trigger);
      }
    }
  }
  // callbackが保持する別ノード参照への変更も含む索引の無効化。
  if (has_position_writer) invalidate_nearest_indexes();
  else sync_nearest_node(node_id);
}

template <typename T_angle, typename T_coord>
int GrowingNeuralGas<T_angle, T_coord>::add_node(const T_angle &w_angle) {
  return add_node(w_angle, calculateFK(w_angle));
}

template <typename T_angle, typename T_coord>
int GrowingNeuralGas<T_angle, T_coord>::add_node(const T_angle &w_angle,
                                                  const T_coord &w_coord) {
  if (addable_node_indicies.empty())
    return -1;
  int node_id = addable_node_indicies.front();
  addable_node_indicies.pop();
  nodes[node_id] =
      NeuronNode<T_angle, T_coord>(node_id, w_angle, w_coord);
  nodes[node_id].weight_coords.assign(
      static_cast<std::size_t>(coord_layer_count_), w_coord);
  nodes[node_id].task_density_ema = 1.0f;
  edges_angle[node_id].clear();
  edges_coord[node_id].clear();
  edges_angle_per_node[node_id].clear();
  edges_coord_per_node[node_id].clear();
  for (int layer = 0; layer < coord_layer_count_; ++layer) {
    edges_coord_per_layer_[layer][node_id].clear();
    edges_coord_per_layer_nodes_[layer][node_id].clear();
  }
  active_indices_.push_back(node_id);
  runStatusProviders(node_id, UpdateTrigger::NODE_ADDED);
  return node_id;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_node(int node) {
  if (node < 0 || (size_t)node >= nodes.size() || nodes[node].id == -1)
    return;

  if (static_cast<std::size_t>(node) < filter_poses_.size()) {
    filter_poses_[node].has_angles = false;
    filter_poses_[node].is_safe = false;
    ++filter_poses_[node].generation;
  }
  if (angle_nearest_index_) angle_nearest_index_->remove_point(node);
  for (auto &index : coord_nearest_indexes_) {
    if (index) index->remove_point(node);
  }

  // 削除IDの再利用時にも残存しない、関節空間辺の双方向削除
  const auto angle_neighbors = edges_angle_per_node[node];
  for (int other : angle_neighbors) remove_edge_angle(node, other);

  if (!edges_coord_per_node[node].empty()) {
    std::vector<int> coord_neighbors(edges_coord_per_node[node].begin(),
                                     edges_coord_per_node[node].end());
    for (int n : coord_neighbors)
      remove_edge_coord(node, n);
  }
  for (int layer = 0; layer < coord_layer_count_; ++layer) {
    if (node < static_cast<int>(edges_coord_per_layer_nodes_[layer].size())) {
      std::vector<int> coord_neighbors(
          edges_coord_per_layer_nodes_[layer][node].begin(),
          edges_coord_per_layer_nodes_[layer][node].end());
      for (int n : coord_neighbors) {
        remove_edge_coord(layer, node, n);
      }
      edges_coord_per_layer_[layer][node].clear();
      edges_coord_per_layer_nodes_[layer][node].clear();
    }
  }

  active_indices_.erase(
      std::remove(active_indices_.begin(), active_indices_.end(), node),
      active_indices_.end());

  edges_angle[node].clear();
  edges_coord[node].clear();
  edges_angle_per_node[node].clear();
  edges_coord_per_node[node].clear();
  nodes[node].weight_coords.clear();
  nodes[node].id = -1;
  nodes[node].task_density_ema = 1.0f;
  addable_node_indicies.push(node);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::add_edge_angle(int node_1,
                                                         int node_2) {
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  if (edges_angle[node_1].count(node_2)) {
    edges_angle[node_1][node_2].age = 1;
    edges_angle[node_2][node_1].age = 1;
  } else {
    edges_angle_per_node[node_1].push_back(node_2);
    edges_angle_per_node[node_2].push_back(node_1);
    edges_angle[node_1][node_2].age = 1;
    edges_angle[node_2][node_1].age = 1;
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_edge_angle(int node_1,
                                                            int node_2) {
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  auto &v1 = edges_angle_per_node[node_1];
  v1.erase(std::remove(v1.begin(), v1.end(), node_2), v1.end());
  auto &v2 = edges_angle_per_node[node_2];
  v2.erase(std::remove(v2.begin(), v2.end(), node_1), v2.end());

  edges_angle[node_1].erase(node_2);
  edges_angle[node_2].erase(node_1);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::add_edge_coord(int node_1,
                                                         int node_2) {
  add_edge_coord(0, node_1, node_2);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_edge_coord(int node_1,
                                                            int node_2) {
  remove_edge_coord(0, node_1, node_2);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::add_edge_coord(int layer_index,
                                                        int node_1,
                                                        int node_2) {
  if (layer_index < 0 ||
      layer_index >= static_cast<int>(edges_coord_per_layer_.size())) {
    return;
  }
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  auto &edges =
      (layer_index == 0) ? edges_coord : edges_coord_per_layer_[layer_index];
  auto &edges_per_node = (layer_index == 0)
                             ? edges_coord_per_node
                             : edges_coord_per_layer_nodes_[layer_index];
  if (edges[node_1].count(node_2)) {
    edges[node_1][node_2].age = 1;
    edges[node_2][node_1].age = 1;
  } else {
    edges_per_node[node_1].push_back(node_2);
    edges_per_node[node_2].push_back(node_1);
    edges[node_1][node_2].age = 1;
    edges[node_2][node_1].age = 1;
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_edge_coord(int layer_index,
                                                           int node_1,
                                                           int node_2) {
  if (layer_index < 0 ||
      layer_index >= static_cast<int>(edges_coord_per_layer_.size())) {
    return;
  }
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  auto &edges =
      (layer_index == 0) ? edges_coord : edges_coord_per_layer_[layer_index];
  auto &edges_per_node = (layer_index == 0)
                             ? edges_coord_per_node
                             : edges_coord_per_layer_nodes_[layer_index];
  auto &v1c = edges_per_node[node_1];
  v1c.erase(std::remove(v1c.begin(), v1c.end(), node_2), v1c.end());
  auto &v2c = edges_per_node[node_2];
  v2c.erase(std::remove(v2c.begin(), v2c.end(), node_1), v2c.end());

  edges[node_1].erase(node_2);
  edges[node_2].erase(node_1);
}

template <typename T_angle, typename T_coord>
float GrowingNeuralGas<T_angle, T_coord>::calc_squaredNorm_angle(
    const T_angle &w1, const T_angle &w2) const {
  return static_cast<float>((w1 - w2).squaredNorm());
}

template <typename T_angle, typename T_coord>
float GrowingNeuralGas<T_angle, T_coord>::calc_squaredNorm_coord(
    const T_coord &w1, const T_coord &w2) const {
  return static_cast<float>((w1 - w2).squaredNorm());
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::update_node_weights(
    int node_id, const T_angle &sample_angle, float step) {
  if (node_id < 0 || (size_t)node_id >= nodes.size() || nodes[node_id].id == -1)
    return;
  nodes[node_id].weight_angle +=
      step * (sample_angle - nodes[node_id].weight_angle);

  if (angle_nearest_index_) {
    angle_nearest_index_->set_point(node_id, nodes[node_id].weight_angle.data());
  }
  // 学習中のFK省略。保存前のrefresh_coord_weightsによる座標更新。
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::refresh_coord_weights() {
  if (!kinematic_chain_)
    return;
  for (int layer = 0; layer < coord_layer_count_; ++layer) {
    refresh_coord_weights(layer);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::refresh_coord_weights(
    int coord_layer_index) {
  coord_nearest_indexes_.clear();
  if (!kinematic_chain_)
    return;
  if (coord_layer_index < 0 ||
      coord_layer_index >= static_cast<int>(coord_layer_count_)) {
    return;
  }
  for (int i : active_indices_) {
    auto &node = nodes[i];
    kinematic_chain_->updateKinematics(node.weight_angle);
    const auto &pts = kinematic_chain_->getLinkPositions();
    const auto &oris = kinematic_chain_->getLinkOrientations();

    if (!pts.empty() && !oris.empty()) {
      const std::size_t arm_index = static_cast<std::size_t>(coord_layer_index);
      Eigen::Vector3d eef_position = kinematic_chain_->getEEFPosition(arm_index);
      Eigen::Quaterniond eef_orientation =
          kinematic_chain_->getEEFOrientation(arm_index);
      if (coord_layer_index == 0) {
        node.weight_coord = eef_position.template cast<typename T_coord::Scalar>();
        node.status.ee_direction =
            (eef_orientation * Eigen::Vector3d::UnitX()).template cast<float>();
        node.status.ee_orientation = eef_orientation.template cast<float>();
      }

      if (node.weight_coords.size() <
          static_cast<std::size_t>(coord_layer_count_)) {
        node.weight_coords.resize(
            static_cast<std::size_t>(coord_layer_count_),
            node.weight_coord);
      }
      node.weight_coords[coord_layer_index] =
          eef_position.template cast<typename T_coord::Scalar>();
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::invalidate_nearest_indexes() {
  angle_nearest_index_.reset();
  coord_nearest_indexes_.clear();
}

template <typename T_angle, typename T_coord>
const T_coord &GrowingNeuralGas<T_angle, T_coord>::node_coord(
    int node_id, int coord_layer_idx) const {
  const auto &node = nodes[node_id];
  return coord_layer_idx == 0 ||
                 node.weight_coords.size() <= static_cast<std::size_t>(coord_layer_idx)
             ? node.weight_coord : node.weight_coords[coord_layer_idx];
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::sync_nearest_node(int node_id) {
  if (angle_nearest_index_) {
    if (nodes[node_id].weight_angle.size() != angle_dimension) {
      angle_nearest_index_.reset();
    } else {
      angle_nearest_index_->set_point(node_id, nodes[node_id].weight_angle.data());
    }
  }
  for (std::size_t layer_idx = 0; layer_idx < coord_nearest_indexes_.size(); ++layer_idx) {
    auto &index = coord_nearest_indexes_[layer_idx];
    if (!index) continue;
    const auto &coord = node_coord(node_id, static_cast<int>(layer_idx));
    if (coord.size() != coord_dimension) index.reset();
    else index->set_point(node_id, coord.data());
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::find_nearest_angle(
    const T_angle &sample_angle, int num_candidates) {
  if (params_.enable_nearest_index && sample_angle.size() == angle_dimension) {
    if (!angle_nearest_index_) {
      angle_nearest_index_ = std::make_unique<nearest_node_index>(angle_dimension);
      if (angle_nearest_index_->is_supported()) {
        for (int node_idx = 0; node_idx < static_cast<int>(nodes.size()); ++node_idx) {
          if (nodes[node_idx].id == -1) continue;
          angle_nearest_index_->set_point(node_idx,
              nodes[node_idx].weight_angle.size() == angle_dimension
                  ? nodes[node_idx].weight_angle.data() : nullptr);
        }
      }
    }
    if (angle_nearest_index_->query(sample_angle.data(), num_candidates,
            [&](int node_idx) { return calc_squaredNorm_angle(sample_angle, nodes[node_idx].weight_angle); },
            candidates_buffer_)) return;
  }
  candidates_buffer_.clear();
  candidates_buffer_.reserve(nodes.size());
  for (int node_idx = 0; node_idx < static_cast<int>(nodes.size()); ++node_idx) {
    if (nodes[node_idx].id != -1) candidates_buffer_.emplace_back(
        calc_squaredNorm_angle(sample_angle, nodes[node_idx].weight_angle), node_idx);
  }
  const auto num_found = std::min(candidates_buffer_.size(), static_cast<std::size_t>(num_candidates));
  std::partial_sort(candidates_buffer_.begin(), candidates_buffer_.begin() + num_found,
                    candidates_buffer_.end());
  candidates_buffer_.resize(num_found);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::find_nearest_coord(
    const T_coord &sample_coord, int coord_layer_idx) {
  if (params_.enable_nearest_index && sample_coord.size() == coord_dimension) {
    if (coord_nearest_indexes_.size() != static_cast<std::size_t>(coord_layer_count_)) {
      coord_nearest_indexes_.resize(coord_layer_count_);
    }
    auto &index = coord_nearest_indexes_[coord_layer_idx];
    if (!index) {
      index = std::make_unique<nearest_node_index>(coord_dimension);
      if (index->is_supported()) {
        for (int node_idx = 0; node_idx < static_cast<int>(nodes.size()); ++node_idx) {
          if (nodes[node_idx].id == -1) continue;
          const auto &coord = node_coord(node_idx, coord_layer_idx);
          index->set_point(node_idx, coord.size() == coord_dimension ? coord.data() : nullptr);
        }
      }
    }
    if (index->query(sample_coord.data(), 2,
            [&](int node_idx) { return calc_squaredNorm_coord(sample_coord, node_coord(node_idx, coord_layer_idx)); },
            candidates_buffer_)) {
      // 従来のTCP走査で候補外となる最大float距離の除外。
      candidates_buffer_.erase(std::remove_if(candidates_buffer_.begin(), candidates_buffer_.end(),
          [](const auto &candidate) { return !(candidate.first < std::numeric_limits<float>::max()); }),
          candidates_buffer_.end());
      return;
    }
  }
  // 従来の走査順と同距離時の小さいID優先を保持した2近傍。
  int first_idx = -1, second_idx = -1;
  float first_dist = std::numeric_limits<float>::max();
  float second_dist = first_dist;
  for (int node_idx = 0; node_idx < static_cast<int>(nodes.size()); ++node_idx) {
    if (nodes[node_idx].id == -1) continue;
    const float dist = calc_squaredNorm_coord(sample_coord, node_coord(node_idx, coord_layer_idx));
    if (dist < first_dist) {
      second_dist = first_dist; second_idx = first_idx;
      first_dist = dist; first_idx = node_idx;
    } else if (dist < second_dist) {
      second_dist = dist; second_idx = node_idx;
    }
  }
  candidates_buffer_.clear();
  if (first_idx != -1) candidates_buffer_.emplace_back(first_dist, first_idx);
  if (second_idx != -1) candidates_buffer_.emplace_back(second_dist, second_idx);
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::internalCheckColliding(
    const T_angle &angles) {
  if (!collision_checker_ || !kinematic_chain_)
    return false;

  kinematic_chain_->forwardKinematicsAt(angles, q_buffer, pos_buffer,
                                        ori_buffer);
  collision_checker_->updateBodyPoses(pos_buffer, ori_buffer);
  return collision_checker_->checkCollision();
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::internalCheckPathColliding(
    const T_angle &q1, const T_angle &q2, int /*steps*/) {

  // 端点と全関節の補間刻みに基づく検査。微小区間の無条件通過なし。
  return simulation::has_joint_segment_collision(
      q1, q2, 0.025, [this](const T_angle &angles) {
        return internalCheckColliding(angles);
      });
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::one_train_update(
    const T_angle &sample_angle) {
  // 従来と同じ100反復ごとの誤差減衰。近傍探索から独立した全ノード更新。
  constexpr int num_decay_steps = 100;
  accumulated_decay_factor_ *= (1.0f - params_.beta);
  ++decay_step_count_;
  if (decay_step_count_ >= num_decay_steps) {
    for (auto &node : nodes) {
      if (node.id != -1) node.error_angle *= accumulated_decay_factor_;
    }
    accumulated_decay_factor_ = 1.0f;
    decay_step_count_ = 0;
  }

  find_nearest_angle(sample_angle, std::max(2, params_.n_best_candidates));
  if (candidates_buffer_.size() < 2) return;
  const int n_search = std::max(0, std::min(
      static_cast<int>(candidates_buffer_.size()), params_.n_best_candidates));

  std::deque<int> candidate_ids;
  for (int i = 0; i < n_search; ++i)
    candidate_ids.push_back(candidates_buffer_[i].second);

  std::vector<int> winners;
  std::vector<float> winners_dist_sq; // 保存用
  int trials = 0;
  while (trials < n_search && winners.size() < 2 && !candidate_ids.empty()) {
    int cid = candidate_ids.front();
    candidate_ids.pop_front();
    // サンプル点への直線経路が衝突しないノードを探す
    if (!collision_aware_ ||
        !internalCheckPathColliding(sample_angle, nodes[cid].weight_angle)) {
      winners.push_back(cid);
      // 対応する距離を検索（n_searchが小さいので線形探索で十分）
      for(int k=0; k<n_search; ++k) {
          if(candidates_buffer_[k].second == cid) {
              winners_dist_sq.push_back(candidates_buffer_[k].first);
              break;
          }
      }
    }
    trials++;
  }

  if (winners.empty()) {
    n_trial_angle++;
    return;
  }

  int s1 = winners[0];
  int s2 = (winners.size() >= 2) ? winners[1] : -1;
  float dist_s1_sq = winners_dist_sq[0];
  float dist_s2_sq = (s2 != -1) ? winners_dist_sq[1] : 0.0f;
  float dist = std::sqrt(dist_s1_sq);

  float task_space_gain = 1.0f;
  if (params_.use_task_density_bias && kinematic_chain_) {
    const T_coord sample_coord = calculateFK(sample_angle);
    const T_coord winner_coord = calculateFK(nodes[s1].weight_angle);
    const float task_error = std::sqrt(calc_squaredNorm_coord(sample_coord, winner_coord));
    if (!task_error_ema_initialized_) {
      task_error_ema_ = task_error;
      task_error_ema_initialized_ = true;
    } else {
      const float task_alpha = std::max(0.0f, params_.task_error_ema_alpha);
      task_error_ema_ = (1.0f - task_alpha) * task_error_ema_ + task_alpha * task_error;
    }

    float normalized_task_error = 1.0f;
    if (task_error_ema_ > 0.0f) {
      normalized_task_error = task_error / task_error_ema_;
    }
    normalized_task_error = std::clamp(
        normalized_task_error, params_.task_error_gain_min,
        params_.task_error_gain_max);

    const float responsibility = std::exp(-normalized_task_error);
    const float density_alpha = std::max(0.0f, params_.task_density_ema_alpha);
    nodes[s1].task_density_ema =
        (1.0f - density_alpha) * nodes[s1].task_density_ema +
        density_alpha * responsibility;

    float density_mean = 0.0f;
    int density_count = 0;
    for (int i : active_indices_) {
      if (nodes[i].id == -1) {
        continue;
      }
      density_mean += nodes[i].task_density_ema;
      density_count++;
    }
    if (density_count > 0) {
      density_mean /= static_cast<float>(density_count);
      const float density_ratio =
          nodes[s1].task_density_ema / density_mean;
      float density_gain = std::pow(
          1.0f / std::max(1.0f, density_ratio),
          std::max(0.0f, params_.task_density_gain_gamma));
      task_space_gain = std::clamp(
          density_gain, params_.task_density_gain_min,
          params_.task_density_gain_max);
    } else {
      task_space_gain = 1.0f;
    }
  }

  // --- 統計収集 (1次・2次遅れフィルタ) ---
  // (dist_s1_sq, dist_s2_sq は計算済みのため再利用)

  float lpf_alpha = params_.lpf_alpha;
  // 1次EMA
  ema1_s1_sq = (1.0f - lpf_alpha) * ema1_s1_sq + lpf_alpha * dist_s1_sq;
  if (s2 != -1) {
    ema1_s2_sq = (1.0f - lpf_alpha) * ema1_s2_sq + lpf_alpha * dist_s2_sq;
  }

  // 2次EMA (直列)
  ema2_s1_sq = (1.0f - lpf_alpha) * ema2_s1_sq + lpf_alpha * ema1_s1_sq;
  if (s2 != -1) {
    ema2_s2_sq = (1.0f - lpf_alpha) * ema2_s2_sq + lpf_alpha * ema1_s2_sq;
  }

  // ファイル出力 (1000回に1回に制限して高速化)
  if (n_learning % 1000 == 0) {
    if (!stats_ofs_.is_open()) {
      stats_ofs_.open(stats_log_path_);
    }
    if (stats_ofs_.is_open()) {
      stats_ofs_ << n_learning << " " << dist_s1_sq << " " << dist_s2_sq << " "
                 << ema1_s1_sq << " " << ema1_s2_sq << " " << ema2_s1_sq << " "
                 << ema2_s2_sq << "\n";
      stats_ofs_.flush();
    }
  }

  // Add-if-Silent (AiS): s2との距離が閾値以上なら新規ノード追加
  if (s2 != -1) {
    float dist2 = std::sqrt(dist_s2_sq);
    if (dist2 > params_.ais_threshold && !addable_node_indicies.empty()) {
      int new_id = add_node(sample_angle);
      if (new_id != -1)
        add_edge_angle(new_id, s1);
      n_trial_angle++;
      n_learning++;
      return;
    }
  }

  nodes[s1].error_angle += dist;
  update_node_weights(s1, sample_angle,
                      params_.learn_rate_s1 * task_space_gain);

  if (s2 != -1) {
    // エッジ生成可能かどうかの干渉チェック
    if (!collision_aware_ ||
        !internalCheckPathColliding(nodes[s1].weight_angle,
                                    nodes[s2].weight_angle)) {
      add_edge_angle(s1, s2);
    } 
    // else if (collision_aware_) {
    //   // 衝突している場合はリフレッシュせず、既存エッジがあれば即座に削除（そうしないでエイジングで削除したほうがいい可能性あり）
    //   remove_edge_angle(s1, s2);
    // }
  }

  // 周辺ノードの更新とエイジング
  const std::vector<int> &neighbors = edges_angle_per_node[s1];
  std::vector<int> to_remove;
  for (int nid : neighbors) {
    if (edges_angle[s1][nid].age > params_.max_edge_age) {
      to_remove.push_back(nid);
    } else {
      update_node_weights(nid, sample_angle,
                          params_.learn_rate_s2 * task_space_gain);
      edges_angle[s1][nid].age++;
      edges_angle[nid][s1].age++;
    }
  }
  for (int nid : to_remove) {
    remove_edge_angle(s1, nid);
    if (edges_angle_per_node[nid].empty())
      remove_node(nid);
  }

  // Lambda check (Node addition between high-error nodes)
  if (n_trial_angle >= (int)params_.lambda) {
    n_trial_angle = 0;
    int q = -1, f = -1;
    float max_e = -1.0f;
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (nodes[i].id != -1 && nodes[i].error_angle > max_e) {
        max_e = nodes[i].error_angle;
        q = i;
      }
    }
    if (q != -1) {
      float max_ef = -1.0f;
      for (int nid : edges_angle_per_node[q]) {
        if (nodes[nid].error_angle > max_ef) {
          max_ef = nodes[nid].error_angle;
          f = nid;
        }
      }
    }
    if (q != -1 && f != -1) {
      T_angle mid = (nodes[q].weight_angle + nodes[f].weight_angle) * 0.5f;
      if (!collision_aware_ || !internalCheckColliding(mid)) {
        int new_id = add_node(mid);
        if (new_id != -1) {
          remove_edge_angle(q, f);
          add_edge_angle(q, new_id);
          add_edge_angle(f, new_id);
          nodes[q].error_angle *= params_.alpha;
          nodes[f].error_angle *= params_.alpha;
          nodes[new_id].error_angle = nodes[q].error_angle;
        }
      }
    }
  }
  n_trial_angle++;
  n_learning++;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::gngTrain(
    const std::vector<T_angle> &samples, int max_iter) {
  // 保持された外部参照による呼出し間の座標変更の反映。
  invalidate_nearest_indexes();
  if (samples.empty())
    return;
  int iters = (max_iter == -1) ? (int)samples.size() : max_iter;
  for (int i = 0; i < iters; ++i) {
    one_train_update(samples[rand() % samples.size()]);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::gngTrainOnTheFly(int max_iter) {
  invalidate_nearest_indexes();
  if (!kinematic_chain_)
    return;
  
  std::cout << "[GNG] Starting Training on-the-fly (" << max_iter << " iterations)..." << std::endl;
  const auto start_time = std::chrono::steady_clock::now();
  const int progress_interval = std::max(1, max_iter / 20);
  for (int i = 0; i < max_iter; ++i) {
    kinematic_chain_->sampleRandomJointValues(random_joint_buffer_);
    auto &q_truncated = random_angle_buffer_;
    for (int j = 0; j < angle_dimension; ++j) {
      q_truncated(j) = static_cast<typename T_angle::Scalar>(random_joint_buffer_[j]);
    }
    one_train_update(q_truncated);

    if (i % progress_interval == 0 || i == max_iter - 1) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed_sec =
          std::chrono::duration<double>(now - start_time).count();
      const double progress = static_cast<double>(i + 1) / std::max(1, max_iter);
      const double eta_sec = (progress > 1e-9) ? elapsed_sec * (1.0 - progress) / progress : 0.0;
      std::cout << "  Progress: " << (progress * 100.0) << "% ("
                << (i + 1) << "/" << max_iter << "), Nodes: "
                << getActiveIndices().size() << ", elapsed: "
                << elapsed_sec << "s, ETA: " << eta_sec << "s" << std::endl;
    }
  }
  std::cout << std::endl << "[GNG] Training Complete. Final Nodes: " << getActiveIndices().size() << std::endl;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::trainCoordEdgesOnTheFly(int max_iter) {
  trainCoordEdgesOnTheFly(max_iter, 0);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::trainCoordEdgesOnTheFly(
    int max_iter, int coord_layer_index) {
  coord_nearest_indexes_.clear();
  if (!kinematic_chain_)
    return;
  if (coord_layer_index < 0 ||
      coord_layer_index >= static_cast<int>(coord_layer_count_)) {
    return;
  }

  std::cout << "[GNG] Training Coordinate Edges On-the-fly (layer "
            << coord_layer_index << ")..." << std::endl;
  const auto start_time = std::chrono::steady_clock::now();
  const int progress_interval = std::max(1, max_iter / 20);
  for (int i = 0; i < max_iter; ++i) {
    kinematic_chain_->sampleRandomJointValues(random_joint_buffer_);
    auto &q_truncated = random_angle_buffer_;
    for (int j = 0; j < angle_dimension; ++j) {
      q_truncated(j) = static_cast<typename T_angle::Scalar>(random_joint_buffer_[j]);
    }
    T_coord s_coord = calculateFK(q_truncated, coord_layer_index);

    find_nearest_coord(s_coord, coord_layer_index);
    const int s1 = candidates_buffer_.empty() ? -1 : candidates_buffer_[0].second;
    const int s2 = candidates_buffer_.size() < 2 ? -1 : candidates_buffer_[1].second;

    if (s1 == -1)
      continue;

    if (s2 != -1) {
      add_edge_coord(coord_layer_index, s1, s2);
    }

    const std::vector<int> &neighbors =
        getNeighborsCoord(s1, coord_layer_index);
    std::vector<int> to_remove;
    for (int nid : neighbors) {
      auto &edges =
          (coord_layer_index == 0) ? edges_coord : edges_coord_per_layer_[coord_layer_index];
      edges[s1][nid].age++;
      edges[nid][s1].age++;
      if (edges[s1][nid].age > params_.max_edge_age) {
        to_remove.push_back(nid);
      }
    }
    for (int nid : to_remove) {
      remove_edge_coord(coord_layer_index, s1, nid);
    }

    if (i % progress_interval == 0 || i == max_iter - 1) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed_sec =
          std::chrono::duration<double>(now - start_time).count();
      const double progress = static_cast<double>(i + 1) / std::max(1, max_iter);
      const double eta_sec = (progress > 1e-9) ? elapsed_sec * (1.0 - progress) / progress : 0.0;
      std::cout << "  Coord edge Progress [" << coord_layer_index << "]: "
                << (progress * 100.0) << "% (" << (i + 1) << "/" << max_iter
                << "), elapsed: " << elapsed_sec << "s, ETA: " << eta_sec
                << "s" << std::endl;
    }
  }
  std::cout << std::endl << "[GNG] Coordinate Edge Training Complete."
            << std::endl;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::trainCoordEdges(
    const std::vector<T_angle> &angle_samples, int max_iter) {
  coord_nearest_indexes_.clear();
  if (angle_samples.empty())
    return;

  std::cout << "[GNG] Training Coordinate Edges (Graph Refinement)..."
            << std::endl;
  int iters = (max_iter == -1) ? (int)angle_samples.size() : max_iter;

  for (int i = 0; i < iters; ++i) {
    // 1. Pick a sample and compute its FK
    const T_angle &s_angle = angle_samples[rand() % angle_samples.size()];
    T_coord s_coord = calculateFK(s_angle);

    // TCP座標の厳密2近傍。
    find_nearest_coord(s_coord, 0);
    const int s1 = candidates_buffer_.empty() ? -1 : candidates_buffer_[0].second;
    const int s2 = candidates_buffer_.size() < 2 ? -1 : candidates_buffer_[1].second;

    if (s1 == -1)
      continue;

    // 3. Update edges (Connect s1-s2, Aging s1's neighbors)
    if (s2 != -1) {
      add_edge_coord(s1, s2);
    }

    const std::vector<int> &neighbors = edges_coord_per_node[s1];
    std::vector<int> to_remove;
    for (int nid : neighbors) {
      edges_coord[s1][nid].age++;
      edges_coord[nid][s1].age++;
      if (edges_coord[s1][nid].age > params_.max_edge_age) {
        to_remove.push_back(nid);
      }
    }
    for (int nid : to_remove) {
      remove_edge_coord(s1, nid);
    }

    // NO node movement, NO node addition/removal.
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::invalidate_collision_cache() {
  filter_poses_.clear();
  filter_safe_edges_.clear();
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::prepare_filter_collision_cache() {
  filter_poses_.resize(nodes.size());
  for (std::size_t idx = 0; idx < nodes.size(); ++idx) {
    auto &record = filter_poses_[idx];
    if (nodes[idx].id == -1) {
      record.has_angles = false;
      record.is_safe = false;
    } else if (!record.has_angles ||
               !has_same_angles(record.angles, nodes[idx].weight_angle)) {
      record.angles = nodes[idx].weight_angle;
      ++record.generation;
      record.has_angles = true;
      record.is_safe = false;
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::strictFilter() {
  if (!collision_checker_)
    return;
  std::cout << "[StrictFilter] Starting efficient self-collision filtering..."
            << std::endl;

  const bool enable_cache = params_.enable_static_collision_cache;
  if (enable_cache) prepare_filter_collision_cache();
  else invalidate_collision_cache();
  std::size_t num_reused_nodes = 0;
  std::size_t num_reused_edges = 0;
  std::size_t num_checked_edges = 0;

  // 干渉姿勢と、その全層の接続辺の先行除去。
  int removed_nodes = 0;
  std::unordered_map<std::string, int> collision_pair_counts;
  int detailed_logs = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1) continue;
    if (enable_cache && filter_poses_[i].is_safe) {
      ++num_reused_nodes;
      continue;
    }
    const bool is_colliding = internalCheckColliding(nodes[i].weight_angle);
    if (enable_cache) filter_poses_[i].is_safe = !is_colliding;
    if (is_colliding) {
      const auto colliding_pairs = collectCollisionPairs(collision_checker_);
      for (const auto &pair : colliding_pairs) {
        collision_pair_counts[makePairKey(pair.first, pair.second)]++;
      }
      if (detailed_logs < 5) {
        std::cout << "[StrictFilter] Colliding node idx=" << i
                  << " id=" << nodes[i].id
                  << " pair_count=" << colliding_pairs.size() << std::endl;
        for (std::size_t k = 0;
             k < std::min<std::size_t>(6, colliding_pairs.size()); ++k) {
          std::cout << "  pair: " << colliding_pairs[k].first << " <-> "
                    << colliding_pairs[k].second << std::endl;
        }
        detailed_logs++;
      }
      // 干渉ノードに接続する関節空間辺の先行除去。
      std::vector<int> neighbors(edges_angle_per_node[i].begin(),
                                 edges_angle_per_node[i].end());
      for (int n : neighbors)
        remove_edge_angle(i, n);
      remove_node(i);
      removed_nodes++;
    }
  }

  printTopCollisionPairs(collision_pair_counts);

  // 全層の辺を関節角で検査。同じ端点ペアの結果を層間で共有。
  int removed_edges = 0;
  std::map<std::pair<int, int>, bool> edge_collisions;
  // 現在のグラフに残る非干渉辺だけの保持。過去の辺の無制限蓄積なし。
  std::unordered_map<std::uint64_t, filter_edge_generations> next_safe_edges;
  const auto is_edge_colliding = [&](int first, int second) {
    const auto key = std::minmax(first, second);
    const std::pair<int, int> ids{key.first, key.second};
    const auto found = edge_collisions.find(ids);
    if (found != edge_collisions.end()) return found->second;
    const std::uint64_t edge_key =
        (static_cast<std::uint64_t>(ids.first) << 32) |
        static_cast<std::uint32_t>(ids.second);
    filter_edge_generations generations{};
    if (enable_cache) {
      generations = {filter_poses_[ids.first].generation,
                     filter_poses_[ids.second].generation};
      const auto cached = filter_safe_edges_.find(edge_key);
      if (cached != filter_safe_edges_.end() && cached->second == generations) {
        ++num_reused_edges;
        next_safe_edges.emplace(edge_key, generations);
        edge_collisions.emplace(ids, false);
        return false;
      }
    }
    ++num_checked_edges;
    const auto &first_angles = nodes[first].weight_angle;
    const auto &second_angles = nodes[second].weight_angle;
    const bool is_colliding = enable_cache
        ? simulation::has_joint_segment_collision(
              first_angles, second_angles, 0.025, [&](const T_angle &angles) {
                // 同じ静的条件で先行検査済みの端点。補間角は従来どおりの全件検査。
                if (has_same_angles(angles, first_angles) ||
                    has_same_angles(angles, second_angles)) return false;
                return internalCheckColliding(angles);
              })
        : internalCheckPathColliding(first_angles, second_angles);
    if (enable_cache && !is_colliding) next_safe_edges.emplace(edge_key, generations);
    edge_collisions.emplace(ids, is_colliding);
    return is_colliding;
  };
  for (int idx = 0; idx < static_cast<int>(nodes.size()); ++idx) {
    if (nodes[idx].id == -1) continue;
    const auto angle_neighbors = edges_angle_per_node[idx];
    for (int other : angle_neighbors) {
      if (idx < other && is_edge_colliding(idx, other)) {
        remove_edge_angle(idx, other);
        ++removed_edges;
      }
    }
    for (int layer_idx = 0; layer_idx < coord_layer_count_; ++layer_idx) {
      const auto coord_neighbors = getNeighborsCoord(idx, layer_idx);
      for (int other : coord_neighbors) {
        if (idx < other && is_edge_colliding(idx, other)) {
          remove_edge_coord(layer_idx, idx, other);
          ++removed_edges;
        }
      }
    }
  }
  std::cout << "[StrictFilter] Result: Removed " << removed_nodes
            << " nodes and " << removed_edges << " edges." << std::endl;

  if (enable_cache) {
    filter_safe_edges_ = std::move(next_safe_edges);
    std::cout << "[StrictFilter] Cache: reused_nodes=" << num_reused_nodes
              << " reused_edges=" << num_reused_edges
              << " checked_edges=" << num_checked_edges << std::endl;
  }

  // 有効ノード一覧の再構成。
  active_indices_.clear();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && nodes[i].status.active) {
      active_indices_.push_back(i);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::removeInactiveElements() {
  std::cout << "[Cleanup] Removing inactive elements..." << std::endl;
  int removed_edges = 0;
  int removed_nodes = 0;

  // 1. Remove Inactive Edges (Angle Space)
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
    if (i < (int)edges_angle_per_node.size()) {
      const std::vector<int> &neighbors = edges_angle_per_node[i];
      // Work on a copy because remove_edge_angle modifies the vector
      std::vector<int> neighbors_copy = neighbors; 
      for (int n : neighbors_copy) {
        if (i < n) {
          if (!edges_angle[i][n].active) {
            remove_edge_angle(i, n);
            removed_edges++;
          }
        }
      }
    }
    }
  }

  // 2. Remove Inactive Nodes
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      if (!nodes[i].status.active) {
        remove_node(i);
        removed_nodes++;
      }
    }
  }

  std::cout << "[Cleanup] Removed " << removed_nodes << " nodes and "
            << removed_edges << " edges." << std::endl;

  // Rebuild active_indices_
  active_indices_.clear();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && nodes[i].status.active) {
      active_indices_.push_back(i);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::pruneToLargestComponent() {
  std::cout << "[IslandPruning] Finding largest connected component..."
            << std::endl;

  std::vector<bool> visited(nodes.size(), false);
  std::vector<std::vector<int>> components;

  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && nodes[i].status.active && !visited[i]) {
      // Start new BFS
      std::vector<int> current_component;
      std::queue<int> q;
      q.push(i);
      visited[i] = true;

      while (!q.empty()) {
        int u = q.front();
        q.pop();
        current_component.push_back(u);

        for (int v : edges_angle_per_node[u]) {
          if (nodes[v].id != -1 && nodes[v].status.active && !visited[v]) {
            visited[v] = true;
            q.push(v);
          }
        }
      }
      components.push_back(current_component);
    }
  }

  if (components.empty()) {
    std::cout << "[IslandPruning] No active nodes found." << std::endl;
    return;
  }

  // Find largest
  size_t largest_idx = 0;
  for (size_t i = 1; i < components.size(); ++i) {
    if (components[i].size() > components[largest_idx].size()) {
      largest_idx = i;
    }
  }

  std::cout << "[IslandPruning] Found " << components.size() << " components."
            << std::endl;
  std::cout << "[IslandPruning] Largest component size: "
            << components[largest_idx].size() << " nodes." << std::endl;

  // Deactivate all nodes NOT in the largest component
  std::vector<bool> keep(nodes.size(), false);
  for (int nid : components[largest_idx]) {
    keep[nid] = true;
  }

  int deactivated_nodes = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && nodes[i].status.active && !keep[i]) {
      nodes[i].status.active = false;
      deactivated_nodes++;

      // Also deactivate edges connected to this node
      for (int v : edges_angle_per_node[i]) {
        edges_angle[i][v].active = false;
        edges_angle[v][i].active = false;
      }
    }
  }

  std::cout << "[IslandPruning] Deactivated " << deactivated_nodes
            << " nodes belonging to smaller islands." << std::endl;

  // Rebuild active_indices_
  active_indices_.clear();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && nodes[i].status.active) {
      active_indices_.push_back(i);
    }
  }
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::save(const std::string &filename) {
  // Always refresh coordinates to match current Kinematic Model (EEF Tip)
  // before saving.
  refresh_coord_weights();

  std::string resolved_path = robot_sim::common::resolvePath(filename);
  std::ofstream ofs(resolved_path, std::ios::binary);
  if (!ofs)
    return false;
  uint32_t version = 9; // Version 9: Removed one obsolete status float
  ofs.write((char *)&version, sizeof(version));
  ofs.write((char *)&coord_layer_count_, sizeof(coord_layer_count_));

  int node_count = 0;
  for (int i = 0; i < (int)nodes.size(); ++i)
    if (nodes[i].id != -1)
      node_count++;
  ofs.write((char *)&node_count, sizeof(node_count));

  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      ofs.write((char *)&nodes[i].id, sizeof(int));
      ofs.write((char *)&nodes[i].error_angle, sizeof(float));
      float error_coord_dummy = 0.0f;
      ofs.write((char *)&error_coord_dummy, sizeof(float));

      GrowingNeuralGas_Internal::write_eigen(ofs, nodes[i].weight_angle);
      GrowingNeuralGas_Internal::write_eigen(ofs, nodes[i].weight_coord);
      if (version >= 6) {
        int coord_count = static_cast<int>(nodes[i].weight_coords.size());
        ofs.write((char *)&coord_count, sizeof(coord_count));
        for (const auto &coord : nodes[i].weight_coords) {
          GrowingNeuralGas_Internal::write_eigen(ofs, coord);
        }
      }

      // Status fields
      ofs.write((char *)&nodes[i].status.level, sizeof(int));
      ofs.write((char *)&nodes[i].status.is_surface, sizeof(bool));
      bool is_active_surface = nodes[i].status.is_active_surface;
      ofs.write((char *)&is_active_surface, sizeof(bool));
      bool self_collision_free = nodes[i].status.self_collision_free;
      ofs.write((char *)&self_collision_free, sizeof(bool));
      bool active = nodes[i].status.active;
      ofs.write((char *)&active, sizeof(bool));
      bool is_boundary = nodes[i].status.is_boundary;
      ofs.write((char *)&is_boundary, sizeof(bool));

      GrowingNeuralGas_Internal::write_eigen(ofs,
                                              nodes[i].status.ee_direction);

      // ManipulabilityInfo (Version 4+)
      ofs.write((char *)&nodes[i].status.manip_info.manipulability,
                sizeof(float));
      ofs.write((char *)&nodes[i].status.min_singular_value, sizeof(float));
      ofs.write((char *)&nodes[i].status.joint_limit_score, sizeof(float));
      ofs.write((char *)&nodes[i].status.manip_info.valid, sizeof(bool));
      ofs.write((char *)&nodes[i].status.dynamic_manipulability, sizeof(float));

      // Rotational manipulability (Version 8+)
      ofs.write((char *)&nodes[i].status.rotational_manip_info.manipulability, sizeof(float));
      ofs.write((char *)&nodes[i].status.rotational_manip_info.valid, sizeof(bool));

      // joint_positions is removed in version 7
    }
  }

  // Angle edges
  int edge_count = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      for (const auto &pair : edges_angle[i]) {
        if (i < pair.first)
          edge_count++;
      }
    }
  }
  ofs.write((char *)&edge_count, sizeof(edge_count));
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      for (const auto &pair : edges_angle[i]) {
        int n = pair.first;
        if (i < n) {
          ofs.write((char *)&i, sizeof(int));
          ofs.write((char *)&n, sizeof(int));
          ofs.write((char *)&pair.second.age, sizeof(int));
          bool e_active = pair.second.active;
          ofs.write((char *)&e_active, sizeof(bool));
        }
      }
    }
  }

  if (version >= 6) {
    for (int layer = 0; layer < coord_layer_count_; ++layer) {
      const auto &edges =
          (layer == 0) ? edges_coord : edges_coord_per_layer_[layer];
      int coord_edge_count = 0;
      for (int i = 0; i < (int)nodes.size(); ++i) {
        if (nodes[i].id != -1) {
          for (const auto &pair : edges[i]) {
            if (i < pair.first)
              coord_edge_count++;
          }
        }
      }
      ofs.write((char *)&coord_edge_count, sizeof(coord_edge_count));
      for (int i = 0; i < (int)nodes.size(); ++i) {
        if (nodes[i].id != -1) {
          for (const auto &pair : edges[i]) {
            int n = pair.first;
            if (i < n) {
              ofs.write((char *)&i, sizeof(int));
              ofs.write((char *)&n, sizeof(int));
              ofs.write((char *)&pair.second.age, sizeof(int));
              bool e_active = pair.second.active;
              ofs.write((char *)&e_active, sizeof(bool));
            }
          }
        }
      }
    }
  } else {
    // Coord edges
    int coord_edge_count = 0;
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (nodes[i].id != -1) {
        for (const auto &pair : edges_coord[i]) {
          if (i < pair.first)
            coord_edge_count++;
        }
      }
    }
    ofs.write((char *)&coord_edge_count, sizeof(coord_edge_count));
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (nodes[i].id != -1) {
        for (const auto &pair : edges_coord[i]) {
          int n = pair.first;
          if (i < n) {
            ofs.write((char *)&i, sizeof(int));
            ofs.write((char *)&n, sizeof(int));
            ofs.write((char *)&pair.second.age, sizeof(int));
            bool e_active = pair.second.active;
            ofs.write((char *)&e_active, sizeof(bool));
          }
        }
      }
    }
  }

  return true;
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::load(const std::string &filename) {
  static constexpr std::size_t kMaxLoadableNodes = 1000000;
  std::string resolved_path = robot_sim::common::resolvePath(filename);
  std::ifstream ifs(resolved_path, std::ios::binary);
  if (!ifs)
    return false;

  // 既存ノードと索引・静的判定結果の破棄。
  invalidate_collision_cache();
  invalidate_nearest_indexes();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    nodes[i] = NeuronNode<T_angle, T_coord>();
    nodes[i].id = -1;
    edges_angle_per_node[i].clear();
    edges_coord_per_node[i].clear();
    edges_angle[i].clear();
    edges_coord[i].clear();
  }

  uint32_t version;
  ifs.read((char *)&version, sizeof(version));
  if (version != 1 && version != 2 && version != 3 && version != 4 &&
      version != 5 && version != 6 && version != 7 && version != 8 &&
      version != 9) {
    std::cerr << "Error: Unsupported GNG file version: " << version
              << " (hex: 0x" << std::hex << version << std::dec << ")" << std::endl;
    return false;
  }

  if (version >= 6) {
    ifs.read((char *)&coord_layer_count_, sizeof(coord_layer_count_));
    coord_layer_count_ = std::max(1, coord_layer_count_);
  } else {
    coord_layer_count_ = 1;
  }
  setCoordLayerCount(coord_layer_count_);

  int node_count = 0;
  ifs.read((char *)&node_count, sizeof(node_count));
  if (!ifs || node_count < 0 ||
      static_cast<std::size_t>(node_count) > kMaxLoadableNodes) {
    std::cerr << "Error: Invalid GNG node count: " << node_count << std::endl;
    return false;
  }

  const auto ensure_node_capacity = [this](std::size_t required) {
    if (required <= nodes.size()) {
      return true;
    }
    if (required > kMaxLoadableNodes) {
      return false;
    }

    const std::size_t old_size = nodes.size();
    const std::size_t doubled = std::max<std::size_t>(1, old_size * 2);
    const std::size_t new_size =
        std::min(kMaxLoadableNodes, std::max(required, doubled));
    nodes.resize(new_size);
    edges_angle.resize(new_size);
    edges_coord.resize(new_size);
    edges_angle_per_node.resize(new_size);
    edges_coord_per_node.resize(new_size);
    for (auto &layer_edges : edges_coord_per_layer_) {
      layer_edges.resize(new_size);
    }
    for (auto &layer_neighbors : edges_coord_per_layer_nodes_) {
      layer_neighbors.resize(new_size);
    }
    for (std::size_t i = old_size; i < new_size; ++i) {
      nodes[i].id = -1;
      nodes[i].status.active = false;
    }
    return true;
  };

  std::size_t loaded_node_index_limit = 0;
  for (int k = 0; k < node_count; ++k) {
    int id;
    ifs.read((char *)&id, sizeof(int));
    if (!ifs || id < 0 ||
        !ensure_node_capacity(static_cast<std::size_t>(id) + 1)) {
      std::cerr << "Error: Invalid GNG node id: " << id << std::endl;
      return false;
    }
    loaded_node_index_limit = std::max(
        loaded_node_index_limit, static_cast<std::size_t>(id) + 1);
    nodes[id].id = id;
    ifs.read((char *)&nodes[id].error_angle, sizeof(float));
    float dummy;
    ifs.read((char *)&dummy, sizeof(float)); // error_coord

    GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].weight_angle);
    GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].weight_coord);
    nodes[id].weight_coords.clear();
    if (version >= 6) {
      int coord_count = 0;
      ifs.read((char *)&coord_count, sizeof(coord_count));
      coord_count = std::max(0, coord_count);
      nodes[id].weight_coords.resize(coord_count);
      for (int c = 0; c < coord_count; ++c) {
        GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].weight_coords[c]);
      }
      if (!nodes[id].weight_coords.empty()) {
        nodes[id].weight_coord = nodes[id].weight_coords.front();
      }
    } else {
      nodes[id].weight_coords.push_back(nodes[id].weight_coord);
    }

    // Status
    ifs.read((char *)&nodes[id].status.level, sizeof(int));
    bool b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.is_surface = b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.is_active_surface = b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.self_collision_free = b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.active = b_tmp;
    if (version >= 5) {
      ifs.read((char *)&b_tmp, sizeof(bool));
      nodes[id].status.is_boundary = b_tmp;
    }

    GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].status.ee_direction);

    if (version >= 4) {
      ifs.read((char *)&nodes[id].status.manip_info.manipulability,
               sizeof(float));
      ifs.read((char *)&nodes[id].status.min_singular_value, sizeof(float));
      ifs.read((char *)&nodes[id].status.joint_limit_score, sizeof(float));
      if (version <= 8) {
        float discarded_legacy_score = 0.0f;
        ifs.read((char *)&discarded_legacy_score, sizeof(float));
      }
      ifs.read((char *)&nodes[id].status.manip_info.valid, sizeof(bool));
      ifs.read((char *)&nodes[id].status.dynamic_manipulability, sizeof(float));
    }

    if (version >= 8) {
      ifs.read((char *)&nodes[id].status.rotational_manip_info.manipulability, sizeof(float));
      ifs.read((char *)&nodes[id].status.rotational_manip_info.valid, sizeof(bool));
    } else {
      nodes[id].status.rotational_manip_info.valid = false;
      nodes[id].status.rotational_manip_info.manipulability = 0.0;
    }

    if (version < 7) {
      int jp_size = 0;
      ifs.read((char *)&jp_size, sizeof(int));
      if (jp_size > 0) {
        for (int i = 0; i < jp_size; ++i) {
          Eigen::Vector3f dummy_vec;
          GrowingNeuralGas_Internal::read_eigen(ifs, dummy_vec);
        }
      }
    }
  }

  // Edges Angle
  int edge_count = 0;
  ifs.read((char *)&edge_count, sizeof(edge_count));
  for (int k = 0; k < edge_count; ++k) {
    int n1, n2, age;
    ifs.read((char *)&n1, sizeof(int));
    ifs.read((char *)&n2, sizeof(int));
    ifs.read((char *)&age, sizeof(int));

    bool active = true;
    if (version >= 3) {
      ifs.read((char *)&active, sizeof(bool));
    }

    if (n1 >= 0 && (size_t)n1 < nodes.size() && n2 >= 0 &&
        (size_t)n2 < nodes.size() && nodes[n1].id >= 0 && nodes[n2].id >= 0) {
      add_edge_angle(n1, n2);
      edges_angle[n1][n2].age = age;
      edges_angle[n2][n1].age = age;
      edges_angle[n1][n2].active = active;
      edges_angle[n2][n1].active = active;
    }
  }

  if (version >= 6) {
    for (int layer = 0; layer < coord_layer_count_; ++layer) {
      int coord_edge_count = 0;
      ifs.read((char *)&coord_edge_count, sizeof(coord_edge_count));
      for (int k = 0; k < coord_edge_count; ++k) {
        int n1, n2, age;
        ifs.read((char *)&n1, sizeof(int));
        ifs.read((char *)&n2, sizeof(int));
        ifs.read((char *)&age, sizeof(int));

        bool active = true;
        if (version >= 3) {
          ifs.read((char *)&active, sizeof(bool));
        }

        if (n1 >= 0 && (size_t)n1 < nodes.size() && n2 >= 0 &&
            (size_t)n2 < nodes.size() && nodes[n1].id >= 0 && nodes[n2].id >= 0) {
          add_edge_coord(layer, n1, n2);
          auto &edges =
              (layer == 0) ? edges_coord : edges_coord_per_layer_[layer];
          edges[n1][n2].age = age;
          edges[n2][n1].age = age;
          edges[n1][n2].active = active;
          edges[n2][n1].active = active;
        }
      }
    }
  } else {
    // Edges Coord
    int coord_edge_count = 0;
    ifs.read((char *)&coord_edge_count, sizeof(coord_edge_count));
    for (int k = 0; k < coord_edge_count; ++k) {
      int n1, n2, age;
      ifs.read((char *)&n1, sizeof(int));
      ifs.read((char *)&n2, sizeof(int));
      ifs.read((char *)&age, sizeof(int));

      bool active = true;
      if (version >= 3) {
        ifs.read((char *)&active, sizeof(bool));
      }

      if (n1 >= 0 && (size_t)n1 < nodes.size() && n2 >= 0 &&
          (size_t)n2 < nodes.size() && nodes[n1].id >= 0 && nodes[n2].id >= 0) {
        add_edge_coord(n1, n2);
        edges_coord[n1][n2].age = age;
        edges_coord[n2][n1].age = age;
        edges_coord[n1][n2].active = active;
        edges_coord[n2][n1].active = active;
      }
    }
  }

  if (loaded_node_index_limit > 0 &&
      loaded_node_index_limit < nodes.size()) {
    nodes.resize(loaded_node_index_limit);
    edges_angle.resize(loaded_node_index_limit);
    edges_coord.resize(loaded_node_index_limit);
    edges_angle_per_node.resize(loaded_node_index_limit);
    edges_coord_per_node.resize(loaded_node_index_limit);
    for (auto &layer_edges : edges_coord_per_layer_) {
      layer_edges.resize(loaded_node_index_limit);
    }
    for (auto &layer_neighbors : edges_coord_per_layer_nodes_) {
      layer_neighbors.resize(loaded_node_index_limit);
    }
  }

  // Make addable indices correct and rebuild active_indices_
  while (!addable_node_indicies.empty())
    addable_node_indicies.pop();
  active_indices_.clear();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1) {
      addable_node_indicies.push(i);
    } else {
      active_indices_.push_back(i);
    }
  }

  // 読み込み後に情報の補完を行う（V5以前のファイル対応）
  refresh_coord_weights();

  return true;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::setParams(
    const GngParameters &params) {
  invalidate_collision_cache();
  invalidate_nearest_indexes();
  bool resize_needed = (params.max_node_num != params_.max_node_num);
  params_ = params;

  if (resize_needed) {
    nodes.assign(params_.max_node_num, NeuronNode<T_angle, T_coord>());
    active_indices_.clear();
    while (!addable_node_indicies.empty())
      addable_node_indicies.pop();
    for (int i = 0; i < (int)nodes.size(); ++i) {
      nodes[i].id = -1;
      nodes[i].status.active = false;
      addable_node_indicies.push(i);
    }
    edges_angle.resize(nodes.size());
    edges_coord.resize(nodes.size());
    for (int i = 0; i < (int)nodes.size(); ++i) {
      edges_angle[i].clear();
      edges_coord[i].clear();
    }
    edges_angle_per_node.assign(nodes.size(), std::vector<int>());
    edges_coord_per_node.assign(nodes.size(), std::vector<int>());
    setCoordLayerCount(coord_layer_count_);
    n_learning = 0;
    n_trial_angle = 0;
    task_error_ema_initialized_ = false;
    task_error_ema_ = 0.0f;

    // 初期ノードの再追加
    for (int i = 0; i < params_.start_node_num; ++i) {
      T_angle wa;
      if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
        wa.setZero(angle_dimension);
      else
        wa.setZero();
      kinematic_chain_->sampleRandomJointValues(random_joint_buffer_);
      for (int j = 0; j < angle_dimension; ++j) {
        wa(j) = static_cast<typename T_angle::Scalar>(random_joint_buffer_[j]);
      }
      add_node(wa);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::triggerBatchUpdates() {
  // Keep cached spatial coordinates in sync with the latest learned joint
  // weights before any downstream consumers read node.position-like fields.
  refresh_coord_weights();
  forEachActive([&](int i, const auto & /*node*/) {
    runStatusProviders(i, UpdateTrigger::BATCH_UPDATE);
  });
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::triggerPeriodicUpdates() {
  forEachActive([&](int i, const auto & /*node*/) {
    runStatusProviders(i, UpdateTrigger::TIME_PERIODIC);
  });
}

template class GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
template class GNG::GrowingNeuralGas<Eigen::Vector3f, Eigen::Vector3f>;
