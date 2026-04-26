#include "GrowingNeuralGas.hpp"
#include "../analysis/iself_collision_checker.hpp"
#include "common/resource_utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

using namespace GNG;

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

  // Standalone analysis / ROS 2 safety nodes may construct GNG without a
  // kinematic chain. In that case, skip the seed-node bootstrap here; the
  // model will be populated later by load().
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

    bool found = false;
    for (int retry = 0; retry < 100; ++retry) {
      std::vector<double> q_vec = kinematic_chain_->sampleRandomJointValues();
      for (int j = 0; j < angle_dimension; ++j) {
        wa(j) = static_cast<typename T_angle::Scalar>(q_vec[j]);
      }
      if (!internalCheckColliding(wa)) {
        found = true;
        break;
      }
    }
    // If no valid node found in 100 tries, just use the last one (it will likely be pruned later)
    add_node(wa, calculateFK(wa));
  }
}

template <typename T_angle, typename T_coord>
GrowingNeuralGas<T_angle, T_coord>::~GrowingNeuralGas() {}

template <typename T_angle, typename T_coord>
T_coord
GrowingNeuralGas<T_angle, T_coord>::calculateFK(const T_angle &angle_values) {
  if (!kinematic_chain_) {
    if constexpr (T_coord::RowsAtCompileTime == Eigen::Dynamic)
      return T_coord::Zero(coord_dimension);
    else
      return T_coord::Zero();
  }
  int dof = kinematic_chain_->getTotalDOF();
  kinematic_chain_->updateKinematics(
      angle_values.head(std::min((int)angle_values.size(), dof)));
  // エンドエフェクタ先端の位置を取得
  Eigen::Vector3d eef_position_double = kinematic_chain_->getEEFPosition();
  return eef_position_double.cast<typename T_coord::Scalar>();
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::runStatusProviders(
    int node_id, UpdateTrigger trigger) {
  if (node_id < 0 || (size_t)node_id >= nodes.size() || nodes[node_id].id == -1)
    return;
  auto &node = nodes[node_id];
  for (auto &provider : providers_) {
    auto triggers = provider->getTriggers();
    if (std::find(triggers.begin(), triggers.end(), trigger) !=
        triggers.end()) {
      if (provider->shouldUpdate(node, trigger)) {
        provider->update(node, trigger);
      }
    }
  }
}

template <typename T_angle, typename T_coord>
int GrowingNeuralGas<T_angle, T_coord>::add_node(T_angle w_angle,
                                                  T_coord w_coord) {
  (void)w_coord;
  if (addable_node_indicies.empty())
    return -1;
  int node_id = addable_node_indicies.front();
  addable_node_indicies.pop();
  T_coord calculated_w_coord = calculateFK(w_angle);
  nodes[node_id] =
      NeuronNode<T_angle, T_coord>(node_id, w_angle, calculated_w_coord);
  edges_angle[node_id].clear();
  edges_coord[node_id].clear();
  edges_angle_per_node[node_id].clear();
  edges_coord_per_node[node_id].clear();
  active_indices_.push_back(node_id);
  runStatusProviders(node_id, UpdateTrigger::NODE_ADDED);
  return node_id;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_node(int node) {
  if (node < 0 || (size_t)node >= nodes.size() || nodes[node].id == -1)
    return;

  if (!edges_coord_per_node[node].empty()) {
    std::vector<int> coord_neighbors(edges_coord_per_node[node].begin(),
                                     edges_coord_per_node[node].end());
    for (int n : coord_neighbors)
      remove_edge_coord(node, n);
  }

  active_indices_.erase(
      std::remove(active_indices_.begin(), active_indices_.end(), node),
      active_indices_.end());

  edges_angle[node].clear();
  edges_coord[node].clear();
  edges_angle_per_node[node].clear();
  edges_coord_per_node[node].clear();
  nodes[node].id = -1;
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
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  if (edges_coord[node_1].count(node_2)) {
    edges_coord[node_1][node_2].age = 1;
    edges_coord[node_2][node_1].age = 1;
  } else {
    edges_coord_per_node[node_1].push_back(node_2);
    edges_coord_per_node[node_2].push_back(node_1);
    edges_coord[node_1][node_2].age = 1;
    edges_coord[node_2][node_1].age = 1;
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_edge_coord(int node_1,
                                                            int node_2) {
  if (node_1 < 0 || (size_t)node_1 >= nodes.size() || nodes[node_1].id == -1 ||
      node_2 < 0 || (size_t)node_2 >= nodes.size() || nodes[node_2].id == -1)
    return;
  auto &v1c = edges_coord_per_node[node_1];
  v1c.erase(std::remove(v1c.begin(), v1c.end(), node_2), v1c.end());
  auto &v2c = edges_coord_per_node[node_2];
  v2c.erase(std::remove(v2c.begin(), v2c.end(), node_1), v2c.end());

  edges_coord[node_1].erase(node_2);
  edges_coord[node_2].erase(node_1);
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

  // NOTE: skip weight_coord update (FK) during training for efficiency.
  // It should be refreshed by calling refresh_coord_weights() before saving.
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::refresh_coord_weights() {
  if (!kinematic_chain_) return;
  for (int i : active_indices_) {
    auto &node = nodes[i];
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts;
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> oris;
    
    // 全リンクのFK計算
    kinematic_chain_->forwardKinematicsAt(node.weight_angle, pts, oris);
    
    if (!pts.empty() && !oris.empty()) {
        // 位置の更新
        node.weight_coord = pts.back().template cast<typename T_coord::Scalar>();
        // 方向と姿勢の更新 (追加情報の補完)
        node.status.ee_direction = (oris.back() * Eigen::Vector3d::UnitX()).template cast<float>();
        node.status.ee_orientation = oris.back().template cast<float>();
        
        // 全関節位置も保存 (providers.hpp と同様の整合性維持)
        node.status.joint_positions.clear();
        for (const auto &p : pts) {
          node.status.joint_positions.push_back(p.template cast<float>());
        }
    }
  }
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

  // 1. リプシッツ性に基づいた早期スキップ検討
  // $\|q2 - q1\|$ が極めて小さければ、ノードの安全マージンの範囲内に収まるため
  // 再帰を回さずに通過させて良い（ノード自体の安全は前提）。
  float dist_q = (q1 - q2).norm();
  if (dist_q < 0.001f)
    return false; // 約0.05度以下の微小区間はスキップ

  // 2. 再帰的な二分探索 (Bisection) 実行用のラムダ
  // Reduced depth from 6 (64 segments) to 3 (8 segments) for massive speedup in Step 3
  std::function<bool(const T_angle &, const T_angle &, int)> check_recursive;
  check_recursive = [&](const T_angle &a, const T_angle &b, int depth) -> bool {
    if (depth >= 3)
      return false;

    T_angle mid = (a + b) * 0.5f;
    if (internalCheckColliding(mid))
      return true; // 発見次第、即座に早期リターン

    // 左右の区間を再帰的にチェック
    return check_recursive(a, mid, depth + 1) ||
           check_recursive(mid, b, depth + 1);
  };

  return check_recursive(q1, q2, 0);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::one_train_update(
    const T_angle &sample_angle) {
  // 1. Find N-best candidates
  candidates_buffer_.clear();
  candidates_buffer_.reserve(nodes.size());

  // 定期的な一括誤差減衰に切り替え (O(I*N) から O(I*N/freq) に削減)
  const int DECAY_FREQ = 100;
  accumulated_decay_factor_ *= (1.0f - params_.beta);
  decay_step_count_++;

  bool should_apply_global_decay = (decay_step_count_ >= DECAY_FREQ);

  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1)
      continue;
    
    // 周期的な減衰の適用
    if (should_apply_global_decay) {
      nodes[i].error_angle *= accumulated_decay_factor_;
    }

    float d2 = calc_squaredNorm_angle(sample_angle, nodes[i].weight_angle);
    candidates_buffer_.push_back({d2, i});
  }

  if (should_apply_global_decay) {
    accumulated_decay_factor_ = 1.0f;
    decay_step_count_ = 0;
  }

  if (candidates_buffer_.size() < 2)
    return;

  int n_search = std::min((int)candidates_buffer_.size(), params_.n_best_candidates);
  std::partial_sort(candidates_buffer_.begin(), candidates_buffer_.begin() + n_search,
                    candidates_buffer_.end());

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
      int new_id = add_node(sample_angle, calculateFK(sample_angle));
      if (new_id != -1)
        add_edge_angle(new_id, s1);
      n_trial_angle++;
      n_learning++;
      return;
    }
  }

  nodes[s1].error_angle += dist;
  update_node_weights(s1, sample_angle, params_.learn_rate_s1);

  if (s2 != -1) {
    // エッジ生成時の干渉チェック
    if (!collision_aware_ ||
        !internalCheckPathColliding(nodes[s1].weight_angle,
                                    nodes[s2].weight_angle)) {
      add_edge_angle(s1, s2);
    } else if (collision_aware_) {
      // 衝突している場合はリフレッシュせず、既存エッジがあれば即座に削除（合理的）
      remove_edge_angle(s1, s2);
    }
  }

  // 周辺ノードの更新とエイジング
  const std::vector<int> &neighbors = edges_angle_per_node[s1];
  std::vector<int> to_remove;
  for (int nid : neighbors) {
    if (edges_angle[s1][nid].age > params_.max_edge_age) {
      to_remove.push_back(nid);
    } else {
      update_node_weights(nid, sample_angle, params_.learn_rate_s2);
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
        int new_id = add_node(mid, T_coord());
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
  if (samples.empty())
    return;
  int iters = (max_iter == -1) ? (int)samples.size() : max_iter;
  for (int i = 0; i < iters; ++i) {
    one_train_update(samples[rand() % samples.size()]);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::gngTrainOnTheFly(int max_iter) {
  if (!kinematic_chain_)
    return;
  
  std::cout << "[GNG] Starting Training on-the-fly (" << max_iter << " iterations)..." << std::endl;
  for (int i = 0; i < max_iter; ++i) {
    std::vector<double> q_vec = kinematic_chain_->sampleRandomJointValues();
    T_angle q_truncated;
    if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
      q_truncated.resize(angle_dimension);
    for (int j = 0; j < angle_dimension; ++j) {
      q_truncated(j) = static_cast<typename T_angle::Scalar>(q_vec[j]);
    }
    one_train_update(q_truncated);

    if (i % 1000 == 0 || i == max_iter - 1) {
      std::cout << "  Progress: " << (i * 100 / max_iter) << "% (" << i << "/" << max_iter 
                << "), Nodes: " << getActiveIndices().size() << "\r" << std::flush;
    }
  }
  std::cout << std::endl << "[GNG] Training Complete. Final Nodes: " << getActiveIndices().size() << std::endl;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::trainCoordEdgesOnTheFly(int max_iter) {
  if (!kinematic_chain_)
    return;

  std::cout << "[GNG] Training Coordinate Edges On-the-fly..." << std::endl;
  for (int i = 0; i < max_iter; ++i) {
    // 1. Generate sample and FK
    std::vector<double> q_vec = kinematic_chain_->sampleRandomJointValues();
    T_angle q_truncated;
    if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
      q_truncated.resize(angle_dimension);
    for (int j = 0; j < angle_dimension; ++j) {
      q_truncated(j) = static_cast<typename T_angle::Scalar>(q_vec[j]);
    }
    T_coord s_coord = calculateFK(q_truncated);

    // 2. Find 2 nearest nodes in COORD space
    int s1 = -1, s2 = -1;
    float d1 = std::numeric_limits<float>::max();
    float d2 = std::numeric_limits<float>::max();

    for (int n = 0; n < (int)nodes.size(); ++n) {
      if (nodes[n].id != -1) {
        float d = calc_squaredNorm_coord(s_coord, nodes[n].weight_coord);
        if (d < d1) {
          d2 = d1;
          s2 = s1;
          d1 = d;
          s1 = n;
        } else if (d < d2) {
          d2 = d;
          s2 = n;
        }
      }
    }

    if (s1 == -1)
      continue;

    // 3. Update edges
    if (s2 != -1) {
      add_edge_coord(s1, s2);
    }

    // Neighbors update
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

    if (i % 1000 == 0 || i == max_iter - 1) {
      std::cout << "  Coord edge Progress: " << (i * 100 / max_iter) << "% (" << i << "/" << max_iter << ")\r" << std::flush;
    }
  }
  std::cout << std::endl << "[GNG] Coordinate Edge Training Complete." << std::endl;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::trainCoordEdges(
    const std::vector<T_angle> &angle_samples, int max_iter) {
  if (angle_samples.empty())
    return;

  std::cout << "[GNG] Training Coordinate Edges (Graph Refinement)..."
            << std::endl;
  int iters = (max_iter == -1) ? (int)angle_samples.size() : max_iter;

  for (int i = 0; i < iters; ++i) {
    // 1. Pick a sample and compute its FK
    const T_angle &s_angle = angle_samples[rand() % angle_samples.size()];
    T_coord s_coord = calculateFK(s_angle);

    // 2. Find 2 nearest nodes in COORD space
    int s1 = -1, s2 = -1;
    float d1 = std::numeric_limits<float>::max();
    float d2 = std::numeric_limits<float>::max();

    for (int n = 0; n < (int)nodes.size(); ++n) {
      if (nodes[n].id != -1) {
        float d = calc_squaredNorm_coord(s_coord, nodes[n].weight_coord);
        if (d < d1) {
          d2 = d1;
          s2 = s1;
          d1 = d;
          s1 = n;
        } else if (d < d2) {
          d2 = d;
          s2 = n;
        }
      }
    }

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
void GrowingNeuralGas<T_angle, T_coord>::strictFilter() {
  if (!collision_checker_)
    return;
  std::cout << "[StrictFilter] Starting efficient self-collision filtering..."
            << std::endl;

  // 1. Remove colliding nodes first (and all their associated edges)
  int removed_nodes = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1 && internalCheckColliding(nodes[i].weight_angle)) {
      // Node itself is in collision.
      // Clear all its edges first (removes from edges_angle_per_node and
      // edges_angle)
      std::vector<int> neighbors(edges_angle_per_node[i].begin(),
                                 edges_angle_per_node[i].end());
      for (int n : neighbors)
        remove_edge_angle(i, n);
      remove_node(i);
      removed_nodes++;
    }
  }

    // 2. Remove colliding edges among the remaining valid nodes
    int removed_edges = 0;
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (nodes[i].id == -1)
        continue;
  
      // Neighbors are already a vector
      std::vector<int> neighbors_copy = edges_angle_per_node[i];
      for (int n : neighbors_copy) {
        if (i < n) { // Check each edge only once
          if (internalCheckPathColliding(nodes[i].weight_angle,
                                         nodes[n].weight_angle, 50)) {
            remove_edge_angle(i, n);
            removed_edges++;
          }
        }
      }
    }
  std::cout << "[StrictFilter] Result: Removed " << removed_nodes
            << " nodes and " << removed_edges << " edges." << std::endl;

  // Rebuild active_indices_
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
  uint32_t version = 5; // Version 5: Added is_boundary flag
  ofs.write((char *)&version, sizeof(version));

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

      // Status fields
      ofs.write((char *)&nodes[i].status.level, sizeof(int));
      ofs.write((char *)&nodes[i].status.is_surface, sizeof(bool));
      bool is_active_surface = nodes[i].status.is_active_surface;
      ofs.write((char *)&is_active_surface, sizeof(bool));
      bool valid = nodes[i].status.valid;
      ofs.write((char *)&valid, sizeof(bool));
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
      ofs.write((char *)&nodes[i].status.combined_score, sizeof(float));
      ofs.write((char *)&nodes[i].status.manip_info.valid, sizeof(bool));
      ofs.write((char *)&nodes[i].status.dynamic_manipulability, sizeof(float));

      int jp_size = (int)nodes[i].status.joint_positions.size();
      ofs.write((char *)&jp_size, sizeof(int));
      for (const auto &v : nodes[i].status.joint_positions) {
        GrowingNeuralGas_Internal::write_eigen(ofs, v);
      }
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

  return true;
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::load(const std::string &filename) {
  std::string resolved_path = robot_sim::common::resolvePath(filename);
  std::ifstream ifs(resolved_path, std::ios::binary);
  if (!ifs)
    return false;

  // Clear existing
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
      version != 5 && version != 6) {
    std::cerr << "Error: Unsupported GNG file version: " << version
              << " (hex: 0x" << std::hex << version << std::dec << ")" << std::endl;
    return false;
  }

  int node_count = 0;
  ifs.read((char *)&node_count, sizeof(node_count));

  for (int k = 0; k < node_count; ++k) {
    int id;
    ifs.read((char *)&id, sizeof(int));
    if (id < 0 || (size_t)id >= nodes.size()) {
      // Should ideally skip appropriate bytes, but format is complex.
      // We assume valid IDs in this pipeline.
      continue;
    }
    nodes[id].id = id;
    ifs.read((char *)&nodes[id].error_angle, sizeof(float));
    float dummy;
    ifs.read((char *)&dummy, sizeof(float)); // error_coord

    GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].weight_angle);
    GrowingNeuralGas_Internal::read_eigen(ifs, nodes[id].weight_coord);

    // Status
    ifs.read((char *)&nodes[id].status.level, sizeof(int));
    bool b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.is_surface = b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.is_active_surface = b_tmp;
    ifs.read((char *)&b_tmp, sizeof(bool));
    nodes[id].status.valid = b_tmp;
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
      ifs.read((char *)&nodes[id].status.combined_score, sizeof(float));
      ifs.read((char *)&nodes[id].status.manip_info.valid, sizeof(bool));
      ifs.read((char *)&nodes[id].status.dynamic_manipulability, sizeof(float));
    }

    int jp_size = 0;
    ifs.read((char *)&jp_size, sizeof(int));
    if (jp_size > 0) {
      nodes[id].status.joint_positions.resize(jp_size);
      for (int i = 0; i < jp_size; ++i) {
        GrowingNeuralGas_Internal::read_eigen(
            ifs, nodes[id].status.joint_positions[i]);
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
        (size_t)n2 < nodes.size()) {
      add_edge_angle(n1, n2);
      edges_angle[n1][n2].age = age;
      edges_angle[n2][n1].age = age;
      edges_angle[n1][n2].active = active;
      edges_angle[n2][n1].active = active;
    }
  }

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
        (size_t)n2 < nodes.size()) {
      add_edge_coord(n1, n2);
      edges_coord[n1][n2].age = age;
      edges_coord[n2][n1].age = age;
      edges_coord[n1][n2].active = active;
      edges_coord[n2][n1].active = active;
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
  bool resize_needed = (params.max_node_num != params_.max_node_num);
  params_ = params;

  if (resize_needed) {
    nodes.assign(params_.max_node_num, NeuronNode<T_angle, T_coord>());
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
    n_learning = 0;
    n_trial_angle = 0;

    // 初期ノードの再追加
    for (int i = 0; i < params_.start_node_num; ++i) {
      T_angle wa;
      if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
        wa.setZero(angle_dimension);
      else
        wa.setZero();
      T_coord dummy_wc;
      add_node(wa, dummy_wc);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::triggerBatchUpdates() {
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
