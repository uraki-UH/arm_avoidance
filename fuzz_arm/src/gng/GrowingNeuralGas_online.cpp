#include "gng/GrowingNeuralGas_online.hpp"
#include "common/config_manager.hpp"
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

using namespace GNG;

namespace GNG {
// Utility functions for writing/reading Eigen objects
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
  } else {
    if (matrix.rows() != rows || matrix.cols() != cols) {
      // Handle error: matrix dimensions mismatch
    }
  }
  in.read((char *)matrix.data(), rows * cols * sizeof(typename T::Scalar));
}
} // namespace GNG

template <typename T_angle, typename T_coord>
GrowingNeuralGas<T_angle, T_coord>::GrowingNeuralGas(
    int angle_dim, int coord_dim, kinematics::KinematicChain *chain)
    : kinematic_chain_(chain), n_learning(0), manipulability_cb_(nullptr),
      n_trial_angle(0), n_trial_coord(0), angle_dimension(angle_dim),
      coord_dimension(coord_dim) {
  nodes.resize(params_.max_node_num);
  for (int i = 0; i < (int)nodes.size(); ++i)
    addable_node_indicies.push(i);

  edges_angle.resize(nodes.size());
  edges_coord.resize(nodes.size());
  edges_angle_per_node.resize(nodes.size());
  edges_coord_per_node.resize(nodes.size());

  // 初期ノード作成
  for (int i = 0; i < params_.start_node_num; ++i) {
    T_angle wa;
    if constexpr (T_angle::RowsAtCompileTime == Eigen::Dynamic)
      wa.setZero(angle_dimension);
    else
      wa.setZero();

    for (int j = 0; j < (int)wa.size(); ++j) {
      wa(j) = static_cast<typename T_angle::Scalar>(rand()) /
              static_cast<double>(RAND_MAX);
    }

    T_coord wc;
    if constexpr (T_coord::RowsAtCompileTime == Eigen::Dynamic)
      wc.setZero(coord_dimension);
    else
      wc.setZero();

    for (int j = 0; j < (int)wc.size(); ++j) {
      wc(j) = static_cast<typename T_coord::Scalar>(rand()) /
              static_cast<double>(RAND_MAX);
    }
    add_node(wa, wc);
  }
}

template <typename T_angle, typename T_coord>
GrowingNeuralGas<T_angle, T_coord>::~GrowingNeuralGas() {}

template <typename T_angle, typename T_coord>
T_coord
GrowingNeuralGas<T_angle, T_coord>::calculateFK(const T_angle &angle_values) {
  if (!kinematic_chain_) {
    std::cerr << "Error: KinematicChain not set for GNG." << std::endl;
    // エラーハンドリング: デフォルト値や例外をスロー
    if constexpr (T_coord::RowsAtCompileTime == Eigen::Dynamic) {
      return T_coord::Zero(coord_dimension);
    } else {
      return T_coord::Zero();
    }
  }

  int dof = kinematic_chain_->getTotalDOF();
  kinematic_chain_->updateKinematics(
      angle_values.head(std::min((int)angle_values.size(), dof)));

  // エンドエフェクタ先端の位置を取得
  Eigen::Vector3d eef_position_double = kinematic_chain_->getEEFPosition();

  // T_coord (Eigen::Vector3f) に変換
  return eef_position_double.cast<typename T_coord::Scalar>();
}

template <typename T_angle, typename T_coord>
class LegacyStatusProvider
    : public GrowingNeuralGas<T_angle, T_coord>::IStatusProvider {
  std::function<void(NeuronNode<T_angle, T_coord> &)> cb_;

public:
  LegacyStatusProvider(std::function<void(NeuronNode<T_angle, T_coord> &)> cb)
      : cb_(cb) {}
  std::vector<typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger>
  getTriggers() const override {
    return {GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::NODE_ADDED,
            GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger::COORD_UPDATED};
  }
  void update(NeuronNode<T_angle, T_coord> &node,
              [[maybe_unused]]
              typename GrowingNeuralGas<T_angle, T_coord>::UpdateTrigger
                  trigger) override {
    cb_(node);
  }
};

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::registerStatusProvider(
    const std::string &name,
    std::function<void(NeuronNode<T_angle, T_coord> &node)> cb) {
  auto provider = std::make_shared<LegacyStatusProvider<T_angle, T_coord>>(cb);
  named_providers_[name] = provider;
  providers_.push_back(provider);
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::unregisterStatusProvider(
    const std::string &name) {
  if (named_providers_.count(name)) {
    auto provider = named_providers_[name];
    providers_.erase(
        std::remove(providers_.begin(), providers_.end(), provider),
        providers_.end());
    named_providers_.erase(name);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::runStatusProviders(
    int node_id, [[maybe_unused]] UpdateTrigger trigger) {
  if (node_id < 0 || (size_t)node_id >= nodes.size())
    return;
  if (nodes[node_id].id == -1)
    return;

  auto &node = nodes[node_id];

  for (auto &provider : providers_) {
    auto triggers = provider->getTriggers();
    if (std::find(triggers.begin(), triggers.end(), trigger) !=
        triggers.end()) {
      if (provider->shouldUpdate(node, trigger)) {
        try {
          provider->update(node, trigger);
        } catch (...) {
        }
      }
    }
  }

  // 特殊扱い: 可操作性コールバック（従来の仕組み）
  if (trigger == UpdateTrigger::NODE_ADDED ||
      trigger == UpdateTrigger::COORD_UPDATED) {
    if (manipulability_cb_) {
      try {
        Status s = manipulability_cb_(node.weight_angle, node.weight_coord);
        // メタデータ等を維持しつつ更新
        // node.status.label = s.label;
        // node.status.radius = s.radius;
        // node.status.velocity = s.velocity;
        node.status.level = s.level;
        node.status.is_surface = s.is_surface;
        node.status.valid = s.valid;
        node.status.manip_info = s.manip_info;
      } catch (...) {
      }
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::triggerBatchUpdates() {
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      runStatusProviders(i, UpdateTrigger::BATCH_UPDATE);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::triggerPeriodicUpdates() {
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      runStatusProviders(i, UpdateTrigger::TIME_PERIODIC);
    }
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::update_node_weights(
    int node_id, const T_angle &sample_angle,
    const T_coord &sample_coord, // sample_coord may be ignored
    float step, bool update_angle, bool update_coord) {
  (void)sample_coord; // Suppress unused parameter warning
  (void)update_coord; // Suppress unused parameter warning
  if (node_id < 0 || (size_t)node_id >= nodes.size())
    return;
  if (nodes[node_id].id == -1)
    return;

  bool changed = false;
  if (update_angle && (size_t)sample_angle.size() > 0) {
    if ((size_t)nodes[node_id].weight_angle.size() ==
        (size_t)sample_angle.size()) {
      nodes[node_id].weight_angle +=
          step * (sample_angle - nodes[node_id].weight_angle);
      changed = true;
      // weight_angleが更新されたので、weight_coordもFKで更新する
      nodes[node_id].weight_coord = calculateFK(nodes[node_id].weight_angle);
    }
  }
  // update_coord フラグはここでは無視され、常にFKから計算される
  // if (update_coord && (size_t)sample_coord.size() > 0) {
  //   if ((size_t)nodes[node_id].weight_coord.size() ==
  //       (size_t)sample_coord.size()) {
  //     nodes[node_id].weight_coord +=
  //         step * (sample_coord - nodes[node_id].weight_coord);
  //     changed = true;
  //   }
  // }

  if (changed) {
    runStatusProviders(node_id, UpdateTrigger::COORD_UPDATED);
  }
}

template <typename T_angle, typename T_coord>
int GrowingNeuralGas<T_angle, T_coord>::add_node(
    T_angle w_angle,
    T_coord w_coord) { // w_coord will be recalculated
  (void)w_coord;       // Suppress unused parameter warning
  if (addable_node_indicies.empty())
    return -1;
  int node_id = addable_node_indicies.front();
  addable_node_indicies.pop();

  // Calculate w_coord from w_angle using FK
  T_coord calculated_w_coord = calculateFK(w_angle);

  nodes[node_id] =
      NeuronNode<T_angle, T_coord>(node_id, w_angle, calculated_w_coord);
  edges_angle[node_id].clear();
  edges_coord[node_id].clear();
  edges_angle_per_node[node_id].clear();
  edges_coord_per_node[node_id].clear();

  runStatusProviders(node_id, UpdateTrigger::NODE_ADDED);
  return node_id;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::remove_node(int node) {
  if (node < 0 || (size_t)node >= nodes.size())
    return;
  if (!edges_angle_per_node[node].empty() ||
      !edges_coord_per_node[node].empty())
    return;

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
  if (node_1 < 0 || node_2 < 0)
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
  if (node_1 < 0 || node_2 < 0)
    return;
  // Erase from maps
  edges_angle[node_1].erase(node_2);
  edges_angle[node_2].erase(node_1);

  // Erase from vector lists
  auto &v1 = edges_angle_per_node[node_1];
  v1.erase(std::remove(v1.begin(), v1.end(), node_2), v1.end());
  auto &v2 = edges_angle_per_node[node_2];
  v2.erase(std::remove(v2.begin(), v2.end(), node_1), v2.end());
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::add_edge_coord(int node_1,
                                                        int node_2) {
  if (node_1 < 0 || node_2 < 0)
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
  if (node_1 < 0 || node_2 < 0)
    return;
  // Erase from maps
  edges_coord[node_1].erase(node_2);
  edges_coord[node_2].erase(node_1);

  // Erase from vector lists
  auto &v1 = edges_coord_per_node[node_1];
  v1.erase(std::remove(v1.begin(), v1.end(), node_2), v1.end());
  auto &v2 = edges_coord_per_node[node_2];
  v2.erase(std::remove(v2.begin(), v2.end(), node_1), v2.end());
}

template <typename T_angle, typename T_coord>
float GrowingNeuralGas<T_angle, T_coord>::calc_squaredNorm_angle(
    const T_angle &w1, const T_angle &w2) {
  if ((size_t)w1.size() == 0 || (size_t)w2.size() == 0)
    return std::numeric_limits<float>::max();
  return static_cast<float>((w1 - w2).squaredNorm());
}

template <typename T_angle, typename T_coord>
float GrowingNeuralGas<T_angle, T_coord>::calc_squaredNorm_coord(
    const T_coord &w1, const T_coord &w2) {
  if ((size_t)w1.size() == 0 || (size_t)w2.size() == 0)
    return std::numeric_limits<float>::max();
  return static_cast<float>((w1 - w2).squaredNorm());
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::clear_all_edges_of_node(int node) {
  if (node < 0)
    return;
  {
    std::vector<int> neigh(edges_angle_per_node[node].begin(),
                           edges_angle_per_node[node].end());
    for (int n : neigh)
      remove_edge_angle(node, n);
  }
  {
    std::vector<int> neigh(edges_coord_per_node[node].begin(),
                           edges_coord_per_node[node].end());
    for (int n : neigh)
      remove_edge_coord(node, n);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::gngTrain(
    const std::vector<T_angle> &angle_samples,
    const std::vector<T_coord> &coord_samples, int mode) {
  int num_samples =
      (mode == 0) ? (int)angle_samples.size() : (int)coord_samples.size();
  if (num_samples == 0)
    return;

  for (int k = 0; k < 100; ++k) {
    int idx = rand() % num_samples;
    T_angle sa =
        (idx < (int)angle_samples.size()) ? angle_samples[idx] : T_angle();
    T_coord sc =
        (idx < (int)coord_samples.size()) ? coord_samples[idx] : T_coord();
    one_train_update(sa, sc, mode);
  }
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::one_train_update(
    const T_angle &sample_angle, const T_coord &sample_coord, int mode) {
  float min_dis1 = std::numeric_limits<float>::max();
  float min_dis2 = std::numeric_limits<float>::max();
  int s1_id = -1, s2_id = -1;
  float dis = 0.0f;

  for (auto &node : nodes) {
    if (node.id == -1)
      continue;
    node.error_angle -= params_.beta * node.error_angle;
    node.error_coord -= params_.beta * node.error_coord;

    if (mode == 0)
      dis = calc_squaredNorm_angle(sample_angle, node.weight_angle);
    else
      dis = calc_squaredNorm_coord(sample_coord, node.weight_coord);

    if (dis < min_dis1) {
      min_dis2 = min_dis1;
      s2_id = s1_id;
      min_dis1 = dis;
      s1_id = node.id;
    } else if (dis < min_dis2) {
      min_dis2 = dis;
      s2_id = node.id;
    }
  }
  if (s1_id == -1)
    return;
  float dist = std::sqrt(min_dis1);

  if (mode == 0) {
    // --- Angle Space Mode: Full GNG Algorithm ---
    nodes[s1_id].error_angle += dist;
    update_node_weights(s1_id, sample_angle, sample_coord,
                        params_.learn_rate_s1, true,
                        ((size_t)sample_coord.size() > 0));
    if (s2_id != -1)
      add_edge_angle(s1_id, s2_id);

    auto &neighbors = edges_angle_per_node[s1_id];
    std::vector<int> delete_edges;
    for (auto nid : neighbors) {
      if (edges_angle[s1_id][nid].age > params_.max_edge_age)
        delete_edges.push_back(nid);
      else {
        update_node_weights(nid, sample_angle, sample_coord,
                            params_.learn_rate_s2, true,
                            ((size_t)sample_coord.size() > 0));
        edges_angle[s1_id][nid].age++;
        edges_angle[nid][s1_id].age++;
      }
    }
    for (auto dn : delete_edges) {
      remove_edge_angle(s1_id, dn);
      if (edges_angle_per_node[dn].empty() && edges_coord_per_node[dn].empty())
        remove_node(dn);
    }
    n_trial_angle++;

    // Node addition check (only in mode 0)
    if (n_trial_angle >= params_.lambda) {
      n_trial_angle = 0;
      if (!addable_node_indicies.empty()) {
        float max_err_q = -1.0f, max_err_f = -1.0f;
        int q_id = -1, f_id = -1;
        for (const auto &node : nodes) {
          if (node.id == -1)
            continue;
          if (node.error_angle > max_err_q) {
            max_err_q = node.error_angle;
            q_id = node.id;
          }
        }
        if (q_id != -1) {
          for (const auto &nid : edges_angle_per_node[q_id]) {
            if (nodes[nid].error_angle > max_err_f) {
              max_err_f = nodes[nid].error_angle;
              f_id = nid;
            }
          }
        }
        if (q_id != -1 && f_id != -1) {
          T_angle mid_a =
              (nodes[q_id].weight_angle + nodes[f_id].weight_angle) * 0.5f;
          // T_coord mid_c = (nodes[q_id].weight_coord +
          // nodes[f_id].weight_coord) * 0.5f; // Remove or comment out
          T_coord mid_c = calculateFK(mid_a); // Calculate w_coord using FK
          int new_id = add_node(mid_a, mid_c);
          remove_edge_angle(q_id, f_id);
          add_edge_angle(q_id, new_id);
          add_edge_angle(f_id, new_id);
          nodes[q_id].error_angle *= params_.alpha;
          nodes[f_id].error_angle *= params_.alpha;
          nodes[new_id].error_angle =
              (nodes[q_id].error_angle + nodes[f_id].error_angle) * 0.5f;
        }
      }
    }
  } else {
    // --- Coordinate Space Mode: Topology Only ---
    // Only connect the two nearest nodes in 3D space && edge aging
    if (s1_id != -1 && s2_id != -1) {
      add_edge_coord(s1_id, s2_id);
    }

    // Age and remove old coordinate edges around winner (s1)
    if (s1_id != -1) {
      auto &neighbors = edges_coord_per_node[s1_id];
      std::vector<int> delete_edges;
      for (auto nid : neighbors) {
        if (edges_coord[s1_id][nid].age > params_.max_edge_age)
          delete_edges.push_back(nid);
        else {
          // increment age (keep symmetry)
          edges_coord[s1_id][nid].age++;
          edges_coord[nid][s1_id].age++;
        }
      }
      for (auto dn : delete_edges) {
        remove_edge_coord(s1_id, dn);
        // remove node if it has no edges in either graph
        if (edges_angle_per_node[dn].empty() &&
            edges_coord_per_node[dn].empty())
          remove_node(dn);
      }
    }
  }
  n_learning++;
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::save(
    const std::string &filename) const {
  std::ofstream ofs(filename, std::ios::binary);
  if (!ofs) {
    std::cerr << "Error: Cannot open file for writing: " << filename
              << std::endl;
    return false;
  }

  const uint32_t version = 4; // Version 4: Extended Status + Manipulability
  ofs.write((char *)&version, sizeof(version));

  int active_node_count = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      active_node_count++;
    }
  }
  ofs.write((char *)&active_node_count, sizeof(active_node_count));

  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      ofs.write((char *)&nodes[i].id, sizeof(int));
      ofs.write((char *)&nodes[i].error_angle, sizeof(float));
      ofs.write((char *)&nodes[i].error_coord, sizeof(float));
      write_eigen(ofs, nodes[i].weight_angle);
      write_eigen(ofs, nodes[i].weight_coord);

      // Status info
      ofs.write((char *)&nodes[i].status.level, sizeof(int));
      ofs.write((char *)&nodes[i].status.is_surface, sizeof(bool));
      ofs.write((char *)&nodes[i].status.is_active_surface, sizeof(bool));
      ofs.write((char *)&nodes[i].status.valid, sizeof(bool));
      ofs.write((char *)&nodes[i].status.active, sizeof(bool));
      write_eigen(ofs, nodes[i].status.ee_direction);

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
      for (const auto &jp : nodes[i].status.joint_positions) {
        write_eigen(ofs, jp);
      }
    }
  }

  int angle_edge_count = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1)
      continue;
    for (const auto &pair : edges_angle[i]) {
      if (i < pair.first) { // Save each edge only once
        angle_edge_count++;
      }
    }
  }
  ofs.write((char *)&angle_edge_count, sizeof(angle_edge_count));
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1)
      continue;
    for (const auto &pair : edges_angle[i]) {
      int neighbor = pair.first;
      if (i < neighbor) {
        ofs.write((char *)&i, sizeof(int));
        ofs.write((char *)&neighbor, sizeof(int));
        ofs.write((char *)&pair.second.age, sizeof(int));
      }
    }
  }

  int coord_edge_count = 0;
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1)
      continue;
    for (const auto &pair : edges_coord[i]) {
      if (i < pair.first) { // Save each edge only once
        coord_edge_count++;
      }
    }
  }
  ofs.write((char *)&coord_edge_count, sizeof(coord_edge_count));
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id == -1)
      continue;
    for (const auto &pair : edges_coord[i]) {
      int neighbor = pair.first;
      if (i < neighbor) {
        ofs.write((char *)&i, sizeof(int));
        ofs.write((char *)&neighbor, sizeof(int));
        ofs.write((char *)&pair.second.age, sizeof(int));
      }
    }
  }

  ofs.close();
  return true;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::loadParameters(
    const std::string &filename) {
  common::ConfigManager &cfg = common::ConfigManager::Instance();
  if (!cfg.Load(filename)) {
    std::cerr << "[GNG] Warning: Failed to load GNG parameters from "
              << filename << ". Using defaults." << std::endl;
    return;
  }

  params_.lambda = cfg.GetInt("lambda", params_.lambda);
  params_.max_node_num = cfg.GetInt("max_node_num", params_.max_node_num);
  params_.num_samples = cfg.GetInt("num_samples", params_.num_samples);
  params_.max_iterations = cfg.GetInt("max_iterations", params_.max_iterations);
  params_.refine_iterations =
      cfg.GetInt("refine_iterations", params_.refine_iterations);
  params_.coord_edge_iterations =
      cfg.GetInt("coord_edge_iterations", params_.coord_edge_iterations);

  params_.learn_rate_s1 =
      (float)cfg.GetDouble("learn_rate_s1", params_.learn_rate_s1);
  params_.learn_rate_s2 =
      (float)cfg.GetDouble("learn_rate_s2", params_.learn_rate_s2);
  params_.alpha = (float)cfg.GetDouble("alpha", params_.alpha);
  params_.beta = (float)cfg.GetDouble("beta", params_.beta);
  params_.max_edge_age = cfg.GetInt("max_edge_age", params_.max_edge_age);
  params_.start_node_num = cfg.GetInt("start_node_num", params_.start_node_num);
  params_.n_best_candidates =
      cfg.GetInt("n_best_candidates", params_.n_best_candidates);
  params_.ais_threshold =
      (float)cfg.GetDouble("ais_threshold", params_.ais_threshold);

  // Resize internal data structures if max_node_num changed
  if (nodes.size() != (size_t)params_.max_node_num) {
    std::cout << "[GNG] Resizing to " << params_.max_node_num << " nodes."
              << std::endl;
    nodes.resize(params_.max_node_num);
    edges_angle.resize(nodes.size());
    edges_coord.resize(nodes.size());
    edges_angle_per_node.resize(nodes.size());
    edges_coord_per_node.resize(nodes.size());

    // Reset addable indices
    std::queue<int> empty_queue;
    addable_node_indicies.swap(empty_queue);
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (nodes[i].id == -1)
        addable_node_indicies.push(i);
    }
  }

  std::cout << "[GNG] Parameters loaded from " << filename << std::endl;
}

template <typename T_angle, typename T_coord>
void GrowingNeuralGas<T_angle, T_coord>::refresh_coord_weights() {
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      nodes[i].weight_coord = calculateFK(nodes[i].weight_angle);
    }
  }
}

template <typename T_angle, typename T_coord>
bool GrowingNeuralGas<T_angle, T_coord>::load(const std::string &filename) {
  std::ifstream ifs(filename, std::ios::binary);
  if (!ifs) {
    std::cerr << "Error: Cannot open file for reading: " << filename
              << std::endl;
    return false;
  }

  // --- Reset GNG state ---
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (nodes[i].id != -1) {
      clear_all_edges_of_node(i);
      nodes[i].id = -1;
    }
  }
  edges_angle.resize(nodes.size());
  edges_coord.resize(nodes.size());
  for (int i = 0; i < (int)nodes.size(); ++i) {
    edges_angle[i].clear();
    edges_coord[i].clear();
  }
  edges_angle_per_node.assign(nodes.size(), std::vector<int>());
  edges_coord_per_node.assign(nodes.size(), std::vector<int>());
  std::queue<int> empty_queue;
  addable_node_indicies.swap(empty_queue);
  for (int i = 0; i < (int)nodes.size(); i++)
    addable_node_indicies.push(i);
  n_learning = 0;
  n_trial_angle = 0;
  n_trial_coord = 0;

  uint32_t version;
  ifs.read((char *)&version, sizeof(version));
  if (version != 1 && version != 2 && version != 3 && version != 4) {
    std::cerr << "Error: Unsupported file version: " << version << std::endl;
    return false;
  }

  int active_node_count;
  ifs.read((char *)&active_node_count, sizeof(active_node_count));

  std::vector<bool> is_node_active(nodes.size(), false);

  for (int i = 0; i < active_node_count; ++i) {
    int id;
    ifs.read((char *)&id, sizeof(int));

    if (id < 0 || (size_t)id >= nodes.size()) {
      std::cerr << "[GNG] Warning: Node ID " << id
                << " exceeds current capacity (" << nodes.size()
                << "). Skipping." << std::endl;
      // Need to skip the rest of this node data...
      // This is tricky without knowing exact size.
      // For now, assume config matches or is larger.
      continue;
    }

    nodes[id].id = id;
    ifs.read((char *)&nodes[id].error_angle, sizeof(float));
    ifs.read((char *)&nodes[id].error_coord, sizeof(float));

    T_angle wa;
    read_eigen(ifs, wa);
    nodes[id].weight_angle = wa;

    T_coord wc;
    read_eigen(ifs, wc);
    nodes[id].weight_coord = wc;

    if (version >= 2) {
      ifs.read((char *)&nodes[id].status.level, sizeof(int));
      ifs.read((char *)&nodes[id].status.is_surface, sizeof(bool));
      ifs.read((char *)&nodes[id].status.is_active_surface, sizeof(bool));
      ifs.read((char *)&nodes[id].status.valid, sizeof(bool));
      ifs.read((char *)&nodes[id].status.active, sizeof(bool));
      read_eigen(ifs, nodes[id].status.ee_direction);

      if (version >= 4) {
        ifs.read((char *)&nodes[id].status.manip_info.manipulability,
                 sizeof(float));
        ifs.read((char *)&nodes[id].status.min_singular_value, sizeof(float));
        ifs.read((char *)&nodes[id].status.joint_limit_score, sizeof(float));
        ifs.read((char *)&nodes[id].status.combined_score, sizeof(float));
        ifs.read((char *)&nodes[id].status.manip_info.valid, sizeof(bool));
        ifs.read((char *)&nodes[id].status.dynamic_manipulability,
                 sizeof(float));
      }

      int jp_size;
      ifs.read((char *)&jp_size, sizeof(int));
      nodes[id].status.joint_positions.resize(jp_size);
      for (int j = 0; j < jp_size; ++j) {
        read_eigen(ifs, nodes[id].status.joint_positions[j]);
      }
    }

    is_node_active[id] = true;
    edges_angle_per_node[id] = std::unordered_set<int>();
    edges_coord_per_node[id] = std::unordered_set<int>();
  }

  // Rebuild addable_node_indicies
  addable_node_indicies = std::queue<int>();
  for (int i = 0; i < (int)nodes.size(); ++i) {
    if (!is_node_active[i]) {
      addable_node_indicies.push(i);
    }
  }

  int angle_edge_count;
  ifs.read((char *)&angle_edge_count, sizeof(angle_edge_count));
  for (int i = 0; i < angle_edge_count; ++i) {
    int u, v, age;
    ifs.read((char *)&u, sizeof(int));
    ifs.read((char *)&v, sizeof(int));
    ifs.read((char *)&age, sizeof(int));
    add_edge_angle(u, v);
    edges_angle[u][v].age = age;
    edges_angle[v][u].age = age;

    if (version >= 3) {
      bool active;
      ifs.read((char *)&active, sizeof(bool));
      edges_angle[u][v].active = active;
      edges_angle[v][u].active = active;
    }
  }

  int coord_edge_count;
  ifs.read((char *)&coord_edge_count, sizeof(coord_edge_count));
  for (int i = 0; i < coord_edge_count; ++i) {
    int u, v, age;
    ifs.read((char *)&u, sizeof(int));
    ifs.read((char *)&v, sizeof(int));
    ifs.read((char *)&age, sizeof(int));
    add_edge_coord(u, v);
    edges_coord[u][v].age = age;
    edges_coord[v][u].age = age;

    if (version >= 3) {
      bool active;
      ifs.read((char *)&active, sizeof(bool));
      edges_coord[u][v].active = active;
      edges_coord[v][u].active = active;
    }
  }

  ifs.close();

  // Update status of all loaded nodes
  triggerBatchUpdates();

  return true;
}

template class GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
template class GNG::GrowingNeuralGas<Eigen::Vector3f, Eigen::Vector3f>;
