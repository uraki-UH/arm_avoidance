#pragma once

#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <queue>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "kinematics/kinematic_chain.hpp"
#include <Eigen/Dense>

// 共通のノード・エッジ定義
#include "gng_parameters.hpp"
#include "node_status.hpp"

namespace simulation {
class ISelfCollisionChecker;
}

namespace GNG {

/**
 * @brief 自己干渉回避を強化したGNGクラス
 * GrowingNeuralGas.hpp のコードをベースにしたスタンドアロン版
 */
template <typename T_angle, typename T_coord> class GrowingNeuralGas {
public:
  uint32_t version = 5; // Version 5: Added is_boundary flag
  using UpdateTrigger = GNG::UpdateTrigger;
  using NodeType = NeuronNode<T_angle, T_coord>;
  using ManipulabilityCallback =
      std::function<Status(const T_angle &angle, const T_coord &coord)>;

  struct IStatusProvider {
    virtual ~IStatusProvider() = default;
    virtual std::vector<UpdateTrigger> getTriggers() const = 0;
    virtual void update(NeuronNode<T_angle, T_coord> &node,
                        UpdateTrigger trigger) = 0;
    virtual bool
    shouldUpdate([[maybe_unused]] const NeuronNode<T_angle, T_coord> &node,
                 [[maybe_unused]] UpdateTrigger trigger) const {
      return true;
    }
  };

  GrowingNeuralGas(int angle_dim, int coord_dim,
                    kinematics::KinematicChain *chain);
  ~GrowingNeuralGas();

  void registerStatusProvider(std::shared_ptr<IStatusProvider> provider) {
    providers_.push_back(provider);
  }

  void gngTrain(const std::vector<T_angle> &angle_samples,
                const std::vector<T_coord> &coord_samples, int mode = 0);

  // 自己干渉チェッカーの設定
  void setCollisionAware(bool enable) { collision_aware_ = enable; }
  bool isCollisionAware() const { return collision_aware_; }
  void setSelfCollisionChecker(simulation::ISelfCollisionChecker *checker) {
    collision_checker_ = checker;
  }

  void gngTrain(const std::vector<T_angle> &samples, int max_iter = -1);
  void gngTrainOnTheFly(int max_iter);

  const std::vector<int> &getNeighborsAngle(int node_id) const {
    return edges_angle_per_node[node_id];
  }

  const std::vector<int> &getNeighborsCoord(int node_id) const {
    return edges_coord_per_node[node_id];
  }

  bool isEdgeActive(int n1, int n2, int mode = 0) const {
    if (n1 < 0 || n2 < 0 || (size_t)n1 >= nodes.size() ||
        (size_t)n2 >= nodes.size())
      return false;
    if (mode == 0) {
      if ((size_t)n1 >= edges_angle.size())
        return false;
      auto it = edges_angle[n1].find(n2);
      return (it != edges_angle[n1].end()) && it->second.active;
    } else {
      if ((size_t)n1 >= edges_coord.size())
        return false;
      auto it = edges_coord[n1].find(n2);
      return (it != edges_coord[n1].end()) && it->second.active;
    }
  }

  void refresh_coord_weights();
  void trainCoordEdges(const std::vector<T_angle> &angle_samples,
                       int max_iter = -1);
  void trainCoordEdgesOnTheFly(int max_iter);

  void strictFilter();
  void removeInactiveElements();
  void pruneToLargestComponent();

  void triggerBatchUpdates();
  void triggerPeriodicUpdates();

  size_t getMaxNodeNum() const { return nodes.size(); }

  const NeuronNode<T_angle, T_coord> &nodeAt(int idx) const {
    return nodes[idx];
  }

  const std::vector<NeuronNode<T_angle, T_coord>> &get_nodes() const {
    return nodes;
  }

  NeuronNode<T_angle, T_coord> &nodeAt(int idx) { return nodes[idx]; }

  /**
   * @brief Get currently active node indices.
   */
  const std::vector<int>& getActiveIndices() const { return active_indices_; }

  /**
   * @brief Iterate over active nodes (mutable).
   */
  template <typename Fn> void forEachActive(Fn &&fn) {
    for (int i : active_indices_) {
      fn(i, nodes[i]);
    }
  }

  /**
   * @brief Iterate over active nodes (const).
   */
  template <typename Fn> void forEachActive(Fn &&fn) const {
    for (int i : active_indices_) {
      fn(i, nodes[i]);
    }
  }

  /**
   * @brief Iterate over active + valid nodes (mutable).
   */
  template <typename Fn> void forEachActiveValid(Fn &&fn) {
    for (int i : active_indices_) {
      auto &n = nodes[i];
      if (n.status.valid)
        fn(i, n);
    }
  }

  /**
   * @brief Iterate over active + valid nodes (const).
   */
  template <typename Fn> void forEachActiveValid(Fn &&fn) const {
    for (int i : active_indices_) {
      const auto &n = nodes[i];
      if (n.status.valid)
        fn(i, n);
    }
  }

  /**
   * @brief Get nodes vector (mutable).
   */
  std::vector<NeuronNode<T_angle, T_coord>> &getNodes() { return nodes; }

  /**
   * @brief Get nodes vector (const).
   */
  const std::vector<NeuronNode<T_angle, T_coord>> &getNodes() const {
    return nodes;
  }

  bool save(const std::string &filename);
  bool load(const std::string &filename);

  void setStatsLogPath(const std::string &path) { stats_log_path_ = path; }

  // パラメータ管理
  void loadParameters(const std::string &filename);
  const GngParameters &getParams() const { return params_; }
  void setParams(const GngParameters &p);

  void setNodeActive(int node_id, bool active) {
    if (node_id >= 0 && (size_t)node_id < nodes.size()) {
      nodes[node_id].status.active = active;
    }
  }

  // mode: 0=Angle, 1=Coord
  void setEdgeActive(int n1, int n2, bool active, int mode = 0) {
    if (n1 < 0 || n2 < 0 || (size_t)n1 >= nodes.size() ||
        (size_t)n2 >= nodes.size())
      return;
    if (mode == 0) {
      auto it1 = edges_angle[n1].find(n2);
      if (it1 != edges_angle[n1].end())
        it1->second.active = active;
      auto it2 = edges_angle[n2].find(n1);
      if (it2 != edges_angle[n2].end())
        it2->second.active = active;
    } else {
      auto it1 = edges_coord[n1].find(n2);
      if (it1 != edges_coord[n1].end())
        it1->second.active = active;
      auto it2 = edges_coord[n2].find(n1);
      if (it2 != edges_coord[n2].end())
        it2->second.active = active;
    }
  }

private:
  void one_train_update(const T_angle &sample_angle,
                        const T_coord &sample_coord, int mode);
  void one_train_update(const T_angle &sample_angle);

  int add_node(T_angle w_angle, T_coord w_coord);
  void remove_node(int node_id);
  void add_edge_angle(int node_1, int node_2);
  void remove_edge_angle(int node_1, int node_2);
  void add_edge_coord(int node_1, int node_2);
  void remove_edge_coord(int node_1, int node_2);
  void update_node_weights(int node_id, const T_angle &sample_angle,
                           float step);

  float calc_squaredNorm_angle(const T_angle &w1, const T_angle &w2) const;
  float calc_squaredNorm_coord(const T_coord &w1, const T_coord &w2) const;
  T_coord calculateFK(const T_angle &angle_values);
  void runStatusProviders(int node_id, UpdateTrigger trigger);

  bool internalCheckColliding(const T_angle &angles);
  bool internalCheckPathColliding(const T_angle &q1, const T_angle &q2,
                                  int steps = 5);

  std::vector<NeuronNode<T_angle, T_coord>> nodes;
  kinematics::KinematicChain *const kinematic_chain_;

  std::vector<std::vector<int>> edges_angle_per_node;
  std::vector<std::unordered_map<int, EdgeInfo>> edges_angle;

  std::vector<std::vector<int>> edges_coord_per_node;
  std::vector<std::unordered_map<int, EdgeInfo>> edges_coord;

  int n_learning = 0;
  int n_trial_angle = 0;
  std::vector<std::shared_ptr<IStatusProvider>> providers_;
  std::vector<int> active_indices_;
  std::queue<int> addable_node_indicies;

  std::vector<double> q_buffer;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      pos_buffer;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      ori_buffer;

  int angle_dimension = 0;
  int coord_dimension = 0;
  bool collision_aware_ = false;
  simulation::ISelfCollisionChecker *collision_checker_ = nullptr;

  // 距離統計用の LPF 変数
  float ema1_s1_sq = 0.0f;
  float ema1_s2_sq = 0.0f;
  float ema2_s1_sq = 0.0f;
  float ema2_s2_sq = 0.0f;

  std::string stats_log_path_ = "gng_distance_stats.dat";
  std::ofstream stats_ofs_;
  GngParameters params_;

  // Optimization buffers
  mutable std::vector<std::pair<float, int>> candidates_buffer_;
  float accumulated_decay_factor_ = 1.0f;
  int decay_step_count_ = 0;
};

} // namespace GNG
