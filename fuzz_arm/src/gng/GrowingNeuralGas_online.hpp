#pragma once

#include <cstdlib>
#include <functional>
#include <queue>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "kinematics/kinematic_chain.hpp" // これを追加

#define EIGEN_NO_DEBUG

// #define MAX_NODE_NUM 1000 // Removed for dynamic sizing

#include "common/gng_parameters.hpp"
#include "common/node_status.hpp"

namespace GNG {

template <typename T_angle, typename T_coord> class GrowingNeuralGas {
public:
  using UpdateTrigger = GNG::UpdateTrigger;

  using ManipulabilityCallback =
      std::function<Status(const T_angle &angle, const T_coord &coord)>;

  // 前方宣言
  struct IStatusProvider {
    virtual ~IStatusProvider() = default;
    virtual std::vector<UpdateTrigger> getTriggers() const = 0;
    virtual void update(NeuronNode<T_angle, T_coord> &node,
                        UpdateTrigger trigger) = 0;
    // デフォルトでは全てのノードを更新。必要に応じてオーバーライドしてフィルタリング。
    virtual bool shouldUpdate(const NeuronNode<T_angle, T_coord> &node,
                              [[maybe_unused]] UpdateTrigger trigger) const {
      (void)node; // Suppress unused parameter warning
      return true;
    }
  };

  GrowingNeuralGas(int angle_dim, int coord_dim,
                   kinematics::KinematicChain *chain);
  ~GrowingNeuralGas();

  // 旧式コールバック互換
  void setManipulabilityCallback(ManipulabilityCallback cb) {
    manipulability_cb_ = std::move(cb);
  }

  // プロバイダ登録
  void registerStatusProvider(std::shared_ptr<IStatusProvider> provider) {
    providers_.push_back(provider);
  }

  // 旧方式互換用（内部でラップする）
  void registerStatusProvider(
      const std::string &name,
      std::function<void(NeuronNode<T_angle, T_coord> &node)> cb);

  void
  unregisterStatusProvider(const std::string &name); // 名前ベースの削除は互換用

  // トレーニングエントリ
  // mode: 0 for Angle Space, 1 for Coordinate Space
  void gngTrain(const std::vector<T_angle> &angle_samples,
                const std::vector<T_coord> &coord_samples, int mode = 0);

  // サンプルの片方しか無い場合のためのショートカット
  void gngTrainAngle(const std::vector<T_angle> &samples) {
    gngTrain(samples, {}, 0);
  }
  void gngTrainCoord(const std::vector<T_coord> &samples) {
    gngTrain({}, samples, 1);
  }

  // 直接ノード参照（読み出しのみ）
  const NeuronNode<T_angle, T_coord> &nodeAt(int idx) const {
    return nodes[idx];
  }

  void setNodeActive(int node_id, bool active) {
    if (node_id >= 0 && (size_t)node_id < nodes.size()) {
      nodes[node_id].status.active = active;
    }
  }

  void setNodeActiveSurface(int node_id, bool active_surface) {
    if (node_id >= 0 && (size_t)node_id < nodes.size()) {
      nodes[node_id].status.is_active_surface = active_surface;
    }
  }

  // 近傍取得
  const std::vector<int> &getNeighborsAngle(int node_id) const {
    return edges_angle_per_node.at(node_id);
  }
  const std::vector<int> &getNeighborsCoord(int node_id) const {
    return edges_coord_per_node.at(node_id);
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

  // providers 実行
  void runStatusProviders(int node_id, UpdateTrigger trigger);
  void triggerBatchUpdates();
  void triggerPeriodicUpdates();

  size_t getMaxNodeNum() const { return nodes.size(); }

  bool save(const std::string &filename) const;
  bool load(const std::string &filename);
  void clear_all_edges_of_node(int node_id);
  void refresh_coord_weights();

  // パラメータ管理
  void loadParameters(const std::string &filename);
  const GngParameters &getParams() const { return params_; }
  void setParams(const GngParameters &p) { params_ = p; }

private:
  // core ops
  void update_node_weights(int node_id, const T_angle &sample_angle,
                           const T_coord &sample_coord, float step,
                           bool update_angle, bool update_coord);
  int add_node(T_angle w_angle, T_coord w_coord);
  void remove_node(int node_id);

  void add_edge_angle(int node_1, int node_2);
  void remove_edge_angle(int node_1, int node_2);
  void add_edge_coord(int node_1, int node_2);
  void remove_edge_coord(int node_1, int node_2);

  float calc_squaredNorm_angle(const T_angle &w1, const T_angle &w2);
  float calc_squaredNorm_coord(const T_coord &w1, const T_coord &w2);

  // mode: 0 for Angle, 1 for Coord
  void one_train_update(const T_angle &sample_angle,
                        const T_coord &sample_coord, int mode);

  // data
  std::vector<NeuronNode<T_angle, T_coord>> nodes;

  kinematics::KinematicChain
      *const kinematic_chain_; // コンストラクタで初期化される

  // Forward Kinematics related
  T_coord calculateFK(const T_angle &angle_values);

  // Angle Space Edges (Sparse Properties)
  std::vector<int> empty_neighbors; // For safe access
  std::vector<std::vector<int>> edges_angle_per_node;
  std::vector<std::unordered_map<int, EdgeInfo>> edges_angle;

  // Coordinate Space Edges (Sparse Properties)
  std::vector<std::vector<int>> edges_coord_per_node;
  std::vector<std::unordered_map<int, EdgeInfo>> edges_coord;

  int n_learning = 0;

  std::vector<std::shared_ptr<IStatusProvider>> providers_;
  std::unordered_map<std::string, std::shared_ptr<IStatusProvider>>
      named_providers_; // 互換用
  ManipulabilityCallback manipulability_cb_;

  std::queue<int> addable_node_indicies;
  int n_trial_angle = 0;
  int n_trial_coord = 0;
  int angle_dimension = 0;
  int coord_dimension = 0;

  GngParameters params_;
};

} // namespace GNG
