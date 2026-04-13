#include "gng/GrowingNeuralGas_offline2.hpp"
#include <queue>
#include <vector>

namespace robot_sim {
namespace analysis {

/**
 * @brief 汎用グラフのためのトポロジー（連結成分）解析クラス
 *
 * テンプレートと事前に確保したバッファを用いることで、
 * アロケーションのオーバーヘッドを無くし、リアルタイムループ内で高速に動作します。
 */
class GraphTopologyAnalyzer {
public:
  GraphTopologyAnalyzer() = default;
  ~GraphTopologyAnalyzer() = default;

  /**
   * @brief トポロジー解析を実行し、最大の連結成分（本土）を特定します。
   *
   * @tparam Validator 関数またはラムダ型: bool(int node_id) ->
   * ノードが安全/有効か判定
   * @tparam NeighborProvider 関数またはラムダ型: const Container&(int node_id)
   * -> 隣接ノード群を返す
   * @param max_nodes グラフ内の最大ノードID (0 to max_nodes - 1)
   * @param is_valid ノードの有効性を判定するコールバック
   * @param get_neighbors 隣接ノードを取得するコールバック
   * @param robot_node_id (Optional) ロボットが現時点でもっとも近いノードID。
   *                      これを指定すると，このノードを含む成分が Mainland
   * となります。
   * @param fallback_to_largest (Optional) robot_node_id が見つからない場合，
   *                            従来通り最大成分を Mainland とするかどうか。
   */
  template <typename Validator, typename NeighborProvider>
  void analyze(int max_nodes, Validator is_valid,
               NeighborProvider get_neighbors, int robot_node_id = -1,
               bool fallback_to_largest = true) {
    std::vector<int> all_indices(max_nodes);
    for (int i = 0; i < max_nodes; ++i) all_indices[i] = i;
    analyze(all_indices, max_nodes, is_valid, get_neighbors, robot_node_id, fallback_to_largest);
  }

  template <typename IndexContainer, typename Validator, typename NeighborProvider>
  void analyze(const IndexContainer& active_indices, int max_nodes, Validator is_valid,
               NeighborProvider get_neighbors, int robot_node_id = -1,
               bool fallback_to_largest = true) {
    // 1. バッファサイズの調整と初期化
    if ((int)visited_.size() < max_nodes) {
      visited_.resize(max_nodes, 0);
      component_ids_.resize(max_nodes, -1);
      valid_mask_.resize(max_nodes, 0);
    }

    // 必要最小限の範囲のみ初期化
    for (int i : active_indices) {
      if (i >= 0 && i < max_nodes) {
        valid_mask_[i] = is_valid(i) ? 1 : 0;
        visited_[i] = 0;
        component_ids_[i] = -1;
      }
    }
    
    components_.clear();

    int current_group_id = 0;
    int max_size = 0;
    int mainland_id = -1;
    bool mainland_found_by_robot = false;

    // 2. アクティブなノードをスキャンし、未訪問の有効なノードからBFSを開始
    for (int i : active_indices) {
      if (i < 0 || i >= max_nodes || visited_[i] || !valid_mask_[i]) {
        continue;
      }

      // BFSの初期化
      bfs_vec_queue_.clear();
      bfs_vec_queue_.push_back(i);
      int head = 0;
      visited_[i] = 1;

      int current_size = 0;
      bool contains_robot = false;

      // 3. 連結している全ノードを探索
      while (head < (int)bfs_vec_queue_.size()) {
        int curr = bfs_vec_queue_[head++];

        component_ids_[curr] = current_group_id;
        current_size++;
        if (curr == robot_node_id) {
          contains_robot = true;
        }

        // 隣接ノードを展開
        const auto &neighbors = get_neighbors(curr);
        for (int neighbor_id : neighbors) {
          if (neighbor_id >= 0 && neighbor_id < max_nodes) {
            // valid_mask_ による高速判定
            if (!visited_[neighbor_id] && valid_mask_[neighbor_id]) {
              visited_[neighbor_id] = 1;
              bfs_vec_queue_.push_back(neighbor_id);
            }
          }
        }
      }

      // コンポーネント情報の記録
      components_.push_back({current_group_id, current_size});

      // 最大連結成分（本土）の更新
      if (robot_node_id != -1) {
        if (contains_robot) {
          mainland_id = current_group_id;
          mainland_found_by_robot = true;
        } else if (fallback_to_largest && !mainland_found_by_robot &&
                   current_size > max_size) {
          max_size = current_size;
          mainland_id = current_group_id;
        }
      } else {
        if (fallback_to_largest && current_size > max_size) {
          max_size = current_size;
          mainland_id = current_group_id;
        }
      }

      current_group_id++;
    }

    // 4. 結果の保存
    mainland_id_ = mainland_id;
  }

  struct TopologyStats {
    int active_count = 0;
    int mainland_count = 0;
    int island_count = 0;
  };

  /**
   * @brief トポロジー解析を実行し、その場で GNG ノードのステータスを更新します (高速版)。
   */
  template <typename T_angle, typename T_coord, typename IndexContainer,
            typename Validator, typename NeighborProvider>
  TopologyStats analyzeIntegrated(
      std::vector<GNG::NeuronNode<T_angle, T_coord>> &nodes,
      const IndexContainer &active_indices, int max_nodes, Validator is_valid,
      NeighborProvider get_neighbors, int robot_node_id = -1,
      bool fallback_to_largest = true) {
    // 1. 通常の解析を実行 (バッファ再利用)
    analyze(active_indices, max_nodes, is_valid, get_neighbors, robot_node_id,
            fallback_to_largest);

    int mainland_id = getMainlandId();
    TopologyStats stats;

    // 2. アクティブノードに対して結果を直接適用 (O(Active) 1回で完結)
    for (int i : active_indices) {
      if (i < 0 || i >= max_nodes)
        continue;
      auto &node = nodes[i];
      int group_id = component_ids_[i];

      node.status.topology_group_id = group_id;
      node.status.is_mainland = (mainland_id != -1 && group_id == mainland_id);
      
      // ロボットが非安全な場合は本土フラグを強制 OFF (SimulationApp の既存ロジック互換)
      if (robot_node_id == -1) {
          node.status.is_mainland = false;
      }

      if (node.status.valid) {
          stats.active_count++;
          if (node.status.is_mainland) {
              stats.mainland_count++;
          } else if (mainland_id != -1 && group_id != -1) {
              stats.island_count++;
          }
      }
    }
    return stats;
  }

  // 特定のノードが本土に属しているか
  bool isMainland(int node_id) const {
    if (node_id < 0 || node_id >= (int)component_ids_.size())
      return false;
    return component_ids_[node_id] == mainland_id_ && mainland_id_ != -1;
  }

  // 特定のノードのグループIDを取得
  int getGroupId(int node_id) const {
    if (node_id < 0 || node_id >= (int)component_ids_.size())
      return -1;
    return component_ids_[node_id];
  }

  // 本土のグループIDを取得
  int getMainlandId() const { return mainland_id_; }

private:
  // 状態保持バッファ（毎フレームのアロケーション防止）
  std::vector<uint8_t> visited_;
  std::vector<int> component_ids_;
  std::vector<uint8_t> valid_mask_;
  std::vector<int> bfs_vec_queue_;

  struct ComponentInfo {
    int id;
    int size;
  };
  std::vector<ComponentInfo> components_;
  int mainland_id_ = -1;
};

} // namespace analysis
} // namespace robot_sim
