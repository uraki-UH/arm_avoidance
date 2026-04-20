#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <vector>

namespace RRT {

/* ノードクラス */
class Node {
public:
  int id;
  std::vector<double> config; // 関節角度
  int parent_id;

  Node() : id(-1), parent_id(-1) {}
  Node(int node_id, const std::vector<double> &cfg, int parent = -1)
      : id(node_id), config(cfg), parent_id(parent) {}
};

/* RRT-Connect アルゴリズム実装 */
class RRTConnect {
public:
  // コンストラクタ
  RRTConnect(int dof, const std::vector<double> &lower_limits,
             const std::vector<double> &upper_limits)
      : dof_(dof), lower_limits_(lower_limits), upper_limits_(upper_limits),
        max_iterations_(10000), step_size_(0.1), goal_bias_(0.1),
        generator_(std::random_device()()), uniform_dist_(0.0, 1.0) {
    if (lower_limits_.size() != dof_ || upper_limits_.size() != dof_) {
      throw std::invalid_argument("Limit sizes must match DOF");
    }
  }

  // 衝突検出関数の設定
  void setCollisionChecker(
      std::function<bool(const std::vector<double> &)> checker) {
    collision_checker_ = checker;
  }

  // 距離計算関数の設定（デフォルトはユークリッド距離）
  void setDistanceMetric(std::function<double(const std::vector<double> &,
                                              const std::vector<double> &)>
                             metric) {
    distance_metric_ = metric;
  }

  // パラメータ設定
  void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
  void setStepSize(double step) { step_size_ = step; }
  void setGoalBias(double bias) { goal_bias_ = bias; }

  // RRT-Connect アルゴリズムの実行
  bool plan(const std::vector<double> &start, const std::vector<double> &goal) {
    if (!isValid(start) || !isValid(goal)) {
      return false;
    }

    // 2つの木を初期化
    tree_start_.clear();
    tree_goal_.clear();

    tree_start_.push_back(Node(0, start, -1));
    tree_goal_.push_back(Node(0, goal, -1));

    int connect_start_idx = -1;
    int connect_goal_idx = -1;
    bool start_to_goal = true; // 木の交換状態を追跡

    for (int iter = 0; iter < max_iterations_; ++iter) {
      // ランダムサンプリング
      std::vector<double> random_config = sampleRandomConfig();

      // Start木を拡張
      int new_start_idx = extend(tree_start_, random_config);

      if (new_start_idx != -1) {
        // Goal木をStart木の新ノードに向けて接続試行
        int connect_result =
            connect(tree_goal_, tree_start_[new_start_idx].config);

        if (connect_result != -1) {
          // 接続成功
          connect_start_idx = new_start_idx;
          connect_goal_idx = connect_result;
          break;
        }
      }

      // 木を交換して同じ処理
      std::swap(tree_start_, tree_goal_);
      start_to_goal = !start_to_goal;
    }

    // 経路が見つかった場合
    if (connect_start_idx != -1 && connect_goal_idx != -1) {
      constructPath(connect_start_idx, connect_goal_idx, start_to_goal);
      return true;
    }

    return false;
  }

  // 計画された経路を取得
  std::vector<std::vector<double>> getPath() const { return path_; }

  // 木の情報を取得（デバッグ用）
  const std::vector<Node> &getStartTree() const { return tree_start_; }
  const std::vector<Node> &getGoalTree() const { return tree_goal_; }

private:
  int dof_; // 自由度
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;

  int max_iterations_;
  double step_size_;
  double goal_bias_;

  std::vector<Node> tree_start_;
  std::vector<Node> tree_goal_;
  std::vector<std::vector<double>> path_;

  std::function<bool(const std::vector<double> &)> collision_checker_;
  std::function<double(const std::vector<double> &,
                       const std::vector<double> &)>
      distance_metric_;

  std::mt19937 generator_;
  std::uniform_real_distribution<double> uniform_dist_;

  // ランダムな設定をサンプリング
  std::vector<double> sampleRandomConfig() {
    std::vector<double> config(dof_);
    for (int i = 0; i < dof_; ++i) {
      double range = upper_limits_[i] - lower_limits_[i];
      config[i] = lower_limits_[i] + uniform_dist_(generator_) * range;
    }
    return config;
  }

  // 設定が有効かチェック
  bool isValid(const std::vector<double> &config) const {
    if (config.size() != dof_)
      return false;

    // 範囲チェック
    for (int i = 0; i < dof_; ++i) {
      if (config[i] < lower_limits_[i] || config[i] > upper_limits_[i]) {
        return false;
      }
    }

    // 衝突チェック
    if (collision_checker_ && collision_checker_(config)) {
      return false;
    }

    return true;
  }

  // 距離計算
  double distance(const std::vector<double> &config1,
                  const std::vector<double> &config2) const {
    if (distance_metric_) {
      return distance_metric_(config1, config2);
    }

    // デフォルト: ユークリッド距離
    double sum = 0.0;
    for (int i = 0; i < dof_; ++i) {
      double diff = config1[i] - config2[i];
      sum += diff * diff;
    }
    return std::sqrt(sum);
  }

  // 最近傍ノードを検索（インデックスを返す）
  int findNearestNode(const std::vector<Node> &tree,
                      const std::vector<double> &config) const {
    double min_dist = std::numeric_limits<double>::max();
    int nearest_idx = -1;

    for (size_t i = 0; i < tree.size(); ++i) {
      double dist = distance(tree[i].config, config);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = static_cast<int>(i);
      }
    }

    return nearest_idx;
  }

  // 2つの設定間を補間
  std::vector<double> steer(const std::vector<double> &from,
                            const std::vector<double> &to, double step) const {
    double dist = distance(from, to);

    if (dist <= step) {
      return to;
    }

    std::vector<double> new_config(dof_);
    double ratio = step / dist;

    for (int i = 0; i < dof_; ++i) {
      new_config[i] = from[i] + ratio * (to[i] - from[i]);
    }

    return new_config;
  }

  // 木を拡張
  int extend(std::vector<Node> &tree,
             const std::vector<double> &target_config) {
    int nearest_idx = findNearestNode(tree, target_config);
    if (nearest_idx == -1)
      return -1;

    std::vector<double> new_config =
        steer(tree[nearest_idx].config, target_config, step_size_);

    if (!isValid(new_config)) {
      return -1;
    }

    // エッジの衝突チェック
    if (!isEdgeValid(tree[nearest_idx].config, new_config)) {
      return -1;
    }

    int new_id = tree.size();
    tree.push_back(Node(new_id, new_config, nearest_idx));

    return static_cast<int>(tree.size() -
                            1); // 新しく追加されたノードのインデックスを返す
  }

  // 目標に向けて接続を試行
  int connect(std::vector<Node> &tree,
              const std::vector<double> &target_config) {
    int nearest_idx = findNearestNode(tree, target_config);
    if (nearest_idx == -1)
      return -1;

    std::vector<double> current_config = tree[nearest_idx].config;
    int current_idx = nearest_idx;

    while (true) {
      double dist = distance(current_config, target_config);

      // 目標に到達
      if (dist < step_size_) {
        if (isValid(target_config) &&
            isEdgeValid(current_config, target_config)) {
          int new_id = tree.size();
          tree.push_back(Node(new_id, target_config, current_idx));
          return static_cast<int>(tree.size() - 1);
        }
        return -1;
      }

      // 1ステップ進む
      std::vector<double> new_config =
          steer(current_config, target_config, step_size_);

      if (!isValid(new_config) || !isEdgeValid(current_config, new_config)) {
        return -1;
      }

      int new_id = tree.size();
      tree.push_back(Node(new_id, new_config, current_idx));

      current_config = new_config;
      current_idx = static_cast<int>(tree.size() - 1);
    }
  }

  // エッジの衝突チェック
  bool isEdgeValid(const std::vector<double> &from,
                   const std::vector<double> &to) const {
    if (!collision_checker_)
      return true;

    double dist = distance(from, to);
    int num_checks =
        std::max(2, static_cast<int>(std::ceil(dist / (step_size_ * 0.5))));

    for (int i = 1; i < num_checks; ++i) {
      double ratio = static_cast<double>(i) / num_checks;
      std::vector<double> intermediate(dof_);

      for (int j = 0; j < dof_; ++j) {
        intermediate[j] = from[j] + ratio * (to[j] - from[j]);
      }

      if (!isValid(intermediate)) {
        return false;
      }
    }

    return true;
  }

  // 経路を構築
  void constructPath(int start_connect_idx, int goal_connect_idx,
                     bool start_to_goal) {
    path_.clear();

    std::vector<std::vector<double>> start_path;
    std::vector<std::vector<double>> goal_path;

    if (start_to_goal) {
      // 通常の状態: tree_start_が開始点から、tree_goal_が終了点から
      // Start木から経路を遡る
      int current = start_connect_idx;
      while (current != -1) {
        start_path.push_back(tree_start_[current].config);
        current = tree_start_[current].parent_id;
      }
      std::reverse(start_path.begin(), start_path.end());

      // Goal木から経路を遡る
      current = goal_connect_idx;
      while (current != -1) {
        goal_path.push_back(tree_goal_[current].config);
        current = tree_goal_[current].parent_id;
      }
      // goal_pathは逆順のまま（goalからconnection pointまで）
    } else {
      // 交換された状態: tree_start_が終了点から、tree_goal_が開始点から
      // Goal木（実際は開始点から）から経路を遡る
      int current = goal_connect_idx;
      while (current != -1) {
        start_path.push_back(tree_goal_[current].config);
        current = tree_goal_[current].parent_id;
      }
      std::reverse(start_path.begin(), start_path.end());

      // Start木（実際は終了点から）から経路を遡る
      current = start_connect_idx;
      while (current != -1) {
        goal_path.push_back(tree_start_[current].config);
        current = tree_start_[current].parent_id;
      }
      // goal_pathは逆順のまま
    }

    // 経路を結合（接続点の重複を避ける）
    path_.insert(path_.end(), start_path.begin(), start_path.end());
    if (!goal_path.empty()) {
      path_.insert(path_.end(), goal_path.begin(), goal_path.end());
    }
  }
};

} // namespace RRT
