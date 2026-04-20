#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <random>
#include <vector>

#include "planner/stomp_params.hpp"

namespace STOMP {

/* STOMP (Stochastic Trajectory Optimization for Motion Planning) */
class STOMPOptimizer {
public:
  // コンストラクタ
  STOMPOptimizer(int dof, int num_timesteps,
                 const robot_sim::planner::StompParams &params =
                     robot_sim::planner::StompParams())
      : dof_(dof), num_timesteps_(num_timesteps), params_(params),
        generator_(std::random_device()()) {
    // 軌道の初期化
    trajectory_ = Eigen::MatrixXd::Zero(num_timesteps_, dof_);

    // ノイズ生成器の初期化
    noise_generator_ = std::make_shared<NoiseGenerator>(dof_, num_timesteps_);
  }

  // コスト関数の設定（軌道全体のコスト）
  void
  setCostFunction(std::function<double(const Eigen::MatrixXd &)> cost_func) {
    trajectory_cost_function_ = cost_func;
  }

  // 状態ごとのコスト関数の設定
  void setStateCostFunction(
      std::function<double(const Eigen::VectorXd &, int)> cost_func) {
    state_cost_function_ = cost_func;
  }

  // パラメータ設定
  void setParams(const robot_sim::planner::StompParams &params) {
    params_ = params;
  }
  void setNumIterations(int iter) { params_.max_iterations = iter; }
  void setNumRollouts(int rollouts) { params_.num_rollouts = rollouts; }
  void setNumReusedRollouts(int reused) {
    params_.num_reused_rollouts = reused;
  }
  void setControlCostWeight(double weight) {
    params_.control_cost_weight = weight;
  }
  void setNoiseStddev(double stddev) { params_.noise_stddev = stddev; }
  void setNoiseDecay(double decay) { params_.noise_decay = decay; }
  void setTemperature(double temp) { params_.temperature = temp; }

  // 最適化の実行
  bool optimize(const Eigen::VectorXd &start, const Eigen::VectorXd &goal) {
    if (start.size() != dof_ || goal.size() != dof_) {
      return false;
    }

    // 初期軌道の生成（線形補間）
    initializeTrajectory(start, goal);

    // 前回のロールアウトを保存
    std::vector<Rollout> previous_rollouts;

    // 反復最適化
    for (int iter = 0; iter < params_.max_iterations; ++iter) {
      // ロールアウトの生成
      std::vector<Rollout> rollouts = generateRollouts(previous_rollouts);

      // コストの計算
      computeCosts(rollouts);

      // 重みの計算
      std::vector<double> weights = computeWeights(rollouts);

      // 軌道の更新
      updateTrajectory(rollouts, weights);

      // 始点と終点を固定
      trajectory_.row(0) = start;
      trajectory_.row(num_timesteps_ - 1) = goal;

      // 再利用するロールアウトを保存
      previous_rollouts =
          selectBestRollouts(rollouts, params_.num_reused_rollouts);

      // ノイズの減衰
      params_.noise_stddev = std::max(
          params_.min_noise_stddev, params_.noise_stddev * params_.noise_decay);

      // デバッグ出力
      if (iter % 10 == 0) {
        double best_cost = rollouts[0].total_cost;
        for (const auto &r : rollouts) {
          if (r.total_cost < best_cost)
            best_cost = r.total_cost;
        }
        // std::cout << "Iteration " << iter << ", Best Cost: " << best_cost <<
        // std::endl;
      }
    }

    return true;
  }

  // 最適化された軌道を取得
  Eigen::MatrixXd getTrajectory() const { return trajectory_; }

  // std::vector形式で軌道を取得
  std::vector<std::vector<double>> getTrajectoryAsVector() const {
    std::vector<std::vector<double>> result;
    for (int i = 0; i < num_timesteps_; ++i) {
      std::vector<double> state(dof_);
      for (int j = 0; j < dof_; ++j) {
        state[j] = trajectory_(i, j);
      }
      result.push_back(state);
    }
    return result;
  }

private:
  int dof_;                               // 自由度
  int num_timesteps_;                     // タイムステップ数
  robot_sim::planner::StompParams params_; // パラメータ構造体

  Eigen::MatrixXd trajectory_; // 現在の軌道 (num_timesteps x dof)

  std::function<double(const Eigen::MatrixXd &)> trajectory_cost_function_;
  std::function<double(const Eigen::VectorXd &, int)> state_cost_function_;

  std::mt19937 generator_;

  // ロールアウト構造体
  struct Rollout {
    Eigen::MatrixXd trajectory;
    Eigen::MatrixXd noise;
    std::vector<double> state_costs;
    double control_cost;
    double total_cost;
  };

  // ノイズ生成器
  class NoiseGenerator {
  public:
    NoiseGenerator(int dof, int num_timesteps)
        : dof_(dof), num_timesteps_(num_timesteps) {
      constructDifferenceMatrix();
    }

    Eigen::MatrixXd generate(std::mt19937 &gen, double stddev) {
      std::normal_distribution<double> dist(0.0, stddev);

      Eigen::MatrixXd noise(num_timesteps_, dof_);
      for (int i = 0; i < num_timesteps_; ++i) {
        for (int j = 0; j < dof_; ++j) {
          noise(i, j) = dist(gen);
        }
      }

      // 相関ノイズの生成（滑らかなノイズ）
      Eigen::MatrixXd smooth_noise = R_inv_ * noise;

      // 始点と終点のノイズをゼロに
      smooth_noise.row(0).setZero();
      smooth_noise.row(num_timesteps_ - 1).setZero();

      return smooth_noise;
    }

  private:
    int dof_;
    int num_timesteps_;
    Eigen::MatrixXd R_;     // 相関行列
    Eigen::MatrixXd R_inv_; // 相関行列の逆行列

    void constructDifferenceMatrix() {
      // 有限差分行列の構築
      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_timesteps_, num_timesteps_);

      for (int i = 0; i < num_timesteps_; ++i) {
        A(i, i) = -2.0;
        if (i > 0)
          A(i, i - 1) = 1.0;
        if (i < num_timesteps_ - 1)
          A(i, i + 1) = 1.0;
      }

      // 境界条件
      A(0, 0) = 1.0;
      A(0, 1) = 0.0;
      A(num_timesteps_ - 1, num_timesteps_ - 1) = 1.0;
      A(num_timesteps_ - 1, num_timesteps_ - 2) = 0.0;

      // 相関行列 R = A^T * A
      R_ = A.transpose() * A;

      // 正則化
      R_ += 1e-6 * Eigen::MatrixXd::Identity(num_timesteps_, num_timesteps_);

      // コレスキー分解で逆行列を計算
      R_inv_ = R_.llt().solve(
          Eigen::MatrixXd::Identity(num_timesteps_, num_timesteps_));
    }
  };

  std::shared_ptr<NoiseGenerator> noise_generator_;

  // 初期軌道の生成（線形補間）
  void initializeTrajectory(const Eigen::VectorXd &start,
                            const Eigen::VectorXd &goal) {
    for (int i = 0; i < num_timesteps_; ++i) {
      double ratio = static_cast<double>(i) / (num_timesteps_ - 1);
      trajectory_.row(i) = (1.0 - ratio) * start + ratio * goal;
    }
  }

  // ロールアウトの生成
  std::vector<Rollout>
  generateRollouts(const std::vector<Rollout> &previous_rollouts) {
    std::vector<Rollout> rollouts;

    // ノイズなしのロールアウト（現在の軌道）
    Rollout noiseless;
    noiseless.trajectory = trajectory_;
    noiseless.noise = Eigen::MatrixXd::Zero(num_timesteps_, dof_);
    rollouts.push_back(noiseless);

    // 新しいロールアウトの生成
    int num_new = params_.num_rollouts - 1 - previous_rollouts.size();
    for (int i = 0; i < num_new; ++i) {
      Rollout rollout;
      rollout.noise =
          noise_generator_->generate(generator_, params_.noise_stddev);
      rollout.trajectory = trajectory_ + rollout.noise;
      rollouts.push_back(rollout);
    }

    // 前回の良いロールアウトを再利用
    for (const auto &prev : previous_rollouts) {
      Rollout rollout;
      // 前回のノイズを少し減衰させて再利用
      rollout.noise = prev.noise * params_.noise_decay;
      rollout.trajectory = trajectory_ + rollout.noise;
      rollouts.push_back(rollout);
    }

    return rollouts;
  }

  // コストの計算
  void computeCosts(std::vector<Rollout> &rollouts) {
    for (auto &rollout : rollouts) {
      rollout.state_costs.clear();
      rollout.state_costs.resize(num_timesteps_, 0.0);

      // 状態コストの計算
      if (state_cost_function_) {
        for (int t = 0; t < num_timesteps_; ++t) {
          rollout.state_costs[t] =
              state_cost_function_(rollout.trajectory.row(t), t);
        }
      } else if (trajectory_cost_function_) {
        double total_traj_cost = trajectory_cost_function_(rollout.trajectory);
        double cost_per_state = total_traj_cost / num_timesteps_;
        for (int t = 0; t < num_timesteps_; ++t) {
          rollout.state_costs[t] = cost_per_state;
        }
      }

      // 制御コストの計算（加速度の2乗）
      rollout.control_cost = 0.0;
      for (int j = 0; j < dof_; ++j) {
        for (int t = 1; t < num_timesteps_ - 1; ++t) {
          double accel = rollout.trajectory(t + 1, j) -
                         2.0 * rollout.trajectory(t, j) +
                         rollout.trajectory(t - 1, j);
          rollout.control_cost += accel * accel;
        }
      }

      // 総コスト
      double state_cost_sum = 0.0;
      for (double c : rollout.state_costs) {
        state_cost_sum += c;
      }
      rollout.total_cost =
          state_cost_sum + params_.control_cost_weight * rollout.control_cost;
    }
  }

  // 重みの計算（ソフトマックス）
  std::vector<double> computeWeights(const std::vector<Rollout> &rollouts) {
    std::vector<double> weights(rollouts.size());

    // タイムステップごとの重み
    std::vector<std::vector<double>> time_weights(num_timesteps_);

    for (int t = 0; t < num_timesteps_; ++t) {
      std::vector<double> costs;
      for (const auto &rollout : rollouts) {
        costs.push_back(rollout.state_costs[t]);
      }

      // ソフトマックスで重みを計算
      time_weights[t] = computeSoftmax(costs);
    }

    // 全体の重みは各タイムステップの重みの平均
    for (size_t i = 0; i < rollouts.size(); ++i) {
      double sum = 0.0;
      for (int t = 0; t < num_timesteps_; ++t) {
        sum += time_weights[t][i];
      }
      weights[i] = sum / num_timesteps_;
    }

    // 正規化
    double sum = 0.0;
    for (double w : weights)
      sum += w;
    if (sum > 0) {
      for (double &w : weights)
        w /= sum;
    }

    return weights;
  }

  // ソフトマックスの計算
  std::vector<double> computeSoftmax(const std::vector<double> &costs) {
    std::vector<double> weights(costs.size());

    // 最小コストを見つける
    double min_cost = *std::min_element(costs.begin(), costs.end());

    // exp(-cost/temperature)を計算
    double sum = 0.0;
    for (size_t i = 0; i < costs.size(); ++i) {
      weights[i] = std::exp(-(costs[i] - min_cost) / params_.temperature);
      sum += weights[i];
    }

    // 正規化
    if (sum > 0) {
      for (double &w : weights)
        w /= sum;
    }

    return weights;
  }

  // 軌道の更新
  void updateTrajectory(const std::vector<Rollout> &rollouts,
                        const std::vector<double> &weights) {
    Eigen::MatrixXd noise_update = Eigen::MatrixXd::Zero(num_timesteps_, dof_);

    // 重み付きノイズの和
    for (size_t i = 0; i < rollouts.size(); ++i) {
      noise_update += weights[i] * rollouts[i].noise;
    }

    // 軌道を更新
    trajectory_ += noise_update;
  }

  // 最良のロールアウトを選択
  std::vector<Rollout> selectBestRollouts(const std::vector<Rollout> &rollouts,
                                          int num_best) {
    std::vector<Rollout> sorted_rollouts = rollouts;

    std::sort(sorted_rollouts.begin(), sorted_rollouts.end(),
              [](const Rollout &a, const Rollout &b) {
                return a.total_cost < b.total_cost;
              });

    std::vector<Rollout> best_rollouts;
    for (int i = 0;
         i < std::min(num_best, static_cast<int>(sorted_rollouts.size()));
         ++i) {
      best_rollouts.push_back(sorted_rollouts[i]);
    }

    return best_rollouts;
  }
};

/* 便利なコスト関数 */
class CostFunctions {
public:
  // 球状障害物のコスト関数
  static std::function<double(const Eigen::VectorXd &, int)>
  sphereObstacle(const Eigen::Vector3d &center, double radius,
                 double max_cost = 1000.0) {
    return [center, radius, max_cost](const Eigen::VectorXd &state,
                                      int t) -> double {
      if (state.size() < 3)
        return 0.0;

      Eigen::Vector3d pos = state.head<3>();
      double dist = (pos - center).norm() - radius;

      if (dist < 0) {
        return max_cost;
      } else if (dist < radius * 0.5) {
        return max_cost * std::exp(-dist / (radius * 0.1));
      } else {
        return 0.0;
      }
    };
  }

  // 複数障害物のコスト結合
  static std::function<double(const Eigen::VectorXd &, int)> combineStateCosts(
      const std::vector<std::function<double(const Eigen::VectorXd &, int)>>
          &cost_functions) {
    return [cost_functions](const Eigen::VectorXd &state, int t) -> double {
      double total_cost = 0.0;
      for (const auto &func : cost_functions) {
        total_cost += func(state, t);
      }
      return total_cost;
    };
  }
};

} // namespace STOMP

// STOMP_HPP

/*
使用例

#include "stomp.hpp"
#include <iostream>

int main() {
    // 3自由度（x, y, z座標）の例
    int dof = 3;
    int num_timesteps = 50;

    STOMP::STOMPOptimizer stomp(dof, num_timesteps);

    // 球状障害物のコスト関数
    Eigen::Vector3d obstacle_center(0.5, 0.5, 0.5);
    double obstacle_radius = 0.2;

    auto cost_func = STOMP::CostFunctions::sphereObstacle(obstacle_center,
obstacle_radius, 100.0); stomp.setStateCostFunction(cost_func);

    // パラメータ設定
    stomp.setNumIterations(100);
    stomp.setNumRollouts(10);
    stomp.setNumReusedRollouts(5);
    stomp.setControlCostWeight(0.1);
    stomp.setNoiseStddev(0.5);
    stomp.setNoiseDecay(0.95);
    stomp.setTemperature(0.1);

    // 始点と終点
    Eigen::VectorXd start(3);
    start << 0.0, 0.0, 0.0;

    Eigen::VectorXd goal(3);
    goal << 1.0, 1.0, 1.0;

    // 最適化実行
    if (stomp.optimize(start, goal)) {
        std::cout << "最適化が成功しました！" << std::endl;

        auto trajectory = stomp.getTrajectoryAsVector();
        std::cout << "軌道のタイムステップ数: " << trajectory.size() <<
std::endl;

        // 軌道の一部を表示
        for (size_t i = 0; i < std::min(size_t(5), trajectory.size()); ++i) {
            std::cout << "Timestep " << i << ": [";
            for (size_t j = 0; j < trajectory[i].size(); ++j) {
                std::cout << trajectory[i][j];
                if (j < trajectory[i].size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    } else {
        std::cout << "最適化が失敗しました。" << std::endl;
    }

    return 0;
}

===============================================

複数障害物の例
// 複数の障害物
std::vector<std::function<double(const Eigen::VectorXd&, int)>> cost_funcs;

cost_funcs.push_back(STOMP::CostFunctions::sphereObstacle(
    Eigen::Vector3d(0.3, 0.3, 0.3), 0.15, 100.0));
cost_funcs.push_back(STOMP::CostFunctions::sphereObstacle(
    Eigen::Vector3d(0.7, 0.7, 0.7), 0.2, 100.0));

auto combined_cost = STOMP::CostFunctions::combineStateCosts(cost_funcs);
stomp.setStateCostFunction(combined_cost);

================================================




*/