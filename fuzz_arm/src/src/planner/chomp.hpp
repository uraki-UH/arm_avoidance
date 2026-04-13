#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <vector>

#include "planner/chomp_params.hpp"

namespace CHOMP {

/* CHOMP (Covariant Hamiltonian Optimization for Motion Planning) */
class CHOMPOptimizer {
public:
  // コンストラクタ
  CHOMPOptimizer(int dof, int num_waypoints,
                 const robot_sim::planner::ChompParams &params =
                     robot_sim::planner::ChompParams())
      : dof_(dof), num_waypoints_(num_waypoints), params_(params) {
    // 軌道の初期化
    trajectory_.resize(num_waypoints_, Eigen::VectorXd::Zero(dof_));

    // 有限差分行列の構築
    constructFiniteDifferenceMatrix();
  }

  // 衝突コスト計算関数の設定
  void setObstacleCostFunction(
      std::function<double(const Eigen::VectorXd &)> cost_func) {
    obstacle_cost_function_ = cost_func;
  }

  // 衝突勾配計算関数の設定
  void setObstacleGradientFunction(
      std::function<Eigen::VectorXd(const Eigen::VectorXd &)> grad_func) {
    obstacle_gradient_function_ = grad_func;
  }

  // パラメータ設定
  void setParams(const robot_sim::planner::ChompParams &params) {
    params_ = params;
  }
  void setNumIterations(int iter) { params_.num_iterations = iter; }
  void setLearningRate(double rate) { params_.learning_rate = rate; }
  void setObstacleCostWeight(double weight) {
    params_.obstacle_cost_weight = weight;
  }
  void setSmoothnessCostWeight(double weight) {
    params_.smoothness_cost_weight = weight;
  }
  void setObstacleThreshold(double threshold) {
    params_.obstacle_threshold = threshold;
  }

  // 最適化の実行
  bool optimize(const Eigen::VectorXd &start, const Eigen::VectorXd &goal) {
    if (start.size() != dof_ || goal.size() != dof_) {
      return false;
    }

    // 初期軌道の生成（線形補間）
    initializeTrajectory(start, goal);

    // 反復最適化
    for (int iter = 0; iter < params_.num_iterations; ++iter) {
      // 勾配の計算
      std::vector<Eigen::VectorXd> gradient = computeGradient();

      // 軌道の更新（始点と終点は固定）
      for (int i = 1; i < num_waypoints_ - 1; ++i) {
        trajectory_[i] -= params_.learning_rate * gradient[i];
      }

      // コストの計算（デバッグ用）
      if (iter % 10 == 0) {
        computeTotalCost();
        // std::cout << "Iteration " << iter << ", Cost: " << total_cost <<
        // std::endl;
      }
    }

    return true;
  }

  // 最適化された軌道を取得
  std::vector<Eigen::VectorXd> getTrajectory() const { return trajectory_; }

  // std::vector<std::vector<double>> 形式で軌道を取得
  std::vector<std::vector<double>> getTrajectoryAsVector() const {
    std::vector<std::vector<double>> result;
    for (const auto &waypoint : trajectory_) {
      std::vector<double> config(waypoint.data(),
                                 waypoint.data() + waypoint.size());
      result.push_back(config);
    }
    return result;
  }

  // コストの取得
  double getTotalCost() const { return computeTotalCost(); }

private:
  int dof_;                               // 自由度
  int num_waypoints_;                     // ウェイポイント数
  robot_sim::planner::ChompParams params_; // パラメータ構造体

  std::vector<Eigen::VectorXd> trajectory_; // 軌道

  Eigen::MatrixXd A_; // 有限差分行列
  Eigen::MatrixXd K_; // 平滑化行列 (A^T * A)

  std::function<double(const Eigen::VectorXd &)> obstacle_cost_function_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)>
      obstacle_gradient_function_;

  // 有限差分行列の構築
  void constructFiniteDifferenceMatrix() {
    // 2階微分の有限差分近似
    A_ = Eigen::MatrixXd::Zero(num_waypoints_, num_waypoints_);

    for (int i = 0; i < num_waypoints_; ++i) {
      A_(i, i) = -2.0;
      if (i > 0)
        A_(i, i - 1) = 1.0;
      if (i < num_waypoints_ - 1)
        A_(i, i + 1) = 1.0;
    }

    // 境界条件の調整
    A_(0, 0) = 1.0;
    A_(0, 1) = 0.0;
    A_(num_waypoints_ - 1, num_waypoints_ - 1) = 1.0;
    A_(num_waypoints_ - 1, num_waypoints_ - 2) = 0.0;

    // 平滑化行列 K = A^T * A
    K_ = A_.transpose() * A_;
  }

  // 初期軌道の生成（線形補間）
  void initializeTrajectory(const Eigen::VectorXd &start,
                            const Eigen::VectorXd &goal) {
    trajectory_[0] = start;
    trajectory_[num_waypoints_ - 1] = goal;

    for (int i = 1; i < num_waypoints_ - 1; ++i) {
      double ratio = static_cast<double>(i) / (num_waypoints_ - 1);
      trajectory_[i] = (1.0 - ratio) * start + ratio * goal;
    }
  }

  // 全体コストの計算
  double computeTotalCost() const {
    double smoothness_cost = computeSmoothnessCost();
    double obstacle_cost = computeObstacleCost();

    return params_.smoothness_cost_weight * smoothness_cost +
           params_.obstacle_cost_weight * obstacle_cost;
  }

  // 滑らかさコストの計算
  double computeSmoothnessCost() const {
    double cost = 0.0;

    // 各自由度について
    for (int j = 0; j < dof_; ++j) {
      Eigen::VectorXd q_j(num_waypoints_);
      for (int i = 0; i < num_waypoints_; ++i) {
        q_j(i) = trajectory_[i](j);
      }

      // コスト = q^T * K * q
      cost += q_j.transpose() * K_ * q_j;
    }

    return cost;
  }

  // 障害物コストの計算
  double computeObstacleCost() const {
    if (!obstacle_cost_function_)
      return 0.0;

    double cost = 0.0;
    for (const auto &waypoint : trajectory_) {
      double c = obstacle_cost_function_(waypoint);
      cost += c;
    }

    return cost;
  }

  // 勾配の計算
  std::vector<Eigen::VectorXd> computeGradient() {
    std::vector<Eigen::VectorXd> gradient(num_waypoints_);

    // 滑らかさ勾配
    for (int j = 0; j < dof_; ++j) {
      Eigen::VectorXd q_j(num_waypoints_);
      for (int i = 0; i < num_waypoints_; ++i) {
        q_j(i) = trajectory_[i](j);
      }

      // 勾配 = 2 * K * q
      Eigen::VectorXd grad_j = 2.0 * params_.smoothness_cost_weight * K_ * q_j;

      for (int i = 0; i < num_waypoints_; ++i) {
        if (gradient[i].size() == 0) {
          gradient[i] = Eigen::VectorXd::Zero(dof_);
        }
        gradient[i](j) = grad_j(i);
      }
    }

    // 障害物勾配
    if (obstacle_gradient_function_) {
      for (int i = 0; i < num_waypoints_; ++i) {
        Eigen::VectorXd obs_grad = obstacle_gradient_function_(trajectory_[i]);
        gradient[i] += params_.obstacle_cost_weight * obs_grad;
      }
    } else if (obstacle_cost_function_) {
      // 数値微分で勾配を近似
      for (int i = 0; i < num_waypoints_; ++i) {
        gradient[i] +=
            params_.obstacle_cost_weight * numericalGradient(trajectory_[i]);
      }
    }

    // 始点と終点の勾配はゼロ（固定）
    gradient[0] = Eigen::VectorXd::Zero(dof_);
    gradient[num_waypoints_ - 1] = Eigen::VectorXd::Zero(dof_);

    return gradient;
  }

  // 数値微分による勾配計算
  Eigen::VectorXd numericalGradient(const Eigen::VectorXd &config) const {
    const double epsilon = 1e-5;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dof_);

    for (int i = 0; i < dof_; ++i) {
      Eigen::VectorXd config_plus = config;
      Eigen::VectorXd config_minus = config;

      config_plus(i) += epsilon;
      config_minus(i) -= epsilon;

      double cost_plus = obstacle_cost_function_(config_plus);
      double cost_minus = obstacle_cost_function_(config_minus);

      gradient(i) = (cost_plus - cost_minus) / (2.0 * epsilon);
    }

    return gradient;
  }
};

/* 便利な障害物コスト関数 */
class ObstacleCostFunctions {
public:
  // 球状障害物のコスト関数
  static std::function<double(const Eigen::VectorXd &)>
  sphereObstacle(const Eigen::Vector3d &center, double radius,
                 double epsilon = 0.1) {
    return [center, radius, epsilon](const Eigen::VectorXd &config) -> double {
      // 設定の最初の3次元を位置として使用
      if (config.size() < 3)
        return 0.0;

      Eigen::Vector3d pos = config.head<3>();
      double dist = (pos - center).norm() - radius;

      if (dist > epsilon) {
        return 0.0;
      } else if (dist < 0) {
        return -dist + 0.5 * epsilon;
      } else {
        return 0.5 * std::pow(dist - epsilon, 2) / epsilon;
      }
    };
  }

  // 球状障害物の勾配関数
  static std::function<Eigen::VectorXd(const Eigen::VectorXd &)>
  sphereObstacleGradient(const Eigen::Vector3d &center, double radius,
                         double epsilon = 0.1) {
    return [center, radius,
            epsilon](const Eigen::VectorXd &config) -> Eigen::VectorXd {
      Eigen::VectorXd gradient = Eigen::VectorXd::Zero(config.size());

      if (config.size() < 3)
        return gradient;

      Eigen::Vector3d pos = config.head<3>();
      Eigen::Vector3d diff = pos - center;
      double dist = diff.norm();
      double signed_dist = dist - radius;

      if (signed_dist > epsilon) {
        return gradient;
      }

      Eigen::Vector3d normal = diff / dist;

      if (signed_dist < 0) {
        gradient.head<3>() = -normal;
      } else {
        gradient.head<3>() = -(signed_dist - epsilon) / epsilon * normal;
      }

      return gradient;
    };
  }

  // 複数障害物のコスト関数を結合
  static std::function<double(const Eigen::VectorXd &)> combineObstacleCosts(
      const std::vector<std::function<double(const Eigen::VectorXd &)>>
          &cost_functions) {
    return [cost_functions](const Eigen::VectorXd &config) -> double {
      double total_cost = 0.0;
      for (const auto &func : cost_functions) {
        total_cost += func(config);
      }
      return total_cost;
    };
  }

  // 複数障害物の勾配関数を結合
  static std::function<Eigen::VectorXd(const Eigen::VectorXd &)>
  combineObstacleGradients(
      const std::vector<std::function<Eigen::VectorXd(const Eigen::VectorXd &)>>
          &gradient_functions) {
    return
        [gradient_functions](const Eigen::VectorXd &config) -> Eigen::VectorXd {
          Eigen::VectorXd total_gradient = Eigen::VectorXd::Zero(config.size());
          for (const auto &func : gradient_functions) {
            total_gradient += func(config);
          }
          return total_gradient;
        };
  }
};

} // namespace CHOMP

// CHOMP_HPP

/*
使用例
#include "chomp.hpp"
#include <iostream>

int main() {
    // 3自由度（x, y, z座標）の例
    int dof = 3;
    int num_waypoints = 20;

    CHOMP::CHOMPOptimizer chomp(dof, num_waypoints);

    // 球状障害物の設定
    Eigen::Vector3d obstacle_center(0.5, 0.5, 0.5);
    double obstacle_radius = 0.3;

    auto cost_func = CHOMP::ObstacleCostFunctions::sphereObstacle(
        obstacle_center, obstacle_radius, 0.2);
    auto grad_func = CHOMP::ObstacleCostFunctions::sphereObstacleGradient(
        obstacle_center, obstacle_radius, 0.2);

    chomp.setObstacleCostFunction(cost_func);
    chomp.setObstacleGradientFunction(grad_func);

    // パラメータ設定
    chomp.setNumIterations(100);
    chomp.setLearningRate(0.05);
    chomp.setObstacleCostWeight(10.0);
    chomp.setSmoothnessCostWeight(1.0);

    // 始点と終点
    Eigen::VectorXd start(3);
    start << 0.0, 0.0, 0.0;

    Eigen::VectorXd goal(3);
    goal << 1.0, 1.0, 1.0;

    // 最適化実行
    if (chomp.optimize(start, goal)) {
        std::cout << "最適化が成功しました！" << std::endl;

        auto trajectory = chomp.getTrajectoryAsVector();
        std::cout << "軌道のウェイポイント数: " << trajectory.size() <<
std::endl;

        // 軌道の表示
        for (size_t i = 0; i < trajectory.size(); ++i) {
            std::cout << "Waypoint " << i << ": [";
            for (size_t j = 0; j < trajectory[i].size(); ++j) {
                std::cout << trajectory[i][j];
                if (j < trajectory[i].size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        std::cout << "総コスト: " << chomp.getTotalCost() << std::endl;
    } else {
        std::cout << "最適化が失敗しました。" << std::endl;
    }

    return 0;
}


*/