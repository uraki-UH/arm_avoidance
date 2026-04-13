#pragma once

#include <Eigen/Dense>
#include <vector>

namespace Manipulability {

/**
 * @brief 可操作性のタイプ
 */
enum ManipulabilityType {
  KINEMATIC, // 運動学的可操作性
  DYNAMIC,   // 動的可操作性
  PENALIZED  // ペナルティ付き可操作性
};

/**
 * @brief 関節の情報を格納する構造体
 */
struct JointInfo {
  Eigen::Vector3d position; // 関節位置
  Eigen::Vector3d axis;     // 関節軸（正規化済み）

  JointInfo()
      : position(Eigen::Vector3d::Zero()), axis(Eigen::Vector3d::UnitZ()) {}
  JointInfo(const Eigen::Vector3d &pos, const Eigen::Vector3d &ax)
      : position(pos), axis(ax.normalized()) {}
};

/**
 * @brief 可操作性楕円体の情報
 */
struct ManipulabilityEllipsoid {
  Eigen::Vector3d center;               // 中心位置
  Eigen::Vector3d singular_values;      // 特異値（3つ）
  Eigen::Matrix3d principal_directions; // 主軸方向（U行列）
  double manipulability;                // 可操作性尺度
  double condition_number;              // 条件数
  double min_singular_value;            // 最小特異値
  double volume;                        // 楕円体の体積
  bool valid;                           // 有効かどうか

  // 方向ごとの動きやすさ (X, Y, Z)
  double manip_x;
  double manip_y;
  double manip_z;

  ManipulabilityEllipsoid()
      : center(Eigen::Vector3d::Zero()),
        singular_values(Eigen::Vector3d::Zero()),
        principal_directions(Eigen::Matrix3d::Identity()), manipulability(0.0),
        condition_number(0.0), min_singular_value(0.0), volume(0.0),
        valid(false), manip_x(0.0), manip_y(0.0), manip_z(0.0) {}
};

/**
 * @brief ヤコビ行列を計算
 * @param joints 関節情報のリスト
 * @param end_effector_pos 手先位置
 * @return ヤコビ行列 (3 x n_joints)
 */
Eigen::MatrixXd calculateJacobian(const std::vector<JointInfo> &joints,
                                  const Eigen::Vector3d &end_effector_pos);

/**
 * @brief 部分ヤコビ行列を計算（特定関節までの寄与）
 * @param joints 関節情報のリスト
 * @param target_joint_index 対象関節のインデックス
 * @param target_pos 対象位置（通常はその関節位置）
 * @return 部分ヤコビ行列
 */
Eigen::MatrixXd calculatePartialJacobian(const std::vector<JointInfo> &joints,
                                         int target_joint_index,
                                         const Eigen::Vector3d &target_pos);

/**
 * @brief 可操作性楕円体を計算
 * @param J ヤコビ行列
 * @param type 可操作性のタイプ
 * @return 可操作性楕円体の情報
 */
ManipulabilityEllipsoid
calculateManipulabilityEllipsoid(const Eigen::MatrixXd &J,
                                 ManipulabilityType type = KINEMATIC,
                                 double scale_factor = 1.0);

/**
 * @brief 可操作性尺度を計算
 * @param J ヤコビ行列
 * @param type 可操作性のタイプ
 * @return 可操作性尺度（スカラー値）
 */
double calculateManipulabilityMeasure(const Eigen::MatrixXd &J,
                                      ManipulabilityType type = KINEMATIC);

/**
 * @brief 特定方向の可操作性を評価
 * @param ellipsoid 可操作性楕円体
 * @param direction 評価する方向ベクトル
 * @return 方向可操作性（その方向への動きやすさ）
 */
double
evaluateDirectionalManipulability(const ManipulabilityEllipsoid &ellipsoid,
                                  const Eigen::Vector3d &direction);

/**
 * @brief 条件数を計算（特異姿勢の判定に使用）
 * @param J ヤコビ行列
 * @return 条件数（大きいほど特異姿勢に近い）
 */
double calculateConditionNumber(const Eigen::MatrixXd &J);

/**
 * @brief 動的可操作性楕円体を計算
 * @param J ヤコビ行列
 * @param M 慣性行列 (n x n)
 * @return 動的可操作性楕円体
 */
ManipulabilityEllipsoid
calculateDynamicManipulabilityEllipsoid(const Eigen::MatrixXd &J,
                                        const Eigen::MatrixXd &M,
                                        double scale_factor = 1.0);

/**
 * @brief 各関節位置での可操作性を計算
 * @param joints 全関節情報
 * @return 各関節での可操作性楕円体のリスト
 */
std::vector<ManipulabilityEllipsoid>
calculateAllJointManipulabilities(const std::vector<JointInfo> &joints,
                                  double scale_factor = 1.0);

/**
 * @brief ヤコビ行列の零空間を計算（冗長自由度の解析用）
 * @param J ヤコビ行列
 * @param tolerance 特異値の閾値
 * @return 零空間の基底ベクトル
 */
Eigen::MatrixXd calculateNullSpace(const Eigen::MatrixXd &J,
                                   double tolerance = 1e-6);

/**
 * @brief 特異値分解の結果を取得
 * @param J ヤコビ行列
 * @param U 左特異ベクトル（出力）
 * @param S 特異値（出力）
 * @param V 右特異ベクトル（出力）
 */
void computeSVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &U,
                Eigen::VectorXd &S, Eigen::MatrixXd &V);

/**
 * @brief ヤコビ行列から全ての可操作性指標を計算
 * @param J ヤコビ行列(3xN)
 * @return 拡張された楕円体情報
 */
ManipulabilityEllipsoid calculateFullMetrics(const Eigen::MatrixXd &J);

} // namespace Manipulability
