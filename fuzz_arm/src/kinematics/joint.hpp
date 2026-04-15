// joint.hpp
#pragma once

#include "utility.hpp"
#include <Eigen/Geometry>
#include <iostream>

namespace kinematics {

// 関節の種類
enum class JointType {
    Fixed,      // 0-DOF, 固定
    Revolute,   // 1-DOF, 回転関節
    Prismatic,  // 1-DOF, 直動関節
    Universal,  // 2-DOF, ユニバーサルジョイント
    Spherical   // 3-DOF, ボールジョイント
};

// 関節のパラメータと状態を保持する構造体
struct Joint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    JointType type = JointType::Fixed;
    std::string name = "";

    // --- 静的なパラメータ (ローカル座標系での定義) ---
    // 親リンクの姿勢に対する、この関節の初期姿勢オフセット
    Eigen::Quaterniond local_rotation = Eigen::Quaterniond::Identity();

    // 回転軸の定義
    Eigen::Vector3d axis1 = Eigen::Vector3d::UnitX(); // Revolute, Universal, Spherical の1軸目
    Eigen::Vector3d axis2 = Eigen::Vector3d::UnitY(); // Universal, Spherical の2軸目
    Eigen::Vector3d axis3 = Eigen::Vector3d::UnitZ(); // Spherical の3軸目

    // 直動軸の定義
    Eigen::Vector3d prismatic_axis = Eigen::Vector3d::UnitX();

    // --- 動的なパラメータ (状態) ---
    // 関節変数の値 (角度 or 移動量)
    std::vector<double> values = {0.0, 0.0, 0.0};

    // 可動域制限
    std::vector<double> min_limits;
    std::vector<double> max_limits;

    // --- ヘルパーメソッド ---
    
    /**
     * @brief valuesから現在の関節の相対的な回転を計算する
     * 
     * Spherical の場合:
     * Body-fixed (intrinsic) rotation sequence として実装されています。
     * R = R_axis1(θ1) * R_axis2(θ2) * R_axis3(θ3)
     * 
     * デフォルトでは XYZ Euler angles として動作:
     * - axis1 = X軸 (Roll)
     * - axis2 = Y軸 (Pitch)  
     * - axis3 = Z軸 (Yaw)
     * 
     * 各軸は初期関節座標系で定義されており、Eigen のクォータニオン乗算により
     * 自動的に intrinsic rotation が適用されます。
     * 
     * @return 関節の相対回転 (親リンク座標系基準)
     */
    inline Eigen::Quaterniond getRelativeRotation() const {
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        switch (type) {
            case JointType::Revolute:
                rotation = Eigen::AngleAxisd(values[0], axis1);
                break;
            case JointType::Universal:
                rotation = Eigen::AngleAxisd(values[0], axis1) * 
                          Eigen::AngleAxisd(values[1], axis2);
                break;
            case JointType::Spherical:
                rotation = Eigen::AngleAxisd(values[0], axis1) *
                          Eigen::AngleAxisd(values[1], axis2) *
                          Eigen::AngleAxisd(values[2], axis3);
                break;
            default:
                break;
        }
        return rotation;
    }

    inline Eigen::Vector3d getRelativeTranslation() const {
        if (type == JointType::Prismatic) {
            return prismatic_axis * values[0];
        }
        return Eigen::Vector3d::Zero();
    }

    inline int getDOF() const {
        switch (type) {
            case JointType::Fixed: return 0;
            case JointType::Revolute: return 1;
            case JointType::Prismatic: return 1;
            case JointType::Universal: return 2;
            case JointType::Spherical: return 3;
        }
        return 0;
    }

    /**
     * @brief Spherical ジョイントの軸が正規直交しているか検証
     * 
     * URDF から読み込んだ軸定義が正しいか確認します。
     * 警告が出た場合、Jacobian 計算や IK の精度に影響する可能性があります。
     * 
     * @param tolerance 直交性の許容誤差 (デフォルト: 1e-6)
     * @return true if axes are orthonormal, false otherwise
     */
    inline bool validateSphericalAxes(double tolerance = 1e-6) const {
        if (type != JointType::Spherical) {
            return true;
        }

        // 軸の正規化チェック
        double norm1 = axis1.norm();
        double norm2 = axis2.norm();
        double norm3 = axis3.norm();
        
        if (std::abs(norm1 - 1.0) > tolerance || 
            std::abs(norm2 - 1.0) > tolerance || 
            std::abs(norm3 - 1.0) > tolerance) {
            std::cerr << "Warning: Spherical joint '" << name 
                      << "' has non-normalized axes: "
                      << "||axis1||=" << norm1 
                      << ", ||axis2||=" << norm2 
                      << ", ||axis3||=" << norm3 << std::endl;
            return false;
        }

        // 直交性チェック
        double dot12 = axis1.dot(axis2);
        double dot23 = axis2.dot(axis3);
        double dot31 = axis3.dot(axis1);
        
        if (std::abs(dot12) > tolerance || 
            std::abs(dot23) > tolerance || 
            std::abs(dot31) > tolerance) {
            std::cerr << "Warning: Spherical joint '" << name 
                      << "' has non-orthogonal axes: "
                      << "axis1·axis2=" << dot12 
                      << ", axis2·axis3=" << dot23 
                      << ", axis3·axis1=" << dot31 << std::endl;
            return false;
        }

        return true;
    }

    /**
     * @brief Spherical ジョイントの軸を正規直交化
     * 
     * URDF の定義が不正確な場合に使用します。
     * Gram-Schmidt 法で axis1 を基準に axis2, axis3 を再計算します。
     */
    inline void normalizeSphericalAxes() {
        if (type != JointType::Spherical) {
            return;
        }

        axis1.normalize();
        axis2 = (axis2 - axis2.dot(axis1) * axis1).normalized();
        axis3 = axis1.cross(axis2).normalized();
    }

    /**
     * @brief Spherical ジョイントの軸をデフォルト (XYZ Euler) に設定
     * 
     * URDF の ball joint から変換する際に使用します。
     * XYZ Euler angles として動作:
     * - axis1 = X軸 (Roll)
     * - axis2 = Y軸 (Pitch)
     * - axis3 = Z軸 (Yaw)
     */
    inline void setDefaultSphericalAxes() {
        if (type != JointType::Spherical) {
            return;
        }
        axis1 = Eigen::Vector3d::UnitX();
        axis2 = Eigen::Vector3d::UnitY();
        axis3 = Eigen::Vector3d::UnitZ();
    }

    /**
     * @brief 指定した1つの軸から残り2軸を自動生成
     * 
     * URDF で単一軸が定義されている場合に使用します。
     * Gram-Schmidt 法で直交する軸を生成します。
     * 
     * @param primary_axis 主軸 (正規化されている必要がある)
     */
    inline void setSphericalAxesFromPrimary(const Eigen::Vector3d& primary_axis) {
        if (type != JointType::Spherical) {
            return;
        }
        
        axis1 = primary_axis.normalized();
        
        // axis1 に直交する任意のベクトルを生成
        Eigen::Vector3d temp = (std::abs(axis1.x()) > 0.9) 
                               ? Eigen::Vector3d::UnitY() 
                               : Eigen::Vector3d::UnitX();
        axis2 = (temp - temp.dot(axis1) * axis1).normalized();
        axis3 = axis1.cross(axis2).normalized();
    }
};

} // namespace kinematics