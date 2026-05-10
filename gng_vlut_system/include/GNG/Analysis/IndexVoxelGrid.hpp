#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <unordered_map>
#include <vector>
#include "common/constants.hpp"

namespace GNG {
namespace Analysis {

/**
 * @brief ボクセルインデックスとフラットIDの相互変換を管理するクラス。
 * ビットパッキングのパラメータ（シフト量、オフセット）を動的に保持し、
 * マジックナンバーを排除します。
 */
class IndexVoxelGrid {
public:
  IndexVoxelGrid(double voxel_size) : voxel_size_(voxel_size) {}

  /**
   * @brief インデックスパラメータの一括設定
   */
  void setIndexingParams(int x, int y, int z, long off) {
    x_shift_ = x;
    y_shift_ = y;
    z_shift_ = z;
    offset_ = off;
  }

  void setVoxelSize(double size) { voxel_size_ = size; }

  double getVoxelSize() const { return voxel_size_; }

  /**
   * @brief 3Dインデックスをフラットな1D IDに変換（パッキング）
   */
  long getFlatVoxelId(const Eigen::Vector3i &idx) const {
    return ((long)idx.x() + offset_) << x_shift_ |
           ((long)idx.y() + offset_) << y_shift_ |
           ((long)idx.z() + offset_) << z_shift_;
  }

  /**
   * @brief フラットな1D IDを3Dインデックスに復元（アンパッキング）
   */
  Eigen::Vector3i getIndexFromFlatId(long id) const {
    long mask = (1L << y_shift_) - 1; // y_shiftがx, y, zの間隔として使われている前提
    // 注: 本来は各軸のビット幅に応じたマスクが必要ですが、
    // 現在の設計（21bitずつ）に合わせ、上位ビットをシフトで落としつつマスクします。
    long x = (id >> x_shift_) - offset_;
    long y = ((id >> y_shift_) & mask) - offset_;
    long z = ((id >> z_shift_) & mask) - offset_;
    return Eigen::Vector3i((int)x, (int)y, (int)z);
  }

  int getXShift() const { return x_shift_; }
  int getYShift() const { return y_shift_; }
  int getZShift() const { return z_shift_; }
  long getOffset() const { return offset_; }

private:
  double voxel_size_; // 必須: コンストラクタで初期化

  // デフォルト値（21bit packing: 2^21 = 2,097,152）
  int x_shift_ = ::robot_sim::common::Constants::DEFAULT_X_SHIFT;
  int y_shift_ = ::robot_sim::common::Constants::DEFAULT_Y_SHIFT;
  int z_shift_ = ::robot_sim::common::Constants::DEFAULT_Z_SHIFT;
  long offset_ = ::robot_sim::common::Constants::DEFAULT_OFFSET;

  // LUTとしての役割（将来的に使用）
  std::unordered_map<long, std::vector<int>> voxel_to_ids_;
  std::unordered_map<int, std::vector<long>> id_to_voxels_;
};

} // namespace Analysis
} // namespace GNG
