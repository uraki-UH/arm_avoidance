#pragma once

#include "status/manipulability.hpp"
#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <vector>

namespace GNG {

/**
 * @brief ステータス更新のタイミングを制御するトリガー
 */
enum class UpdateTrigger {
  NODE_ADDED,
  COORD_UPDATED,
  TIME_PERIODIC,
  BATCH_UPDATE,
  MANUAL
};

/**
 * @brief GNGノードに付与されるステータス情報。
 * コアロジックから分離され、物理指標（可操作性等）を一括管理する。
 */
struct Status {
  int level = 0;           // 階層レベル (0 = base/finest)
  bool is_surface = false; // 表面ノードフラグ (デフォルト: false, 更新で設定)
  bool is_active_surface =
      false;                // 有効な表面ノード (デフォルト: false, 更新で設定)
  bool is_boundary = false; // 境界ノードフラグ
  bool valid = true;        // 全体的な有効性
  bool active = true;       // アクティブフラグ

  // --- Topological Status ---
  bool is_mainland = true;    // True: 本土 (最大の安全な連結成分)に属する
  int topology_group_id = -1; // -1: 衝突/無効, >= 0: 所属する連結成分のID

  // --- Dynamic Collision Status ---
  int collision_count = 0;   // Count of occupied voxels that are colliding
  bool is_colliding = false; // True if collision_count > 0
  int danger_count = 0;   // Count of occupied/nearby voxels that are dangerous
  bool is_danger = false; // True if danger_count > 0

  // 手先方向ベクトル
  Eigen::Vector3f ee_direction = Eigen::Vector3f::UnitX();

  // --- 物理指標 (include/status/ 内のライブラリを利用) ---

  // 静的（運動学的）可操作性
  Manipulability::ManipulabilityEllipsoid manip_info;

  // 以前の ManipulabilityInfo にあった追加スコア（互換性のため）
  float min_singular_value = 0.0f;
  float joint_limit_score = 1.0f;
  float combined_score = 0.0f;
  float dynamic_manipulability = 0.0f;

  // 動的可操作性
  Manipulability::ManipulabilityEllipsoid dynamic_manip_info;

  // --- その他 ---
  std::vector<Eigen::Vector3f> joint_positions;    // 全関節位置
  std::unordered_map<std::string, float> metadata; // 汎用メタデータ

  Status() {
    manip_info.valid = false;
    dynamic_manip_info.valid = false;
  }
};

/**
 * @brief GNGノードの基本クラス
 */
template <typename T_angle, typename T_coord> class NeuronNode {
public:
  int id = -1;
  float error_angle = 0.0f;
  float error_coord = 0.0f;
  T_angle weight_angle;
  T_coord weight_coord;
  Status status;
  NeuronNode() {};
  NeuronNode(int id_, T_angle w_angle, T_coord w_coord)
      : id(id_), weight_angle(w_angle), weight_coord(w_coord) {};
  ~NeuronNode() {};
};

/**
 * @brief GNGエッジの属性情報
 */
struct EdgeInfo {
  int age = 0;             // GNG aging
  float weight = 1.0f;     // For planning (distance/cost)
  float confidence = 1.0f; // For topological reliability
  bool active = true;      // Active state for pruning
  bool exists() const { return age > 0; }
};

} // namespace GNG
