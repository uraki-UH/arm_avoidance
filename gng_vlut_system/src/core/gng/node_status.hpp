#pragma once

#include "metrics/manipulability.hpp"
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
  // NOTE: is_surface / is_active_surface は現状どこからも利用しておらず、
  // 当面使う予定もない。将来の想定用途は複数ロボットの作業空間の重なり判定。
  // 各ロボットの到達領域の表面を取れば、それらが重なるエリアが
  // 協調動作を要する領域として抽出できる。
  bool is_surface = false; // 表面ノードフラグ (デフォルト: false)
  bool is_active_surface =
      false;                // 有効な表面ノード (デフォルト: false, 更新で設定)
  bool is_boundary = false; // 境界ノードフラグ

  // 静的な自己干渉の判定結果。オフライン学習時に自己干渉プロバイダが確定させる。
  // 環境障害物とは無関係であり、実行時に書き換えてはならない。
  // 旧名は valid。「全体的な有効性」という曖昧な名前だったため改称した。
  // NOTE: 現状 topological_map_avoidance_node が環境障害物の反映時にここへ
  // 書き込んでおり、責務が混線している。詳細は TASK_CANDIDATES.md を参照。
  bool self_collision_free = true;

  // 構造上ノードが存在するか。GNG 管理側(島の刈り込み等)が設定する。
  // 自己干渉や環境衝突とは別軸であり、統合してはならない。
  bool active = true;

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
  // 手先姿勢 (回転)
  Eigen::Quaternionf ee_orientation = Eigen::Quaternionf::Identity();

  // --- 物理指標 (include/metrics/ 内のライブラリを利用) ---

  // 静的（運動学的）可操作性
  Manipulability::ManipulabilityEllipsoid manip_info;

  // 以前の ManipulabilityInfo にあった追加指標
  float min_singular_value = 0.0f;
  float joint_limit_score = 1.0f;
  float dynamic_manipulability = 0.0f;

  // 動的可操作性
  Manipulability::ManipulabilityEllipsoid dynamic_manip_info;

  // 回転可操作性
  Manipulability::ManipulabilityEllipsoid rotational_manip_info;

  // --- その他 ---
  std::unordered_map<std::string, float> metadata; // 汎用メタデータ

  Status() {
    manip_info.valid = false;
    dynamic_manip_info.valid = false;
    rotational_manip_info.valid = false;
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
  float task_density_ema = 1.0f;
  T_angle weight_angle;
  T_coord weight_coord;
  std::vector<T_coord> weight_coords;
  Status status;
  NeuronNode() {};
  NeuronNode(int id_, T_angle w_angle, T_coord w_coord)
      : id(id_), weight_angle(w_angle), weight_coord(w_coord),
        weight_coords{w_coord} {};
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
