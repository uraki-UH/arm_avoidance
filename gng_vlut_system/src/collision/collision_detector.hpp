#pragma once

#include <Eigen/Dense>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

namespace collision {

struct AABB {
  Eigen::Vector3d min, max;

  AABB()
      : min(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
        max(Eigen::Vector3d::Constant(-std::numeric_limits<double>::max())) {}

  AABB(const Eigen::Vector3d &min_pt, const Eigen::Vector3d &max_pt)
      : min(min_pt), max(max_pt) {}

  void expand(const Eigen::Vector3d &point) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  void expand(const AABB &other) {
    min = min.cwiseMin(other.min);
    max = max.cwiseMax(other.max);
  }

  bool intersects(const AABB &other) const {
    return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
           (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
           (min.z() <= other.max.z() && max.z() >= other.min.z());
  }

  double surfaceArea() const {
    Eigen::Vector3d d = max - min;
    return 2.0 * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
  }
};

// ============================================================================
// 1. データ構造 (出力)
// ============================================================================

struct Contact {
  bool colliding = false;      // 衝突しているかどうか
  double depth = 0.0;          // 貫通深度 (衝突している場合は正の値)
  Eigen::Vector3d pos_a;       // 形状Aの表面上の接触点
  Eigen::Vector3d pos_b;       // 形状Bの表面上の接触点
  Eigen::Vector3d normal;      // 接触法線 (AからBへの方向)
  double link_parameter = 0.0; // カプセルやエッジ上の位置 [0, 1]
  int primitive_id = -1;       // サブプリミティブID (例: メッシュ内の三角形ID)
};

/**
 * @brief 距離情報を表し、符号付き距離場 (SDF) をサポート。
 */
struct DistanceInfo {
  double distance =
      std::numeric_limits<double>::max(); // 重なっている場合は負の値
  Eigen::Vector3d nearest_a;              // 形状Aの最近傍点
  Eigen::Vector3d nearest_b;              // 形状Bの最近傍点
};

/**
 * @brief リプシッツ連続性を使用した経路検証の結果を表す。
 */
struct EdgeValidationResult {
  bool is_safe = true;   // 経路が安全かどうか
  double unsafe_t = 1.0; // 安全性が保証されない、または衝突が発生した時間 (0-1)
  Contact witness_contact; // 失敗時の接触情報
};

// ============================================================================
// 2. プリミティブ (入力)
// ============================================================================

struct Sphere {
  Eigen::Vector3d center; // 球の中心
  double radius;          // 球の半径
};

struct Capsule {
  Eigen::Vector3d a; // カプセルの端点A
  Eigen::Vector3d b; // カプセルの端点B
  double radius;     // カプセルの半径
};

struct Box {
  Eigen::Vector3d center;   // ボックスの中心
  Eigen::Matrix3d rotation; // ボックスの回転行列 (ローカル軸)
  Eigen::Vector3d extents;  // 各軸方向の半分の長さ (half-extents)
};

struct Triangle {
  Eigen::Vector3d v0, v1, v2; // 三角形の頂点
};

struct Point {
  Eigen::Vector3d p; // 点の座標
};

// 前方宣言
class AABBTree;

struct Mesh {
  std::vector<Triangle> triangles; // メッシュを構成する三角形のリスト
  std::shared_ptr<AABBTree> bvh;   // 境界ボリューム階層 (高速衝突検出用)
  AABB bounds;                     // メッシュ全体の境界ボックス
};

// ============================================================================
// 3. マルチレベルクエリインターフェース
// ============================================================================

class CollisionQuery {
public:
  // --- レベル1: 高速なブールチェック (早期終了) ---
  static bool testCollision(const Capsule &a, const Sphere &b);
  static bool testCollision(const Capsule &a, const Triangle &b);
  static bool testCollision(const Capsule &a, const Mesh &b);

  // --- レベル2: マージンを考慮したチェック ---
  static bool testCollision(const Capsule &a, const Sphere &b, double margin);
  static bool testCollision(const Capsule &a, const Point &b, double margin);

  // --- レベル3: 詳細な接触情報 ---
  static Contact computeContact(const Capsule &a, const Sphere &b);
  static Contact computeContact(const Capsule &a, const Triangle &b);

  // --- レベル4: 距離と符号付き距離場 (SDF) ---
  static DistanceInfo computeDistance(const Capsule &a, const Sphere &b);
  static DistanceInfo computeDistance(const Capsule &a, const Triangle &b);
  static double computeSDF(const Mesh &mesh, const Eigen::Vector3d &p);

  // --- レベル5: 連続的な衝突検出 (CCD) ---
  static bool computeTOI(const Sphere &a, const Eigen::Vector3d &velocity_a,
                         const Sphere &b, const Eigen::Vector3d &velocity_b,
                         double &out_toi);

  // --- レベル6: リプシッツ連続性に基づくエッジ検証 ---
  static EdgeValidationResult validateEdge(const Capsule &link_at_q0,
                                           const Capsule &link_at_q1,
                                           const Mesh &environment,
                                           double lipschitz_constant);

  // --- Box Collision (SAT) ---
  static bool testCollision(const Box &a, const Box &b);
  static bool testCollision(const Box &a, const Capsule &b);
};

// ============================================================================
// 4. 衝突ワールド (マネージャー)
// ============================================================================

class CollisionWorld {
public:
  void addObstacle(const Sphere &s) { spheres.push_back(s); }
  void addObstacle(const Mesh &m) { meshes.push_back(m); }
  void addObstacle(const Box &b) { boxes.push_back(b); }

  bool checkRobotCollision(const std::vector<Capsule> &robot_links) const;
  std::vector<Contact>
  getAllContacts(const std::vector<Capsule> &robot_links) const;

private:
  std::vector<Sphere> spheres;
  std::vector<Mesh> meshes;
  std::vector<Box> boxes;
};

} // namespace collision