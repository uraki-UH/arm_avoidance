#include "collision/collision_detector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace collision {

// ============================================================================
// Helper Functions
// ============================================================================

namespace {
// 点と線分の最近傍点を計算
Eigen::Vector3d closestPointOnSegment(const Eigen::Vector3d &p,
                                      const Eigen::Vector3d &a,
                                      const Eigen::Vector3d &b) {
  Eigen::Vector3d ab = b - a;
  double t = (p - a).dot(ab) / ab.dot(ab);
  t = std::max(0.0, std::min(1.0, t));
  return a + t * ab;
}

// 点と三角形の最近傍点を計算
Eigen::Vector3d closestPointOnTriangle(const Eigen::Vector3d &p,
                                       const Triangle &tri) {
  Eigen::Vector3d ab = tri.v1 - tri.v0;
  Eigen::Vector3d ac = tri.v2 - tri.v0;
  Eigen::Vector3d ap = p - tri.v0;

  double d1 = ab.dot(ap);
  double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0)
    return tri.v0;

  Eigen::Vector3d bp = p - tri.v1;
  double d3 = ab.dot(bp);
  double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3)
    return tri.v1;

  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    double v = d1 / (d1 - d3);
    return tri.v0 + v * ab;
  }

  Eigen::Vector3d cp = p - tri.v2;
  double d5 = ab.dot(cp);
  double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6)
    return tri.v2;

  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    double w = d2 / (d2 - d6);
    return tri.v0 + w * ac;
  }

  double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return tri.v1 + w * (tri.v2 - tri.v1);
  }

  double denom = 1.0 / (va + vb + vc);
  double v = vb * denom;
  double w = vc * denom;
  return tri.v0 + ab * v + ac * w;
}

} // namespace

// ============================================================================
// CollisionQuery Implementation
// ============================================================================

bool CollisionQuery::testCollision(const Capsule &a, const Sphere &b) {
  Eigen::Vector3d closest = closestPointOnSegment(b.center, a.a, a.b);
  double distance = (closest - b.center).norm();
  return distance < (a.radius + b.radius);
}

bool CollisionQuery::testCollision(const Capsule &a, const Triangle &b) {
  // カプセルの線分と三角形の最近傍点を計算
  Eigen::Vector3d closest_on_triangle = closestPointOnTriangle(a.a, b);
  Eigen::Vector3d closest_on_segment =
      closestPointOnSegment(closest_on_triangle, a.a, a.b);

  // 三角形上の他の点も考慮
  closest_on_triangle = closestPointOnTriangle(closest_on_segment, b);

  double distance = (closest_on_segment - closest_on_triangle).norm();
  return distance < a.radius;
}

bool CollisionQuery::testCollision(const Capsule &a, const Mesh &b) {
  // 簡易実装：全ての三角形をチェック
  for (const auto &triangle : b.triangles) {
    if (testCollision(a, triangle)) {
      return true;
    }
  }
  return false;
}

bool CollisionQuery::testCollision(const Capsule &a, const Sphere &b,
                                   double margin) {
  Eigen::Vector3d closest = closestPointOnSegment(b.center, a.a, a.b);
  double distance = (closest - b.center).norm();
  return distance < (a.radius + b.radius + margin);
}

bool CollisionQuery::testCollision(const Capsule &a, const Point &b,
                                   double margin) {
  Eigen::Vector3d closest = closestPointOnSegment(b.p, a.a, a.b);
  double distance = (closest - b.p).norm();
  return distance < (a.radius + margin);
}

Contact CollisionQuery::computeContact(const Capsule &a, const Sphere &b) {
  Contact contact;

  Eigen::Vector3d closest_on_capsule =
      closestPointOnSegment(b.center, a.a, a.b);
  double distance = (closest_on_capsule - b.center).norm();

  contact.colliding = distance < (a.radius + b.radius);

  if (contact.colliding) {
    contact.depth = (a.radius + b.radius) - distance;
    contact.pos_a = closest_on_capsule;

    if (distance > 1e-6) {
      Eigen::Vector3d direction = (b.center - closest_on_capsule).normalized();
      contact.pos_b = b.center - direction * b.radius;
      contact.normal = direction;
    } else {
      // 完全に重なっている場合
      contact.pos_b = b.center;
      contact.normal = Eigen::Vector3d::UnitZ(); // 適当な法線
    }

    // カプセル上の位置パラメータを計算
    Eigen::Vector3d seg_dir = a.b - a.a;
    if (seg_dir.norm() > 1e-6) {
      contact.link_parameter =
          (closest_on_capsule - a.a).dot(seg_dir) / seg_dir.dot(seg_dir);
      contact.link_parameter =
          std::max(0.0, std::min(1.0, contact.link_parameter));
    }
  }

  return contact;
}

Contact CollisionQuery::computeContact(const Capsule &a, const Triangle &b) {
  Contact contact;

  Eigen::Vector3d closest_on_triangle = closestPointOnTriangle(a.a, b);
  Eigen::Vector3d closest_on_capsule =
      closestPointOnSegment(closest_on_triangle, a.a, a.b);

  // より正確な最近傍点を反復的に計算
  for (int iter = 0; iter < 3; ++iter) {
    closest_on_triangle = closestPointOnTriangle(closest_on_capsule, b);
    closest_on_capsule = closestPointOnSegment(closest_on_triangle, a.a, a.b);
  }

  double distance = (closest_on_capsule - closest_on_triangle).norm();
  contact.colliding = distance < a.radius;

  if (contact.colliding) {
    contact.depth = a.radius - distance;
    contact.pos_a = closest_on_capsule;
    contact.pos_b = closest_on_triangle;

    if (distance > 1e-6) {
      contact.normal = (closest_on_capsule - closest_on_triangle).normalized();
    } else {
      // 三角形の法線を使用
      Eigen::Vector3d edge1 = b.v1 - b.v0;
      Eigen::Vector3d edge2 = b.v2 - b.v0;
      contact.normal = edge1.cross(edge2).normalized();
    }

    // カプセル上の位置パラメータを計算
    Eigen::Vector3d seg_dir = a.b - a.a;
    if (seg_dir.norm() > 1e-6) {
      contact.link_parameter =
          (closest_on_capsule - a.a).dot(seg_dir) / seg_dir.dot(seg_dir);
      contact.link_parameter =
          std::max(0.0, std::min(1.0, contact.link_parameter));
    }
  }

  return contact;
}

DistanceInfo CollisionQuery::computeDistance(const Capsule &a,
                                             const Sphere &b) {
  DistanceInfo info;

  info.nearest_a = closestPointOnSegment(b.center, a.a, a.b);
  double center_distance = (info.nearest_a - b.center).norm();

  if (center_distance > 1e-6) {
    Eigen::Vector3d direction = (b.center - info.nearest_a).normalized();
    info.nearest_b = b.center - direction * b.radius;
    info.distance = center_distance - a.radius - b.radius;
  } else {
    info.nearest_b = b.center;
    info.distance = -a.radius - b.radius; // 完全に重なっている
  }

  return info;
}

DistanceInfo CollisionQuery::computeDistance(const Capsule &a,
                                             const Triangle &b) {
  DistanceInfo info;

  info.nearest_b = closestPointOnTriangle(a.a, b);
  info.nearest_a = closestPointOnSegment(info.nearest_b, a.a, a.b);

  // より正確な最近傍点を反復的に計算
  for (int iter = 0; iter < 3; ++iter) {
    info.nearest_b = closestPointOnTriangle(info.nearest_a, b);
    info.nearest_a = closestPointOnSegment(info.nearest_b, a.a, a.b);
  }

  double distance = (info.nearest_a - info.nearest_b).norm();
  info.distance = distance - a.radius;

  return info;
}

double CollisionQuery::computeSDF(const Mesh &mesh, const Eigen::Vector3d &p) {
  double min_distance = std::numeric_limits<double>::max();

  // 簡易実装：全三角形との距離を計算
  for (const auto &triangle : mesh.triangles) {
    Eigen::Vector3d closest = closestPointOnTriangle(p, triangle);
    double distance = (p - closest).norm();

    // 内部判定（レイキャスティングの簡易版）
    Eigen::Vector3d edge1 = triangle.v1 - triangle.v0;
    Eigen::Vector3d edge2 = triangle.v2 - triangle.v0;
    Eigen::Vector3d normal = edge1.cross(edge2).normalized();

    if ((p - closest).dot(normal) < 0) {
      distance = -distance;
    }

    if (std::abs(distance) < std::abs(min_distance)) {
      min_distance = distance;
    }
  }

  return min_distance;
}

bool CollisionQuery::computeTOI(const Sphere &a,
                                const Eigen::Vector3d &velocity_a,
                                const Sphere &b,
                                const Eigen::Vector3d &velocity_b,
                                double &out_toi) {
  Eigen::Vector3d relative_velocity = velocity_a - velocity_b;
  Eigen::Vector3d relative_position = a.center - b.center;

  double radius_sum = a.radius + b.radius;

  // 2次方程式の係数
  double a_coeff = relative_velocity.dot(relative_velocity);
  double b_coeff = 2.0 * relative_position.dot(relative_velocity);
  double c_coeff =
      relative_position.dot(relative_position) - radius_sum * radius_sum;

  // 判別式
  double discriminant = b_coeff * b_coeff - 4.0 * a_coeff * c_coeff;

  if (discriminant < 0.0) {
    return false; // 衝突しない
  }

  double sqrt_discriminant = std::sqrt(discriminant);
  double t1 = (-b_coeff - sqrt_discriminant) / (2.0 * a_coeff);
  double t2 = (-b_coeff + sqrt_discriminant) / (2.0 * a_coeff);

  out_toi = std::min(t1, t2);

  // 有効な時間範囲内かチェック
  return out_toi >= 0.0 && out_toi <= 1.0;
}

EdgeValidationResult CollisionQuery::validateEdge(const Capsule &link_at_q0,
                                                  const Capsule &link_at_q1,
                                                  const Mesh &environment,
                                                  double lipschitz_constant) {
  EdgeValidationResult result;

  // 簡易実装：中間点でのチェック
  const int num_samples = 10;

  for (int i = 0; i <= num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;

    // 線形補間でカプセル位置を計算
    Capsule interpolated_capsule;
    interpolated_capsule.a = (1.0 - t) * link_at_q0.a + t * link_at_q1.a;
    interpolated_capsule.b = (1.0 - t) * link_at_q0.b + t * link_at_q1.b;
    interpolated_capsule.radius =
        (1.0 - t) * link_at_q0.radius + t * link_at_q1.radius;

    // マージンを考慮した衝突チェック
    double safety_margin = lipschitz_constant * 0.1; // 適切なマージンを設定

    for (const auto &triangle : environment.triangles) {
      DistanceInfo dist_info = computeDistance(interpolated_capsule, triangle);

      if (dist_info.distance < safety_margin) {
        result.is_safe = false;
        result.unsafe_t = t;
        result.witness_contact = computeContact(interpolated_capsule, triangle);
        return result;
      }
    }
  }

  return result;
}

// ============================================================================
// Box Collision Implementation (SAT)
// ============================================================================

namespace {
// Helper for SAT: Project shape onto axis
void projectBox(const Box &box, const Eigen::Vector3d &axis, double &min,
                double &max) {
  // Box center projection
  double center_proj = box.center.dot(axis);

  // Extent projection (sum of absolute dot products)
  // extents are half-widths along local axes.
  // global_axis = R * local_axis => projected length = sum(|axis . (R *
  // local_axis_i) * extent_i|) Simplified: sum(|axis . (R.col(i))| * extent(i))
  double extent_proj = 0.0;
  for (int i = 0; i < 3; ++i) {
    extent_proj += std::abs(axis.dot(box.rotation.col(i))) * box.extents(i);
  }

  min = center_proj - extent_proj;
  max = center_proj + extent_proj;
}

void projectCapsule(const Capsule &capsule, const Eigen::Vector3d &axis,
                    double &min, double &max) {
  double p1 = capsule.a.dot(axis);
  double p2 = capsule.b.dot(axis);
  min = std::min(p1, p2) - capsule.radius;
  max = std::max(p1, p2) + capsule.radius;
}

} // namespace

bool CollisionQuery::testCollision(const Box &a, const Box &b) {
  // Axes to test:
  // 3 face normals of A
  // 3 face normals of B
  // 9 cross products of edges (A_i x B_j)

  std::vector<Eigen::Vector3d> axes;
  axes.reserve(15);

  for (int i = 0; i < 3; ++i)
    axes.push_back(a.rotation.col(i));
  for (int i = 0; i < 3; ++i)
    axes.push_back(b.rotation.col(i));

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d cross = a.rotation.col(i).cross(b.rotation.col(j));
      if (cross.norm() > 1e-6) { // Avoid zero vectors
        axes.push_back(cross.normalized());
      }
    }
  }

  for (const auto &axis : axes) {
    double minA, maxA, minB, maxB;
    projectBox(a, axis, minA, maxA);
    projectBox(b, axis, minB, maxB);

    if (maxA < minB || maxB < minA) {
      return false; // Separating axis found
    }
  }

  return true;
}

bool CollisionQuery::testCollision(const Box &a, const Capsule &b) {
  // Axes to test:
  // 3 face normals of Box A
  // 1 axis of Capsule B
  // 3 cross products (Box axis x Capsule axis)

  std::vector<Eigen::Vector3d> axes;
  axes.reserve(7);

  for (int i = 0; i < 3; ++i)
    axes.push_back(a.rotation.col(i));

  Eigen::Vector3d caps_axis = (b.b - b.a);
  if (caps_axis.norm() > 1e-6) {
    caps_axis.normalize();
    axes.push_back(caps_axis);
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d cross = a.rotation.col(i).cross(caps_axis);
      if (cross.norm() > 1e-6) {
        axes.push_back(cross.normalized());
      }
    }
  }

  for (const auto &axis : axes) {
    double minA, maxA, minB, maxB;
    projectBox(a, axis, minA, maxA);
    projectCapsule(b, axis, minB, maxB);

    if (maxA < minB || maxB < minA) {
      return false; // Separating axis found
    }
  }

  return true;
}

// ============================================================================
// CollisionWorld Implementation
// ============================================================================

bool CollisionWorld::checkRobotCollision(
    const std::vector<Capsule> &robot_links) const {
  // 球との衝突チェック
  for (const auto &link : robot_links) {
    for (const auto &sphere : spheres) {
      if (CollisionQuery::testCollision(link, sphere)) {
        return true;
      }
    }
  }

  // メッシュとの衝突チェック
  for (const auto &link : robot_links) {
    for (const auto &mesh : meshes) {
      if (CollisionQuery::testCollision(link, mesh)) {
        return true;
      }
    }
  }

  return false;
}

std::vector<Contact>
CollisionWorld::getAllContacts(const std::vector<Capsule> &robot_links) const {
  std::vector<Contact> contacts;

  // 球との接触情報を収集
  for (const auto &link : robot_links) {
    for (const auto &sphere : spheres) {
      Contact contact = CollisionQuery::computeContact(link, sphere);
      if (contact.colliding) {
        contacts.push_back(contact);
      }
    }
  }

  // メッシュとの接触情報を収集
  for (const auto &link : robot_links) {
    for (const auto &mesh : meshes) {
      for (const auto &triangle : mesh.triangles) {
        Contact contact = CollisionQuery::computeContact(link, triangle);
        if (contact.colliding) {
          contacts.push_back(contact);
        }
      }
    }
  }

  return contacts;
}

} // namespace collision