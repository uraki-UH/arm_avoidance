#include "common/geometry_utils.hpp"

namespace common {
namespace geometry {

Eigen::Vector3d
PrimitiveSupport::support(const Eigen::Vector3d &direction) const {
  // 1. 方向をローカル座標系に変換
  Eigen::Vector3d local_dir = transform_.linear().transpose() * direction;
  Eigen::Vector3d local_p = Eigen::Vector3d::Zero();

  // 2. ローカル座標系でのサポートポイントを計算
  if (geom_.type == simulation::GeometryType::BOX) {
    Eigen::Vector3d h = geom_.size * 0.5;
    local_p = Eigen::Vector3d(local_dir.x() > 0 ? h.x() : -h.x(),
                              local_dir.y() > 0 ? h.y() : -h.y(),
                              local_dir.z() > 0 ? h.z() : -h.z());
  } else if (geom_.type == simulation::GeometryType::SPHERE) {
    double r = geom_.size.x();
    if (local_dir.norm() > 1e-9) {
      local_p = local_dir.normalized() * r;
    }
  } else if (geom_.type == simulation::GeometryType::CYLINDER) {
    // URDF Cylinder: radius=size.x, length=size.y, aligned along Z
    double r = geom_.size.x();
    double h = geom_.size.y() * 0.5;

    local_p.z() = (local_dir.z() > 0) ? h : -h;
    Eigen::Vector2d circle_dir(local_dir.x(), local_dir.y());
    if (circle_dir.norm() > 1e-9) {
      local_p.head<2>() = circle_dir.normalized() * r;
    }
  }

  // 3. ワールド座標系に戻す
  return transform_ * local_p;
}

Eigen::Vector3d VoxelSupport::support(const Eigen::Vector3d &direction) const {
  return center_ +
         Eigen::Vector3d(direction.x() > 0 ? extents_.x() : -extents_.x(),
                         direction.y() > 0 ? extents_.y() : -extents_.y(),
                         direction.z() > 0 ? extents_.z() : -extents_.z());
}

Eigen::Vector3d GJK::minkowskiSupport(const SupportProvider &shapeA,
                                      const SupportProvider &shapeB,
                                      const Eigen::Vector3d &direction) {
  return shapeA.support(direction) - shapeB.support(-direction);
}

// GJK Simplex structure (up to 4 points)
struct Simplex {
  Eigen::Vector3d points[4];
  int size = 0;

  void add(const Eigen::Vector3d &p) {
    points[3] = points[2];
    points[2] = points[1];
    points[1] = points[0];
    points[0] = p;
    if (size < 4)
      size++;
  }
};

static bool doSimplex(Simplex &simplex, Eigen::Vector3d &direction) {
  if (simplex.size == 2) { // Line
    Eigen::Vector3d A = simplex.points[0];
    Eigen::Vector3d B = simplex.points[1];
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AO = -A;

    if (AB.dot(AO) > 0) {
      direction = AB.cross(AO).cross(AB);
      if (direction.norm() < 1e-9) { // AO is on AB, or parallel
        // Need a perpendicular vector
        direction = (std::abs(AB.x()) < std::abs(AB.y()))
                        ? Eigen::Vector3d(1, 0, 0).cross(AB)
                        : Eigen::Vector3d(0, 1, 0).cross(AB);
      }
    } else {
      simplex.size = 1;
      direction = AO;
    }
  } else if (simplex.size == 3) { // Triangle
    Eigen::Vector3d A = simplex.points[0];
    Eigen::Vector3d B = simplex.points[1];
    Eigen::Vector3d C = simplex.points[2];
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    Eigen::Vector3d ABC = AB.cross(AC);
    Eigen::Vector3d AO = -A;

    if (ABC.cross(AC).dot(AO) > 0) {
      if (AC.dot(AO) > 0) {
        simplex.points[1] = C;
        simplex.size = 2;
        direction = AC.cross(AO).cross(AC);
      } else {
        if (AB.dot(AO) > 0) {
          simplex.size = 2;
          direction = AB.cross(AO).cross(AB);
        } else {
          simplex.size = 1;
          direction = AO;
        }
      }
    } else {
      if (AB.cross(ABC).dot(AO) > 0) {
        if (AB.dot(AO) > 0) {
          simplex.size = 2;
          direction = AB.cross(AO).cross(AB);
        } else {
          simplex.size = 1;
          direction = AO;
        }
      } else {
        if (ABC.dot(AO) > 0) {
          direction = ABC;
        } else {
          simplex.points[1] = C;
          simplex.points[2] = B;
          direction = -ABC;
        }
      }
    }
  } else if (simplex.size == 4) { // Tetrahedron
    Eigen::Vector3d A = simplex.points[0];
    Eigen::Vector3d B = simplex.points[1];
    Eigen::Vector3d C = simplex.points[2];
    Eigen::Vector3d D = simplex.points[3];
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    Eigen::Vector3d AD = D - A;
    Eigen::Vector3d ABC = AB.cross(AC);
    Eigen::Vector3d ACD = AC.cross(AD);
    Eigen::Vector3d ADB = AD.cross(AB);
    Eigen::Vector3d AO = -A;

    if (ABC.dot(AO) > 0) {
      simplex.size = 3;
      return doSimplex(simplex, direction);
    }
    if (ACD.dot(AO) > 0) {
      simplex.points[1] = C;
      simplex.points[2] = D;
      simplex.size = 3;
      return doSimplex(simplex, direction);
    }
    if (ADB.dot(AO) > 0) {
      simplex.points[1] = D;
      simplex.points[2] = B;
      simplex.size = 3;
      return doSimplex(simplex, direction);
    }
    return true; // Origin is inside tetrahedron
  }
  return false;
}

bool GJK::intersect(const SupportProvider &shapeA,
                    const SupportProvider &shapeB, int max_iterations) {
  Eigen::Vector3d direction(1, 0, 0); // Initial direction
  Simplex simplex;

  Eigen::Vector3d first = minkowskiSupport(shapeA, shapeB, direction);
  simplex.add(first);
  direction = -first;

  for (int i = 0; i < max_iterations; ++i) {
    Eigen::Vector3d next = minkowskiSupport(shapeA, shapeB, direction);
    if (next.dot(direction) < 0)
      return false; // Not passed the origin

    simplex.add(next);
    if (doSimplex(simplex, direction))
      return true;
  }
  return false;
}

} // namespace geometry
} // namespace common
