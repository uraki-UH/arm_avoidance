#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include "kinematics/joint.hpp" // For JointType enum and Joint struct

namespace simulation {

// ジオメトリの形状を定義するenum
enum class GeometryType { BOX, CYLINDER, SPHERE, MESH };

// ジオメトリ（衝突・可視）の情報を保持する構造体
struct Geometry {
  GeometryType type;
  Eigen::Vector3d
      size; // BOX, CYLINDER(radius, length, 0), SPHERE(radius, 0, 0)
  std::string mesh_filename;
};

// 材質（色）の情報を保持する構造体
struct Material {
  std::string name;
  Eigen::Vector4d color; // RGBA
};

// リンクの慣性特性
struct Inertial {
  double mass = 0.0;
  Eigen::Isometry3d origin =
      Eigen::Isometry3d::Identity(); // リンク原点からの重心位置オフセット
  Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity();
};

// リンクの可視情報
struct Visual {
  std::string name;
  Eigen::Isometry3d origin =
      Eigen::Isometry3d::Identity(); // リンク原点からのオフセット
  Geometry geometry;                 // Original geometry (may be mesh)
  Geometry simplified_geometry;      // Auto-generated simplified geometry (box)
  Eigen::Vector3d simplified_offset =
      Eigen::Vector3d::Zero(); // Center offset for simplified geometry
  Eigen::Isometry3d simplified_offset_transform = Eigen::Isometry3d::Identity();
  bool has_simplified =
      false; // Flag indicating if simplified geometry is available
  Material material;
};

// リンクの衝突情報
struct Collision {
  std::string name;
  Eigen::Isometry3d origin =
      Eigen::Isometry3d::Identity(); // リンク原点からのオフセット
  Geometry geometry;
};

// リンクのプロパティ
struct LinkProperties {
  std::string name;
  Inertial inertial;
  std::vector<Visual> visuals;
  std::vector<Collision> collisions;
};

// ジョイントの可動域制限
struct JointLimits {
  double lower = 0.0;
  double upper = 0.0;
  double effort = 0.0;
  double velocity = 0.0;
};

// ジョイントの物理特性
struct JointDynamics {
  double damping = 0.0;
  double friction = 0.0;
};

// ジョイントのプロパティ
struct JointProperties {
  std::string name;
  kinematics::JointType type = kinematics::JointType::Fixed;
  std::string parent_link;
  std::string child_link;
  Eigen::Isometry3d origin =
      Eigen::Isometry3d::Identity(); // 親リンク原点からのオフセット
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  JointLimits limits;
  JointDynamics dynamics;
  bool has_limits = false; // 可動域が存在するかどうか
};

// ロボット全体を表現するモデルクラス
class RobotModel {
public:
  std::string getName() const { return name_; }
  void setName(const std::string &name) { name_ = name; }

  void addLink(const LinkProperties &link) { links_[link.name] = link; }
  void addJoint(const JointProperties &joint) { joints_[joint.name] = joint; }

  const LinkProperties *getLink(const std::string &name) const {
    auto it = links_.find(name);
    return it != links_.end() ? &it->second : nullptr;
  }

  const JointProperties *getJoint(const std::string &name) const {
    auto it = joints_.find(name);
    return it != joints_.end() ? &it->second : nullptr;
  }

  const std::map<std::string, LinkProperties> &getLinks() const {
    return links_;
  }
  const std::map<std::string, JointProperties> &getJoints() const {
    return joints_;
  }

  std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
  getFixedLinkInfo() const {
    std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> info;
    for (const auto &[name, joint] : joints_) {
      if (joint.type == kinematics::JointType::Fixed) {
        info[joint.child_link] = {joint.parent_link, joint.origin};
      }
    }
    return info;
  }

  std::string getRootLinkName() const { return root_link_name_; }
  void setRootLinkName(const std::string &name) { root_link_name_ = name; }

private:
  std::string name_;
  std::string root_link_name_;
  std::map<std::string, LinkProperties> links_;
  std::map<std::string, JointProperties> joints_;
};

} // namespace simulation
