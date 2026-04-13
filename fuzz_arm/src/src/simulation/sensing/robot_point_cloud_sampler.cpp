#include "simulation/sensing/robot_point_cloud_sampler.hpp"
#include <cmath>
#include <random>

namespace simulation {

RobotPointCloudSampler::RobotPointCloudSampler(
    const RobotModel &model, const kinematics::KinematicChain &chain)
    : model_(model), chain_(chain) {
  sampleModel();
}

void RobotPointCloudSampler::setSamplingDensity(double points_per_m2) {
  density_ = points_per_m2;
  sampleModel(); // Resample
}

void RobotPointCloudSampler::sampleModel() {
  link_point_data_.clear();

  for (const auto &link_pair : model_.getLinks()) {
    const auto &link = link_pair.second;
    if (link.name == "base_link") {
      continue;
    }
    LinkPoints lp;
    lp.link_name = link.name;

    for (const auto &col : link.collisions) {
      std::vector<Eigen::Vector3d> col_points;
      const auto &geom = col.geometry;

      if (geom.type == GeometryType::BOX) {
        sampleBox(geom.size, col_points);
      } else if (geom.type == GeometryType::CYLINDER) {
        sampleCylinder(geom.size[0], geom.size[1], col_points);
      } else if (geom.type == GeometryType::SPHERE) {
        sampleSphere(geom.size[0], col_points);
      }

      // Transform local collision points to link origin
      for (auto &p : col_points) {
        lp.local_points.push_back(col.origin * p);
      }
    }
    link_point_data_.push_back(lp);
  }
}

void RobotPointCloudSampler::sampleBox(const Eigen::Vector3d &size,
                                       std::vector<Eigen::Vector3d> &points) {
  auto sampleFace = [&](int axes[3], double sign) {
    double area = size[axes[1]] * size[axes[2]];
    int num_points = static_cast<int>(area * density_);
    if (num_points < 1)
      num_points = 1;

    std::mt19937 gen(axes[0] + (int)(sign * 100));
    std::uniform_real_distribution<> dis(-0.5, 0.5);

    for (int i = 0; i < num_points; ++i) {
      Eigen::Vector3d p;
      p[axes[0]] = 0.5 * size[axes[0]] * sign;
      p[axes[1]] = size[axes[1]] * dis(gen);
      p[axes[2]] = size[axes[2]] * dis(gen);
      points.push_back(p);
    }
  };

  int axes_x[3] = {0, 1, 2};
  int axes_y[3] = {1, 0, 2};
  int axes_z[3] = {2, 0, 1};

  sampleFace(axes_x, 1.0);
  sampleFace(axes_x, -1.0);
  sampleFace(axes_y, 1.0);
  sampleFace(axes_y, -1.0);
  sampleFace(axes_z, 1.0);
  sampleFace(axes_z, -1.0);
}

void RobotPointCloudSampler::sampleCylinder(
    double radius, double length, std::vector<Eigen::Vector3d> &points) {
  // Side
  double side_area = 2.0 * M_PI * radius * length;
  int num_side = static_cast<int>(side_area * density_);

  std::mt19937 gen(42);
  std::uniform_real_distribution<> dis_theta(0, 2.0 * M_PI);
  std::uniform_real_distribution<> dis_z(-0.5 * length, 0.5 * length);

  for (int i = 0; i < num_side; ++i) {
    double theta = dis_theta(gen);
    double z = dis_z(gen);
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta), z);
  }

  // Caps
  double cap_area = M_PI * radius * radius;
  int num_cap = static_cast<int>(cap_area * density_);
  std::uniform_real_distribution<> dis_r(0, 1.0);

  auto sampleCap = [&](double z_sign) {
    for (int i = 0; i < num_cap; ++i) {
      double r = radius *
                 std::sqrt(dis_r(gen)); // sqrt for uniform distribution on disk
      double theta = dis_theta(gen);
      points.emplace_back(r * std::cos(theta), r * std::sin(theta),
                          0.5 * length * z_sign);
    }
  };
  sampleCap(1.0);
  sampleCap(-1.0);
}

void RobotPointCloudSampler::sampleSphere(
    double radius, std::vector<Eigen::Vector3d> &points) {
  double area = 4.0 * M_PI * radius * radius;
  int num_points = static_cast<int>(area * density_);

  std::mt19937 gen(123);
  std::uniform_real_distribution<> dis_u(-1.0, 1.0);
  std::uniform_real_distribution<> dis_phi(0, 2.0 * M_PI);

  for (int i = 0; i < num_points; ++i) {
    double u = dis_u(gen);
    double phi = dis_phi(gen);
    double r_flat = std::sqrt(1.0 - u * u);
    points.emplace_back(radius * r_flat * std::cos(phi),
                        radius * r_flat * std::sin(phi), radius * u);
  }
}

void RobotPointCloudSampler::update(
    const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
        &fixed_link_info) {
  world_points_.clear();

  std::map<std::string, Eigen::Isometry3d> link_tfs;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> j_pos;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>
      j_ori;

  // Get base position/orientations from joint values
  chain_.forwardKinematicsAt(chain_.getJointValues(), j_pos, j_ori);
  chain_.buildAllLinkTransforms(j_pos, j_ori, fixed_link_info, link_tfs);

  for (const auto &lp : link_point_data_) {
    auto it = link_tfs.find(lp.link_name);
    if (it != link_tfs.end()) {
      const auto &tf = it->second;
      for (const auto &p : lp.local_points) {
        world_points_.push_back(tf * p);
      }
    }
  }
}

std::vector<Eigen::Vector3d>
RobotPointCloudSampler::getLinkPoints(const std::string & /*link_name*/) const {
  std::vector<Eigen::Vector3d> points;
  return points;
}

} // namespace simulation
