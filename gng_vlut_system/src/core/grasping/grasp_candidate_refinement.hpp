#pragma once

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasp_refinement
{
struct options
{
  double min_width=0.005, max_width=0.074;
  double finger_span=0.061, finger_length=0.0883, finger_thickness=0.034;
  double base_span=0.092, base_width=0.161, min_base_depth=0.080, max_base_depth=0.111;
  double opening_margin=0.003, collision_margin=0.001;
  double approach_length=0.060, contact_band=0.002;
  double min_contact_depth=0.003, max_contact_depth=0.070;
  double min_contact_spread=0.002, max_contact_variation=0.08;
  double max_contact_normal_deg=30.0, max_center_shift=0.025;
  std::size_t min_contact_points=8;
  // 元候補の進入方向から実TCP座標系へのX軸補正角 [deg]
  double tcp_rotation_x_deg=180.0;
  std::vector<double> yaw_offsets_deg{-15.0,0.0,15.0};
  std::vector<double> insertion_depths{0.025,0.040,0.055};
};

struct result
{
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  Eigen::Vector3d left=Eigen::Vector3d::Zero(), right=Eigen::Vector3d::Zero();
  double contact_width=std::numeric_limits<double>::quiet_NaN();
  double observed_width=std::numeric_limits<double>::quiet_NaN();
  double opening_width=std::numeric_limits<double>::quiet_NaN();
  std::size_t left_support_num=0, right_support_num=0;
  bool has_contact_pair=false, has_gripper_check=false, has_observed_collision=false;
  std::string reason="no_contact_points";
};

inline void validate(const options &config)
{
  const std::vector<double> positive{config.min_width,config.max_width,config.finger_span,
    config.finger_length,config.finger_thickness,config.base_span,config.base_width,
    config.max_base_depth,config.contact_band,config.max_contact_depth,
    config.min_contact_spread,config.max_contact_variation,config.max_center_shift};
  for (double value:positive)
    if (!std::isfinite(value) || value<=0) throw std::invalid_argument("invalid refinement dimension");
  for (double value:{config.opening_margin,config.collision_margin,config.approach_length,
      config.min_contact_depth,config.min_base_depth})
    if (!std::isfinite(value) || value<0) throw std::invalid_argument("invalid refinement margin");
  if (config.min_width>=config.max_width || config.min_contact_depth>=config.max_contact_depth ||
    config.max_contact_depth>config.finger_length || config.min_base_depth>=config.max_base_depth ||
    config.min_contact_points<3 || config.max_contact_variation>=1 ||
    !(config.max_contact_normal_deg>0 && config.max_contact_normal_deg<90) ||
    !std::isfinite(config.tcp_rotation_x_deg) || config.yaw_offsets_deg.empty() ||
    config.insertion_depths.empty() || config.yaw_offsets_deg.size()*config.insertion_depths.size()>128)
    throw std::invalid_argument("invalid refinement search options");
  for (double value:config.yaw_offsets_deg)
    if (!std::isfinite(value)) throw std::invalid_argument("invalid yaw offset");
  for (double value:config.insertion_depths)
    if (!std::isfinite(value) || value<0 || value>config.finger_length)
      throw std::invalid_argument("invalid insertion depth");
}

// 全探索姿勢の指・基部・進入掃引と中心補正を包含する局所検索箱
inline std::pair<Eigen::Vector3d,Eigen::Vector3d> search_bounds(
  const Eigen::Isometry3d &source,const options &config)
{
  const double x=0.5*std::max(config.finger_span,config.base_span)+config.collision_margin+config.max_center_shift;
  const double y=std::max(0.5*config.max_width+config.finger_thickness,0.5*config.base_width)+config.collision_margin+config.max_center_shift;
  const double z=std::max(config.finger_length,config.max_base_depth)+config.approach_length+config.collision_margin;
  Eigen::Vector3d min_corner=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_corner=-min_corner;
  constexpr double angle_scale=3.14159265358979323846/180;
  for (double yaw:config.yaw_offsets_deg) for (double depth:config.insertion_depths) {
    const Eigen::Matrix3d rotation=source.linear()*
      Eigen::AngleAxisd(config.tcp_rotation_x_deg*angle_scale,Eigen::Vector3d::UnitX()).toRotationMatrix()*
      Eigen::AngleAxisd(yaw*angle_scale,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (double a:{-x,x}) for (double b:{-y,y}) for (double c:{-config.collision_margin,z}) {
      const Eigen::Vector3d point=source.translation()+rotation*Eigen::Vector3d(a,b,c-depth);
      min_corner=min_corner.cwiseMin(point); max_corner=max_corner.cwiseMax(point);
    }
  }
  return {min_corner,max_corner};
}

inline bool contact_normal(const std::vector<Eigen::Vector3d> &points,
  const options &config, Eigen::Vector3d &center, Eigen::Vector3d &normal)
{
  if (points.size()<config.min_contact_points) return false;
  center.setZero();
  for (const auto &point:points) center+=point;
  center/=static_cast<double>(points.size());
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
  for (const auto &point:points) {
    const Eigen::Vector3d delta=point-center;
    covariance.noalias()+=delta*delta.transpose();
  }
  covariance/=static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
  if (eigen.info()!=Eigen::Success || eigen.eigenvalues()(1)<config.min_contact_spread*config.min_contact_spread ||
    eigen.eigenvalues()(0)>config.max_contact_variation*covariance.trace()) return false;
  normal=eigen.eigenvectors().col(0);
  return true;
}

// 指の閉鎖と上方進入の保守的な直方体掃引。接触帯の内側は対象物の許容領域
inline bool observed_collision(const std::vector<Eigen::Vector3d> &points,
  const Eigen::Isometry3d &pose, double contact_width, double opening_width, const options &config)
{
  const Eigen::Isometry3d inverse=pose.inverse();
  for (const auto &point:points) {
    if (!point.allFinite()) continue;
    const Eigen::Vector3d p=inverse*point;
    if (std::abs(p.x())<=0.5*config.base_span+config.collision_margin &&
      std::abs(p.y())<=0.5*config.base_width+config.collision_margin &&
      p.z()>=config.min_base_depth-config.collision_margin &&
      p.z()<=config.max_base_depth+config.approach_length+config.collision_margin) return true;
    if (std::abs(p.x())<=0.5*config.finger_span+config.collision_margin &&
      std::abs(p.y())>0.5*contact_width+config.contact_band &&
      std::abs(p.y())<=0.5*opening_width+config.finger_thickness+config.collision_margin &&
      p.z()>=-config.collision_margin && p.z()<=config.finger_length+config.approach_length+config.collision_margin) return true;
  }
  return false;
}

inline result evaluate_pose(const std::vector<Eigen::Vector3d> &points,
  const Eigen::Isometry3d &pose, const options &config)
{
  result out;
  out.pose=pose;
  const Eigen::Isometry3d inverse=pose.inverse();
  std::vector<Eigen::Vector3d> local;
  double min_y=std::numeric_limits<double>::infinity(), max_y=-min_y;
  for (const auto &point:points) {
    if (!point.allFinite()) continue;
    const Eigen::Vector3d p=inverse*point;
    if (std::abs(p.x())>0.5*config.finger_span ||
      p.z()<config.min_contact_depth || p.z()>config.max_contact_depth ||
      std::abs(p.y())>0.5*config.max_width+config.max_center_shift) continue;
    local.push_back(p); min_y=std::min(min_y,p.y()); max_y=std::max(max_y,p.y());
  }
  if (local.size()<2*config.min_contact_points) return out;
  const double observed_width=max_y-min_y;
  out.observed_width=observed_width;
  if (observed_width<config.min_width || observed_width+2*config.opening_margin>config.max_width) {
    out.reason="width_out_of_range"; return out;
  }
  std::vector<Eigen::Vector3d> left, right;
  for (const auto &point:local) {
    if (point.y()<=min_y+config.contact_band) left.push_back(point);
    if (point.y()>=max_y-config.contact_band) right.push_back(point);
  }
  Eigen::Vector3d left_center,right_center,left_normal,right_normal;
  out.left_support_num=left.size(); out.right_support_num=right.size();
  if (!contact_normal(left,config,left_center,left_normal) ||
    !contact_normal(right,config,right_center,right_normal)) {
    out.reason="insufficient_contact_support"; return out;
  }
  const Eigen::Vector3d contact_axis=(right_center-left_center).normalized();
  const double min_cos=std::cos(config.max_contact_normal_deg*3.14159265358979323846/180);
  if (std::abs(left_normal.y())<min_cos || std::abs(right_normal.y())<min_cos ||
    std::abs(left_normal.dot(contact_axis))<min_cos || std::abs(right_normal.dot(contact_axis))<min_cos) {
    out.reason="non_opposing_contacts"; return out;
  }
  const Eigen::Vector3d shift(0.5*(left_center.x()+right_center.x()),0.5*(min_y+max_y),0);
  if (shift.norm()>config.max_center_shift) { out.reason="center_shift_limit"; return out; }
  out.pose.translation()+=pose.linear()*shift;
  out.left=pose*left_center; out.right=pose*right_center;
  const double contact_width=right_center.y()-left_center.y();
  if (contact_width<config.min_width) { out.reason="width_out_of_range"; return out; }
  out.contact_width=contact_width;
  out.opening_width=observed_width+2*config.opening_margin;
  out.has_contact_pair=true;
  out.has_gripper_check=true;
  out.has_observed_collision=observed_collision(points,out.pose,out.contact_width,out.opening_width,config);
  out.reason=out.has_observed_collision ? "observed_gripper_collision":"contacts_only";
  return out;
}

inline result refine(const std::vector<Eigen::Vector3d> &points,
  const Eigen::Isometry3d &source, const options &config)
{
  result best;
  best.pose=source;
  if (!source.matrix().allFinite()) { best.reason="invalid_pose"; return best; }
  constexpr double angle_scale=3.14159265358979323846/180;
  double best_score=-std::numeric_limits<double>::infinity();
  for (double yaw:config.yaw_offsets_deg) for (double depth:config.insertion_depths) {
    Eigen::Isometry3d pose=source;
    pose.linear()=source.linear()*Eigen::AngleAxisd(config.tcp_rotation_x_deg*angle_scale,Eigen::Vector3d::UnitX()).toRotationMatrix()*
      Eigen::AngleAxisd(yaw*angle_scale,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation()-=pose.linear().col(2)*depth;
    auto candidate=evaluate_pose(points,pose,config);
    // 両側支持数を優先し、同数では小さい姿勢変更。成功確率としてのスコア出力なし
    const int stage=candidate.has_contact_pair ? (candidate.has_observed_collision ? 3:4):
      (std::isfinite(candidate.observed_width) ? (candidate.reason=="width_out_of_range" ? 1:2):0);
    const double score=stage*1e6+
      static_cast<double>(std::min<std::size_t>(100000,std::min(candidate.left_support_num,candidate.right_support_num)))-
      0.01*std::abs(yaw)-depth;
    if (score>best_score) { best_score=score; best=std::move(candidate); }
  }
  return best;
}
}  // 把持候補の局所補正
