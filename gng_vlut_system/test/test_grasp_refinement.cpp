#include "nodes/planning/grasp_candidate_refinement.hpp"
#include <gtest/gtest.h>
#include <random>

namespace
{
using namespace grasp_refinement;
std::vector<Eigen::Vector3d> box_sides(double width=0.04)
{
  std::vector<Eigen::Vector3d> points;
  for (double side:{-1.0,1.0})
    for (int x=-5;x<=5;++x) for (int z=0;z<8;++z)
      points.emplace_back(0.004*x,side*width/2,0.010+z*0.004);
  return points;
}
}

TEST(grasp_refinement, contact_width_and_opening_are_separate)
{
  options config;
  const auto points=box_sides();
  const auto result=evaluate_pose(points,Eigen::Isometry3d::Identity(),config);
  ASSERT_TRUE(result.has_contact_pair);
  EXPECT_TRUE(result.has_gripper_check);
  EXPECT_FALSE(result.has_observed_collision);
  EXPECT_NEAR(result.contact_width,0.040,1e-9);
  EXPECT_NEAR(result.opening_width,0.046,1e-9);
  EXPECT_EQ(result.left_support_num,88U);
  EXPECT_EQ(result.right_support_num,88U);
}

TEST(grasp_refinement, missing_side_top_only_and_lines_do_not_fabricate_width)
{
  options config;
  auto points=box_sides();
  points.resize(points.size()/2);
  EXPECT_FALSE(evaluate_pose(points,Eigen::Isometry3d::Identity(),config).has_contact_pair);
  points.clear();
  for (int x=-10;x<=10;++x) for (int y=-10;y<=10;++y) points.emplace_back(x*.002,y*.002,.02);
  EXPECT_FALSE(evaluate_pose(points,Eigen::Isometry3d::Identity(),config).has_contact_pair);
  points.clear();
  for (int x=-10;x<=10;++x) for (double y:{-.02,.02}) points.emplace_back(x*.002,y,.02);
  EXPECT_FALSE(evaluate_pose(points,Eigen::Isometry3d::Identity(),config).has_contact_pair);
}

TEST(grasp_refinement, opening_limit_is_not_clamped_to_a_false_fit)
{
  const auto result=evaluate_pose(box_sides(.072),Eigen::Isometry3d::Identity(),options{});
  EXPECT_FALSE(result.has_contact_pair);
  EXPECT_EQ(result.reason,"width_out_of_range");
  EXPECT_TRUE(std::isnan(result.opening_width));
}

TEST(grasp_refinement, finger_base_and_approach_obstacles_are_rejected)
{
  for (const Eigen::Vector3d obstacle:{Eigen::Vector3d(0,.03,.02),Eigen::Vector3d(0,0,.09),
      Eigen::Vector3d(0,.03,.13),Eigen::Vector3d(0,0,.16)}) {
    auto points=box_sides(); points.push_back(obstacle);
    const auto result=evaluate_pose(points,Eigen::Isometry3d::Identity(),options{});
    // 指位置の障害物は接触支持の不足、または掃引衝突としての棄却
    EXPECT_TRUE(!result.has_contact_pair || result.has_observed_collision);
  }
}

TEST(grasp_refinement, rigid_transform_and_recentering_preserve_contact_geometry)
{
  auto points=box_sides();
  const Eigen::Isometry3d pose=Eigen::Translation3d(1,2,3)*Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized());
  for (auto &point:points) point=pose*(point+Eigen::Vector3d(.003,.006,0));
  const auto result=evaluate_pose(points,pose,options{});
  ASSERT_TRUE(result.has_contact_pair);
  EXPECT_NEAR(result.contact_width,.04,1e-9);
  EXPECT_LT((result.pose.translation()-pose*Eigen::Vector3d(.003,.006,0)).norm(),1e-9);
}

TEST(grasp_refinement, real_tcp_axis_correction_and_local_search)
{
  options config;
  config.yaw_offsets_deg={-15,0,15}; config.insertion_depths={.025,.04,.055};
  Eigen::Isometry3d source=Eigen::Isometry3d::Identity();
  source.linear()=Eigen::AngleAxisd(3.14159265358979323846,Eigen::Vector3d::UnitX()).toRotationMatrix();
  source.translation().z()=.05;
  const auto result=refine(box_sides(),source,config);
  ASSERT_TRUE(result.has_contact_pair);
  EXPECT_FALSE(result.has_observed_collision);
  EXPECT_GT(result.pose.linear()(2,2),.999);
  EXPECT_NEAR(result.contact_width,.04,1e-9);
}

TEST(grasp_refinement, invalid_points_and_options)
{
  options config;
  auto points=box_sides();
  points.emplace_back(std::numeric_limits<double>::quiet_NaN(),0,0);
  EXPECT_TRUE(evaluate_pose(points,Eigen::Isometry3d::Identity(),config).has_contact_pair);
  EXPECT_FALSE(refine({},Eigen::Isometry3d::Identity(),config).has_contact_pair);
  config.max_width=0;
  EXPECT_THROW(validate(config),std::invalid_argument);
  config=options{}; config.min_contact_points=2;
  EXPECT_THROW(validate(config),std::invalid_argument);
  config=options{}; config.insertion_depths={1};
  EXPECT_THROW(validate(config),std::invalid_argument);
}

TEST(grasp_refinement, search_box_preserves_full_point_evaluation)
{
  options config;
  std::mt19937 random(915);
  std::uniform_real_distribution<double> value(-.3,.3);
  for (int frame=0;frame<20;++frame) {
    Eigen::Isometry3d source=Eigen::Isometry3d::Identity();
    source.translation()=Eigen::Vector3d(value(random),value(random),value(random));
    source.rotate(Eigen::AngleAxisd(value(random)*8,Eigen::Vector3d(1,2,3).normalized()));
    auto points=box_sides();
    for (int i=0;i<3000;++i) points.emplace_back(value(random),value(random),value(random));
    for (auto &point:points) point=source*point;
    const auto bounds=search_bounds(source,config);
    std::vector<Eigen::Vector3d> cropped;
    for (const auto &point:points)
      if ((point.array()>=bounds.first.array()).all() && (point.array()<=bounds.second.array()).all()) cropped.push_back(point);
    const auto full=refine(points,source,config), local=refine(cropped,source,config);
    EXPECT_EQ(full.has_contact_pair,local.has_contact_pair);
    EXPECT_EQ(full.has_observed_collision,local.has_observed_collision);
    EXPECT_EQ(full.reason,local.reason);
    EXPECT_EQ(full.left_support_num,local.left_support_num);
    EXPECT_EQ(full.right_support_num,local.right_support_num);
    if (std::isfinite(full.observed_width)) EXPECT_NEAR(full.observed_width,local.observed_width,1e-12);
  }
}
