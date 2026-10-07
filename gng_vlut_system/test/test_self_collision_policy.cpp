#include <gtest/gtest.h>
#include <Eigen/Core>
#include "collision/self_collision_policy.hpp"
#include "collision/independent_arm_model.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/joint_segment_collision.hpp"
#include "robot_model/kinematic_adapter.hpp"

namespace {

void add_link(simulation::RobotModel &model, const std::string &name, bool has_shape = true) {
  simulation::LinkProperties link;
  link.name = name;
  if (has_shape) {
    simulation::Collision shape;
    shape.origin = Eigen::Isometry3d::Identity();
    shape.geometry.type = simulation::GeometryType::SPHERE;
    shape.geometry.size = Eigen::Vector3d(0.03, 0.0, 0.0);
    link.collisions.push_back(shape);
  }
  model.addLink(link);
}

void add_joint(simulation::RobotModel &model, const std::string &parent,
               const std::string &child, kinematics::JointType type,
               const Eigen::Vector3d &position = Eigen::Vector3d::Zero()) {
  simulation::JointProperties joint;
  joint.name = parent + "_to_" + child;
  joint.parent_link = parent;
  joint.child_link = child;
  joint.type = type;
  joint.origin = Eigen::Isometry3d::Identity();
  joint.origin.translation() = position;
  joint.axis = Eigen::Vector3d::UnitZ();
  joint.limits.lower = -3.14;
  joint.limits.upper = 3.14;
  model.addJoint(joint);
}

simulation::RobotModel make_model() {
  simulation::RobotModel model;
  model.setRootLinkName("body");
  for (const auto *name : {"body", "mount", "upper", "cover", "tool", "other_arm"}) {
    add_link(model, name, std::string(name) != "mount");
  }
  add_joint(model, "body", "mount", kinematics::JointType::Fixed);
  add_joint(model, "mount", "upper", kinematics::JointType::Revolute, {0.3, 0.0, 0.0});
  add_joint(model, "upper", "cover", kinematics::JointType::Fixed);
  add_joint(model, "upper", "tool", kinematics::JointType::Revolute, {-0.3, 0.0, 0.0});
  add_joint(model, "mount", "other_arm", kinematics::JointType::Revolute, {0.0, 0.3, 0.0});
  return model;
}

bool has_exclusion(const std::set<std::pair<std::string, std::string>> &pairs,
                   std::string first, std::string second) {
  if (second < first) std::swap(first, second);
  return pairs.count({first, second}) != 0;
}

} // 無名名前空間の終端

TEST(self_collision_policy, fixed_parts_and_direct_joint_only) {
  const auto pairs = simulation::collect_self_collision_exclusions(make_model());
  EXPECT_TRUE(has_exclusion(pairs, "body", "mount"));
  EXPECT_TRUE(has_exclusion(pairs, "upper", "cover"));
  EXPECT_TRUE(has_exclusion(pairs, "body", "upper"));
  EXPECT_FALSE(has_exclusion(pairs, "body", "cover"));
  EXPECT_FALSE(has_exclusion(pairs, "cover", "tool"));
  EXPECT_FALSE(has_exclusion(pairs, "body", "tool"));
  EXPECT_FALSE(has_exclusion(pairs, "upper", "other_arm"));
  EXPECT_FALSE(has_exclusion(pairs, "tool", "other_arm"));
}

TEST(self_collision_policy, root_shape_and_nonadjacent_collision_included) {
  const auto model = make_model();
  auto chain = simulation::createMultiArmKinematicChain(model, {{"body", "tool", ""}});
  simulation::GeometricSelfCollisionChecker checker(model, *chain);
  chain->updateKinematics(std::vector<double>(chain->getTotalDOF(), 0.0));
  checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  bool has_root = false;
  for (std::size_t idx = 0; idx < checker.getCollisionObjects().size(); ++idx) {
    has_root = has_root || checker.getLinkNameForObject(idx) == "body";
  }
  EXPECT_TRUE(has_root);
  EXPECT_FALSE(checker.shouldSkipCollision("body", "tool"));
  EXPECT_TRUE(checker.checkCollision());
  const auto pairs = checker.collectSelfCollisionPairs();
  EXPECT_NE(std::find(pairs.begin(), pairs.end(), std::make_pair(std::string("body"), std::string("tool"))), pairs.end());
}

TEST(self_collision_policy, missing_mesh_is_an_error) {
  auto model = make_model();
  auto link = *model.getLink("tool");
  link.collisions.front().geometry.type = simulation::GeometryType::MESH;
  link.collisions.front().geometry.mesh_filename = "/missing_self_collision_mesh.stl";
  link.collisions.front().geometry.size = Eigen::Vector3d::Ones();
  model.addLink(link);
  auto chain = simulation::createMultiArmKinematicChain(model, {{"body", "tool", ""}});
  EXPECT_THROW(simulation::GeometricSelfCollisionChecker(model, *chain), std::runtime_error);
}

TEST(joint_segment_collision, endpoints_and_fine_intermediate_samples) {
  Eigen::VectorXf first(1), second(1);
  first << 0.0f; second << 1.0f;
  EXPECT_TRUE(simulation::has_joint_segment_collision(first, second, 0.025,
      [](const Eigen::VectorXf &angles) { return angles[0] > 0.02f && angles[0] < 0.04f; }));
  EXPECT_TRUE(simulation::has_joint_segment_collision(first, second, 0.025,
      [](const Eigen::VectorXf &angles) { return angles[0] == 1.0f; }));
}

TEST(joint_segment_collision, exact_saved_endpoint_despite_float_rounding) {
  Eigen::VectorXf first(1), second(1);
  first << -3.0f; second << 0.1f;
  ASSERT_NE((first + (second - first))[0], second[0]);
  EXPECT_TRUE(simulation::has_joint_segment_collision(first, second, 0.025,
      [&second](const Eigen::VectorXf &angles) { return angles[0] == second[0]; }));
}

TEST(joint_segment_collision, tiny_segments_are_not_unconditionally_safe) {
  Eigen::VectorXf first(1), second(1);
  first << 0.0f; second << 0.0001f;
  EXPECT_TRUE(simulation::has_joint_segment_collision(first, second, 0.025,
      [](const Eigen::VectorXf &) { return true; }));
  EXPECT_FALSE(simulation::has_joint_segment_collision(first, second, 0.025,
      [](const Eigen::VectorXf &) { return false; }));
}

TEST(joint_segment_collision, invalid_input_is_an_error) {
  Eigen::VectorXf first(1), second(2);
  first.setZero(); second.setZero();
  EXPECT_THROW(simulation::has_joint_segment_collision(first, second, 0.025,
      [](const Eigen::VectorXf &) { return false; }), std::invalid_argument);
  EXPECT_THROW(simulation::has_joint_segment_collision(first, first, 0.0,
      [](const Eigen::VectorXf &) { return false; }), std::invalid_argument);
}

TEST(self_collision_policy, unregistered_backend_cannot_be_enabled_after_construction) {
  const auto model = make_model();
  auto chain = simulation::createMultiArmKinematicChain(model, {{"body", "tool", ""}});
  simulation::GeometricSelfCollisionChecker checker(model, *chain, false);
  EXPECT_THROW(checker.setUseFCLBackend(true), std::invalid_argument);
}

// 対象腕・胴体・固定外装を保持し、反対腕の可動部分だけを除いた学習形状。
TEST(independent_arm_model, preserves_body_and_mount_geometry) {
  auto model = make_model();
  add_link(model, "other_mount", false);
  add_link(model, "other_cover");
  add_link(model, "other_tip");
  add_joint(model, "other_arm", "other_tip", kinematics::JointType::Fixed);
  auto joint = *model.getJoint("mount_to_other_arm");
  joint.parent_link = "other_mount";
  model.addJoint(joint);
  add_joint(model, "body", "other_mount", kinematics::JointType::Fixed);
  add_joint(model, "other_mount", "other_cover", kinematics::JointType::Fixed);
  const auto selected = simulation::make_independent_arm_collision_model(
      model, "mount", {"mount", "other_mount"});
  EXPECT_TRUE(selected.getLink("other_arm")->collisions.empty());
  EXPECT_TRUE(selected.getLink("other_tip")->collisions.empty());
  for (const auto *name : {"body", "upper", "cover", "tool", "other_cover"}) {
    EXPECT_FALSE(selected.getLink(name)->collisions.empty()) << name;
  }
  EXPECT_FALSE(model.getLink("other_arm")->collisions.empty());
  auto chain = simulation::createMultiArmKinematicChain(selected, {{"body", "tool", ""}});
  simulation::GeometricSelfCollisionChecker checker(selected, *chain);
  chain->updateKinematics(std::vector<double>(chain->getTotalDOF(), 0.0));
  checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  EXPECT_TRUE(checker.checkCollision());
}

TEST(independent_arm_model, rejects_body_root_and_overlapping_arms) {
  const auto model = make_model();
  EXPECT_THROW(simulation::collect_moving_arm_links(model, "body"), std::invalid_argument);
  EXPECT_THROW(simulation::collect_moving_arm_links(model, "missing"), std::invalid_argument);
  EXPECT_THROW(simulation::make_independent_arm_collision_model(
      model, "mount", {"mount", "upper"}), std::invalid_argument);
  EXPECT_THROW(simulation::make_independent_arm_collision_model(
      model, "mount", {"mount", "mount"}), std::invalid_argument);
}

TEST(independent_arm_model, inter_arm_collision_is_deferred_to_combination) {
  auto model = make_model();
  add_link(model, "other_mount", false);
  add_joint(model, "body", "other_mount", kinematics::JointType::Fixed);
  auto other_joint = *model.getJoint("mount_to_other_arm");
  other_joint.parent_link = "other_mount";
  other_joint.origin.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);
  model.addJoint(other_joint);
  auto tool_joint = *model.getJoint("upper_to_tool");
  tool_joint.origin.translation() = Eigen::Vector3d(0.3, 0.0, 0.0);
  model.addJoint(tool_joint);
  const auto selected = simulation::make_independent_arm_collision_model(
      model, "mount", {"mount", "other_mount"});
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"mount", "tool", ""}, {"other_mount", "other_arm", ""}});
  chain->updateKinematics(std::vector<double>(chain->getTotalDOF(), 0.0));
  simulation::GeometricSelfCollisionChecker single_checker(selected, *chain);
  single_checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  EXPECT_FALSE(single_checker.checkCollision());
  simulation::GeometricSelfCollisionChecker full_checker(model, *chain);
  full_checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  EXPECT_TRUE(full_checker.checkCollision());
  bool has_inter_arm_pair = false;
  for (const auto &pair : full_checker.collectSelfCollisionPairs()) {
    has_inter_arm_pair = has_inter_arm_pair ||
        (pair.first == "tool" && pair.second == "other_arm") ||
        (pair.first == "other_arm" && pair.second == "tool");
  }
  EXPECT_TRUE(has_inter_arm_pair);
}
