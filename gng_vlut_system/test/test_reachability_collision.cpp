#include <gtest/gtest.h>
#include "collision/geometric_self_collision_checker.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "reachability/joint_sampling.hpp"

TEST(reachability_collision, one_mesh_object_per_collision_shape) {
  const std::string root = std::string(workspace_source_dir) + "/urdf/topo_dual_arm_max";
  const auto model = simulation::loadRobotFromUrdf(
      root + "/topo_dual_arm_max.urdf", root, root + "/meshes");
  auto chain = simulation::createMultiArmKinematicChain(
      model, {{model.getRootLinkName(), "L_tcp", ""}});
  simulation::GeometricSelfCollisionChecker checker(model, *chain);
  checker.setStrictMode(true);
  const auto &objects = checker.getCollisionObjects();
  ASSERT_FALSE(objects.empty());
  EXPECT_EQ(checker.getFCLDetector().getRobotLinks().size(), objects.size());
  auto values = std::vector<double>(chain->getTotalDOF(), 0.0);
  values[0] = 0.4;
  chain->updateKinematics(values);
  checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
  bool has_checked_link = false;
  for (std::size_t idx = 0; idx < objects.size(); ++idx) {
    ASSERT_NE(checker.getFCLObject(idx), nullptr);
    EXPECT_EQ(checker.getFCLObject(idx), checker.getFCLDetector().getRobotLink(idx));
    if (checker.getLinkNameForObject(idx) != "L_link4") continue;
    Eigen::Isometry3d transform;
    ASSERT_TRUE(chain->getLinkTransform("L_link4", transform));
    const auto &collision = model.getLink("L_link4")->collisions.front();
    EXPECT_LT((checker.getFCLObject(idx)->getTransform().matrix() -
               (transform * collision.origin).matrix()).norm(), 1e-10);
    has_checked_link = true;
  }
  EXPECT_TRUE(has_checked_link);
  EXPECT_EQ(checker.checkCollision(), !checker.collectSelfCollisionPairs().empty());
}

TEST(reachability_collision, sampling_uses_urdf_limits) {
  const std::string root = std::string(workspace_source_dir) + "/urdf/topo_dual_arm_max";
  const auto model = simulation::loadRobotFromUrdf(
      root + "/topo_dual_arm_max.urdf", root, root + "/meshes");
  auto chain = simulation::createMultiArmKinematicChain(
      model, {{"L_shoulder_mount", "L_tcp", ""}});
  const auto limits = robot_sim::reachability::collect_joint_limits(model, *chain);
  ASSERT_EQ(limits.size(), 7U);
  EXPECT_DOUBLE_EQ(limits[1].first, model.getJoint("L_joint2")->limits.lower);
  EXPECT_DOUBLE_EQ(limits[3].second, model.getJoint("L_joint4")->limits.upper);
  for (std::uint64_t idx = 1; idx <= 100; ++idx) {
    const auto values = robot_sim::reachability::make_halton_joint_values(limits, idx);
    chain->updateKinematics(values);
    const auto actual = chain->getJointValues();
    ASSERT_EQ(actual.size(), values.size());
    for (std::size_t joint = 0; joint < values.size(); ++joint) {
      EXPECT_GE(values[joint], limits[joint].first);
      EXPECT_LE(values[joint], limits[joint].second);
      EXPECT_DOUBLE_EQ(actual[joint], values[joint]);
    }
  }
}


namespace {

Eigen::Isometry3d make_test_origin(const Eigen::Vector3d &translation,
                                   double angle = 0.0) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(translation);
  transform.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
  return transform;
}

void add_test_joint(simulation::RobotModel &model, const std::string &name,
                    const std::string &parent, const std::string &child,
                    kinematics::JointType type, const Eigen::Isometry3d &origin) {
  simulation::LinkProperties link;
  link.name = child;
  model.addLink(link);
  simulation::JointProperties joint;
  joint.name = name;
  joint.parent_link = parent;
  joint.child_link = child;
  joint.type = type;
  joint.origin = origin;
  joint.axis = Eigen::Vector3d::UnitZ();
  joint.limits.lower = -2.0;
  joint.limits.upper = 2.0;
  model.addJoint(joint);
}

simulation::RobotModel make_dual_arm_test_model() {
  simulation::RobotModel model;
  model.setRootLinkName("world_root");
  simulation::LinkProperties root;
  root.name = model.getRootLinkName();
  model.addLink(root);
  add_test_joint(model, "body_fixed", "world_root", "body",
                 kinematics::JointType::Fixed,
                 make_test_origin({0.1, -0.2, 0.3}, 0.15));
  for (const auto &side : {std::string("left"), std::string("right")}) {
    const double sign = side == "left" ? 1.0 : -1.0;
    add_test_joint(model, side + "_mount", "body", side + "_shoulder",
                   kinematics::JointType::Fixed,
                   make_test_origin({0.2, sign * 0.4, 0.1}, sign * 0.2));
    add_test_joint(model, side + "_joint", side + "_shoulder", side + "_link",
                   kinematics::JointType::Revolute,
                   make_test_origin({0.05, 0.0, 0.02}, sign * 0.1));
    add_test_joint(model, side + "_tip", side + "_link", side + "_tcp",
                   kinematics::JointType::Fixed,
                   make_test_origin({0.5, 0.0, 0.02}));
    add_test_joint(model, side + "_guard_fixed", side + "_link", side + "_guard",
                   kinematics::JointType::Fixed,
                   make_test_origin({0.1, 0.2, -0.03}));
  }
  add_test_joint(model, "head_joint", "body", "head",
                 kinematics::JointType::Revolute,
                 make_test_origin({0.0, 0.0, 0.5}, 0.3));
  return model;
}

std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
all_test_joint_origins(const simulation::RobotModel &model) {
  std::map<std::string, std::pair<std::string, Eigen::Isometry3d>> result;
  for (const auto &entry : model.getJoints()) {
    result[entry.second.child_link] = {entry.second.parent_link, entry.second.origin};
  }
  return result;
}

void expect_dual_arm_transforms(
    const simulation::RobotModel &model,
    const std::map<std::string, Eigen::Isometry3d> &transforms,
    const Eigen::Isometry3d &global_base, double left_angle, double right_angle,
    const std::string &left_prefix = "", const std::string &right_prefix = "") {
  const auto body = global_base * model.getJoint("body_fixed")->origin;
  for (const auto &side : {std::string("left"), std::string("right")}) {
    const std::string &prefix = side == "left" ? left_prefix : right_prefix;
    const double angle = side == "left" ? left_angle : right_angle;
    const auto shoulder = body * model.getJoint(side + "_mount")->origin;
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    motion.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    const auto link = shoulder * model.getJoint(side + "_joint")->origin * motion;
    const std::map<std::string, Eigen::Isometry3d> expected{
        {"world_root", global_base}, {"body", body},
        {side + "_shoulder", shoulder}, {side + "_link", link},
        {side + "_tcp", link * model.getJoint(side + "_tip")->origin},
        {side + "_guard", link * model.getJoint(side + "_guard_fixed")->origin}};
    for (const auto &entry : expected) {
      SCOPED_TRACE(prefix + entry.first);
      const auto actual = transforms.find(prefix + entry.first);
      ASSERT_NE(actual, transforms.end());
      EXPECT_TRUE(actual->second.matrix().isApprox(entry.second.matrix(), 1e-12));
    }
  }
}

}  // 無名名前空間の終端

TEST(reachability_collision, both_nonzero_arms_keep_actual_full_tree_transforms) {
  const auto model = make_dual_arm_test_model();
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"left_shoulder", "left_tcp", ""}, {"right_shoulder", "right_tcp", ""}});
  chain->updateKinematics(std::vector<double>{0.65, -0.45});
  std::map<std::string, Eigen::Isometry3d> transforms;
  chain->buildAllLinkTransforms(chain->getLinkPositions(), chain->getLinkOrientations(),
                                all_test_joint_origins(model), transforms);
  expect_dual_arm_transforms(model, transforms, Eigen::Isometry3d::Identity(), 0.65, -0.45);
  EXPECT_EQ(transforms.size(), model.getLinks().size());
  const auto expected_head = model.getJoint("body_fixed")->origin * model.getJoint("head_joint")->origin;
  ASSERT_NE(transforms.find("head"), transforms.end());
  EXPECT_TRUE(transforms.at("head").matrix().isApprox(expected_head.matrix(), 1e-12));
}

TEST(reachability_collision, factory_applies_global_translation_once) {
  const auto model = make_dual_arm_test_model();
  const Eigen::Vector3d position(0.8, -0.4, 1.2);
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"left_shoulder", "left_tcp", ""}, {"right_shoulder", "right_tcp", ""}}, position);
  chain->updateKinematics(std::vector<double>{0.65, -0.45});
  std::map<std::string, Eigen::Isometry3d> transforms;
  chain->buildAllLinkTransforms(chain->getLinkPositions(), chain->getLinkOrientations(),
                                model.getFixedLinkInfo(), transforms);
  expect_dual_arm_transforms(model, transforms, make_test_origin(position), 0.65, -0.45);
}

TEST(reachability_collision, fixed_branches_follow_rotated_global_base) {
  const auto model = make_dual_arm_test_model();
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"left_shoulder", "left_tcp", ""}, {"right_shoulder", "right_tcp", ""}});
  const auto global_base = make_test_origin({0.8, -0.4, 1.2}, 0.35);
  chain->setBase(global_base.translation(), Eigen::Quaterniond(global_base.rotation()));
  chain->updateKinematics(std::vector<double>{0.65, -0.45});
  std::map<std::string, Eigen::Isometry3d> transforms;
  chain->buildAllLinkTransforms(chain->getLinkPositions(), chain->getLinkOrientations(),
                                model.getFixedLinkInfo(), transforms);
  expect_dual_arm_transforms(model, transforms, global_base, 0.65, -0.45);
}

TEST(reachability_collision, supplied_fk_state_keeps_both_arms_without_mutation) {
  const auto model = make_dual_arm_test_model();
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"left_shoulder", "left_tcp", ""}, {"right_shoulder", "right_tcp", ""}});
  chain->updateKinematics(std::vector<double>{0.0, 0.0});
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> positions;
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> orientations;
  chain->forwardKinematicsAt(std::vector<double>{0.65, -0.45}, positions, orientations);
  std::map<std::string, Eigen::Isometry3d> transforms;
  chain->buildAllLinkTransforms(positions, orientations, all_test_joint_origins(model), transforms);
  expect_dual_arm_transforms(model, transforms, Eigen::Isometry3d::Identity(), 0.65, -0.45);
  EXPECT_EQ(chain->getJointValues(), (std::vector<double>{0.0, 0.0}));
}

TEST(reachability_collision, prefixes_keep_model_roots_and_arm_transforms_separate) {
  const auto model = make_dual_arm_test_model();
  const Eigen::Vector3d position(0.8, -0.4, 1.2);
  auto chain = simulation::createMultiArmKinematicChain(model,
      {{"left_shoulder", "left_tcp", "first_"}, {"right_shoulder", "right_tcp", "second_"}}, position);
  chain->updateKinematics(std::vector<double>{0.65, -0.45});
  std::map<std::string, Eigen::Isometry3d> transforms;
  chain->buildAllLinkTransforms(chain->getLinkPositions(), chain->getLinkOrientations(),
                                all_test_joint_origins(model), transforms);
  expect_dual_arm_transforms(model, transforms, make_test_origin(position), 0.65, -0.45,
                            "first_", "second_");
}
