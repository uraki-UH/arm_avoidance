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
