// 保存2ノードの補間角度と通常strictFilterの呼出し列の照合
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/joint_segment_collision.hpp"
#include "gng/GrowingNeuralGas.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;
using graph_type = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

class traced_chain : public kinematics::KinematicChain {
public:
  explicit traced_chain(kinematics::KinematicChain &base) : base_(base) {}
  int getTotalDOF() const override { return base_.getTotalDOF(); }
  std::vector<double> sampleRandomJointValues() const override { return base_.getJointValues(); }
  void sampleRandomJointValues(std::vector<double> &values) const override { values = base_.getJointValues(); }
  void updateKinematics(const std::vector<double> &values) override { base_.updateKinematics(values); }
  Eigen::Vector3d getEEFPosition(std::size_t idx) const override { return base_.getEEFPosition(idx); }
  void forwardKinematicsAt(const std::vector<double> &values,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> &orientations) const override {
    samples.push_back(values);
    base_.forwardKinematicsAt(values, positions, orientations);
  }
  mutable json samples = json::array();
private:
  kinematics::KinematicChain &base_;
};

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  std::ifstream input(argv[1]);
  const json config = json::parse(input);
  auto model = simulation::loadRobotFromUrdf(config.at("urdf"), config.at("resource_root"), config.at("mesh_root"));
  auto chain = simulation::createMultiArmKinematicChain(model, {{config.at("root"), config.at("eef"), ""}});
  simulation::GeometricSelfCollisionChecker checker(model, *chain, true, config.at("voxel_size"));
  checker.setStrictMode(true);
  for (const auto &pair : config.at("exclusions")) checker.addCollisionExclusion(pair[0], pair[1]);
  traced_chain traced(*chain);
  graph_type graph(chain->getTotalDOF(), 3, &traced);
  if (!graph.load(argv[2]) || graph.getActiveIndices().size() != 2) return 3;
  const auto ids = graph.getActiveIndices();
  const auto first = graph.nodeAt(ids[0]).weight_angle;
  const auto second = graph.nodeAt(ids[1]).weight_angle;
  json shared_samples = json::array();
  const bool is_shared_colliding = simulation::has_joint_segment_collision(first, second, 0.025,
      [&](const Eigen::VectorXf &q) {
        std::vector<double> values(q.data(), q.data() + q.size());
        shared_samples.push_back(values);
        chain->updateKinematics(values);
        checker.updateBodyPoses(chain->getLinkPositions(), chain->getLinkOrientations());
        return checker.checkCollision();
      });
  traced.samples.clear();
  graph.setSelfCollisionChecker(&checker);
  graph.strictFilter();
  json result = {{"is_shared_colliding", is_shared_colliding}, {"shared_samples", shared_samples},
      {"strict_samples", traced.samples}, {"num_remaining_edges", graph.getNeighborsAngle(ids[0]).size()}};
  std::ofstream output(argv[3]);
  output << result.dump(2) << '\n';
  std::cout << "PROBE " << argv[3] << std::endl;
}
