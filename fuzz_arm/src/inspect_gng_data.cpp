#include "common/config_manager.hpp"
#include "gng/GrowingNeuralGas_offline.hpp"
#include "simulation/robot/kinematic_adapter.hpp"
#include "simulation/robot/urdf_loader.hpp"
#include <iostream>

int main() {
  common::ConfigManager &config = common::ConfigManager::Instance();
  if (!config.Load("config.txt")) {
    std::cerr << "Failed to load config.txt" << std::endl;
    return 1;
  }

  std::string urdf_name = config.Get("robot_urdf_path", "custom_robot");
  std::string urdf_file = "urdf/" + urdf_name + ".urdf";
  std::string leaf_link = config.Get("leaf_link_name", "link_7");
  std::string exp_id = config.Get("experiment_id", "default_run");
  std::string suffix = config.Get("online_input_suffix", "_final");
  std::string data_dir = config.Get("data_directory", "gng_results");
  if (!data_dir.empty() && data_dir.back() != '/')
    data_dir += "/";

  std::string gng_file = data_dir + exp_id + suffix + ".bin";

  auto model_obj = simulation::loadRobotFromUrdf(urdf_file);
  simulation::RobotModel robot_model(model_obj);
  auto arm = simulation::createKinematicChainFromModel(robot_model, leaf_link);
  arm.setBase(Eigen::Vector3d(0, 0, 0));

  GNG::GrowingNeuralGas2<Eigen::VectorXf, Eigen::Vector3f> gng(
      arm.getTotalDOF(), 3, &arm);
  if (!gng.load(gng_file)) {
    std::cerr << "Failed to load GNG: " << gng_file << std::endl;
    return 1;
  }

  std::cout << "GNG Loaded. DOF=" << arm.getTotalDOF() << std::endl;

  int count = 0;
  for (int i = 0; i < (int)gng.getMaxNodeNum() && count < 10; ++i) {
    const auto &node = gng.nodeAt(i);
    if (node.id != -1 && node.status.active) {
      std::vector<double> joint_vals(arm.getTotalDOF());
      for (int j = 0; j < arm.getTotalDOF(); ++j)
        joint_vals[j] = node.weight_angle(j);

      arm.setJointValues(joint_vals);
      arm.forwardKinematics();
      Eigen::Vector3d fresh_eef = arm.getEEFPosition();

      std::cout << "Node ID " << node.id << ":\n"
                << "  Stored: [" << node.weight_coord.transpose() << "]\n"
                << "  Fresh:  [" << fresh_eef.cast<float>().transpose() << "]\n"
                << "  Error:  "
                << (node.weight_coord - fresh_eef.cast<float>()).norm()
                << std::endl;
      count++;
    }
  }

  return 0;
}
