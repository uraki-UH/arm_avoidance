//
// GNGマップファイル (.bin) のデータ一貫性を検証するための診断ツール。
//
// 機能:
// 1. GNGマップを読み込む。
// 2. 各ノードに保存されている関節角度 (weight_angle) から順運動学を再計算し、
//    エンドエフェクタの座標を求める。
// 3. 再計算した座標と、ノードに直接保存されている座標 (weight_coord) を比較し、
//    その差 (エラー) を出力する。
//
// 目的:
// - GNGマップ内のデータの自己矛盾をチェックする。
// - 順運動学モデルやGNG生成プロセスのデバッグに利用する。
//
#include "common/config_manager.hpp"
#include "core_safety/gng/GrowingNeuralGas.hpp"
#include "description/kinematic_adapter.hpp"
#include "description/urdf_loader.hpp"
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <experiment_id>" << std::endl;
    return 1;
  }
  std::string experiment_id = argv[1];
  std::string gng_file = "gng_results/" + experiment_id + "/gng.bin";

  common::ConfigManager &config = common::ConfigManager::Instance();
  if (!config.Load("config.txt")) {
    std::cerr << "Failed to load config.txt" << std::endl;
    return 1;
  }

  std::string urdf_name = config.Get("robot_urdf_path", "custom_robot");
  std::string urdf_file = "urdf/" + urdf_name + ".urdf";
  std::string leaf_link = config.Get("leaf_link_name", "link_7");

  auto model_obj = simulation::loadRobotFromUrdf(urdf_file);
  simulation::RobotModel robot_model(model_obj);
  auto arm = simulation::createKinematicChainFromModel(robot_model, leaf_link);
  arm.setBase(Eigen::Vector3d(0, 0, 0));

  GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f> gng(
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
