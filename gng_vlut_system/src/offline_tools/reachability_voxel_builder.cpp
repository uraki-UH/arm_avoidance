#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <random>
#include <set>
#include <limits>
#include <nlohmann/json.hpp>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "collision/geometric_self_collision_checker.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/urdf_loader.hpp"
#include "visualization/visualization_gng.hpp"
#include "reachability/joint_sampling.hpp"

namespace {

using CellIndex = std::array<std::int32_t, 3>;

class ReachabilityVoxelBuilderNode : public rclcpp::Node {
 public:
  ReachabilityVoxelBuilderNode()
      : Node("reachability_voxel_builder") {
    std::string robot_urdf_path =
        declare_parameter<std::string>("robot_urdf_path", "");
    if (robot_urdf_path.empty()) {
      robot_urdf_path = declare_parameter<std::string>("urdf_path", "");
    }
    const std::string resource_root_dir =
        declare_parameter<std::string>("resource_root_dir", "");
    const std::string mesh_root_dir =
        declare_parameter<std::string>("mesh_root_dir", "");
    const std::string profile_name =
        declare_parameter<std::string>("reachability_voxel.profile_name", "left_arm");
    const std::string root_link = declare_parameter<std::string>(
        "reachability_voxel.root_link", "");
    const std::string eef_link = declare_parameter<std::string>(
        "reachability_voxel.eef_link", "");
    const std::string output_path = declare_parameter<std::string>(
        "reachability_voxel.output_path", "reachability_voxel_map.bin");
    const std::string frame_id = declare_parameter<std::string>(
        "reachability_voxel.frame_id", "base_link");
    const double voxel_size =
        declare_parameter<double>("reachability_voxel.voxel_size", 0.05);
    const Eigen::Vector3d min_corner(
        declare_parameter<double>("reachability_voxel.min_x",
                                  declare_parameter<double>("gng_params.min_x", -0.1)),
        declare_parameter<double>("reachability_voxel.min_y",
                                  declare_parameter<double>("gng_params.min_y", -1.0)),
        declare_parameter<double>("reachability_voxel.min_z",
                                  declare_parameter<double>("gng_params.min_z", -1.0)));
    const Eigen::Vector3d max_corner(
        declare_parameter<double>("reachability_voxel.max_x",
                                  declare_parameter<double>("gng_params.max_x", 0.5)),
        declare_parameter<double>("reachability_voxel.max_y",
                                  declare_parameter<double>("gng_params.max_y", 1.0)),
        declare_parameter<double>("reachability_voxel.max_z",
                                  declare_parameter<double>("gng_params.max_z", 1.0)));
    const int max_sample_count = declare_parameter<int>(
        "reachability_voxel.max_sample_count", 200000);
    const int max_no_new_voxel_samples = declare_parameter<int>(
        "reachability_voxel.max_no_new_voxel_samples", 50000);
    const int num_supplement_samples = declare_parameter<int>(
        "reachability_voxel.num_supplement_samples", 20000);
    const int num_validation_samples = declare_parameter<int>(
        "reachability_voxel.num_validation_samples", 10000);
    const int seed = declare_parameter<int>("reachability_voxel.seed", 20260928);
    const bool enable_self_collision = declare_parameter<bool>(
        "reachability_voxel.enable_self_collision", true);
    const auto collision_exclusions =
        declare_parameter<std::vector<std::string>>(
            "collision.self_collision_exclusion_pairs", std::vector<std::string>{});

    if (robot_urdf_path.empty() || !std::isfinite(voxel_size) || voxel_size <= 0.0 ||
        !min_corner.allFinite() || !max_corner.allFinite() ||
        num_supplement_samples < 0 || num_validation_samples < 1 || seed < 0 ||
        (max_corner.array() <= min_corner.array()).any() ||
        max_sample_count < 1 || max_no_new_voxel_samples < 1) {
      throw std::invalid_argument("reachability voxel parameter is invalid");
    }

    const std::string profile_prefix = "gng.profiles." + profile_name + ".";
    const std::string resolved_root = root_link.empty()
                                          ? declare_parameter<std::string>(
                                                profile_prefix + "root", "")
                                          : root_link;
    const std::string resolved_eef = eef_link.empty()
                                         ? declare_parameter<std::string>(
                                               profile_prefix + "eef", "")
                                         : eef_link;
    if (resolved_eef.empty()) {
      throw std::invalid_argument("reachability voxel EEF link is required");
    }

    const simulation::RobotModel model = simulation::loadRobotFromUrdf(
        robot_urdf_path, resource_root_dir, mesh_root_dir);
    // URDFの既存関節・リンク名を維持。プロファイル名は分類用であり接頭辞ではない
    const simulation::ArmConfig arm_config{resolved_root, resolved_eef, ""};
    auto chain = simulation::createMultiArmKinematicChain(model, {arm_config});
    const auto joint_limits = robot_sim::reachability::collect_joint_limits(model, *chain);
    // 全身衝突の基準はURDFルート。肩原点を全身原点へ誤適用しない構成
    auto collision_chain = simulation::createMultiArmKinematicChain(
        model, {{model.getRootLinkName(), resolved_eef, ""}});
    std::map<std::string, std::size_t> sampled_joint_indices;
    std::size_t sampled_idx = 0;
    for (int idx = 0; idx < chain->getNumJoints(); ++idx) {
      if (chain->getJointDOF(idx) == 1) sampled_joint_indices.emplace(chain->getJointName(idx), sampled_idx++);
      else if (chain->getJointDOF(idx) > 1) throw std::runtime_error("multi-DOF joint is unsupported");
    }
    std::unique_ptr<simulation::GeometricSelfCollisionChecker> self_collision_checker;
    if (enable_self_collision) {
      self_collision_checker =
          std::make_unique<simulation::GeometricSelfCollisionChecker>(model, *collision_chain);
#ifdef USE_FCL
      self_collision_checker->setStrictMode(true);
#else
      throw std::runtime_error("reachability self collision requires FCL");
#endif
      for (const auto &entry : collision_exclusions) {
        const auto separator = entry.find_first_of("|:,");
        if (separator == std::string::npos) {
          continue;
        }
        const std::string first = entry.substr(0, separator);
        const std::string second = entry.substr(separator + 1);
        if (!first.empty() && !second.empty()) {
          self_collision_checker->addCollisionExclusion(first, second);
        }
      }
    }

    const auto grid_extent = ((max_corner - min_corner) / voxel_size).array().ceil().eval();
    if (!grid_extent.isFinite().all() || (grid_extent > 1000000.0).any()) {
      throw std::invalid_argument("reachability grid extent is too large");
    }
    const Eigen::Array3i grid_size = grid_extent.cast<int>();
    const std::uint64_t target_count = static_cast<std::uint64_t>(grid_size.x()) *
                                       static_cast<std::uint64_t>(grid_size.y()) *
                                       static_cast<std::uint64_t>(grid_size.z());
    if (target_count == 0) {
      throw std::invalid_argument(
          "reachability voxel grid must contain target cells");
    }

    std::map<CellIndex, robot_sim::visualization::VisualizationGngStaticNode>
        reachable_nodes;
    std::uint64_t collision_reject_count = 0;
    std::uint64_t num_outside = 0;
    // 登録済みセルも検証時は衝突検査。登録時の証拠姿勢は平均化せず保持
    auto evaluate = [&](const std::vector<double> &joint_values, CellIndex &cell) {
      chain->updateKinematics(joint_values);
      const Eigen::Vector3d position = chain->getEEFPosition();
      if (!position.allFinite() || (position.array() < min_corner.array()).any() ||
          (position.array() >= max_corner.array()).any()) {
        ++num_outside;
        return false;
      }
      const Eigen::Array3i idx =
          ((position - min_corner) / voxel_size).array().floor().cast<int>();
      cell = {idx.x(), idx.y(), idx.z()};
      if (self_collision_checker) {
        std::vector<double> full_values;
        for (int idx = 0; idx < collision_chain->getNumJoints(); ++idx) {
          if (collision_chain->getJointDOF(idx) == 0) continue;
          const auto it = sampled_joint_indices.find(collision_chain->getJointName(idx));
          full_values.push_back(it == sampled_joint_indices.end() ? 0.0 : joint_values[it->second]);
        }
        collision_chain->updateKinematics(full_values);
        self_collision_checker->updateBodyPoses(
            collision_chain->getLinkPositions(), collision_chain->getLinkOrientations());
        if (self_collision_checker->checkCollision()) {
          if (collision_reject_count == 0) {
            for (const auto &pair : self_collision_checker->collectSelfCollisionPairs()) {
              RCLCPP_INFO(get_logger(), "First rejected sample: %s <-> %s",
                          pair.first.c_str(), pair.second.c_str());
            }
          }
          ++collision_reject_count;
          return false;
        }
      }
      return true;
    };
    auto insert_witness = [&](const CellIndex &cell, const std::vector<double> &joint_values) {
      if (reachable_nodes.count(cell)) return false;
      if (reachable_nodes.size() >= 65535U) {
        throw std::runtime_error("reachable cells exceed uint16 map capacity; increase voxel_size");
      }
      Eigen::Vector3d normal = chain->getEEFOrientation() * Eigen::Vector3d::UnitZ();
      if (normal.norm() <= 1e-9) normal = Eigen::Vector3d::UnitZ();
      else normal.normalize();
      robot_sim::visualization::VisualizationGngStaticNode node;
      node.position = (min_corner + voxel_size *
          Eigen::Vector3d(cell[0] + 0.5, cell[1] + 0.5, cell[2] + 0.5)).cast<float>();
      node.normal = normal.cast<float>();
      node.label = 1;
      node.num_safe_states = 1;
      node.representative_joint_angle = Eigen::Map<const Eigen::VectorXd>(
          joint_values.data(), static_cast<Eigen::Index>(joint_values.size())).cast<float>();
      reachable_nodes.emplace(cell, std::move(node));
      return true;
    };
    std::uint64_t no_new_voxel_sample_count = 0;
    std::uint64_t sample_count = 0;
    for (std::uint64_t sample_idx = 1;
         sample_idx <= static_cast<std::uint64_t>(max_sample_count) &&
         no_new_voxel_sample_count < static_cast<std::uint64_t>(max_no_new_voxel_samples);
         ++sample_idx) {
      if (!rclcpp::ok()) throw std::runtime_error("reachability generation interrupted");
      const auto values = robot_sim::reachability::make_halton_joint_values(joint_limits, sample_idx);
      CellIndex cell;
      ++sample_count;
      if (evaluate(values, cell) && insert_witness(cell, values)) no_new_voxel_sample_count = 0;
      else ++no_new_voxel_sample_count;
      if (sample_idx % 10000 == 0) {
        RCLCPP_INFO(get_logger(), "Reachability sampling: %llu cells=%zu",
                    static_cast<unsigned long long>(sample_idx), reachable_nodes.size());
      }
    }
    std::set<CellIndex> initial_cells;
    for (const auto &entry : reachable_nodes) initial_cells.insert(entry.first);
    auto random_values = [&](std::mt19937 &rng) {
      std::vector<double> values;
      for (const auto &limits : joint_limits) {
        values.push_back(std::uniform_real_distribution<double>(limits.first, limits.second)(rng));
      }
      return values;
    };
    std::mt19937 supplement_rng(static_cast<std::uint32_t>(seed));
    for (int idx = 0; idx < num_supplement_samples; ++idx) {
      if (!rclcpp::ok()) throw std::runtime_error("reachability supplementation interrupted");
      const auto values = random_values(supplement_rng);
      CellIndex cell;
      if (evaluate(values, cell)) insert_witness(cell, values);
    }
    // 補完とは別系列の検査。検査中のmap更新なし
    std::mt19937 validation_rng(static_cast<std::uint32_t>(seed) ^ 0x9e3779b9U);
    std::uint64_t num_valid = 0, num_hit_before = 0, num_hit_after = 0;
    const auto num_collision_before = collision_reject_count;
    const auto num_outside_before = num_outside;
    for (int idx = 0; idx < num_validation_samples; ++idx) {
      if (!rclcpp::ok()) throw std::runtime_error("reachability validation interrupted");
      CellIndex cell;
      if (!evaluate(random_values(validation_rng), cell)) continue;
      ++num_valid;
      num_hit_before += initial_cells.count(cell);
      num_hit_after += reachable_nodes.count(cell);
    }
    if (reachable_nodes.empty() || num_valid == 0) {
      throw std::runtime_error("no collision-free reachability samples; check model and exclusions");
    }
    nlohmann::json report = {
        {"profile", profile_name}, {"root_link", resolved_root}, {"eef_link", resolved_eef},
        {"urdf_path", robot_urdf_path}, {"frame_id", frame_id}, {"voxel_size", voxel_size},
        {"min_corner", {min_corner.x(), min_corner.y(), min_corner.z()}},
        {"max_corner", {max_corner.x(), max_corner.y(), max_corner.z()}},
        {"seed", seed}, {"num_initial_samples", sample_count},
        {"num_initial_cells", initial_cells.size()}, {"num_cells", reachable_nodes.size()},
        {"num_supplement_samples", num_supplement_samples},
        {"enable_self_collision", enable_self_collision}, {"collision_exclusions", collision_exclusions},
        {"other_joints", "zero"}, {"environment_collision_checked", false},
        {"validation", {{"num_samples", num_validation_samples}, {"num_valid", num_valid},
          {"num_collision_rejected", collision_reject_count - num_collision_before},
          {"num_outside", num_outside - num_outside_before},
          {"num_hit_before", num_hit_before}, {"num_hit_after", num_hit_after},
          {"hit_fraction_before", static_cast<double>(num_hit_before) / num_valid},
          {"hit_fraction_after", static_cast<double>(num_hit_after) / num_valid}}}};
    report["joint_names"] = nlohmann::json::array();
    for (int idx = 0; idx < chain->getNumJoints(); ++idx) {
      if (chain->getJointDOF(idx) > 0) report["joint_names"].push_back(chain->getJointName(idx));
    }
    report["joint_limits"] = joint_limits;

    robot_sim::visualization::VisualizationGngStaticModel map_model;
    map_model.joint_angle_dimension = static_cast<std::uint32_t>(joint_limits.size());
    map_model.nodes.reserve(reachable_nodes.size());
    std::map<CellIndex, std::uint32_t> node_ids;
    for (const auto &[cell, node] : reachable_nodes) {
      node_ids.emplace(cell, static_cast<std::uint32_t>(map_model.nodes.size()));
      map_model.nodes.push_back(node);
    }
    const std::array<CellIndex, 3> neighbor_steps{
        CellIndex{1, 0, 0}, CellIndex{0, 1, 0}, CellIndex{0, 0, 1}};
    for (const auto &[cell, source_id] : node_ids) {
      for (const auto &step : neighbor_steps) {
        const CellIndex neighbor{cell[0] + step[0], cell[1] + step[1],
                                 cell[2] + step[2]};
        const auto target_it = node_ids.find(neighbor);
        if (target_it != node_ids.end()) {
          map_model.edges.emplace_back(source_id, target_it->second);
        }
      }
    }
    std::string error;
    if (!map_model.save(output_path, &error)) {
      throw std::runtime_error(error);
    }
    std::ofstream report_file(output_path + ".json");
    report_file << report.dump(2) << '\n';
    report_file.close();
    if (!report_file) throw std::runtime_error("failed to save reachability report");
    RCLCPP_INFO(get_logger(), "Independent cell coverage: before=%.5f after=%.5f valid=%llu",
                static_cast<double>(num_hit_before) / num_valid,
                static_cast<double>(num_hit_after) / num_valid,
                static_cast<unsigned long long>(num_valid));
    RCLCPP_INFO(
        get_logger(),
        "Reachability voxel map: frame=%s profile=%s target_cells=%llu reachable_cells=%zu sample_count=%llu collision_reject=%llu voxel_size=%.4f output=%s",
        frame_id.c_str(), profile_name.c_str(),
        static_cast<unsigned long long>(target_count), map_model.nodes.size(),
        static_cast<unsigned long long>(sample_count),
        static_cast<unsigned long long>(collision_reject_count), voxel_size,
        output_path.c_str());
  }
};

}  // 無名名前空間

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    std::make_shared<ReachabilityVoxelBuilderNode>();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "reachability_voxel_builder: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
