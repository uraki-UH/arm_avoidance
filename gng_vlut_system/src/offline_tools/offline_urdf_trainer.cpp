#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <sstream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "collision/composite_collision_checker.hpp"
#include "collision/environment_collision_checker.hpp"
#include "collision/geometric_self_collision_checker.hpp"
#include "collision/voxel_collision_checker.hpp"
#include "common/resource_utils.hpp"
#include "common/voxel_utils.hpp"
#include "kinematics/kinematic_chain.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/urdf_loader.hpp"
#include "safety_engine/gng/GrowingNeuralGas.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/runtime/gng_geometric_self_collision_provider.hpp"
#include "safety_engine/runtime/gng_status_providers.hpp"

#ifdef USE_FCL
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#endif

using GNG2 = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;

static std::vector<std::string> splitCommaSeparated(const std::string &text) {
  std::vector<std::string> items;
  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    auto begin = token.find_first_not_of(" \t");
    auto end = token.find_last_not_of(" \t");
    if (begin == std::string::npos) {
      continue;
    }
    items.push_back(token.substr(begin, end - begin + 1));
  }
  return items;
}

static std::string trimWhitespace(const std::string &text) {
  const auto begin = text.find_first_not_of(" \t");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t");
  return text.substr(begin, end - begin + 1);
}

static std::vector<std::pair<std::string, std::string>>
parseLinkPairs(const std::vector<std::string> &entries) {
  std::vector<std::pair<std::string, std::string>> pairs;
  for (const auto &entry : entries) {
    const auto sep = entry.find_first_of("|:,\t");
    if (sep == std::string::npos) {
      continue;
    }
    std::string lhs = trimWhitespace(entry.substr(0, sep));
    std::string rhs = trimWhitespace(entry.substr(sep + 1));
    if (!lhs.empty() && !rhs.empty()) {
      pairs.emplace_back(lhs, rhs);
    }
  }
  return pairs;
}

static std::string makePairKey(std::string a, std::string b) {
  a = trimWhitespace(a);
  b = trimWhitespace(b);
  if (a > b) {
    std::swap(a, b);
  }
  return a + "|" + b;
}

struct JointSelectionSpec {
  std::vector<int> selected_joint_indices;
  std::vector<std::string> selected_joint_names;
  std::unordered_set<int> selected_joint_index_set;
  int selected_dof = 0;
};

static JointSelectionSpec buildJointSelectionSpec(
    const kinematics::KinematicChain &chain,
    const std::vector<std::string> &exclude_joint_names) {
  JointSelectionSpec spec;
  std::unordered_set<std::string> exclude_set(exclude_joint_names.begin(),
                                              exclude_joint_names.end());
  for (int joint_idx = 0; joint_idx < chain.getNumJoints(); ++joint_idx) {
    const std::string joint_name = chain.getJointName(joint_idx);
    if (exclude_set.count(joint_name) > 0) {
      continue;
    }
    const int joint_dof = chain.getJointDOF(joint_idx);
    if (joint_dof <= 0) {
      continue;
    }
    spec.selected_joint_indices.push_back(joint_idx);
    spec.selected_joint_names.push_back(joint_name);
    spec.selected_joint_index_set.insert(joint_idx);
    spec.selected_dof += joint_dof;
  }
  return spec;
}

static std::vector<double> packSelectedJointValues(
    const kinematics::KinematicChain &chain, const JointSelectionSpec &spec,
    const std::vector<double> &full_values) {
  std::vector<double> selected_values;
  selected_values.reserve(static_cast<std::size_t>(std::max(0, spec.selected_dof)));

  std::size_t full_cursor = 0;
  for (int joint_idx = 0; joint_idx < chain.getNumJoints(); ++joint_idx) {
    const int joint_dof = chain.getJointDOF(joint_idx);
    if (joint_dof <= 0) {
      continue;
    }

    if (spec.selected_joint_index_set.count(joint_idx) > 0) {
      for (int d = 0; d < joint_dof; ++d) {
        const std::size_t full_pos = full_cursor + static_cast<std::size_t>(d);
        if (full_pos < full_values.size()) {
          selected_values.push_back(full_values[full_pos]);
        } else {
          selected_values.push_back(0.0);
        }
      }
    }
    full_cursor += static_cast<std::size_t>(joint_dof);
  }

  return selected_values;
}

static std::vector<double> expandSelectedJointValues(
    const kinematics::KinematicChain &chain, const JointSelectionSpec &spec,
    const std::vector<double> &selected_values) {
  std::vector<double> full_values(
      static_cast<std::size_t>(std::max(0, chain.getTotalDOF())), 0.0);

  std::size_t selected_cursor = 0;
  std::size_t full_cursor = 0;
  for (int joint_idx = 0; joint_idx < chain.getNumJoints(); ++joint_idx) {
    const int joint_dof = chain.getJointDOF(joint_idx);
    if (joint_dof <= 0) {
      continue;
    }

    if (spec.selected_joint_index_set.count(joint_idx) > 0) {
      for (int d = 0; d < joint_dof; ++d) {
        if (selected_cursor < selected_values.size() &&
            full_cursor + static_cast<std::size_t>(d) < full_values.size()) {
          full_values[full_cursor + static_cast<std::size_t>(d)] =
              selected_values[selected_cursor];
        }
        ++selected_cursor;
      }
    }

    full_cursor += static_cast<std::size_t>(joint_dof);
  }

  return full_values;
}

class SelectedJointKinematicChain : public kinematics::KinematicChain {
public:
  SelectedJointKinematicChain(kinematics::KinematicChain *base_chain,
                              JointSelectionSpec selection)
      : base_chain_(base_chain), selection_(std::move(selection)) {}

  int getTotalDOF() const override { return selection_.selected_dof; }

  int getNumJoints() const override {
    return base_chain_ ? base_chain_->getNumJoints() : 0;
  }

  std::size_t getArmCount() const override {
    return base_chain_ ? base_chain_->getArmCount() : 0;
  }

  std::string getJointName(int joint_index) const override {
    return base_chain_ ? base_chain_->getJointName(joint_index) : "";
  }

  int getJointDOF(int joint_index) const override {
    return base_chain_ ? base_chain_->getJointDOF(joint_index) : 0;
  }

  std::vector<double> getJointValues() const override {
    if (!base_chain_) {
      return {};
    }
    return packSelectedJointValues(*base_chain_, selection_,
                                   base_chain_->getJointValues());
  }

  bool setJointValues(const std::vector<double> &values) override {
    if (!base_chain_) {
      return false;
    }
    return base_chain_->setJointValues(
        expandSelectedJointValues(*base_chain_, selection_, values));
  }

  void updateKinematics(const std::vector<double> &values) override {
    if (!base_chain_) {
      return;
    }
    base_chain_->updateKinematics(
        expandSelectedJointValues(*base_chain_, selection_, values));
  }

  void forwardKinematicsAt(
      const std::vector<double> &values,
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          &out_positions,
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          &out_orientations) const override {
    if (!base_chain_) {
      out_positions.clear();
      out_orientations.clear();
      return;
    }
    base_chain_->forwardKinematicsAt(
        expandSelectedJointValues(*base_chain_, selection_, values),
        out_positions, out_orientations);
  }

  void forwardKinematicsAt(const std::vector<double> &values) override {
    if (!base_chain_) {
      return;
    }
    base_chain_->forwardKinematicsAt(
        expandSelectedJointValues(*base_chain_, selection_, values));
  }

  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &
  getLinkPositions() const override {
    static const std::vector<Eigen::Vector3d,
                             Eigen::aligned_allocator<Eigen::Vector3d>>
        empty;
    return base_chain_ ? base_chain_->getLinkPositions() : empty;
  }

  const std::vector<Eigen::Quaterniond,
                    Eigen::aligned_allocator<Eigen::Quaterniond>> &
  getLinkOrientations() const override {
    static const std::vector<Eigen::Quaterniond,
                             Eigen::aligned_allocator<Eigen::Quaterniond>>
        empty;
    return base_chain_ ? base_chain_->getLinkOrientations() : empty;
  }

  Eigen::Vector3d getEEFPosition() const override {
    return base_chain_ ? base_chain_->getEEFPosition()
                       : Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond getEEFOrientation() const override {
    return base_chain_ ? base_chain_->getEEFOrientation()
                       : Eigen::Quaterniond::Identity();
  }

  Eigen::Vector3d getEEFPosition(std::size_t arm_index) const override {
    return base_chain_ ? base_chain_->getEEFPosition(arm_index)
                       : Eigen::Vector3d::Zero();
  }

  Eigen::Quaterniond getEEFOrientation(std::size_t arm_index) const override {
    return base_chain_ ? base_chain_->getEEFOrientation(arm_index)
                       : Eigen::Quaterniond::Identity();
  }

  void setBase(const Eigen::Vector3d &position,
               const Eigen::Quaterniond &orientation =
                   Eigen::Quaterniond::Identity()) override {
    if (base_chain_) {
      base_chain_->setBase(position, orientation);
    }
  }

  void buildAllLinkTransforms(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &positions,
      const std::vector<Eigen::Quaterniond,
                        Eigen::aligned_allocator<Eigen::Quaterniond>>
          &orientations,
      const std::map<std::string, std::pair<std::string, Eigen::Isometry3d>>
          &fixed_link_info,
      std::map<std::string, Eigen::Isometry3d> &link_transforms) const override {
    if (base_chain_) {
      base_chain_->buildAllLinkTransforms(positions, orientations,
                                          fixed_link_info, link_transforms);
    } else {
      link_transforms.clear();
    }
  }

  Eigen::MatrixXd calculateJacobianAt(
      int target_joint_index, const std::vector<double> &values) const override {
    if (!base_chain_) {
      return Eigen::MatrixXd::Zero(0, 0);
    }
    return base_chain_->calculateJacobianAt(
        target_joint_index,
        expandSelectedJointValues(*base_chain_, selection_, values));
  }

  Eigen::MatrixXd calculateJacobian(int target_joint_index) const override {
    return base_chain_ ? base_chain_->calculateJacobian(target_joint_index)
                       : Eigen::MatrixXd::Zero(0, 0);
  }

  std::vector<double> sampleRandomJointValues() const override {
    if (!base_chain_) {
      return {};
    }
    return packSelectedJointValues(*base_chain_, selection_,
                                   base_chain_->sampleRandomJointValues());
  }

  std::vector<double> sampleRandomJointValue(int joint_index) const override {
    if (!base_chain_) {
      return {};
    }
    return packSelectedJointValues(*base_chain_, selection_,
                                   base_chain_->sampleRandomJointValue(joint_index));
  }

  std::vector<double> sampleRandomJointValues(
      const std::vector<int> &joint_indices) const override {
    if (!base_chain_) {
      return {};
    }
    return packSelectedJointValues(
        *base_chain_, selection_,
        base_chain_->sampleRandomJointValues(joint_indices));
  }

  bool isWithinLimits(const std::vector<double> &values) const override {
    if (!base_chain_) {
      return false;
    }
    return base_chain_->isWithinLimits(
        expandSelectedJointValues(*base_chain_, selection_, values));
  }

  void clampToLimits(std::vector<double> &values) const override {
    if (!base_chain_) {
      return;
    }
    auto full = expandSelectedJointValues(*base_chain_, selection_, values);
    base_chain_->clampToLimits(full);
    values = packSelectedJointValues(*base_chain_, selection_, full);
  }

private:
  kinematics::KinematicChain *base_chain_;
  JointSelectionSpec selection_;
};

static void writeInitialCollisionApprovalYaml(
    const std::filesystem::path &output_dir,
    const std::vector<std::pair<std::string, std::string>> &pairs) {
  std::filesystem::create_directories(output_dir);
  std::ofstream ofs(output_dir / "initial_collision_approval.yaml");
  if (!ofs.is_open()) {
    return;
  }

  ofs << "# Generated by offline_urdf_trainer\n";
  ofs << "# Paste this under the `collision:` section of your YAML.\n";
  ofs << "collision:\n";
  if (pairs.empty()) {
    ofs << "  approved_initial_collision_pairs: []\n";
    return;
  }
  ofs << "  approved_initial_collision_pairs:\n";

  std::vector<std::pair<std::string, std::string>> sorted_pairs = pairs;
  std::sort(sorted_pairs.begin(), sorted_pairs.end());
  sorted_pairs.erase(
      std::unique(sorted_pairs.begin(), sorted_pairs.end()),
      sorted_pairs.end());

  for (const auto &pair : sorted_pairs) {
    ofs << "    - \"" << pair.first << "|" << pair.second << "\"\n";
  }
}

static std::string vec3ToString(const Eigen::Vector3d &v, int prec = 3) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(prec)
      << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
  return oss.str();
}

static std::string vec3iToString(const Eigen::Vector3i &v) {
  std::ostringstream oss;
  oss << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
  return oss.str();
}

static void writeVoxelValidationReport(
    const rclcpp::Logger &logger,
    const simulation::VoxelCollisionChecker &voxel_checker,
    double voxel_size,
    const std::vector<std::string> &focus_links,
    int max_print_voxels,
    const std::string &dump_path) {
  const auto &link_data = voxel_checker.getLinkVoxelDataList();
  const auto checker_masks = voxel_checker.getLinkVoxelMasks();
  const auto named_tfs = voxel_checker.getCurrentLinkTransforms();

  std::unordered_set<std::string> focus_set(focus_links.begin(), focus_links.end());
  const bool show_all = focus_set.empty();

  GNG::Analysis::IndexVoxelGrid grid(voxel_size);

  std::ostringstream report;
  report << "Voxel link-mask validation report\n";
  report << "voxel_size=" << voxel_size << "\n";
  report << "link_count=" << link_data.size() << "\n";
  report << "focus_links=";
  if (show_all) {
    report << "(all)\n\n";
  } else {
    for (std::size_t i = 0; i < focus_links.size(); ++i) {
      report << (i == 0 ? " " : ", ") << focus_links[i];
    }
    report << "\n\n";
  }

  std::size_t mismatch_count = 0;
  for (std::size_t i = 0; i < link_data.size(); ++i) {
    const auto &data = link_data[i];
    const bool has_tf = i < named_tfs.size() && named_tfs[i].first == data.name;
    const Eigen::Isometry3d tf =
        has_tf ? named_tfs[i].second : Eigen::Isometry3d::Identity();

    std::vector<long> direct_mask;
    direct_mask.reserve(data.local_voxel_centers.size());
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3i, long>>
        samples;
    samples.reserve(static_cast<std::size_t>(std::max(0, max_print_voxels)));

    for (std::size_t k = 0; k < data.local_voxel_centers.size(); ++k) {
      const Eigen::Vector3d world = tf * data.local_voxel_centers[k];
      const Eigen::Vector3i idx = ::common::geometry::VoxelUtils::worldToVoxel(
          world.template cast<float>(), static_cast<float>(voxel_size));
      const long flat = grid.getFlatVoxelId(idx);
      direct_mask.push_back(flat);
      if (static_cast<int>(k) < max_print_voxels) {
        const Eigen::Vector3i roundtrip_idx = grid.getIndexFromFlatId(flat);
        const Eigen::Vector3d roundtrip_world =
            ::common::geometry::VoxelUtils::voxelToWorld(
                roundtrip_idx, static_cast<float>(voxel_size))
                .template cast<double>();
        samples.emplace_back(world, roundtrip_world, idx, flat);
      }
    }

    if (direct_mask.size() > 1) {
      ::common::geometry::VoxelUtils::radixSort(direct_mask);
      direct_mask.erase(std::unique(direct_mask.begin(), direct_mask.end()),
                        direct_mask.end());
    }

    const bool has_checker_mask = i < checker_masks.size();
    const bool match = has_checker_mask && direct_mask == checker_masks[i];
    if (!match) {
      ++mismatch_count;
    }

    if (!show_all && focus_set.count(data.name) == 0) {
      continue;
    }

    report << "[" << (match ? "OK" : "MISMATCH") << "] " << data.name << "\n";
    report << "  local_voxels=" << data.local_voxel_centers.size()
           << " direct_unique=" << direct_mask.size()
           << " checker_unique="
           << (has_checker_mask ? checker_masks[i].size() : 0)
           << " tf=" << (has_tf ? "yes" : "missing") << "\n";
    report << "  local_min=" << vec3ToString(data.local_min)
           << " local_max=" << vec3ToString(data.local_max) << "\n";

    if (has_tf) {
      const auto &t = tf.translation();
      report << "  tf_translation=" << vec3ToString(t) << "\n";
    } else {
      report << "  tf_translation=(missing)\n";
    }

    report << "  samples=" << samples.size() << "\n";
    for (std::size_t s = 0; s < samples.size(); ++s) {
      const auto &sample = samples[s];
      const auto &world = std::get<0>(sample);
      const auto &roundtrip_world = std::get<1>(sample);
      const auto &idx = std::get<2>(sample);
      const auto flat = std::get<3>(sample);
      report << "    [" << s << "] world=" << vec3ToString(world)
             << " idx=" << vec3iToString(idx) << " flat=" << flat
             << " roundtrip_world=" << vec3ToString(roundtrip_world)
             << " delta=" << vec3ToString(world - roundtrip_world) << "\n";
    }

    if (direct_mask.size() <= 24) {
      report << "  ids=";
      for (std::size_t j = 0; j < direct_mask.size(); ++j) {
        report << (j == 0 ? " " : ", ") << direct_mask[j];
      }
      report << "\n";
    } else {
      report << "  ids(first8)=";
      for (std::size_t j = 0; j < std::min<std::size_t>(8, direct_mask.size()); ++j) {
        report << (j == 0 ? " " : ", ") << direct_mask[j];
      }
      report << " ... total=" << direct_mask.size() << "\n";
    }
    report << "\n";
  }

  report << "Summary:\n";
  report << "  mismatched_links=" << mismatch_count << "\n";

  RCLCPP_INFO(logger, "%s", report.str().c_str());
  if (mismatch_count > 0) {
    RCLCPP_WARN(logger, "Voxel validation found %zu mismatched link(s).",
                mismatch_count);
  } else {
    RCLCPP_INFO(logger, "Voxel validation passed.");
  }

  if (!dump_path.empty()) {
    std::ofstream ofs(dump_path);
    if (!ofs.is_open()) {
      RCLCPP_WARN(logger, "Failed to open validation dump path: %s",
                  dump_path.c_str());
      return;
    }
    ofs << report.str();
    RCLCPP_INFO(logger, "Wrote voxel validation report to: %s",
                dump_path.c_str());
  }
}

#ifdef USE_FCL
// --- Fast Triangle-Box Overlap Test (Akenine-Möller) ---
static bool planeBoxOverlap(const fcl::Vector3d &normal,
                            const fcl::Vector3d &vert,
                            const fcl::Vector3d &maxbox) {
  fcl::Vector3d vmin, vmax;
  for (int q = 0; q < 3; q++) {
    double v = vert[q];
    if (normal[q] > 0.0) {
      vmin[q] = -maxbox[q] - v;
      vmax[q] = maxbox[q] - v;
    } else {
      vmin[q] = maxbox[q] - v;
      vmax[q] = -maxbox[q] - v;
    }
  }
  if (normal.dot(vmin) > 0.0)
    return false;
  if (normal.dot(vmax) >= 0.0)
    return true;
  return false;
}

#define AXISTEST_X01(a, b, fa, fb)                                             \
  p0 = a * v0.y() - b * v0.z();                                                \
  p2 = a * v2.y() - b * v2.z();                                                \
  if (p0 < p2) {                                                               \
    min = p0;                                                                  \
    max = p2;                                                                  \
  } else {                                                                     \
    min = p2;                                                                  \
    max = p0;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

#define AXISTEST_X2(a, b, fa, fb)                                              \
  p0 = a * v0.y() - b * v0.z();                                                \
  p1 = a * v1.y() - b * v1.z();                                                \
  if (p0 < p1) {                                                               \
    min = p0;                                                                  \
    max = p1;                                                                  \
  } else {                                                                     \
    min = p1;                                                                  \
    max = p0;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.y() + fb * boxhalfsize.z();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

#define AXISTEST_Y02(a, b, fa, fb)                                             \
  p0 = -a * v0.x() + b * v0.z();                                               \
  p2 = -a * v2.x() + b * v2.z();                                               \
  if (p0 < p2) {                                                               \
    min = p0;                                                                  \
    max = p2;                                                                  \
  } else {                                                                     \
    min = p2;                                                                  \
    max = p0;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

#define AXISTEST_Y1(a, b, fa, fb)                                              \
  p0 = -a * v0.x() + b * v0.z();                                               \
  p1 = -a * v1.x() + b * v1.z();                                               \
  if (p0 < p1) {                                                               \
    min = p0;                                                                  \
    max = p1;                                                                  \
  } else {                                                                     \
    min = p1;                                                                  \
    max = p0;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.x() + fb * boxhalfsize.z();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

#define AXISTEST_Z12(a, b, fa, fb)                                             \
  p1 = a * v1.x() - b * v1.y();                                                \
  p2 = a * v2.x() - b * v2.y();                                                \
  if (p1 < p2) {                                                               \
    min = p1;                                                                  \
    max = p2;                                                                  \
  } else {                                                                     \
    min = p2;                                                                  \
    max = p1;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

#define AXISTEST_Z0(a, b, fa, fb)                                              \
  p0 = a * v0.x() - b * v0.y();                                                \
  p1 = a * v1.x() - b * v1.y();                                                \
  if (p0 < p1) {                                                               \
    min = p0;                                                                  \
    max = p1;                                                                  \
  } else {                                                                     \
    min = p1;                                                                  \
    max = p0;                                                                  \
  }                                                                            \
  rad = fa * boxhalfsize.x() + fb * boxhalfsize.y();                           \
  if (min > rad || max < -rad)                                                 \
    return false;

static bool triBoxOverlap(const fcl::Vector3d &boxcenter,
                          const fcl::Vector3d &boxhalfsize,
                          const fcl::Vector3d triverts[3]) {
  fcl::Vector3d v0 = triverts[0] - boxcenter, v1 = triverts[1] - boxcenter,
                v2 = triverts[2] - boxcenter;
  fcl::Vector3d e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
  double min, max, p0, p1, p2, rad, fex, fey, fez;
  fex = std::abs(e0.x());
  fey = std::abs(e0.y());
  fez = std::abs(e0.z());
  AXISTEST_X01(e0.z(), e0.y(), fez, fey);
  AXISTEST_Y02(e0.z(), e0.x(), fez, fex);
  AXISTEST_Z12(e0.y(), e0.x(), fey, fex);
  fex = std::abs(e1.x());
  fey = std::abs(e1.y());
  fez = std::abs(e1.z());
  AXISTEST_X01(e1.z(), e1.y(), fez, fey);
  AXISTEST_Y02(e1.z(), e1.x(), fez, fex);
  AXISTEST_Z0(e1.y(), e1.x(), fey, fex);
  fex = std::abs(e2.x());
  fey = std::abs(e2.y());
  fez = std::abs(e2.z());
  AXISTEST_X2(e2.z(), e2.y(), fez, fey);
  AXISTEST_Y1(e2.z(), e2.x(), fez, fex);
  AXISTEST_Z12(e2.y(), e2.x(), fey, fex);
  fcl::Vector3d bmin = -boxhalfsize, bmax = boxhalfsize;
  if (std::max({v0.x(), v1.x(), v2.x()}) < bmin.x() ||
      std::min({v0.x(), v1.x(), v2.x()}) > bmax.x())
    return false;
  if (std::max({v0.y(), v1.y(), v2.y()}) < bmin.y() ||
      std::min({v0.y(), v1.y(), v2.y()}) > bmax.y())
    return false;
  if (std::max({v0.z(), v1.z(), v2.z()}) < bmin.z() ||
      std::min({v0.z(), v1.z(), v2.z()}) > bmax.z())
    return false;
  fcl::Vector3d normal = e0.cross(e1);
  return planeBoxOverlap(normal, v0, bmax);
}
#endif

class OfflineUrdfTrainerNode : public rclcpp::Node {
public:
  OfflineUrdfTrainerNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("offline_urdf_trainer", options) {
    RCLCPP_INFO(this->get_logger(),
                "--- Standalone URDF-based Unified GNG/VLUT Pipeline ---");

    // 0. Declare and Get Parameters
    experiment_id_ = this->declare_parameter<std::string>("experiment_id",
                                                          "standalone_train");
    data_directory_ = robot_sim::common::resolveDataPath(
        this->declare_parameter<std::string>("data_directory", "gng_results"));
    robot_urdf_path_ = this->declare_parameter<std::string>("robot_urdf_path",
                                                            "package://topoarm_description/urdf/topo_dual_arm.urdf.xacro");
    gng_model_filename_ = this->declare_parameter<std::string>(
        "gng_model_filename", "gng.bin");
    vlut_filename_ = this->declare_parameter<std::string>(
        "vlut_filename", "vlut.bin");
    ground_z_threshold_ =
        this->declare_parameter<double>("ground_z_threshold", 0.0);
    enable_ground_collision_ = this->declare_parameter<bool>(
        "collision.enable_ground_collision", true);
    enable_self_collision_ = this->declare_parameter<bool>(
        "collision.enable_self_collision", true);
    apply_environment_ignore_links_ = this->declare_parameter<bool>(
        "collision.apply_environment_ignore_links", true);
    apply_self_collision_exclusion_pairs_ = this->declare_parameter<bool>(
        "collision.apply_self_collision_exclusion_pairs", true);
    eef_link_names_ = this->declare_parameter<std::string>(
        "eef_link_names", "");
    arm_leaf_link_names_ = this->declare_parameter<std::string>(
        "arm_leaf_link_names", "");
    gng_exclude_joint_names_ = this->declare_parameter<std::vector<std::string>>(
        "gng_exclude_joint_names", std::vector<std::string>{});
    leaf_link_name_ = this->declare_parameter<std::string>("leaf_link_name",
                                                           "end_effector_link");
    paired_leaf_link_name_ = this->declare_parameter<std::string>(
        "paired_leaf_link_name", "");
    gng_dimension_ = this->declare_parameter<int>("gng_dimension", 12);
    spatial_map_resolution_ =
        this->declare_parameter<double>("spatial_map.resolution", 0.02);
    sensing_resolution_ =
        this->declare_parameter<double>("sensing_resolution", 0.02);
    arm_cache_resolution_ =
        this->declare_parameter<double>("arm_cache_resolution", 0.008);
    danger_threshold_ =
        this->declare_parameter<double>("danger_threshold", 0.025);
    vlut_resolution_ =
        this->declare_parameter<double>("vlut.resolution", 0.02);
    vlut_only_ = this->declare_parameter<bool>("vlut_only", false);
    use_voxel_collision_ =
        this->declare_parameter<bool>("use_voxel_collision", false);
    spatial_map_inflation_ =
        this->declare_parameter<double>("spatial_map.inflation", 0.0);
    self_recognition_inflation_ =
        this->declare_parameter<double>("self_recognition.inflation", 0.02);
    environment_ignore_links_ = this->declare_parameter<std::vector<std::string>>(
        "collision.environment_ignore_links", std::vector<std::string>{});
    self_collision_exclusion_pairs_ =
        this->declare_parameter<std::vector<std::string>>(
            "collision.self_collision_exclusion_pairs",
            std::vector<std::string>{});
    require_initial_collision_approval_ =
        this->declare_parameter<bool>(
            "collision.require_initial_collision_approval", false);
    approved_initial_collision_pairs_ =
        this->declare_parameter<std::vector<std::string>>(
            "collision.approved_initial_collision_pairs",
            std::vector<std::string>{});
    validate_voxel_link_masks_ =
        this->declare_parameter<bool>("collision.validate_voxel_link_masks", false);
    validation_focus_links_ = this->declare_parameter<std::string>(
        "collision.validation_focus_links", "");
    validation_max_print_voxels_ = this->declare_parameter<int>(
        "collision.validation_max_print_voxels", 8);
    validation_dump_path_ = this->declare_parameter<std::string>(
        "collision.validation_dump_path", "");
    initial_collision_only_ =
        this->declare_parameter<bool>("initial_collision_only", false);
    initial_collision_only_ = this->declare_parameter<bool>(
        "collision.initial_collision_only", false);
    initial_collision_only_ = initial_collision_only_ ||
                              this->get_parameter("initial_collision_only").as_bool();
    generate_initial_collision_approval_only_ =
        this->declare_parameter<bool>("generate_initial_collision_approval_only", false);
    generate_initial_collision_approval_only_ =
        this->declare_parameter<bool>(
            "collision.generate_initial_collision_approval_only", false);
    generate_initial_collision_approval_only_ =
        generate_initial_collision_approval_only_ || initial_collision_only_ ||
        this->get_parameter("generate_initial_collision_approval_only").as_bool();

    // GNG Parameters (nested under gng_params)
    gng_params_.lambda = this->declare_parameter<int>("gng_params.lambda", 50);
    gng_params_.max_node_num =
        this->declare_parameter<int>("gng_params.max_node_num", 10000);
    gng_params_.num_samples =
        this->declare_parameter<int>("gng_params.num_samples", 1000000);
    gng_params_.max_iterations =
        this->declare_parameter<int>("gng_params.max_iterations", 1000000);
    gng_params_.refine_iterations =
        this->declare_parameter<int>("gng_params.refine_iterations", 100000);
    gng_params_.coord_edge_iterations = this->declare_parameter<int>(
        "gng_params.coord_edge_iterations", 200000);
    gng_params_.learn_rate_s1 =
        this->declare_parameter<double>("gng_params.learn_rate_s1", 0.08);
    gng_params_.learn_rate_s2 =
        this->declare_parameter<double>("gng_params.learn_rate_s2", 0.008);
    gng_params_.max_edge_age =
        this->declare_parameter<int>("gng_params.max_edge_age", 500);
    gng_params_.alpha =
        this->declare_parameter<double>("gng_params.alpha", 0.5);
    gng_params_.beta =
        this->declare_parameter<double>("gng_params.beta", 0.0005);
    gng_params_.n_best_candidates =
        this->declare_parameter<int>("gng_params.n_best_candidates", 4);
    gng_params_.ais_threshold =
        this->declare_parameter<double>("gng_params.ais_threshold", 1.0);
    gng_params_.start_node_num =
        this->declare_parameter<int>("gng_params.start_node_num", 2);

    // Log parameters for verification
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  experiment_id: %s",
                experiment_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  robot_urdf_path: %s",
                robot_urdf_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  spatial_map_resolution: %f",
                spatial_map_resolution_);
    RCLCPP_INFO(this->get_logger(), "  vlut_only: %s",
                vlut_only_ ? "true" : "false");
  } // Constructor

  void run_training_pipeline() {
    // 1. Robot Setup
    std::unique_ptr<kinematics::KinematicChain> arm;
    simulation::RobotModel *model = nullptr;
    try {
      std::string resolved_path =
          robot_sim::common::resolvePath(robot_urdf_path_);

      if (resolved_path.empty() || !std::filesystem::exists(resolved_path) ||
          std::filesystem::is_directory(resolved_path)) {
        // Try common extensions as a fallback
        std::string with_xacro =
            robot_sim::common::resolvePath(robot_urdf_path_ + ".xacro");
        if (!with_xacro.empty() && std::filesystem::exists(with_xacro)) {
          resolved_path = with_xacro;
        } else {
          std::string with_urdf =
              robot_sim::common::resolvePath(robot_urdf_path_ + ".urdf");
          if (!with_urdf.empty() && std::filesystem::exists(with_urdf)) {
            resolved_path = with_urdf;
          }
        }
      }

      if (resolved_path.empty() || !std::filesystem::exists(resolved_path)) {
        throw std::runtime_error("Could not find robot model for: " +
                                 robot_urdf_path_);
      }

      RCLCPP_INFO(this->get_logger(), "[Robot] Loading from resolved path: %s",
                  resolved_path.c_str());
      auto model_obj = simulation::loadRobotFromUrdf(resolved_path);
      model = new simulation::RobotModel(model_obj);
      std::vector<std::string> eef_names =
          splitCommaSeparated(eef_link_names_);
      std::vector<std::string> arm_leaf_names =
          splitCommaSeparated(arm_leaf_link_names_);
      if (!eef_names.empty()) {
        if (eef_names.size() == 1) {
          arm = std::make_unique<kinematics::KinematicChain>(
              simulation::createKinematicChainFromModel(
                  *model, eef_names.front()));
        } else {
          arm = simulation::createMultiArmKinematicChainFromModels(
              *model, eef_names);
        }
      } else if (!arm_leaf_names.empty()) {
        if (arm_leaf_names.size() == 1) {
          arm = std::make_unique<kinematics::KinematicChain>(
              simulation::createKinematicChainFromModel(
                  *model, arm_leaf_names.front()));
        } else {
          arm = simulation::createMultiArmKinematicChainFromModels(
              *model, arm_leaf_names);
        }
      } else if (!paired_leaf_link_name_.empty()) {
        arm = simulation::createMultiArmKinematicChainFromModels(
            *model, {leaf_link_name_, paired_leaf_link_name_});
      } else if (leaf_link_name_.rfind("left_", 0) == 0) {
        std::string inferred_right = "right_" + leaf_link_name_.substr(5);
        arm = simulation::createMultiArmKinematicChainFromModels(
            *model, {leaf_link_name_, inferred_right});
      } else {
        arm = std::make_unique<kinematics::KinematicChain>(
            simulation::createKinematicChainFromModel(*model, leaf_link_name_));
      }
      arm->setBase(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
      RCLCPP_INFO(this->get_logger(), "[Robot] Loaded: %s, DOF: %d",
                  resolved_path.c_str(), arm->getTotalDOF());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[Error] Robot setup failed: %s",
                   e.what());
      if (model)
        delete model;
      throw; // Re-throw to indicate failure
    }

    std::shared_ptr<SelectedJointKinematicChain> gng_chain;
    if (!gng_exclude_joint_names_.empty()) {
      auto selection = buildJointSelectionSpec(*arm, gng_exclude_joint_names_);
      if (selection.selected_dof <= 0) {
        throw std::runtime_error(
            "gng_exclude_joint_names removed all movable joints.");
      }
      if (gng_dimension_ != selection.selected_dof) {
        RCLCPP_WARN(this->get_logger(),
                    "[GNG] gng_dimension (%d) differs from selected DOF (%d). "
                    "Using the selected DOF derived from gng_exclude_joint_names.",
                    gng_dimension_, selection.selected_dof);
      }
      gng_dimension_ = selection.selected_dof;
      gng_chain = std::make_shared<SelectedJointKinematicChain>(arm.get(),
                                                                std::move(selection));
      std::ostringstream excluded_oss;
      for (std::size_t i = 0; i < gng_exclude_joint_names_.size(); ++i) {
        if (i > 0) {
          excluded_oss << ", ";
        }
        excluded_oss << gng_exclude_joint_names_[i];
      }
      const std::string excluded_joint_names = excluded_oss.str();
      RCLCPP_INFO(this->get_logger(),
                  "[GNG] Excluding joint names from learning: %s",
                  excluded_joint_names.c_str());
      RCLCPP_INFO(this->get_logger(),
                  "[GNG] Selected learning DOF: %d", gng_dimension_);
    }

    // 2. Setup Checkers
    auto self_checker =
        std::make_shared<simulation::GeometricSelfCollisionChecker>(*model,
                                                                    *arm);
    auto env_checker =
        std::make_shared<simulation::EnvironmentCollisionChecker>();
    if (enable_ground_collision_) {
      env_checker->addBoxObstacle(
          "ground", Eigen::Vector3d(0, 0, ground_z_threshold_ - 0.05),
          Eigen::Matrix3d::Identity(), Eigen::Vector3d(10.0, 10.0, 0.05));
    }

#ifdef USE_FCL
    self_checker->setStrictMode(true);
#endif

    auto composite_checker =
        std::make_shared<simulation::CompositeCollisionChecker>();
    composite_checker->setSelfCollisionChecker(self_checker);
    composite_checker->setEnvironmentCollisionChecker(env_checker);

    // 3. GNG Setup
    std::shared_ptr<simulation::ISelfCollisionChecker> final_checker;
    std::shared_ptr<simulation::VoxelCollisionChecker> voxel_checker;
    if (use_voxel_collision_) {
      RCLCPP_INFO(this->get_logger(),
                  "[Collision] Using VoxelCollisionChecker (Res: %f)",
                  spatial_map_resolution_);
      voxel_checker = std::make_shared<simulation::VoxelCollisionChecker>(
          *model, *arm, spatial_map_resolution_, spatial_map_inflation_);
      voxel_checker->setGroundZThreshold(enable_ground_collision_
                                             ? ground_z_threshold_
                                             : -std::numeric_limits<double>::infinity());
      voxel_checker->setEnableSelfCollision(enable_self_collision_);
      voxel_checker->setEnvironmentCollisionChecker(env_checker);
      if (apply_environment_ignore_links_) {
        for (const auto &link_name : environment_ignore_links_) {
          voxel_checker->addEnvironmentIgnoreLink(link_name);
        }
      }
      final_checker = voxel_checker;
    } else {
      composite_checker->setEnableSelfCollision(enable_self_collision_);
      if (apply_environment_ignore_links_) {
        for (const auto &link_name : environment_ignore_links_) {
          composite_checker->addEnvironmentIgnoreLink(link_name);
        }
      }
      final_checker = composite_checker;
    }

    // 初期姿勢での自己衝突候補を先に評価する。
    // ここでは手動の除外ペアをまだ checker に入れず、生の候補を取得する。
    Eigen::VectorXd zero_q(arm->getTotalDOF());
    zero_q.setZero();
    arm->updateKinematics(zero_q);
    std::map<std::string, double> joint_hints;
    const auto joint_values = arm->getJointValues();
    std::size_t joint_cursor = 0;
    for (int i = 0; i < arm->getNumJoints(); ++i) {
      const int dof = arm->getJointDOF(i);
      if (dof == 1 && joint_cursor < joint_values.size()) {
        joint_hints[arm->getJointName(i)] = joint_values[joint_cursor];
      }
      joint_cursor += static_cast<std::size_t>(std::max(0, dof));
    }
    if (voxel_checker) {
      voxel_checker->setJointValueHints(joint_hints);
    }
    self_checker->updateBodyPoses(arm->getLinkPositions(),
                                  arm->getLinkOrientations());
    if (voxel_checker) {
      voxel_checker->updateBodyPoses(arm->getLinkPositions(),
                                     arm->getLinkOrientations());
    }

    if (validate_voxel_link_masks_ && voxel_checker) {
      std::vector<std::string> focus_links = splitCommaSeparated(validation_focus_links_);
      writeVoxelValidationReport(
          this->get_logger(), *voxel_checker, spatial_map_resolution_,
          focus_links, validation_max_print_voxels_, validation_dump_path_);
    } else if (validate_voxel_link_masks_ && !voxel_checker) {
      RCLCPP_WARN(this->get_logger(),
                  "[Collision][Validation] Requested voxel validation, but voxel collision is disabled.");
    }

    std::vector<std::pair<std::string, std::string>> initial_collisions;
    if (enable_self_collision_) {
      if (voxel_checker) {
        initial_collisions = voxel_checker->collectSelfCollisionPairs();
      } else {
        initial_collisions = self_checker->collectSelfCollisionPairs();
      }
    }

    std::set<std::string> manual_exclusion_pairs;
    if (apply_self_collision_exclusion_pairs_) {
      for (const auto &pair : parseLinkPairs(self_collision_exclusion_pairs_)) {
        manual_exclusion_pairs.insert(makePairKey(pair.first, pair.second));
      }
    }

    std::vector<std::pair<std::string, std::string>> filtered_initial_collisions;
    filtered_initial_collisions.reserve(initial_collisions.size());
    for (const auto &pair : initial_collisions) {
      if (manual_exclusion_pairs.count(makePairKey(pair.first, pair.second)) == 0) {
        filtered_initial_collisions.push_back(pair);
      }
    }

    std::set<std::string> approved_initial_pairs;
    if (apply_self_collision_exclusion_pairs_) {
      for (const auto &pair : parseLinkPairs(approved_initial_collision_pairs_)) {
        approved_initial_pairs.insert(makePairKey(pair.first, pair.second));
      }
    }

    if (!filtered_initial_collisions.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[Collision][Initial] Found %zu self-collision candidate pair(s) in the default pose.",
                  filtered_initial_collisions.size());
      for (const auto &pair : filtered_initial_collisions) {
        RCLCPP_WARN(this->get_logger(), "  candidate: %s <-> %s",
                    pair.first.c_str(), pair.second.c_str());
      }
    }

    const std::filesystem::path approval_output_dir =
        std::filesystem::path(data_directory_) / experiment_id_;
    writeInitialCollisionApprovalYaml(approval_output_dir,
                                      filtered_initial_collisions);
    const std::string approval_yaml_path =
        (approval_output_dir / "initial_collision_approval.yaml").string();
    RCLCPP_INFO(this->get_logger(),
                "[Collision][Initial] Wrote approval YAML to: %s",
                approval_yaml_path.c_str());

    if (generate_initial_collision_approval_only_) {
      RCLCPP_INFO(this->get_logger(),
                  "[Collision][Initial] Generation-only mode is enabled. "
                  "Stopping before GNG training.");
      if (model)
        delete model;
      return;
    }

    if (require_initial_collision_approval_ && apply_self_collision_exclusion_pairs_ &&
        enable_self_collision_) {
      std::vector<std::pair<std::string, std::string>> unapproved_pairs;
      for (const auto &pair : filtered_initial_collisions) {
        if (approved_initial_pairs.count(makePairKey(pair.first, pair.second)) ==
            0) {
          unapproved_pairs.push_back(pair);
        }
      }

      if (!unapproved_pairs.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "[Collision][Initial] Approval required, but %zu pair(s) are not approved.",
                     unapproved_pairs.size());
        for (const auto &pair : unapproved_pairs) {
          RCLCPP_ERROR(this->get_logger(), "  unapproved: %s <-> %s",
                       pair.first.c_str(), pair.second.c_str());
        }
        throw std::runtime_error(
            "Initial self-collision approval is required before training.");
      }
    }

    // 承認済みの初期衝突候補は、そのまま除外ペアとして採用。承認制OFFの場合は、初期候補を自動採用。
    const bool auto_accept_initial_collisions =
        !require_initial_collision_approval_ ||
        !apply_self_collision_exclusion_pairs_ || !enable_self_collision_;
    if (apply_self_collision_exclusion_pairs_ && enable_self_collision_) {
      for (const auto &pair : filtered_initial_collisions) {
        const std::string key = makePairKey(pair.first, pair.second);
        if (auto_accept_initial_collisions ||
            approved_initial_pairs.count(key) > 0) {
          manual_exclusion_pairs.insert(key);
        }
      }
    }

    for (const auto &key : manual_exclusion_pairs) {
      const auto sep = key.find('|');
      if (sep == std::string::npos) {
        continue;
      }
      const std::string lhs = key.substr(0, sep);
      const std::string rhs = key.substr(sep + 1);
      if (apply_self_collision_exclusion_pairs_) {
        self_checker->addCollisionExclusion(lhs, rhs);
        if (voxel_checker) {
          voxel_checker->addCollisionExclusion(lhs, rhs);
        }
      }
    }

    kinematics::KinematicChain *gng_chain_ptr =
        gng_chain ? static_cast<kinematics::KinematicChain *>(gng_chain.get())
                  : arm.get();

    GNG2 gng(gng_dimension_, 3, gng_chain_ptr);
    gng.setCoordLayerCount(static_cast<int>(arm->getArmCount()));

    // Initialize GNG parameters from ROS 2 parameters
    gng.setParams(gng_params_);

    gng.setSelfCollisionChecker(final_checker.get());
    // Initialize Status Providers
    gng.registerStatusProvider(
        std::make_shared<GNG::GeometricSelfCollisionProvider<Eigen::VectorXf,
                                                             Eigen::Vector3f>>(
            final_checker.get(), gng_chain_ptr));
    gng.registerStatusProvider(
        std::make_shared<
            GNG::ManipulabilityProvider<Eigen::VectorXf, Eigen::Vector3f>>(
            gng_chain_ptr));
    gng.registerStatusProvider(
        std::make_shared<
            GNG::EEDirectionProvider<Eigen::VectorXf, Eigen::Vector3f>>(
            gng_chain_ptr));

    std::filesystem::path output_dir =
        std::filesystem::path(data_directory_) / experiment_id_;
    gng.setStatsLogPath(
        (output_dir / (experiment_id_ + "_distance_stats.dat")).string());

    // Define standard file paths
    std::string gng_file_path = (output_dir / gng_model_filename_).string();

    // 4. Training Steps
    if (vlut_only_) {
      RCLCPP_INFO(this->get_logger(),
                  "[Step 0] Skipping GNG training. Loading existing map: %s",
                  gng_file_path.c_str());
      if (!gng.load(gng_file_path)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[Error] Failed to load GNG map for VLUT reconstruction from %s.",
            gng_file_path.c_str());
        if (model)
          delete model;
        throw std::runtime_error("Failed to load GNG map.");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "[Step 1] Initial Exploration...");
      gng.setCollisionAware(false);
      gng.gngTrainOnTheFly(gng_params_.max_iterations);

      RCLCPP_INFO(this->get_logger(), "[Step 2] Intermediate Filter...");
      gng.strictFilter();

      RCLCPP_INFO(this->get_logger(),
                  "[Step 3] Refinement (Self-Collision Aware)...");
      gng.setCollisionAware(true);
      gng.gngTrainOnTheFly(gng_params_.refine_iterations);

      RCLCPP_INFO(this->get_logger(), "[Step 4] Final Verification...");
      gng.strictFilter();
      gng.refresh_coord_weights();

      RCLCPP_INFO(this->get_logger(),
                  "[Step 5] Coordinate Space Edge construction...");
      for (int layer = 0; layer < gng.getCoordLayerCount(); ++layer) {
        gng.trainCoordEdgesOnTheFly(gng_params_.coord_edge_iterations, layer);
      }

      // Metadata update for finale
      gng.triggerBatchUpdates();
    }

    // 5. High-Fidelity Voxelization (VLUT Generation)
#ifdef USE_FCL
    struct VRel {
      long vid;
      int nid;
      int lid;
    };
    std::vector<VRel> v_rels;
    double res = vlut_resolution_;
    fcl::Vector3d box_half_size(res * 0.5, res * 0.5, res * 0.5);
    GNG::Analysis::IndexVoxelGrid voxel_grid(res);
    auto active_ids = gng.getActiveIndices();
    int processed = 0;

    RCLCPP_INFO(this->get_logger(), "[Step 6] Pre-voxelizing Robot Links...");

    // Cache for local voxel centers of each link
    struct LocalVoxelCloud {
      std::vector<fcl::Vector3d> centers;
    };
    std::map<int, LocalVoxelCloud> link_voxel_clouds;

    const auto &objects = self_checker->getCollisionObjects();
    for (size_t i = 0; i < objects.size(); ++i) {
      if (objects[i].type != collision::SelfCollisionChecker::ShapeType::MESH) {
        continue;
      }
      auto fcl_obj = self_checker->getFCLObject(i);
      if (!fcl_obj || !fcl_obj->collisionGeometry())
        continue;
      auto mesh = dynamic_cast<const fcl::BVHModel<fcl::OBBRSS<double>> *>(
          fcl_obj->collisionGeometry().get());
      if (!mesh || !mesh->vertices || !mesh->tri_indices)
        continue;

      LocalVoxelCloud &cloud = link_voxel_clouds[i];
      fcl::Vector3d mesh_min, mesh_max;
      mesh_min.setConstant(std::numeric_limits<double>::infinity());
      mesh_max.setConstant(-std::numeric_limits<double>::infinity());
      for (int vidx = 0; vidx < mesh->num_vertices; ++vidx) {
        mesh_min = mesh_min.cwiseMin(mesh->vertices[vidx]);
        mesh_max = mesh_max.cwiseMax(mesh->vertices[vidx]);
      }

      Eigen::Vector3i b_min =
          (mesh_min / res).array().floor().cast<int>().matrix() -
          Eigen::Vector3i::Ones();
      Eigen::Vector3i b_max =
          (mesh_max / res).array().ceil().cast<int>().matrix() +
          Eigen::Vector3i::Ones();

      for (int vx = b_min.x(); vx <= b_max.x(); ++vx) {
        for (int vy = b_min.y(); vy <= b_max.y(); ++vy) {
          for (int vz = b_min.z(); vz <= b_max.z(); ++vz) {
            fcl::Vector3d box_center =
                (Eigen::Vector3i(vx, vy, vz).cast<double>() +
                 Eigen::Vector3d::Constant(0.5)) *
                res;
            bool occupied = false;
            for (int t = 0; t < mesh->num_tris; ++t) {
              const fcl::Triangle &tri = mesh->tri_indices[t];
              fcl::Vector3d v[3] = {mesh->vertices[tri[0]],
                                    mesh->vertices[tri[1]],
                                    mesh->vertices[tri[2]]};
              if (triBoxOverlap(box_center, box_half_size, v)) {
                occupied = true;
                break;
              }
            }
            if (occupied)
              cloud.centers.push_back(box_center);
          }
        }
      }
      RCLCPP_INFO(
          this->get_logger(),
          "  Link [%zu] (%s) voxelized: %zu voxels. Local Z-range: [%f, %f]", i,
          objects[i].name.c_str(), cloud.centers.size(), mesh_min.z(),
          mesh_max.z());
    }

    RCLCPP_INFO(
        this->get_logger(),
        "[Step 6] Building Spatial Index (Voxel Cloud Transformation)...");
    v_rels.reserve(active_ids.size() *
                   500); // Pre-reserve to avoid reallocations
    std::unordered_set<long> seen;

    // --- Track Actual Reachable Bounds ---
    fcl::Vector3d actual_min(std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity());
    fcl::Vector3d actual_max(-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity());

    for (int nid : active_ids) {
      auto &node = gng.nodeAt(nid);
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
          positions;
      std::vector<Eigen::Quaterniond,
                  Eigen::aligned_allocator<Eigen::Quaterniond>>
          orientations;
      gng_chain_ptr->forwardKinematicsAt(node.weight_angle.template cast<double>(),
                                         positions, orientations);
      self_checker->updateBodyPoses(positions, orientations);

      for (auto const &[link_idx, cloud] : link_voxel_clouds) {
        fcl::Transform3d tf =
            self_checker->getFCLObject(link_idx)->getTransform();
        seen.clear(); // Re-use memory

        for (const auto &local_p : cloud.centers) {
          fcl::Vector3d world_p = tf * local_p;

          // Update actual bounds
          actual_min = actual_min.cwiseMin(world_p);
          actual_max = actual_max.cwiseMax(world_p);

          Eigen::Vector3i idx = (world_p / res).array().floor().cast<int>();
          long vid = voxel_grid.getFlatVoxelId(idx);
          if (seen.insert(vid).second) {
            v_rels.push_back({vid, nid, link_idx});
          }
        }
      }
      processed++;
      if (processed % 100 == 0 || processed == (int)active_ids.size()) {
        RCLCPP_INFO(this->get_logger(), "  Analyzed Node %d/%zu (%zu rels)",
                    processed, active_ids.size(), v_rels.size());
      }
    }
    RCLCPP_INFO(this->get_logger(), "[Step 6] Analysis completed.");

    // Calculate Relative Margin (10% of the detected range, min 2cm)
    fcl::Vector3d range = actual_max - actual_min;
    fcl::Vector3d margin = range * 0.1;
    for (int i = 0; i < 3; ++i)
      if (margin[i] < 0.02)
        margin[i] = 0.02;

    actual_min -= margin;
    actual_max += margin;

    Eigen::Vector3i final_dims = ((actual_max - actual_min) / res)
                                     .array()
                                     .ceil()
                                     .cast<int>()
                                     .cwiseMax(1);
    RCLCPP_INFO(this->get_logger(),
                "[Step 6] Final Voxel Grid: [%ld, %ld, %ld] (%ld total voxels)",
                (long)final_dims.x(), (long)final_dims.y(),
                (long)final_dims.z(),
                (long)final_dims.x() * final_dims.y() * final_dims.z());
    RCLCPP_INFO(this->get_logger(),
                "[Step 6] Workspace Bounds: Min(%f, %f, %f), Max(%f, %f, %f)",
                actual_min.x(), actual_min.y(), actual_min.z(), actual_max.x(),
                actual_max.y(), actual_max.z());
#endif

    // 6. Save Everything
    std::filesystem::path output_dir_path =
        std::filesystem::path(data_directory_) / experiment_id_;
    std::filesystem::create_directories(output_dir_path);

    // Save GNG Map
    if (gng.save(gng_file_path)) {
      RCLCPP_INFO(this->get_logger(), "[Success] GNG saved to: %s",
                  gng_file_path.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[Error] Failed to save GNG to: %s",
                   gng_file_path.c_str());
    }

    // Save VLUT
#ifdef USE_FCL
    std::string vlut_file_path = (output_dir_path / vlut_filename_).string();
    std::ofstream ofs(vlut_file_path, std::ios::binary);
    if (ofs) {
      // --- Add Self-Describing Header ---
      auto pack4CharsToUint32 = [](const char *s) {
        return (uint32_t)(s[0] << 24 | s[1] << 16 | s[2] << 8 | s[3]);
      };
      uint32_t file_id = pack4CharsToUint32("VLUT");
      uint32_t version = 2; // Updated version to include bounds
      float save_res = (float)res;
      float min_b[3] = {(float)actual_min.x(), (float)actual_min.y(),
                        (float)actual_min.z()};
      float max_b[3] = {(float)actual_max.x(), (float)actual_max.y(),
                        (float)actual_max.z()};

      ofs.write((char *)&file_id, sizeof(uint32_t));
      ofs.write((char *)&version, sizeof(uint32_t));
      ofs.write((char *)&save_res, sizeof(float));
      ofs.write((char *)min_b, sizeof(float) * 3);
      ofs.write((char *)max_b, sizeof(float) * 3);

      std::sort(v_rels.begin(), v_rels.end(), [](const auto &a, const auto &b) {
        return (a.vid != b.vid) ? a.vid < b.vid : a.nid < b.nid;
      });
      size_t total = v_rels.size();
      ofs.write((char *)&total, sizeof(size_t));
      float d0 = 0.0f;
      for (const auto &rel : v_rels) {
        ofs.write((char *)&rel.vid, sizeof(long));
        ofs.write((char *)&rel.nid, sizeof(int));
        ofs.write((char *)&d0, sizeof(float));
        ofs.write((char *)&rel.lid, sizeof(int));
      }
      RCLCPP_INFO(this->get_logger(), "[Success] VLUT saved to: %s (Res: %f m)",
                  vlut_file_path.c_str(), res);
    } else {
      RCLCPP_ERROR(this->get_logger(), "[Error] Failed to save VLUT to: %s",
                   vlut_file_path.c_str());
    }
#endif

    delete model;
  }

private:
  // Member variables to store parameters
  std::string experiment_id_;
  std::string data_directory_;
  std::string robot_urdf_path_;
  std::string gng_model_filename_;
  std::string vlut_filename_;
  double ground_z_threshold_;
  bool enable_ground_collision_ = true;
  bool enable_self_collision_ = true;
  bool apply_environment_ignore_links_ = true;
  bool apply_self_collision_exclusion_pairs_ = true;
  std::string eef_link_names_;
  std::string arm_leaf_link_names_;
  std::string leaf_link_name_;
  std::string paired_leaf_link_name_;
  int gng_dimension_;
  std::vector<std::string> gng_exclude_joint_names_;
  double spatial_map_resolution_;
  double vlut_resolution_;
  double sensing_resolution_;
  double arm_cache_resolution_;
  double danger_threshold_;
  double spatial_map_inflation_ = 0.0;
  double self_recognition_inflation_ = 0.0;
  std::vector<std::string> environment_ignore_links_;
  std::vector<std::string> self_collision_exclusion_pairs_;
  bool require_initial_collision_approval_ = false;
  std::vector<std::string> approved_initial_collision_pairs_;
  bool validate_voxel_link_masks_ = false;
  std::string validation_focus_links_;
  int validation_max_print_voxels_ = 8;
  std::string validation_dump_path_;
  bool initial_collision_only_ = false;
  bool generate_initial_collision_approval_only_ = false;
  bool vlut_only_ = false;
  bool use_voxel_collision_ = false;
  double voxel_padding_ = 0.0;

  // GNG Parameters struct
  GNG::GngParameters gng_params_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Pass command-line arguments to the node options
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);

  auto node = std::make_shared<OfflineUrdfTrainerNode>(options);

  try {
    node->run_training_pipeline();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Caught exception during training pipeline: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "Training pipeline finished successfully.");
  rclcpp::shutdown();
  return 0;
}
