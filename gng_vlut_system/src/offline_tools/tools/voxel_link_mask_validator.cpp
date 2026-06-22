#include <algorithm>
#include <fstream>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "core/common/constants.hpp"
#include "common/resource_utils.hpp"
#include "common/voxel_utils.hpp"
#include "robot_model/kinematic_adapter.hpp"
#include "robot_model/robot_model.hpp"
#include "robot_model/robot_voxelizer.hpp"
#include "robot_model/urdf_loader.hpp"
#include "safety_engine/indexing/index_voxel_grid.hpp"
#include "safety_engine/recognition/self_recognition_manager.hpp"

namespace {

struct VoxelSample {
  Eigen::Vector3d local;
  Eigen::Vector3d world;
  Eigen::Vector3i idx;
  Eigen::Vector3d roundtrip_world;
  long flat_id = -1;
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
};

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

struct LinkReport {
  std::string name;
  std::size_t local_count = 0;
  std::size_t unique_count = 0;
  bool match_manager = false;
  Eigen::Vector3d local_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_max = Eigen::Vector3d::Zero();
  std::vector<long> direct_mask;
  std::vector<long> manager_mask;
  std::vector<VoxelSample> samples;
};

} // namespace

class VoxelLinkMaskValidator : public rclcpp::Node {
public:
  explicit VoxelLinkMaskValidator(const rclcpp::NodeOptions &options)
      : Node("voxel_link_mask_validator", options) {
    declare_parameter("robot_urdf_path", "");
    declare_parameter<double>("voxel_size",
                              ::robot_sim::common::Constants::DEFAULT_VOXEL_SIZE);
    declare_parameter<double>("self_recognition.inflation", 0.02);
    declare_parameter<std::vector<std::string>>("root_links",
                                                std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("leaf_links",
                                                std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("exclude_links",
                                                std::vector<std::string>{});
    declare_parameter<std::string>("root_link", "");
    declare_parameter<std::vector<double>>("joint_values",
                                           std::vector<double>{});
    declare_parameter<std::vector<std::string>>("focus_links",
                                                std::vector<std::string>{});
    declare_parameter<int>("max_print_voxels_per_link", 8);
    declare_parameter<std::string>("dump_path", "");
    declare_parameter("voxel_idx_shift.x_shift",
                      ::robot_sim::common::Constants::DEFAULT_X_SHIFT);
    declare_parameter("voxel_idx_shift.y_shift",
                      ::robot_sim::common::Constants::DEFAULT_Y_SHIFT);
    declare_parameter("voxel_idx_shift.z_shift",
                      ::robot_sim::common::Constants::DEFAULT_Z_SHIFT);
    declare_parameter("voxel_idx_shift.offset",
                      ::robot_sim::common::Constants::DEFAULT_OFFSET);
  }

  int run() {
    try {
      loadRobot();
      buildKinematics();
      buildVoxelData();
      validate();
      writeReportIfNeeded();
      return mismatched_links_.empty() ? 0 : 2;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Validation failed: %s", e.what());
      return 1;
    }
  }

private:
  void loadRobot() {
    const std::string urdf_rel = get_parameter("robot_urdf_path").as_string();
    const std::string urdf_path = robot_sim::common::resolvePath(urdf_rel);
    if (urdf_path.empty()) {
      throw std::runtime_error("Could not resolve URDF path: " + urdf_rel);
    }
    RCLCPP_INFO(get_logger(), "Loading URDF from: %s", urdf_path.c_str());

    model_ = std::make_shared<simulation::RobotModel>(
        simulation::loadRobotFromUrdf(urdf_path));

    std::string global_root_link = get_parameter("root_link").as_string();
    if (global_root_link.empty()) {
      std::string base_name = model_->getRootLinkName();
      std::string ns = get_namespace();
      if (ns != "/" && !ns.empty()) {
        if (ns[0] == '/') ns = ns.substr(1);
        root_link_ = ns + "/" + base_name;
      } else {
        root_link_ = base_name;
      }
    } else {
      root_link_ = global_root_link;
    }

    auto root_links = get_parameter("root_links").as_string_array();
    auto leaf_links = get_parameter("leaf_links").as_string_array();

    std::vector<std::string> collision_links;
    std::vector<std::string> current_leaf_links = leaf_links;

    for (const auto &[link_name, link_props] : model_->getLinks()) {
      if (!link_props.collisions.empty()) {
        collision_links.push_back(link_name);
      }

      if (leaf_links.empty()) {
        bool has_child = false;
        for (const auto &[joint_name, joint_props] : model_->getJoints()) {
          (void)joint_name;
          if (joint_props.parent_link == link_name) {
            has_child = true;
            break;
          }
        }
        if (!has_child && link_name != model_->getRootLinkName()) {
          current_leaf_links.push_back(link_name);
        }
      }
    }

    std::vector<simulation::ArmConfig> arm_configs;
    arm_configs.reserve(current_leaf_links.size());
    for (std::size_t i = 0; i < current_leaf_links.size(); ++i) {
      simulation::ArmConfig cfg;
      cfg.leaf_link = current_leaf_links[i];
      cfg.prefix = "";
      cfg.root_link = (i < root_links.size() && !root_links[i].empty())
                          ? root_links[i]
                          : root_link_;
      arm_configs.push_back(cfg);
    }

    auto chain_unique = simulation::createMultiArmKinematicChain(
        *model_, arm_configs, Eigen::Vector3d::Zero());
    chain_ = std::shared_ptr<kinematics::KinematicChain>(std::move(chain_unique));

    RCLCPP_INFO(get_logger(), "Root link: %s", root_link_.c_str());
    RCLCPP_INFO(get_logger(), "Collision links: %zu, leaf links: %zu",
                collision_links.size(), current_leaf_links.size());

    auto* grid = manager_.getIndexGrid();
    grid->setVoxelSize(get_parameter("voxel_size").as_double());
    grid->setIndexingParams(
        get_parameter("voxel_idx_shift.x_shift").as_int(),
        get_parameter("voxel_idx_shift.y_shift").as_int(),
        get_parameter("voxel_idx_shift.z_shift").as_int(),
        get_parameter("voxel_idx_shift.offset").as_int());

    const double inflation = get_parameter("self_recognition.inflation").as_double();
    voxel_data_ = simulation::RobotVoxelizer::build(
        *model_, collision_links, *grid, {}, inflation);

    manager_.initialize(chain_, model_, voxel_data_, grid->getVoxelSize());

    if (voxel_data_.empty()) {
      throw std::runtime_error("No voxelized link data was produced.");
    }

    RCLCPP_INFO(get_logger(), "Voxelized links: %zu", voxel_data_.size());
  }

  void buildKinematics() {
    const auto requested_joints = get_parameter("joint_values").as_double_array();
    const std::size_t dof = static_cast<std::size_t>(chain_->getTotalDOF());
    if (!requested_joints.empty() && requested_joints.size() != dof) {
      std::ostringstream oss;
      oss << "joint_values size (" << requested_joints.size()
          << ") does not match DOF (" << dof << "), falling back to zeros.";
      RCLCPP_WARN(get_logger(), "%s", oss.str().c_str());
    }

    if (!requested_joints.empty() && requested_joints.size() == dof) {
      joints_ = requested_joints;
    } else {
      joints_.assign(dof, 0.0);
    }

    manager_.updateRobotState(joints_);
    RCLCPP_INFO(get_logger(), "Using joint vector of size %zu", joints_.size());
  }

  void buildVoxelData() {
    auto fixed_info = model_->getFixedLinkInfo();
    chain_->buildAllLinkTransforms(
        chain_->getLinkPositions(),
        chain_->getLinkOrientations(),
        fixed_info,
        direct_link_tfs_);
    if (direct_link_tfs_.empty()) {
      throw std::runtime_error("Failed to build link transforms.");
    }
  }

  static bool isFocused(const std::unordered_set<std::string> &focus,
                        const std::string &name) {
    return focus.empty() || focus.count(name) > 0;
  }

  void validate() {
    const auto focus_list = get_parameter("focus_links").as_string_array();
    const std::unordered_set<std::string> focus_set(focus_list.begin(), focus_list.end());
    const int max_print =
        std::max(0, static_cast<int>(get_parameter("max_print_voxels_per_link").as_int()));

    auto manager_masks = manager_.getLinkVoxelMasks();
    reports_.clear();
    mismatched_links_.clear();
    reports_.reserve(voxel_data_.size());

    std::ostringstream report;
    report << "Voxel link mask validation report\n";
    report << "root_link=" << root_link_ << "\n";
    report << "voxel_size=" << manager_.getVoxelSize() << "\n";
    report << "link_count=" << voxel_data_.size() << "\n";
    report << "joints=" << joints_.size() << "\n\n";

    for (std::size_t i = 0; i < voxel_data_.size(); ++i) {
      const auto &data = voxel_data_[i];
      auto tf_it = direct_link_tfs_.find(data.name);
      const Eigen::Isometry3d tf =
          (tf_it != direct_link_tfs_.end()) ? tf_it->second : Eigen::Isometry3d::Identity();

      LinkReport link;
      link.name = data.name;
      link.local_count = data.local_voxel_centers.size();
      link.local_min = data.local_min;
      link.local_max = data.local_max;

      std::vector<long> direct_mask;
      direct_mask.reserve(data.local_voxel_centers.size());
      std::vector<VoxelSample> samples;
      samples.reserve(static_cast<std::size_t>(std::min<std::size_t>(data.local_voxel_centers.size(),
                                                                     static_cast<std::size_t>(max_print))));

      for (std::size_t k = 0; k < data.local_voxel_centers.size(); ++k) {
        const Eigen::Vector3d &local = data.local_voxel_centers[k];
        const Eigen::Vector3d world = tf * local;
        const Eigen::Vector3i idx = ::common::geometry::VoxelUtils::worldToVoxel(
            world.template cast<float>(), static_cast<float>(manager_.getVoxelSize()));
        const long flat = manager_.getIndexGrid()->getFlatVoxelId(idx);
        const Eigen::Vector3i roundtrip_idx = manager_.getIndexGrid()->getIndexFromFlatId(flat);
        const Eigen::Vector3d roundtrip_world =
            ::common::geometry::VoxelUtils::voxelToWorld(
                roundtrip_idx, static_cast<float>(manager_.getVoxelSize()))
                .template cast<double>();

        direct_mask.push_back(flat);

        if (static_cast<int>(k) < max_print) {
          samples.push_back(VoxelSample{
              local,
              world,
              idx,
              roundtrip_world,
              flat,
              world - roundtrip_world});
        }
      }

      if (direct_mask.size() > 1) {
        ::common::geometry::VoxelUtils::radixSort(direct_mask);
        direct_mask.erase(std::unique(direct_mask.begin(), direct_mask.end()),
                          direct_mask.end());
      }

      link.direct_mask = direct_mask;
      if (i < manager_masks.size()) {
        link.manager_mask = manager_masks[i];
        link.match_manager = (link.direct_mask == link.manager_mask);
      }
      link.unique_count = link.direct_mask.size();
      link.samples = samples;
      reports_.push_back(link);

      if (!link.match_manager) {
        mismatched_links_.push_back(link.name);
      }

      if (!isFocused(focus_set, link.name)) {
        continue;
      }

      report << "[" << (link.match_manager ? "OK" : "MISMATCH") << "] "
             << link.name << "\n";
      report << "  local_count=" << link.local_count
             << " unique_count=" << link.unique_count
             << " manager_count="
             << (i < manager_masks.size() ? manager_masks[i].size() : 0)
             << "\n";
      report << "  local_min=" << vec3ToString(link.local_min)
             << " local_max=" << vec3ToString(link.local_max) << "\n";

      if (i < manager_masks.size()) {
        report << "  manager_match=" << (link.match_manager ? "true" : "false")
               << "\n";
      }

      report << "  sample_voxels=" << link.samples.size() << "\n";
      for (std::size_t s = 0; s < link.samples.size(); ++s) {
        const auto &sample = link.samples[s];
        report << "    [" << s << "] local=" << vec3ToString(sample.local)
               << " world=" << vec3ToString(sample.world)
               << " idx=" << vec3iToString(sample.idx)
               << " flat=" << sample.flat_id
               << " roundtrip_world=" << vec3ToString(sample.roundtrip_world)
               << " delta=" << vec3ToString(sample.delta)
               << "\n";
      }
      if (link.direct_mask.size() <= 24) {
        report << "  ids=";
        for (std::size_t j = 0; j < link.direct_mask.size(); ++j) {
          report << (j == 0 ? " " : ", ") << link.direct_mask[j];
        }
        report << "\n";
      } else {
        report << "  ids(first8)=";
        for (std::size_t j = 0; j < std::min<std::size_t>(8, link.direct_mask.size()); ++j) {
          report << (j == 0 ? " " : ", ") << link.direct_mask[j];
        }
        report << " ... total=" << link.direct_mask.size() << "\n";
      }
      report << "\n";
    }

    const auto manager_union = manager_.getSelfVoxelMask();
    report << "Summary:\n";
    report << "  manager_union_count=" << manager_union.size() << "\n";
    report << "  mismatched_links=" << mismatched_links_.size() << "\n";
    if (!mismatched_links_.empty()) {
      report << "  mismatch_names=";
      for (std::size_t i = 0; i < mismatched_links_.size(); ++i) {
        report << (i == 0 ? " " : ", ") << mismatched_links_[i];
      }
      report << "\n";
    }

    report_ = report.str();
    RCLCPP_INFO(get_logger(), "%s", report_.c_str());

    if (!mismatched_links_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Voxel validation found %zu mismatched link(s).",
                  mismatched_links_.size());
    } else {
      RCLCPP_INFO(get_logger(), "Voxel validation passed: all checked links matched the manager masks.");
    }
  }

  void writeReportIfNeeded() {
    const std::string dump_path = get_parameter("dump_path").as_string();
    if (dump_path.empty()) {
      return;
    }
    std::ofstream ofs(dump_path);
    if (!ofs) {
      throw std::runtime_error("Failed to open dump_path: " + dump_path);
    }
    ofs << report_;
    ofs.close();
    RCLCPP_INFO(get_logger(), "Wrote voxel validation report to: %s", dump_path.c_str());
  }

  std::shared_ptr<simulation::RobotModel> model_;
  std::shared_ptr<kinematics::KinematicChain> chain_;
  robot_sim::recognition::SelfRecognitionManager manager_;
  std::vector<simulation::LinkVoxelData> voxel_data_;
  std::vector<double> joints_;
  std::map<std::string, Eigen::Isometry3d> direct_link_tfs_;
  std::vector<LinkReport> reports_;
  std::vector<std::string> mismatched_links_;
  std::string root_link_;
  std::string report_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelLinkMaskValidator>(rclcpp::NodeOptions());
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
