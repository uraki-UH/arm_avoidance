#include <core/grasp_types.hpp>
#include <rigid/rigid_grasp_lifecycle_manager.hpp>

#include "gng/GrowingNeuralGas.hpp"
#include "safety_engine/indexing/dense_spatial_index.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{

using GraspLifecycleState = grasping_system::rigid::GraspLifecycleState;
using RigidGraspVlut = grasping_system::rigid::RigidGraspVlut;
using RigidGraspVlutSeed = grasping_system::rigid::RigidGraspVlutSeed;

struct Options
{
  std::string gng_path;
  std::string vlut_path;
  int seed_limit{500};
  int angle_dim{7};
  std::array<double, 3> box_size{0.08, 0.08, 0.08};
  std::array<double, 3> object_offset{0.0, 0.0, 0.10};
};

struct Probe
{
  long voxel_id{0};
  int node_id{-1};
};

void printUsage(const char *program)
{
  std::cout
    << "Usage:\n"
    << "  " << program << " --gng PATH [--vlut PATH] [options]\n\n"
    << "Options:\n"
    << "  --seed-limit N        Active GNG nodes to inspect (default: 500, -1: all)\n"
    << "  --angle-dim N         GNG joint-space dimension (default: 7)\n"
    << "  --box-size X Y Z      Virtual payload dimensions in metres\n"
    << "  --object-offset X Y Z Payload origin in the end-effector frame\n"
    << "  --help                Show this help\n\n"
    << "If --vlut is omitted, vlut.bin next to the GNG file is used.\n"
    << "The legacy form '<gng_bin_path> [seed_limit]' is also accepted.\n";
}

bool parseInt(const char *text, int &value)
{
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < INT_MIN || parsed > INT_MAX) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parseDouble(const char *text, double &value)
{
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const double parsed = std::strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseTriple(int argc, char **argv, int &index, std::array<double, 3> &values)
{
  if (index + 3 >= argc) {
    return false;
  }
  for (std::size_t axis = 0; axis < values.size(); ++axis) {
    if (!parseDouble(argv[++index], values[axis])) {
      return false;
    }
  }
  return true;
}

bool parseOptions(int argc, char **argv, Options &options)
{
  if (argc < 2 || argv[1] == nullptr) {
    return false;
  }

  if (argv[1][0] != '-') {
    options.gng_path = argv[1];
    if (argc >= 3 && !parseInt(argv[2], options.seed_limit)) {
      return false;
    }
    if (argc > 3) {
      return false;
    }
  } else {
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--gng" || argument == "--vlut") {
        if (++i >= argc) {
          return false;
        }
        (argument == "--gng" ? options.gng_path : options.vlut_path) = argv[i];
      } else if (argument == "--seed-limit") {
        if (++i >= argc || !parseInt(argv[i], options.seed_limit)) {
          return false;
        }
      } else if (argument == "--angle-dim") {
        if (++i >= argc || !parseInt(argv[i], options.angle_dim)) {
          return false;
        }
      } else if (argument == "--box-size") {
        if (!parseTriple(argc, argv, i, options.box_size)) {
          return false;
        }
      } else if (argument == "--object-offset") {
        if (!parseTriple(argc, argv, i, options.object_offset)) {
          return false;
        }
      } else {
        return false;
      }
    }
  }

  if (options.gng_path.empty() || options.angle_dim <= 0 || options.seed_limit == 0 ||
      options.seed_limit < -1 ||
      std::any_of(options.box_size.begin(), options.box_size.end(),
                  [](double size) { return size <= 0.0; })) {
    return false;
  }
  if (options.vlut_path.empty()) {
    options.vlut_path =
      (std::filesystem::path(options.gng_path).parent_path() / "vlut.bin").string();
  }
  return true;
}

geometry_msgs::msg::Pose makePose(const Eigen::Vector3f &position,
                                  const Eigen::Quaternionf &orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

std::vector<int> effectivityNodesForVoxel(
  const robot_sim::analysis::ISpatialIndex &robot_vlut,
  const RigidGraspVlut *active_grasp_vlut,
  long voxel_id)
{
  auto node_ids = robot_vlut.getNodesInVoxel(voxel_id);
  if (active_grasp_vlut != nullptr) {
    if (const auto *grasp_node_ids = active_grasp_vlut->findNodesByVoxel(voxel_id)) {
      node_ids.insert(node_ids.end(), grasp_node_ids->begin(), grasp_node_ids->end());
    }
  }
  std::sort(node_ids.begin(), node_ids.end());
  node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
  return node_ids;
}

bool containsNode(const std::vector<int> &node_ids, int node_id)
{
  return std::find(node_ids.begin(), node_ids.end(), node_id) != node_ids.end();
}

std::optional<Probe> inspectPayloadRelations(
  const RigidGraspVlut &grasp_vlut,
  const robot_sim::analysis::ISpatialIndex &robot_vlut,
  std::size_t &payload_relation_count,
  std::size_t &added_relation_count,
  std::size_t &added_node_count,
  std::size_t &unique_voxel_count)
{
  std::optional<Probe> probe;
  std::unordered_set<int> added_nodes;
  std::unordered_set<long> unique_voxels;

  for (const auto &entry : grasp_vlut.entries()) {
    payload_relation_count += entry.related_voxel_ids.size();
    for (long voxel_id : entry.related_voxel_ids) {
      unique_voxels.insert(voxel_id);
      const auto baseline_nodes = robot_vlut.getNodesInVoxel(voxel_id);
      if (!containsNode(baseline_nodes, entry.gng_node_id)) {
        ++added_relation_count;
        added_nodes.insert(entry.gng_node_id);
        if (!probe.has_value()) {
          probe = Probe{voxel_id, entry.gng_node_id};
        }
      }
    }
  }

  added_node_count = added_nodes.size();
  unique_voxel_count = unique_voxels.size();
  return probe;
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc >= 2 && std::string(argv[1]) == "--help") {
    printUsage(argv[0]);
    return 0;
  }

  Options options;
  if (!parseOptions(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }

  using GngType = GNG::GrowingNeuralGas<Eigen::VectorXf, Eigen::Vector3f>;
  auto gng = std::make_shared<GngType>(options.angle_dim, 3, nullptr);
  if (!gng->load(options.gng_path)) {
    std::cerr << "FAIL: could not load GNG: " << options.gng_path << "\n";
    return 1;
  }

  auto robot_vlut = std::make_shared<robot_sim::analysis::DenseSpatialIndex>(
    0.02, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  if (!robot_vlut->load(options.vlut_path)) {
    std::cerr << "FAIL: could not load robot VLUT: " << options.vlut_path << "\n";
    return 1;
  }

  auto indexing = robot_vlut->getGrid().schema();
  indexing.voxel_size = robot_vlut->getVoxelSize();
  auto model = grasping_system::voxel::GraspVoxelModel::makeBox(
    options.box_size, indexing);

  std::vector<RigidGraspVlutSeed> seeds;
  seeds.reserve(static_cast<std::size_t>(std::max(0, options.seed_limit)));
  gng->forEachActiveValid([&](int id, const auto &node) {
    if (options.seed_limit >= 0 &&
        seeds.size() >= static_cast<std::size_t>(options.seed_limit)) {
      return;
    }
    RigidGraspVlutSeed seed;
    seed.gng_node_id = id;
    seed.eef_pose_in_world = makePose(node.weight_coord, node.status.ee_orientation);
    seed.object_pose_in_eef.position.x = options.object_offset[0];
    seed.object_pose_in_eef.position.y = options.object_offset[1];
    seed.object_pose_in_eef.position.z = options.object_offset[2];
    seed.object_pose_in_eef.orientation.w = 1.0;
    seed.traversal_cost = node.error_angle;
    seeds.push_back(seed);
  });
  if (seeds.empty()) {
    std::cerr << "FAIL: the GNG contains no active valid nodes\n";
    return 1;
  }

  grasping_system::core::GraspObject object;
  object.object_id = "virtual_validation_payload";
  object.object_class = "box";
  object.reference_frame = "world";
  object.shape_kind = grasping_system::core::ObjectShapeKind::kRigidPrimitive;
  object.representation_kind = grasping_system::core::ObjectRepresentationKind::kVoxel;
  object.pose_in_world.orientation.w = 1.0;
  object.voxel_resolution = indexing.voxel_size;

  grasping_system::rigid::RigidGraspLifecycleManager lifecycle;
  if (!lifecycle.upsertObject(object, model) ||
      !lifecycle.setSeeds(object.object_id, std::move(seeds)) ||
      !lifecycle.buildVlut(object.object_id)) {
    std::cerr << "FAIL: could not build the virtual payload VLUT\n";
    return 1;
  }
  if (lifecycle.state(object.object_id) != GraspLifecycleState::kTargetIdentified ||
      lifecycle.activeVlut() != nullptr) {
    std::cerr << "FAIL: invalid lifecycle state before grasp activation\n";
    return 1;
  }

  const auto *cached_vlut = lifecycle.cachedVlut(object.object_id);
  if (cached_vlut == nullptr || cached_vlut->empty()) {
    std::cerr << "FAIL: the virtual payload VLUT is empty\n";
    return 1;
  }

  std::size_t payload_relation_count = 0;
  std::size_t added_relation_count = 0;
  std::size_t added_node_count = 0;
  std::size_t unique_voxel_count = 0;
  const auto probe = inspectPayloadRelations(
    *cached_vlut, *robot_vlut, payload_relation_count, added_relation_count,
    added_node_count, unique_voxel_count);
  if (!probe.has_value()) {
    std::cerr
      << "FAIL: the virtual payload added no new node/voxel relations; "
      << "move it with --object-offset or increase --box-size\n";
    return 1;
  }

  const bool affected_before = containsNode(
    effectivityNodesForVoxel(*robot_vlut, lifecycle.activeVlut(), probe->voxel_id),
    probe->node_id);

  if (!lifecycle.beginGraspAttempt(object.object_id) ||
      lifecycle.activate(object.object_id) == nullptr) {
    std::cerr << "FAIL: could not activate the virtual grasp\n";
    return 1;
  }
  const bool affected_during = containsNode(
    effectivityNodesForVoxel(*robot_vlut, lifecycle.activeVlut(), probe->voxel_id),
    probe->node_id);

  if (lifecycle.deactivateActive() == nullptr) {
    std::cerr << "FAIL: could not release the virtual grasp\n";
    return 1;
  }
  const bool affected_after = containsNode(
    effectivityNodesForVoxel(*robot_vlut, lifecycle.activeVlut(), probe->voxel_id),
    probe->node_id);

  const bool lifecycle_ok =
    lifecycle.state(object.object_id) == GraspLifecycleState::kReleasedCached &&
    lifecycle.activeVlut() == nullptr &&
    lifecycle.lastReleasedObjectId() == object.object_id;
  if (affected_before || !affected_during || affected_after || !lifecycle_ok) {
    std::cerr << "FAIL: effectivity overlay did not follow the grasp lifecycle\n";
    return 1;
  }

  std::cout << "Virtual grasp effectivity validation: PASS\n"
            << "  gng_nodes_checked: " << cached_vlut->size() << "\n"
            << "  object_voxels: " << model.cells().size() << "\n"
            << "  payload_relations: " << payload_relation_count << "\n"
            << "  unique_world_voxels: " << unique_voxel_count << "\n"
            << "  added_relations: " << added_relation_count << "\n"
            << "  nodes_with_added_shape: " << added_node_count << "\n"
            << "  probe_voxel_id: " << probe->voxel_id << "\n"
            << "  probe_gng_node_id: " << probe->node_id << "\n"
            << "  affected_before_grasp: false\n"
            << "  affected_while_grasped: true\n"
            << "  affected_after_release: false\n";
  return 0;
}
