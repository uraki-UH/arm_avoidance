#include <graph/gripper_volume_graph_builder.hpp>
#include <graph/gripper_volume_topological_map.hpp>

#include <cassert>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{

std::string writeTestTriangleStl()
{
  const std::string path = "/tmp/grasping_system_test_triangle.stl";
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  assert(output);

  const std::array<char, 80> header{};
  const std::uint32_t triangle_count = 1;
  const std::array<float, 3> normal{0.0F, 0.0F, 1.0F};
  const std::array<std::array<float, 3>, 3> vertices{
    std::array<float, 3>{-0.01F, -0.01F, 0.0F},
    std::array<float, 3>{0.01F, -0.01F, 0.0F},
    std::array<float, 3>{0.0F, 0.01F, 0.0F}};
  const std::uint16_t attributes = 0;
  output.write(header.data(), static_cast<std::streamsize>(header.size()));
  output.write(
    reinterpret_cast<const char *>(&triangle_count), sizeof(triangle_count));
  output.write(reinterpret_cast<const char *>(normal.data()), sizeof(normal));
  output.write(reinterpret_cast<const char *>(vertices.data()), sizeof(vertices));
  output.write(reinterpret_cast<const char *>(&attributes), sizeof(attributes));
  assert(output);
  return path;
}

}  // namespace

int main()
{
  using grasping_system::graph::GripperVolumeGraphBuilder;
  using grasping_system::graph::GripperVolumeGraphSpec;
  using grasping_system::graph::GripperVolumeShape;

  GripperVolumeGraphSpec box_spec;
  box_spec.shape = GripperVolumeShape::kBox;
  box_spec.dimensions = {0.02, 0.02, 0.02};
  box_spec.resolution = 0.01;
  box_spec.pose_in_frame.position.x = 1.0;
  box_spec.pose_in_frame.orientation.w = 1.0;

  const auto box = GripperVolumeGraphBuilder::build(box_spec);
  assert(box.graph.nodes().size() == 8U);
  assert(box.graph.edges().size() == 12U);

  std_msgs::msg::Header header;
  header.frame_id = "test_tool";
  const auto map = grasping_system::graph::toTopologicalMap(box, header);
  assert(map.header.frame_id == "test_tool");
  assert(map.nodes.size() == box.graph.nodes().size());
  assert(map.edges.size() == box.graph.edges().size() * 2U);
  assert(map.clusters.size() == 1U);
  assert(map.clusters.front().nodes.size() == map.nodes.size());
  assert(std::abs(map.clusters.front().scale.x - 0.02F) < 1e-6F);
  assert(map.nodes.front().pos.x > 0.99F);

  const auto map_without_cluster = grasping_system::graph::toTopologicalMap(
    box, header, ais_gng_msgs::msg::TopologicalMap::DEFAULT,
    ais_gng_msgs::msg::TopologicalMap::SEMANTIC_DEFAULT, false);
  assert(map_without_cluster.nodes.size() == map.nodes.size());
  assert(map_without_cluster.edges.size() == map.edges.size());
  assert(map_without_cluster.clusters.empty());

  GripperVolumeGraphSpec ellipsoid_spec = box_spec;
  ellipsoid_spec.shape = GripperVolumeShape::kEllipsoid;
  ellipsoid_spec.dimensions = {0.04, 0.04, 0.04};
  const auto ellipsoid = GripperVolumeGraphBuilder::build(ellipsoid_spec);
  assert(!ellipsoid.graph.nodes().empty());
  assert(ellipsoid.graph.nodes().size() < 64U);

  const std::string mesh_path = writeTestTriangleStl();
  GripperVolumeGraphSpec mesh_spec;
  mesh_spec.dimensions = {0.01, 0.01, 0.01};
  mesh_spec.resolution = 0.01;
  mesh_spec.pose_in_frame.orientation.w = 1.0;
  grasping_system::graph::GripperVolumeMeshExclusion mesh;
  mesh.path = mesh_path;
  mesh.pose_in_frame.orientation.w = 1.0;
  mesh_spec.mesh_exclusions.push_back(mesh);

  const auto excluded_mesh = GripperVolumeGraphBuilder::build(mesh_spec);
  if (!excluded_mesh.graph.nodes().empty()) {
    return 1;
  }

  mesh_spec.retain_occupied_meshes = true;
  const auto retained_mesh = GripperVolumeGraphBuilder::build(mesh_spec);
  if (retained_mesh.graph.nodes().size() != 1U) {
    return 1;
  }

  bool rejected_missing_mesh = false;
  try {
    GripperVolumeGraphSpec invalid_spec;
    invalid_spec.retain_occupied_meshes = true;
    GripperVolumeGraphBuilder::build(invalid_spec);
  } catch (const std::invalid_argument &) {
    rejected_missing_mesh = true;
  }
  if (!rejected_missing_mesh) {
    return 1;
  }
  std::remove(mesh_path.c_str());

  return 0;
}
