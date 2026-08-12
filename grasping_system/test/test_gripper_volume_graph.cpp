#include <graph/gripper_volume_graph_builder.hpp>
#include <graph/gripper_volume_topological_map.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>

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

  GripperVolumeGraphSpec ellipsoid_spec = box_spec;
  ellipsoid_spec.shape = GripperVolumeShape::kEllipsoid;
  ellipsoid_spec.dimensions = {0.04, 0.04, 0.04};
  const auto ellipsoid = GripperVolumeGraphBuilder::build(ellipsoid_spec);
  assert(!ellipsoid.graph.nodes().empty());
  assert(ellipsoid.graph.nodes().size() < 64U);

  return 0;
}
