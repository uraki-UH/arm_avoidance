#include <graph/gripper_volume_graph_builder.hpp>
#include <graph/gripper_volume_topological_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace grasping_system::nodes
{

class GripperVolumeGraphNode : public rclcpp::Node
{
public:
  GripperVolumeGraphNode()
  : Node("gripper_volume_graph_node")
  {
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "grip_V_topological_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "tool0");
    shape_ = declare_parameter<std::string>("shape", "box");
    dimensions_ = declare_parameter<std::vector<double>>(
      "dimensions", {0.08, 0.04, 0.10});
    center_ = declare_parameter<std::vector<double>>(
      "center", {0.0, 0.0, 0.05});
    orientation_xyzw_ = declare_parameter<std::vector<double>>(
      "orientation_xyzw", {0.0, 0.0, 0.0, 1.0});
    resolution_ = declare_parameter<double>("resolution", 0.01);
    exclusion_mesh_paths_ = declare_parameter<std::vector<std::string>>(
      "exclusion_mesh_paths", std::vector<std::string>{});
    exclusion_mesh_scales_ = declare_parameter<std::vector<double>>(
      "exclusion_mesh_scales", std::vector<double>{});
    exclusion_mesh_positions_ = declare_parameter<std::vector<double>>(
      "exclusion_mesh_positions", std::vector<double>{});
    exclusion_mesh_orientations_xyzw_ = declare_parameter<std::vector<double>>(
      "exclusion_mesh_orientations_xyzw", std::vector<double>{});
    exclusion_clearance_ = declare_parameter<double>("exclusion_clearance", 0.0);
    retain_occupied_meshes_ = declare_parameter<bool>("retain_occupied_meshes", false);
    retain_internal_only_ = declare_parameter<bool>("retain_internal_only", false);
    closing_axis_ = declare_parameter<std::vector<double>>(
      "closing_axis", {0.0, 1.0, 0.0});
    positive_finger_mesh_indices_ = declare_parameter<std::vector<std::int64_t>>(
      "positive_finger_mesh_indices", std::vector<std::int64_t>{});
    negative_finger_mesh_indices_ = declare_parameter<std::vector<std::int64_t>>(
      "negative_finger_mesh_indices", std::vector<std::int64_t>{});
    include_cluster_ = declare_parameter<bool>("include_cluster", true);
    label_ = declare_parameter<int>("label", 0);
    semantic_label_ = declare_parameter<int>("semantic_label", 0);

    publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
      output_topic, rclcpp::QoS(1).reliable().transient_local());
    publishGraph();
  }

private:
  void publishGraph()
  {
    try {
      if (frame_id_.empty()) {
        throw std::invalid_argument("frame_id must not be empty");
      }
      if (dimensions_.size() != 3U || center_.size() != 3U ||
        orientation_xyzw_.size() != 4U || closing_axis_.size() != 3U)
      {
        throw std::invalid_argument(
                "dimensions, center, and closing_axis need 3 values; "
                "orientation_xyzw needs 4 values");
      }
      if (label_ < 0 || label_ > 255 || semantic_label_ < 0 || semantic_label_ > 255) {
        throw std::invalid_argument("label values must be in the uint8 range [0, 255]");
      }

      graph::GripperVolumeGraphSpec spec;
      spec.shape = graph::parseGripperVolumeShape(shape_);
      std::copy_n(dimensions_.begin(), 3, spec.dimensions.begin());
      spec.resolution = resolution_;
      spec.pose_in_frame.position.x = center_[0];
      spec.pose_in_frame.position.y = center_[1];
      spec.pose_in_frame.position.z = center_[2];
      spec.pose_in_frame.orientation.x = orientation_xyzw_[0];
      spec.pose_in_frame.orientation.y = orientation_xyzw_[1];
      spec.pose_in_frame.orientation.z = orientation_xyzw_[2];
      spec.pose_in_frame.orientation.w = orientation_xyzw_[3];
      spec.mesh_exclusion_clearance = exclusion_clearance_;
      spec.retain_occupied_meshes = retain_occupied_meshes_;
      spec.retain_internal_only = retain_internal_only_;
      std::copy_n(closing_axis_.begin(), 3, spec.closing_axis.begin());

      const std::size_t exclusion_count = exclusion_mesh_paths_.size();
      if (exclusion_mesh_scales_.size() != exclusion_count * 3U ||
        exclusion_mesh_positions_.size() != exclusion_count * 3U ||
        exclusion_mesh_orientations_xyzw_.size() != exclusion_count * 4U)
      {
        throw std::invalid_argument(
                "each exclusion mesh needs 3 scale, 3 position, and 4 orientation values");
      }
      spec.mesh_exclusions.reserve(exclusion_count);
      for (std::size_t index = 0; index < exclusion_count; ++index) {
        graph::GripperVolumeMeshExclusion exclusion;
        exclusion.path = exclusion_mesh_paths_[index];
        std::copy_n(
          exclusion_mesh_scales_.begin() + static_cast<std::ptrdiff_t>(index * 3U),
          3, exclusion.scale.begin());
        exclusion.pose_in_frame.position.x = exclusion_mesh_positions_[index * 3U];
        exclusion.pose_in_frame.position.y = exclusion_mesh_positions_[index * 3U + 1U];
        exclusion.pose_in_frame.position.z = exclusion_mesh_positions_[index * 3U + 2U];
        exclusion.pose_in_frame.orientation.x =
          exclusion_mesh_orientations_xyzw_[index * 4U];
        exclusion.pose_in_frame.orientation.y =
          exclusion_mesh_orientations_xyzw_[index * 4U + 1U];
        exclusion.pose_in_frame.orientation.z =
          exclusion_mesh_orientations_xyzw_[index * 4U + 2U];
        exclusion.pose_in_frame.orientation.w =
          exclusion_mesh_orientations_xyzw_[index * 4U + 3U];
        spec.mesh_exclusions.push_back(std::move(exclusion));
      }
      const auto copy_mesh_indices = [exclusion_count](
          const std::vector<std::int64_t> &source,
          std::vector<std::size_t> &target) {
          target.reserve(source.size());
          for (const std::int64_t index : source) {
            if (index < 0 || static_cast<std::size_t>(index) >= exclusion_count) {
              throw std::invalid_argument("finger mesh index is out of range");
            }
            target.push_back(static_cast<std::size_t>(index));
          }
        };
      copy_mesh_indices(
        positive_finger_mesh_indices_, spec.positive_finger_mesh_indices);
      copy_mesh_indices(
        negative_finger_mesh_indices_, spec.negative_finger_mesh_indices);

      const auto volume = graph::GripperVolumeGraphBuilder::build(spec);
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;
      const auto map = graph::toTopologicalMap(
        volume, header, static_cast<std::uint8_t>(label_),
        static_cast<std::uint8_t>(semantic_label_), include_cluster_);
      publisher_->publish(map);
      RCLCPP_INFO(
        get_logger(),
        "Published gripper volume graph: frame=%s shape=%s nodes=%zu edges=%zu "
        "meshes=%zu occupied_only=%s internal_only=%s clusters=%zu topic=%s",
        frame_id_.c_str(), shape_.c_str(), map.nodes.size(), map.edges.size() / 2U,
        exclusion_count, retain_occupied_meshes_ ? "true" : "false",
        retain_internal_only_ ? "true" : "false", map.clusters.size(),
        publisher_->get_topic_name());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to build gripper volume graph: %s", error.what());
    }
  }

  std::string frame_id_;
  std::string shape_;
  std::vector<double> dimensions_;
  std::vector<double> center_;
  std::vector<double> orientation_xyzw_;
  std::vector<std::string> exclusion_mesh_paths_;
  std::vector<double> exclusion_mesh_scales_;
  std::vector<double> exclusion_mesh_positions_;
  std::vector<double> exclusion_mesh_orientations_xyzw_;
  std::vector<double> closing_axis_;
  std::vector<std::int64_t> positive_finger_mesh_indices_;
  std::vector<std::int64_t> negative_finger_mesh_indices_;
  double resolution_{0.01};
  double exclusion_clearance_{0.0};
  bool retain_occupied_meshes_{false};
  bool retain_internal_only_{false};
  bool include_cluster_{true};
  int label_{0};
  int semantic_label_{0};
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<grasping_system::nodes::GripperVolumeGraphNode>());
  rclcpp::shutdown();
  return 0;
}
