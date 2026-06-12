#include <ais_gng/topological_grid/topological_grid_node.hpp>

#include <cstdint>
#include <sstream>

namespace fuzzrobo::topological_grid
{

TopologicalGridNode::TopologicalGridNode(const rclcpp::NodeOptions &options)
: Node("topological_grid_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/topological_map/merged");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/topological_grid_assignments");
  summary_topic_ = this->declare_parameter<std::string>("summary_topic", "/topological_grid_assignments/summary");
  grid_spec_.cell_size = this->declare_parameter<double>("grid_size", 0.5);
  grid_spec_.origin_x = this->declare_parameter<double>("origin_x", 0.0);
  grid_spec_.origin_y = this->declare_parameter<double>("origin_y", 0.0);
  grid_spec_.origin_z = this->declare_parameter<double>("origin_z", 0.0);
  x_shift_ = this->declare_parameter<int>("x_shift", 42);
  y_shift_ = this->declare_parameter<int>("y_shift", 21);
  z_shift_ = this->declare_parameter<int>("z_shift", 0);
  offset_ = this->declare_parameter<long>("offset", 1000000L);

  voxel_pub_ = this->create_publisher<voxel_msgs::msg::Voxel>(
    output_topic_,
    rclcpp::QoS(1).reliable().transient_local());
  summary_pub_ = this->create_publisher<std_msgs::msg::String>(summary_topic_, 10);
  map_sub_ = this->create_subscription<ais_gng_msgs::msg::TopologicalMap>(
    input_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TopologicalGridNode::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "TopologicalGridNode ready: input=%s output=%s grid_size=%.3f origin=(%.3f, %.3f, %.3f)",
    input_topic_.c_str(),
    output_topic_.c_str(),
    grid_spec_.cell_size,
    grid_spec_.origin_x,
    grid_spec_.origin_y,
    grid_spec_.origin_z);
}

void TopologicalGridNode::mapCallback(const ais_gng_msgs::msg::TopologicalMap::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  publishResult(*msg);
}

void TopologicalGridNode::publishResult(const ais_gng_msgs::msg::TopologicalMap &map)
{
  const auto assignments = assignNodesToGrid(map, grid_spec_);

  voxel_msgs::msg::Voxel voxel_msg;
  voxel_msg.header.frame_id = map.header.frame_id;
  voxel_msg.header.stamp = map.header.stamp;
  voxel_msg.voxel_size = static_cast<float>(grid_spec_.cell_size);
  voxel_msg.x_shift = x_shift_;
  voxel_msg.y_shift = y_shift_;
  voxel_msg.z_shift = z_shift_;
  voxel_msg.offset = offset_;
  voxel_msg.data.reserve(assignments.size());
  for (const auto &assignment : assignments) {
    const std::uint64_t flat_id =
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(assignment.cell.x) + offset_) << x_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(assignment.cell.y) + offset_) << y_shift_) |
      (static_cast<std::uint64_t>(static_cast<std::int64_t>(assignment.cell.z) + offset_) << z_shift_);
    voxel_msg.data.push_back(flat_id);
  }
  voxel_pub_->publish(voxel_msg);

  std_msgs::msg::String summary_msg;
  std::ostringstream oss;
  oss << "{";
  oss << "\"node_count\":" << assignments.size() << ",";
  oss << "\"grid_size\":" << grid_spec_.cell_size << ",";
  oss << "\"voxel_size\":" << voxel_msg.voxel_size << ",";
  oss << "\"origin\":[" << grid_spec_.origin_x << "," << grid_spec_.origin_y << "," << grid_spec_.origin_z << "],";
  oss << "\"x_shift\":" << x_shift_ << ",";
  oss << "\"y_shift\":" << y_shift_ << ",";
  oss << "\"z_shift\":" << z_shift_ << ",";
  oss << "\"offset\":" << offset_ << ",";
  oss << "\"assignments\":[";
  for (std::size_t i = 0; i < assignments.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    const auto &a = assignments[i];
    oss << "{\"node_index\":" << a.node_index
        << ",\"cell\":[" << a.cell.x << "," << a.cell.y << "," << a.cell.z << "]"
        << ",\"voxel_id\":" << voxel_msg.data[i] << "}";
  }
  oss << "]";
  oss << "}";
  summary_msg.data = oss.str();
  summary_pub_->publish(summary_msg);
}

}  // namespace fuzzrobo::topological_grid
