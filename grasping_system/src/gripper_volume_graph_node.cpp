#include <graph/gripper_volume_graph_builder.hpp>
#include <graph/gripper_volume_topological_map.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace grasping_system::nodes
{

namespace
{

constexpr std::array<char, 8> kCacheMagic{{'G', 'V', 'O', 'L', 'C', 'A', 'C', 'H'}};
constexpr std::uint32_t kCacheVersion = 1U;

class CacheSignature
{
public:
  void appendBytes(const void *data, std::size_t size)
  {
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    for (std::size_t index = 0; index < size; ++index) {
      value_ ^= static_cast<std::uint64_t>(bytes[index]);
      value_ *= 1099511628211ULL;
    }
  }

  template<typename ValueT>
  void appendValue(const ValueT &value)
  {
    appendBytes(&value, sizeof(ValueT));
  }

  void appendString(const std::string &value)
  {
    const std::uint64_t size = value.size();
    appendValue(size);
    appendBytes(value.data(), value.size());
  }

  void appendFile(const std::string &path)
  {
    appendString(path);
    std::ifstream input(path, std::ios::binary);
    if (!input) {
      throw std::runtime_error("キャッシュ署名用メッシュの読み込み失敗: " + path);
    }

    std::array<char, 8192> buffer{};
    while (input.read(buffer.data(), static_cast<std::streamsize>(buffer.size())) || input.gcount() > 0) {
      appendBytes(buffer.data(), static_cast<std::size_t>(input.gcount()));
    }
    if (!input.eof()) {
      throw std::runtime_error("キャッシュ署名用メッシュの読み込み異常: " + path);
    }
  }

  std::uint64_t value() const noexcept
  {
    return value_;
  }

private:
  std::uint64_t value_{1469598103934665603ULL};
};

template<typename ValueT>
bool readValue(std::ifstream &input, ValueT &value)
{
  input.read(reinterpret_cast<char *>(&value), sizeof(ValueT));
  return static_cast<bool>(input);
}

template<typename ValueT>
void writeValue(std::ofstream &output, const ValueT &value)
{
  output.write(reinterpret_cast<const char *>(&value), sizeof(ValueT));
  if (!output) {
    throw std::runtime_error("グリッパ体積キャッシュの書き込み失敗");
  }
}

std::optional<ais_gng_msgs::msg::TopologicalMap> loadCachedMap(
  const std::filesystem::path &cache_path, std::uint64_t expected_signature,
  std::string &reason)
{
  std::ifstream input(cache_path, std::ios::binary);
  if (!input) {
    reason = "キャッシュ未作成";
    return std::nullopt;
  }

  std::array<char, kCacheMagic.size()> magic{};
  std::uint32_t version = 0U;
  std::uint64_t signature = 0U;
  std::uint64_t payload_size = 0U;
  input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!input || !readValue(input, version) || !readValue(input, signature) ||
    !readValue(input, payload_size))
  {
    reason = "キャッシュヘッダ不正";
    return std::nullopt;
  }
  if (magic != kCacheMagic || version != kCacheVersion) {
    reason = "キャッシュ形式不一致";
    return std::nullopt;
  }
  if (signature != expected_signature) {
    reason = "定義またはメッシュ更新";
    return std::nullopt;
  }
  if (payload_size == 0U || payload_size > static_cast<std::uint64_t>(
      std::numeric_limits<std::size_t>::max()))
  {
    reason = "キャッシュペイロード長不正";
    return std::nullopt;
  }

  rclcpp::SerializedMessage serialized_message(static_cast<std::size_t>(payload_size));
  auto &serialized = serialized_message.get_rcl_serialized_message();
  input.read(
    reinterpret_cast<char *>(serialized.buffer),
    static_cast<std::streamsize>(payload_size));
  if (!input) {
    reason = "キャッシュペイロード破損";
    return std::nullopt;
  }
  serialized.buffer_length = static_cast<std::size_t>(payload_size);

  try {
    ais_gng_msgs::msg::TopologicalMap map;
    rclcpp::Serialization<ais_gng_msgs::msg::TopologicalMap> serializer;
    serializer.deserialize_message(&serialized_message, &map);
    return map;
  } catch (const std::exception &error) {
    reason = std::string("キャッシュ復元失敗: ") + error.what();
    return std::nullopt;
  }
}

void saveCachedMap(
  const std::filesystem::path &cache_path, std::uint64_t signature,
  const ais_gng_msgs::msg::TopologicalMap &map)
{
  std::error_code error;
  if (!cache_path.parent_path().empty()) {
    std::filesystem::create_directories(cache_path.parent_path(), error);
    if (error) {
      throw std::runtime_error("キャッシュディレクトリ作成失敗: " + error.message());
    }
  }

  rclcpp::Serialization<ais_gng_msgs::msg::TopologicalMap> serializer;
  rclcpp::SerializedMessage serialized_message;
  serializer.serialize_message(&map, &serialized_message);
  const auto &serialized = serialized_message.get_rcl_serialized_message();
  const std::filesystem::path temporary_path = cache_path.string() + "." +
    std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) + ".tmp";

  {
    std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("グリッパ体積キャッシュの作成失敗: " +
              temporary_path.string());
    }
    output.write(kCacheMagic.data(), static_cast<std::streamsize>(kCacheMagic.size()));
    writeValue(output, kCacheVersion);
    writeValue(output, signature);
    const std::uint64_t payload_size = serialized.buffer_length;
    writeValue(output, payload_size);
    output.write(
      reinterpret_cast<const char *>(serialized.buffer),
      static_cast<std::streamsize>(serialized.buffer_length));
    if (!output) {
      throw std::runtime_error("グリッパ体積キャッシュのペイロード書き込み失敗");
    }
  }

  std::filesystem::rename(temporary_path, cache_path, error);
  if (error) {
    std::filesystem::remove(temporary_path);
    throw std::runtime_error("グリッパ体積キャッシュの確定失敗: " + error.message());
  }
}

}  // namespace

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
    cache_file_ = declare_parameter<std::string>("cache_file", "");
    cache_mode_ = declare_parameter<std::string>("cache_mode", "disabled");
    publish_graph_ = declare_parameter<bool>("publish_graph", true);
    exit_after_publish_ = declare_parameter<bool>("exit_after_publish", false);

    publisher_ = create_publisher<ais_gng_msgs::msg::TopologicalMap>(
      output_topic, rclcpp::QoS(1).reliable().transient_local());
    publishGraph();
  }

  bool exitAfterPublish() const noexcept
  {
    return exit_after_publish_;
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

      const std::uint64_t cache_signature = makeCacheSignature();
      const bool can_use_cache = !cache_file_.empty() && cache_mode_ == "use";
      const bool can_write_cache = !cache_file_.empty() &&
        (cache_mode_ == "use" || cache_mode_ == "refresh");
      if (!cache_file_.empty() && cache_mode_ != "use" && cache_mode_ != "refresh" &&
        cache_mode_ != "disabled")
      {
        throw std::invalid_argument(
                "cache_mode は use、refresh、disabled のいずれかを指定");
      }

      ais_gng_msgs::msg::TopologicalMap map;
      bool is_cache_hit = false;
      if (can_use_cache) {
        std::string cache_reason;
        const auto cached_map = loadCachedMap(cache_file_, cache_signature, cache_reason);
        if (cached_map.has_value()) {
          map = *cached_map;
          is_cache_hit = true;
          RCLCPP_INFO(
            get_logger(), "グリッパ体積キャッシュ利用: file=%s", cache_file_.c_str());
        } else {
          RCLCPP_INFO(
            get_logger(), "グリッパ体積キャッシュ再構築: file=%s reason=%s",
            cache_file_.c_str(), cache_reason.c_str());
        }
      }
      if (!is_cache_hit) {
        const auto volume = graph::GripperVolumeGraphBuilder::build(spec);
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id_;
        map = graph::toTopologicalMap(
          volume, header, static_cast<std::uint8_t>(label_),
          static_cast<std::uint8_t>(semantic_label_), include_cluster_);
        if (can_write_cache) {
          saveCachedMap(cache_file_, cache_signature, map);
        }
      }
      map.header.stamp = now();
      map.header.frame_id = frame_id_;
      if (publish_graph_) {
        publisher_->publish(map);
      }
      RCLCPP_INFO(
        get_logger(),
        "グリッパ体積グラフ%s: frame=%s shape=%s nodes=%zu edges=%zu "
        "meshes=%zu occupied_only=%s internal_only=%s clusters=%zu cache=%s topic=%s",
        publish_graph_ ? "公開" : "キャッシュ構築", frame_id_.c_str(), shape_.c_str(),
        map.nodes.size(), map.edges.size() / 2U,
        exclusion_count, retain_occupied_meshes_ ? "true" : "false",
        retain_internal_only_ ? "true" : "false", map.clusters.size(),
        is_cache_hit ? "hit" : "miss",
        publisher_->get_topic_name());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "グリッパ体積グラフ構築失敗: %s", error.what());
    }
  }

  std::uint64_t makeCacheSignature() const
  {
    CacheSignature signature;
    signature.appendValue(kCacheVersion);
    signature.appendString(shape_);
    signature.appendString(frame_id_);
    signature.appendString(outputTopic());
    appendValues(signature, dimensions_);
    appendValues(signature, center_);
    appendValues(signature, orientation_xyzw_);
    appendValues(signature, closing_axis_);
    signature.appendValue(resolution_);
    signature.appendValue(exclusion_clearance_);
    signature.appendValue(retain_occupied_meshes_);
    signature.appendValue(retain_internal_only_);
    signature.appendValue(include_cluster_);
    signature.appendValue(label_);
    signature.appendValue(semantic_label_);
    appendValues(signature, exclusion_mesh_scales_);
    appendValues(signature, exclusion_mesh_positions_);
    appendValues(signature, exclusion_mesh_orientations_xyzw_);
    appendValues(signature, positive_finger_mesh_indices_);
    appendValues(signature, negative_finger_mesh_indices_);
    for (const std::string &path : exclusion_mesh_paths_) {
      signature.appendFile(path);
    }
    return signature.value();
  }

  template<typename ValueT>
  static void appendValues(CacheSignature &signature, const std::vector<ValueT> &values)
  {
    const std::uint64_t size = values.size();
    signature.appendValue(size);
    for (const ValueT &value : values) {
      signature.appendValue(value);
    }
  }

  std::string outputTopic() const
  {
    return publisher_->get_topic_name();
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
  std::string cache_file_;
  std::string cache_mode_;
  bool publish_graph_{true};
  bool exit_after_publish_{false};
  rclcpp::Publisher<ais_gng_msgs::msg::TopologicalMap>::SharedPtr publisher_;
};

}  // namespace grasping_system::nodes

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<grasping_system::nodes::GripperVolumeGraphNode>();
  if (!node->exitAfterPublish()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
