#include <ais_gng_msgs/msg/topological_map.hpp>
#include <ais_gng_msgs/srv/save_object_gng_dataset.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

namespace
{

using json = nlohmann::json;
using TopologicalMap = ais_gng_msgs::msg::TopologicalMap;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

bool is_object_name(const std::string &value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '_';
  });
}

const char *nodeLabel(std::uint8_t label)
{
  if (label == TopologicalMap::SAFE_TERRAIN) {
    return "safe";
  }
  if (label == TopologicalMap::WALL) {
    return "wall";
  }
  if (label == TopologicalMap::HUMAN) {
    return "human";
  }
  if (label == TopologicalMap::CAR) {
    return "car";
  }
  return "unknown";
}

std::string currentTimestamp()
{
  const std::time_t now = std::time(nullptr);
  std::tm utc_time{};
  gmtime_r(&now, &utc_time);
  std::ostringstream stream;
  stream << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

std::string current_dataset_id_base(const std::string &object_name)
{
  const std::time_t now = std::time(nullptr);
  std::tm utc_time{};
  gmtime_r(&now, &utc_time);
  std::ostringstream stream;
  stream << object_name << '_' << std::put_time(&utc_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::string automatic_dataset_id(
  const std::filesystem::path &output_dir, const std::string &object_name)
{
  const std::regex file_name_pattern(
    "^" + object_name +
    R"(_[0-9]{8}_[0-9]{6}_([0-9]{3,9})_gng_template_v1\.json\.gz$)");
  std::uint32_t max_dataset_num = 0;
  if (std::filesystem::exists(output_dir)) {
    for (const auto &entry : std::filesystem::directory_iterator(output_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      std::smatch match;
      const std::string file_name = entry.path().filename().string();
      if (std::regex_match(file_name, match, file_name_pattern)) {
        max_dataset_num = std::max(
          max_dataset_num, static_cast<std::uint32_t>(std::stoul(match.str(1))));
      }
    }
  }
  if (max_dataset_num >= 999999999U) {
    throw std::runtime_error("自動採番できるdataset_idがありません。");
  }
  std::ostringstream id_stream;
  id_stream << current_dataset_id_base(object_name) << '_' << std::setw(3) << std::setfill('0') <<
    (max_dataset_num + 1U);
  return id_stream.str();
}

float finiteOrZero(float value)
{
  return std::isfinite(value) ? value : 0.0F;
}

bool has_nonzero_covariance(const std::array<float, 9> &covariance)
{
  return std::any_of(covariance.begin(), covariance.end(), [](float value) {
    return std::isfinite(value) && std::fabs(value) > 0.0F;
  });
}

bool has_same_stamp(
  const builtin_interfaces::msg::Time &first,
  const builtin_interfaces::msg::Time &second)
{
  return first.sec == second.sec && first.nanosec == second.nanosec;
}

void write_gzip_json(const std::filesystem::path &output_path, const json &value)
{
  const std::string text = value.dump();
  gzFile file = gzopen(output_path.c_str(), "wb9");
  if (file == nullptr) {
    throw std::runtime_error("gzip保存先を開けません: " + output_path.string());
  }
  std::size_t offset = 0;
  bool has_error = false;
  while (offset < text.size()) {
    const std::size_t remaining = text.size() - offset;
    const unsigned int chunk_size = static_cast<unsigned int>(
      std::min<std::size_t>(remaining, 1024U * 1024U));
    const int written = gzwrite(file, text.data() + offset, chunk_size);
    if (written <= 0) {
      has_error = true;
      break;
    }
    offset += static_cast<std::size_t>(written);
  }
  if (gzclose(file) != Z_OK || has_error) {
    throw std::runtime_error("gzip書込失敗: " + output_path.string());
  }
}

void write_point_cloud(
  const std::filesystem::path &output_path, const PointCloud2 &point_cloud)
{
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(point_cloud, pcl_cloud);
  pcl::PCDWriter writer;
  if (writer.writeBinaryCompressed(output_path.string(), pcl_cloud) < 0) {
    throw std::runtime_error("PCD書込失敗: " + output_path.string());
  }
}

class ObjectGngDatasetExporterNode : public rclcpp::Node
{
public:
  ObjectGngDatasetExporterNode()
  : Node("object_gng_dataset_exporter_node")
  {
    output_dir_ = declare_parameter<std::string>("output_dir", "/datasets");
    const std::string map_topic = declare_parameter<std::string>("map_topic", "topological_map");
    point_cloud_topic_ = declare_parameter<std::string>("point_cloud_topic", "");
    point_cloud_cache_num_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("point_cloud_cache_num", 4)));
    const std::string save_service =
      declare_parameter<std::string>("save_service", "/save_gng_data");
    if (output_dir_.empty()) {
      throw std::runtime_error("output_dir の指定が必要です。");
    }

    map_sub_ = create_subscription<TopologicalMap>(
      map_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ObjectGngDatasetExporterNode::onMap, this, std::placeholders::_1));
    if (!point_cloud_topic_.empty()) {
      point_cloud_sub_ = create_subscription<PointCloud2>(
        point_cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(
          &ObjectGngDatasetExporterNode::on_point_cloud, this,
          std::placeholders::_1));
    }
    save_service_ = create_service<ais_gng_msgs::srv::SaveObjectGngDataset>(
      save_service,
      std::bind(
        &ObjectGngDatasetExporterNode::saveDataset, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "物体GNG保存待機: map=%s point_cloud=%s service=%s output_dir=%s",
      map_topic.c_str(), point_cloud_topic_.empty() ? "無効" : point_cloud_topic_.c_str(),
      save_service.c_str(), output_dir_.c_str());
  }

private:
  void onMap(const TopologicalMap::SharedPtr map)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_map_ = *map;
    latest_matched_point_cloud_.reset();
    for (auto it = point_cloud_cache_.rbegin(); it != point_cloud_cache_.rend(); ++it) {
      if (has_same_stamp(map->header.stamp, (*it)->header.stamp)) {
        latest_matched_point_cloud_ = *it;
        break;
      }
    }
  }

  void on_point_cloud(const PointCloud2::ConstSharedPtr point_cloud)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    point_cloud_cache_.push_back(point_cloud);
    while (point_cloud_cache_.size() > point_cloud_cache_num_) {
      point_cloud_cache_.pop_front();
    }
    if (latest_map_ && has_same_stamp(latest_map_->header.stamp, point_cloud->header.stamp)) {
      latest_matched_point_cloud_ = point_cloud;
    }
  }

  void saveDataset(
    const std::shared_ptr<ais_gng_msgs::srv::SaveObjectGngDataset::Request> request,
    std::shared_ptr<ais_gng_msgs::srv::SaveObjectGngDataset::Response> response)
  {
    TopologicalMap map;
    PointCloud2::ConstSharedPtr point_cloud;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_map_) {
        response->success = false;
        response->message = "topological_map をまだ受信していません。";
        return;
      }
      map = *latest_map_;
      if (request->enable_point_cloud_save) {
        if (!latest_matched_point_cloud_) {
          response->success = false;
          response->message = point_cloud_topic_.empty() ?
            "元点群保存は無効です。point_cloud_topicを指定してください。" :
            "topological_mapと同じ時刻のPointCloud2を保持していません。";
          return;
        }
        point_cloud = latest_matched_point_cloud_;
      }
    }

    try {
      const std::string output_dir = request->output_dir.empty() ? output_dir_ : request->output_dir;
      if (output_dir.empty()) {
        throw std::runtime_error("output_dir の指定が必要です。");
      }
      if (!is_object_name(request->object_name)) {
        throw std::runtime_error(
                "object_name は英字開始の英数字または_だけで指定してください: " +
                request->object_name);
      }
      const std::string dataset_id = automatic_dataset_id(output_dir, request->object_name);
      const std::filesystem::path output_path =
        std::filesystem::path(output_dir) /
        (dataset_id + "_gng_template_v1.json.gz");
      const std::filesystem::path point_cloud_path =
        std::filesystem::path(output_dir) / (dataset_id + "_source.pcd");
      std::filesystem::create_directories(output_path.parent_path());
      try {
        if (point_cloud) {
          write_point_cloud(point_cloud_path, *point_cloud);
        }
        write_gzip_json(
          output_path,
          buildDataset(
            map, dataset_id, request->object_name, point_cloud.get(),
            point_cloud ? point_cloud_path.filename().string() : ""));
      } catch (...) {
        std::error_code error;
        std::filesystem::remove(output_path, error);
        std::filesystem::remove(point_cloud_path, error);
        throw;
      }
      response->success = true;
      response->message = "保存完了: " + output_path.string() +
        " nodes=" + std::to_string(map.nodes.size()) +
        " edges=" + std::to_string(map.edges.size() / 2U) +
        " clusters=" + std::to_string(map.clusters.size()) +
        " bytes=" + std::to_string(std::filesystem::file_size(output_path));
      if (point_cloud) {
        response->message += " point_cloud=" + point_cloud_path.string() +
          " point_cloud_bytes=" + std::to_string(std::filesystem::file_size(point_cloud_path));
      }
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception &error) {
      response->success = false;
      response->message = error.what();
      RCLCPP_ERROR(get_logger(), "物体GNGデータセット保存失敗: %s", error.what());
    }
  }

  json buildDataset(
    const TopologicalMap &map, const std::string &dataset_id,
    const std::string &object_name, const PointCloud2 *point_cloud,
    const std::string &point_cloud_file_name) const
  {
    json nodes = json::array();
    std::unordered_map<std::uint16_t, std::size_t> node_indices;
    for (std::size_t index = 0; index < map.nodes.size(); ++index) {
      const auto &source = map.nodes.at(index);
      json node = {
        {"x", finiteOrZero(source.pos.x)},
        {"y", finiteOrZero(source.pos.y)},
        {"z", finiteOrZero(source.pos.z)},
        {"nx", finiteOrZero(source.normal.x)},
        {"ny", finiteOrZero(source.normal.y)},
        {"nz", finiteOrZero(source.normal.z)},
      };
      if (std::fabs(source.rho) > 0.0F) {
        node["rho"] = finiteOrZero(source.rho);
      }
      const std::string label = nodeLabel(source.label);
      if (label != "unknown") {
        node["label"] = label;
      }
      if (!source.inpcl_ids.empty()) {
        node["inpcl_ids"] = source.inpcl_ids;
      }
      if (source.winner_point_count > 0U) {
        node["winner_point_count"] = source.winner_point_count;
      }
      if (has_nonzero_covariance(source.winner_point_covariance)) {
        json covariance = json::array();
        for (const float value : source.winner_point_covariance) {
          covariance.push_back(finiteOrZero(value));
        }
        node["winner_point_covariance"] = std::move(covariance);
      }
      nodes.push_back(std::move(node));
      node_indices.emplace(source.id, index);
    }

    const auto resolve_index = [&node_indices, &map](std::uint16_t value) {
        const auto found = node_indices.find(value);
        if (found != node_indices.end()) {
          return found->second;
        }
        return static_cast<std::size_t>(value) < map.nodes.size() ?
               static_cast<std::size_t>(value) : map.nodes.size();
      };

    json edges = json::array();
    for (std::size_t index = 0; index + 1U < map.edges.size(); index += 2U) {
      const std::size_t first = resolve_index(map.edges.at(index));
      const std::size_t second = resolve_index(map.edges.at(index + 1U));
      if (first == map.nodes.size() || second == map.nodes.size() || first == second) {
        continue;
      }
      edges.push_back({first, second});
    }

    json clusters = json::array();
    for (const auto &source : map.clusters) {
      json indices = json::array();
      for (const std::uint16_t node_id : source.nodes) {
        const std::size_t index = resolve_index(node_id);
        if (index != map.nodes.size()) {
          indices.push_back(index);
        }
      }
      if (!indices.empty()) {
        clusters.push_back({{"id", source.id}, {"idx", indices}});
      }
    }

    const std::string timestamp = currentTimestamp();
    json source = {
      {"sample", "ros2_ais_gng"},
      {"point_cloud_source", "topological_map"},
      {"gng_algorithm", "ais_gng"},
      {"map_frame_id", map.header.frame_id},
      {"map_stamp", {
          {"sec", map.header.stamp.sec},
          {"nanosec", map.header.stamp.nanosec},
        }},
    };
    if (point_cloud != nullptr) {
      json fields = json::array();
      bool has_packed_color = false;
      bool has_r = false;
      bool has_g = false;
      bool has_b = false;
      for (const auto &field : point_cloud->fields) {
        fields.push_back({
          {"name", field.name},
          {"offset", field.offset},
          {"datatype", field.datatype},
          {"count", field.count},
        });
        has_packed_color = has_packed_color || field.name == "rgb" || field.name == "rgba";
        has_r = has_r || field.name == "r";
        has_g = has_g || field.name == "g";
        has_b = has_b || field.name == "b";
      }
      const bool has_color = has_packed_color || (has_r && has_g && has_b);
      source["point_cloud"] = {
        {"file_name", point_cloud_file_name},
        {"format", "pcd_binary_compressed"},
        {"topic", point_cloud_topic_},
        {"frame_id", point_cloud->header.frame_id},
        {"stamp", {
            {"sec", point_cloud->header.stamp.sec},
            {"nanosec", point_cloud->header.stamp.nanosec},
          }},
        {"width", point_cloud->width},
        {"height", point_cloud->height},
        {"point_count", static_cast<std::uint64_t>(point_cloud->width) * point_cloud->height},
        {"is_dense", point_cloud->is_dense},
        {"has_color", has_color},
        {"fields", std::move(fields)},
      };
    }
    json graph = {
      {"nodes", nodes},
      {"edges", edges},
    };
    if (!clusters.empty()) {
      graph["node_clusters"] = std::move(clusters);
    }
    return {
      {"schema_version", 1},
      {"kind", "object_template"},
      {"template_id", dataset_id},
      {"display_name", object_name},
      {"canonical_yaw_deg", 0.0},
      {"created_at", timestamp},
      {"source", source},
      {"gng", graph},
    };
  }

  std::string output_dir_;
  std::string point_cloud_topic_;
  std::size_t point_cloud_cache_num_ = 4;
  std::mutex data_mutex_;
  std::optional<TopologicalMap> latest_map_;
  std::deque<PointCloud2::ConstSharedPtr> point_cloud_cache_;
  PointCloud2::ConstSharedPtr latest_matched_point_cloud_;
  rclcpp::Subscription<TopologicalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Service<ais_gng_msgs::srv::SaveObjectGngDataset>::SharedPtr save_service_;
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObjectGngDatasetExporterNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("object_gng_dataset_exporter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
