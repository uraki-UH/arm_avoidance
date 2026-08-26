#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <voxel_msgs/msg/voxel.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <nlohmann/json.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nodes/bridge/reachability_voxel_accumulator.hpp"
#include "nodes/bridge/world_point_bucket_index.hpp"
#include "safety_engine/indexing/voxel_id_codec.hpp"

namespace robot_sim::bridge
{

class WorldIndexToVoxelNode : public rclcpp::Node
{
private:
  struct additional_consumer
  {
    std::string name;
    std::string target_frame_id;
    reachability_bounds bounds;
    std::unique_ptr<robot_sim::analysis::VoxelIdCodec> voxel_codec;
    std::unique_ptr<reachability_voxel_accumulator> voxel_accumulator;
    rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr roi_publisher;
  };

public:
  explicit WorldIndexToVoxelNode(const rclcpp::NodeOptions &options)
  : Node("world_index_to_voxel_node", options),
    voxel_codec_(0.02),
    world_bucket_codec_(0.2)
  {
    declare_parameter<std::string>("input_topic", "/points");
    declare_parameter<std::string>("output_topic", "/voxel_ids");
    declare_parameter<std::string>("source_frame_id", "");
    declare_parameter<std::string>("world_frame_id", "world");
    declare_parameter<std::string>("target_frame_id", "world");
    declare_parameter<bool>("enable_world_index", true);
    declare_parameter<bool>("enable_roi_query", true);
    declare_parameter<std::string>("world_bucket_topic", "/world_index/buckets");
    declare_parameter<bool>("enable_world_bucket_publish", true);
    declare_parameter<double>("voxel_size", 0.02);
    declare_parameter<double>("bucket_size", 0.2);
    declare_parameter<int>("x_shift", 42);
    declare_parameter<int>("y_shift", 21);
    declare_parameter<int>("z_shift", 0);
    declare_parameter<long>("offset", 1000000L);
    declare_parameter<bool>("enable_reachability_filter", true);
    declare_parameter<double>("min_reachability_x", -0.1);
    declare_parameter<double>("max_reachability_x", 0.5);
    declare_parameter<double>("min_reachability_y", -1.0);
    declare_parameter<double>("max_reachability_y", 1.0);
    declare_parameter<double>("min_reachability_z", -1.0);
    declare_parameter<double>("max_reachability_z", 1.0);
    declare_parameter<double>("reachability_margin_x", 0.2);
    declare_parameter<double>("reachability_margin_y", 0.2);
    declare_parameter<double>("reachability_margin_z", 0.2);
    declare_parameter<int>("max_dense_voxel_num", 8000000);
    declare_parameter<std::string>("additional_consumers_json", "[]");

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    source_frame_id_ = get_parameter("source_frame_id").as_string();
    world_frame_id_ = get_parameter("world_frame_id").as_string();
    target_frame_id_ = get_parameter("target_frame_id").as_string();
    enable_world_index_ = get_parameter("enable_world_index").as_bool();
    enable_roi_query_ = get_parameter("enable_roi_query").as_bool();
    world_bucket_topic_ = get_parameter("world_bucket_topic").as_string();
    enable_world_bucket_publish_ = get_parameter("enable_world_bucket_publish").as_bool();
    if (world_frame_id_.empty() || target_frame_id_.empty()) {
      throw rclcpp::exceptions::InvalidParametersException(
        "world_frame_idとtarget_frame_idには空でないframeが必要");
    }
    if (enable_roi_query_ && !enable_world_index_) {
      throw rclcpp::exceptions::InvalidParametersException(
        "enable_roi_queryにはenable_world_indexが必要");
    }

    const double voxel_size = get_parameter("voxel_size").as_double();
    const double bucket_size = get_parameter("bucket_size").as_double();
    if (voxel_size <= 0.0 || bucket_size <= 0.0) {
      throw rclcpp::exceptions::InvalidParametersException(
        "voxel_sizeとbucket_sizeには正の値が必要");
    }
    const int x_shift = get_parameter("x_shift").as_int();
    const int y_shift = get_parameter("y_shift").as_int();
    const int z_shift = get_parameter("z_shift").as_int();
    const long offset = static_cast<long>(get_parameter("offset").as_int());
    voxel_codec_.setVoxelSize(voxel_size);
    voxel_codec_.setIndexingParams(x_shift, y_shift, z_shift, offset);
    world_bucket_codec_.setVoxelSize(bucket_size);
    world_bucket_codec_.setIndexingParams(x_shift, y_shift, z_shift, offset);
    world_index_ = std::make_unique<world_point_bucket_index>(bucket_size);

    reachability_bounds_.enable_filter =
      get_parameter("enable_reachability_filter").as_bool();
    reachability_bounds_.min_corner = Eigen::Vector3d(
      get_parameter("min_reachability_x").as_double(),
      get_parameter("min_reachability_y").as_double(),
      get_parameter("min_reachability_z").as_double());
    reachability_bounds_.max_corner = Eigen::Vector3d(
      get_parameter("max_reachability_x").as_double(),
      get_parameter("max_reachability_y").as_double(),
      get_parameter("max_reachability_z").as_double());
    reachability_bounds_.margin = Eigen::Vector3d(
      get_parameter("reachability_margin_x").as_double(),
      get_parameter("reachability_margin_y").as_double(),
      get_parameter("reachability_margin_z").as_double());
    try {
      reachability_bounds_.validate();
    } catch (const std::invalid_argument &error) {
      throw rclcpp::exceptions::InvalidParametersException(error.what());
    }
    const std::size_t max_dense_voxel_num = static_cast<std::size_t>(
      std::max<std::int64_t>(0, get_parameter("max_dense_voxel_num").as_int()));
    voxel_accumulator_ = std::make_unique<reachability_voxel_accumulator>(
      voxel_codec_, reachability_bounds_, max_dense_voxel_num);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS voxel_qos(1);
    voxel_qos.reliable().transient_local();
    roi_publisher_ = create_publisher<voxel_msgs::msg::Voxel>(output_topic_, voxel_qos);
    if (enable_world_index_ && enable_world_bucket_publish_) {
      world_bucket_publisher_ = create_publisher<voxel_msgs::msg::Voxel>(
        world_bucket_topic_, voxel_qos);
    }
    configureAdditionalConsumers(
      get_parameter("additional_consumers_json").as_string(), max_dense_voxel_num, voxel_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&WorldIndexToVoxelNode::pointCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "WorldIndexToVoxelNode initialized. input=%s output=%s world_index=%s roi_query=%s additional_consumer_num=%zu world_bucket=%s world_frame=%s target_frame=%s bucket_size=%.3f voxel_size=%.4f accumulator=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      enable_world_index_ ? "enabled" : "disabled",
      enable_roi_query_ ? "index" : "direct",
      additional_consumers_.size(),
      enable_world_index_ && enable_world_bucket_publish_
        ? world_bucket_topic_.c_str() : "disabled",
      world_frame_id_.c_str(), target_frame_id_.c_str(), world_index_->bucket_size(),
      voxel_codec_.voxelSize(),
      voxel_accumulator_->uses_dense_bitmap() ? "dense_bitmap" : "hash");
  }

private:
  std::string resolveSourceFrameId(const sensor_msgs::msg::PointCloud2 &msg) const
  {
    if (!source_frame_id_.empty()) {
      return source_frame_id_;
    }
    if (msg.header.frame_id.empty() || msg.header.frame_id == "map") {
      return world_frame_id_;
    }
    return msg.header.frame_id;
  }

  bool lookupTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    const sensor_msgs::msg::PointCloud2 &msg,
    Eigen::Isometry3d &source_to_target) const
  {
    if (target_frame == source_frame) {
      source_to_target = Eigen::Isometry3d::Identity();
      return true;
    }
    const bool has_cloud_stamp =
      msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0;
    try {
      const geometry_msgs::msg::TransformStamped transform = has_cloud_stamp
        ? tf_buffer_->lookupTransform(
          target_frame, source_frame, rclcpp::Time(msg.header.stamp))
        : tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      source_to_target = tf2::transformToEigen(transform.transform);
      return true;
    } catch (const tf2::TransformException &) {
      if (!has_cloud_stamp) {
        return false;
      }
      try {
        const geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        source_to_target = tf2::transformToEigen(transform.transform);
        return true;
      } catch (const tf2::TransformException &) {
        return false;
      }
    }
  }

  static bool isIdentityTransform(const Eigen::Isometry3d &transform)
  {
    return transform.translation().norm() <= 1e-9 &&
      (transform.linear() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= 1e-9;
  }

  void configureAdditionalConsumers(
    const std::string &additional_consumers_json,
    std::size_t max_dense_voxel_num,
    const rclcpp::QoS &voxel_qos)
  {
    nlohmann::json entries;
    try {
      entries = nlohmann::json::parse(additional_consumers_json);
    } catch (const nlohmann::json::exception &error) {
      throw rclcpp::exceptions::InvalidParametersException(
        std::string("additional_consumers_jsonのJSON形式不正: ") + error.what());
    }
    if (!entries.is_array()) {
      throw rclcpp::exceptions::InvalidParametersException(
        "additional_consumers_jsonにはJSON配列が必要");
    }

    for (const nlohmann::json &entry : entries) {
      if (!entry.is_object()) {
        throw rclcpp::exceptions::InvalidParametersException(
          "additional_consumers_jsonの各要素にはJSON objectが必要");
      }
      additional_consumer consumer;
      consumer.name = entry.value("name", "additional");
      consumer.target_frame_id = entry.value("target_frame_id", "");
      const std::string output_topic = entry.value("output_topic", "");
      const double voxel_size = entry.value("voxel_size", 0.0);
      if (consumer.name.empty() || consumer.target_frame_id.empty() || output_topic.empty() ||
        voxel_size <= 0.0)
      {
        throw rclcpp::exceptions::InvalidParametersException(
          "additional consumerにはname、target_frame_id、output_topic、正のvoxel_sizeが必要");
      }
      consumer.bounds.enable_filter = entry.value("enable_reachability_filter", true);
      consumer.bounds.min_corner = Eigen::Vector3d(
        entry.value("min_reachability_x", -0.1),
        entry.value("min_reachability_y", -1.0),
        entry.value("min_reachability_z", -1.0));
      consumer.bounds.max_corner = Eigen::Vector3d(
        entry.value("max_reachability_x", 0.5),
        entry.value("max_reachability_y", 1.0),
        entry.value("max_reachability_z", 1.0));
      consumer.bounds.margin = Eigen::Vector3d(
        entry.value("reachability_margin_x", 0.2),
        entry.value("reachability_margin_y", 0.2),
        entry.value("reachability_margin_z", 0.2));
      try {
        consumer.bounds.validate();
      } catch (const std::invalid_argument &error) {
        throw rclcpp::exceptions::InvalidParametersException(
          "additional consumer=" + consumer.name + ": " + error.what());
      }
      consumer.voxel_codec = std::make_unique<robot_sim::analysis::VoxelIdCodec>(
        voxel_size);
      consumer.voxel_codec->setIndexingParams(
        entry.value("x_shift", 42),
        entry.value("y_shift", 21),
        entry.value("z_shift", 0),
        entry.value("offset", 1000000L));
      const std::int64_t configured_max_dense_voxel_num = entry.value(
        "max_dense_voxel_num", static_cast<std::int64_t>(max_dense_voxel_num));
      const std::size_t consumer_max_dense_voxel_num = static_cast<std::size_t>(
        std::max<std::int64_t>(0, configured_max_dense_voxel_num));
      consumer.voxel_accumulator = std::make_unique<reachability_voxel_accumulator>(
        *consumer.voxel_codec, consumer.bounds, consumer_max_dense_voxel_num);
      consumer.roi_publisher = create_publisher<voxel_msgs::msg::Voxel>(
        output_topic, voxel_qos);
      additional_consumers_.push_back(std::move(consumer));
    }
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> makeWorldQueryBounds(
    const Eigen::Isometry3d &target_to_world) const
  {
    const Eigen::Vector3d min_target = reachability_bounds_.min_corner -
      reachability_bounds_.margin;
    const Eigen::Vector3d max_target = reachability_bounds_.max_corner +
      reachability_bounds_.margin;
    Eigen::Vector3d min_world = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_world = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
    for (int corner_idx = 0; corner_idx < 8; ++corner_idx) {
      Eigen::Vector3d corner;
      for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
        corner[axis_idx] = (corner_idx & (1 << axis_idx)) != 0
          ? max_target[axis_idx] : min_target[axis_idx];
      }
      const Eigen::Vector3d world_corner = target_to_world * corner;
      min_world = min_world.cwiseMin(world_corner);
      max_world = max_world.cwiseMax(world_corner);
    }
    return {min_world, max_world};
  }

  std::pair<Eigen::Vector3d, Eigen::Vector3d> makeWorldQueryBounds(
    const reachability_bounds &bounds,
    const Eigen::Isometry3d &target_to_world) const
  {
    const Eigen::Vector3d min_target = bounds.min_corner - bounds.margin;
    const Eigen::Vector3d max_target = bounds.max_corner + bounds.margin;
    Eigen::Vector3d min_world = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
    Eigen::Vector3d max_world = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
    for (int corner_idx = 0; corner_idx < 8; ++corner_idx) {
      Eigen::Vector3d corner;
      for (int axis_idx = 0; axis_idx < 3; ++axis_idx) {
        corner[axis_idx] = (corner_idx & (1 << axis_idx)) != 0
          ? max_target[axis_idx] : min_target[axis_idx];
      }
      const Eigen::Vector3d world_corner = target_to_world * corner;
      min_world = min_world.cwiseMin(world_corner);
      max_world = max_world.cwiseMax(world_corner);
    }
    return {min_world, max_world};
  }

  void publishWorldBuckets(const std_msgs::msg::Header &header)
  {
    if (!world_bucket_publisher_) {
      return;
    }
    std::vector<long> world_bucket_ids;
    world_bucket_ids.reserve(world_index_->bucket_num());
    world_index_->visit_buckets([&world_bucket_ids, this](
      const world_bucket_key &key,
      std::size_t) {
      world_bucket_ids.push_back(world_bucket_codec_.toFlatId(
        Eigen::Vector3i(key.x, key.y, key.z)));
    });
    std::sort(world_bucket_ids.begin(), world_bucket_ids.end());
    world_bucket_publisher_->publish(
      world_bucket_codec_.makeMessage(header, world_bucket_ids));
  }

  void publishAdditionalConsumers(const sensor_msgs::msg::PointCloud2 &msg)
  {
    for (additional_consumer &consumer : additional_consumers_) {
      Eigen::Isometry3d world_to_target = Eigen::Isometry3d::Identity();
      if (!lookupTransform(
          consumer.target_frame_id, world_frame_id_, msg, world_to_target))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "追加ROI用TF取得失敗: consumer=%s target=%s source=%s",
          consumer.name.c_str(), consumer.target_frame_id.c_str(), world_frame_id_.c_str());
        continue;
      }

      const bool is_world_to_target_identity = isIdentityTransform(world_to_target);
      const auto [min_world, max_world] = makeWorldQueryBounds(
        consumer.bounds, world_to_target.inverse());
      consumer.voxel_accumulator->begin_frame(world_index_->point_num());
      world_bucket_query_stats query_stats;
      try {
        query_stats = world_index_->query_aabb(
          min_world, max_world,
          [&consumer, &world_to_target, is_world_to_target_identity](
            const Eigen::Vector3f &world_point) {
            if (is_world_to_target_identity) {
              consumer.voxel_accumulator->add_point_in_target_frame(
                world_point.cast<double>());
            } else {
              consumer.voxel_accumulator->add_point(
                world_point.cast<double>(), world_to_target);
            }
          });
      } catch (const std::exception &error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "追加ROI抽出失敗: consumer=%s error=%s",
          consumer.name.c_str(), error.what());
        continue;
      }

      const reachability_voxelization_stats voxel_stats =
        consumer.voxel_accumulator->stats();
      const std::vector<long> &roi_voxel_ids =
        consumer.voxel_accumulator->finish_voxel_ids();
      std_msgs::msg::Header roi_header = msg.header;
      roi_header.frame_id = consumer.target_frame_id;
      if (roi_header.stamp.sec == 0 && roi_header.stamp.nanosec == 0) {
        roi_header.stamp = now();
      }
      consumer.roi_publisher->publish(
        consumer.voxel_codec->makeMessage(roi_header, roi_voxel_ids));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "追加ROI出力: consumer=%s candidate=%zu accepted=%zu roi_points=%zu roi_voxels=%zu target_identity=%s",
        consumer.name.c_str(), query_stats.candidate_point_num,
        query_stats.accepted_point_num, voxel_stats.accepted_point_count,
        roi_voxel_ids.size(), is_world_to_target_identity ? "true" : "false");
    }
  }

  void accumulateDirectPoints(
    const sensor_msgs::msg::PointCloud2 &msg,
    const Eigen::Isometry3d &source_to_target,
    reachability_voxel_accumulator &accumulator) const
  {
    const bool is_source_to_target_identity = isIdentityTransform(source_to_target);
    accumulator.begin_frame(static_cast<std::size_t>(msg.width) * msg.height);
    sensor_msgs::PointCloud2ConstIterator<float> point_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> point_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> point_z(msg, "z");
    for (; point_x != point_x.end(); ++point_x, ++point_y, ++point_z) {
      const Eigen::Vector3d source_point(*point_x, *point_y, *point_z);
      if (is_source_to_target_identity) {
        accumulator.add_point_in_target_frame(source_point);
      } else {
        accumulator.add_point(source_point, source_to_target);
      }
    }
  }

  bool publishPrimaryDirect(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &source_frame,
    reachability_voxelization_stats &voxel_stats,
    std::size_t &roi_voxel_num,
    bool &is_source_to_target_identity)
  {
    Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
    if (!lookupTransform(target_frame_id_, source_frame, msg, source_to_target)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "直接ROI用TF取得失敗: target=%s source=%s",
        target_frame_id_.c_str(), source_frame.c_str());
      return false;
    }
    is_source_to_target_identity = isIdentityTransform(source_to_target);
    accumulateDirectPoints(msg, source_to_target, *voxel_accumulator_);
    voxel_stats = voxel_accumulator_->stats();
    const std::vector<long> &voxel_ids = voxel_accumulator_->finish_voxel_ids();
    roi_voxel_num = voxel_ids.size();
    std_msgs::msg::Header header = msg.header;
    header.frame_id = target_frame_id_;
    if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
      header.stamp = now();
    }
    roi_publisher_->publish(voxel_codec_.makeMessage(header, voxel_ids));
    return true;
  }

  void publishAdditionalConsumersDirect(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &source_frame)
  {
    for (additional_consumer &consumer : additional_consumers_) {
      Eigen::Isometry3d source_to_target = Eigen::Isometry3d::Identity();
      if (!lookupTransform(
          consumer.target_frame_id, source_frame, msg, source_to_target))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "追加直接ROI用TF取得失敗: consumer=%s target=%s source=%s",
          consumer.name.c_str(), consumer.target_frame_id.c_str(), source_frame.c_str());
        continue;
      }
      accumulateDirectPoints(msg, source_to_target, *consumer.voxel_accumulator);
      const reachability_voxelization_stats voxel_stats =
        consumer.voxel_accumulator->stats();
      const std::vector<long> &voxel_ids =
        consumer.voxel_accumulator->finish_voxel_ids();
      std_msgs::msg::Header header = msg.header;
      header.frame_id = consumer.target_frame_id;
      if (header.stamp.sec == 0 && header.stamp.nanosec == 0) {
        header.stamp = now();
      }
      consumer.roi_publisher->publish(
        consumer.voxel_codec->makeMessage(header, voxel_ids));
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "追加直接ROI出力: consumer=%s roi_points=%zu roi_voxels=%zu source_to_target_identity=%s",
        consumer.name.c_str(), voxel_stats.accepted_point_count, voxel_ids.size(),
        isIdentityTransform(source_to_target) ? "true" : "false");
    }
  }

  void pointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    const std::string source_frame = resolveSourceFrameId(*msg);
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "空の点群frame_idのためworld index更新を中止");
      return;
    }

    const std::size_t input_point_num = static_cast<std::size_t>(msg->width) * msg->height;
    reachability_voxelization_stats direct_voxel_stats;
    std::size_t direct_roi_voxel_num = 0;
    bool has_direct_primary_output = false;
    bool is_direct_primary_identity = false;
    if (!enable_roi_query_) {
      has_direct_primary_output = publishPrimaryDirect(
        *msg, source_frame, direct_voxel_stats, direct_roi_voxel_num,
        is_direct_primary_identity);
      publishAdditionalConsumersDirect(*msg, source_frame);
      if (!enable_world_index_) {
        const double processing_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - processing_start).count();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "直接ROI voxel化: input=%zu accepted=%zu outside=%zu nonfinite=%zu roi_voxels=%zu additional_consumer_num=%zu processing_ms=%.3f source_to_target_identity=%s",
          direct_voxel_stats.input_point_count, direct_voxel_stats.accepted_point_count,
          direct_voxel_stats.outside_point_count, direct_voxel_stats.nonfinite_point_count,
          direct_roi_voxel_num, additional_consumers_.size(), processing_ms,
          is_direct_primary_identity ? "true" : "false");
        return;
      }
    }

    Eigen::Isometry3d source_to_world = Eigen::Isometry3d::Identity();
    if (!lookupTransform(world_frame_id_, source_frame, *msg, source_to_world)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "world index用TF取得失敗: target=%s source=%s",
        world_frame_id_.c_str(), source_frame.c_str());
      return;
    }
    const bool is_source_to_world_identity = isIdentityTransform(source_to_world);
    if (!enable_roi_query_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "world indexは可視化用に構築し、ROI voxel化は直接方式を使用");
    }

    world_index_->begin_frame(input_point_num);
    sensor_msgs::PointCloud2ConstIterator<float> point_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> point_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> point_z(*msg, "z");
    for (; point_x != point_x.end(); ++point_x, ++point_y, ++point_z) {
      const Eigen::Vector3f source_point(*point_x, *point_y, *point_z);
      const Eigen::Vector3f world_point = is_source_to_world_identity
        ? source_point
        : (source_to_world * source_point.cast<double>()).cast<float>();
      world_index_->add_point(world_point);
    }

    if (!enable_roi_query_) {
      std_msgs::msg::Header world_header = msg->header;
      world_header.frame_id = world_frame_id_;
      if (world_header.stamp.sec == 0 && world_header.stamp.nanosec == 0) {
        world_header.stamp = now();
      }
      publishWorldBuckets(world_header);
      const double processing_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - processing_start).count();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "world index可視化更新: input=%zu world_points=%zu world_buckets=%zu primary_output=%s roi_voxels=%zu additional_consumer_num=%zu processing_ms=%.3f source_to_world_identity=%s",
        input_point_num, world_index_->point_num(), world_index_->bucket_num(),
        has_direct_primary_output ? "true" : "false", direct_roi_voxel_num,
        additional_consumers_.size(), processing_ms,
        is_source_to_world_identity ? "true" : "false");
      return;
    }

    world_bucket_query_stats query_stats;
    reachability_voxelization_stats voxel_stats;
    std::size_t roi_voxel_num = 0;
    bool has_primary_output = false;
    bool is_world_to_target_identity = false;
    Eigen::Isometry3d world_to_target = Eigen::Isometry3d::Identity();
    if (!lookupTransform(target_frame_id_, world_frame_id_, *msg, world_to_target)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROI用TF取得失敗: target=%s source=%s",
        target_frame_id_.c_str(), world_frame_id_.c_str());
    } else {
      is_world_to_target_identity = isIdentityTransform(world_to_target);
      const auto [min_world, max_world] = makeWorldQueryBounds(world_to_target.inverse());
      voxel_accumulator_->begin_frame(world_index_->point_num());
      try {
        query_stats = world_index_->query_aabb(
          min_world, max_world,
          [this, &world_to_target, is_world_to_target_identity](
            const Eigen::Vector3f &world_point) {
            if (is_world_to_target_identity) {
              voxel_accumulator_->add_point_in_target_frame(world_point.cast<double>());
            } else {
              voxel_accumulator_->add_point(world_point.cast<double>(), world_to_target);
            }
          });
        voxel_stats = voxel_accumulator_->stats();
        const std::vector<long> &roi_voxel_ids = voxel_accumulator_->finish_voxel_ids();
        roi_voxel_num = roi_voxel_ids.size();
        std_msgs::msg::Header roi_header = msg->header;
        roi_header.frame_id = target_frame_id_;
        if (roi_header.stamp.sec == 0 && roi_header.stamp.nanosec == 0) {
          roi_header.stamp = now();
        }
        roi_publisher_->publish(voxel_codec_.makeMessage(roi_header, roi_voxel_ids));
        has_primary_output = true;
      } catch (const std::exception &error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "world index ROI抽出失敗: %s", error.what());
      }
    }

    publishAdditionalConsumers(*msg);
    std_msgs::msg::Header world_header = msg->header;
    world_header.frame_id = world_frame_id_;
    if (world_header.stamp.sec == 0 && world_header.stamp.nanosec == 0) {
      world_header.stamp = now();
    }
    publishWorldBuckets(world_header);

    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "world index更新: input=%zu world_points=%zu world_buckets=%zu primary_output=%s candidate=%zu accepted=%zu roi_points=%zu roi_voxels=%zu additional_consumer_num=%zu processing_ms=%.3f source_to_world_identity=%s world_to_target_identity=%s",
      input_point_num, world_index_->point_num(), world_index_->bucket_num(),
      has_primary_output ? "true" : "false",
      query_stats.candidate_point_num, query_stats.accepted_point_num,
      voxel_stats.accepted_point_count, roi_voxel_num, additional_consumers_.size(), processing_ms,
      is_source_to_world_identity ? "true" : "false",
      is_world_to_target_identity ? "true" : "false");
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string source_frame_id_;
  std::string world_frame_id_;
  std::string target_frame_id_;
  std::string world_bucket_topic_;
  bool enable_world_index_{true};
  bool enable_roi_query_{true};
  bool enable_world_bucket_publish_{true};
  reachability_bounds reachability_bounds_;
  robot_sim::analysis::VoxelIdCodec voxel_codec_;
  robot_sim::analysis::VoxelIdCodec world_bucket_codec_;
  std::unique_ptr<world_point_bucket_index> world_index_;
  std::unique_ptr<reachability_voxel_accumulator> voxel_accumulator_;
  std::vector<additional_consumer> additional_consumers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr roi_publisher_;
  rclcpp::Publisher<voxel_msgs::msg::Voxel>::SharedPtr world_bucket_publisher_;
};

}  // robot_sim::bridge名前空間終端

RCLCPP_COMPONENTS_REGISTER_NODE(robot_sim::bridge::WorldIndexToVoxelNode)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_sim::bridge::WorldIndexToVoxelNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
