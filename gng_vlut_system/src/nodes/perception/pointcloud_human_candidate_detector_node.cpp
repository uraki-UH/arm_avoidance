#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

class pointcloud_human_candidate_detector_node : public rclcpp::Node
{
public:
  using point_type = pcl::PointXYZ;
  using point_cloud_type = pcl::PointCloud<point_type>;

  pointcloud_human_candidate_detector_node()
  : Node("pointcloud_human_candidate_detector_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/lidar/points");
    const auto detections_topic = declare_parameter<std::string>(
      "detections_3d_topic", "/perception/human_candidate/detections_3d");
    const auto candidate_points_topic = declare_parameter<std::string>(
      "candidate_points_topic", "/perception/human_candidate/points");
    const auto human_candidate_detected_topic = declare_parameter<std::string>(
      "human_candidate_detected_topic", "/perception/human_candidate/is_detected");
    const auto inference_healthy_topic = declare_parameter<std::string>(
      "inference_healthy_topic", "/perception/human_candidate/is_inference_healthy");
    voxel_size_m_ = declare_parameter<double>("voxel_size_m", 0.08);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.3);
    max_range_m_ = declare_parameter<double>("max_range_m", 20.0);
    min_ground_clearance_m_ = declare_parameter<double>("min_ground_clearance_m", 0.08);
    max_ground_height_m_ = declare_parameter<double>("max_ground_height_m", 2.8);
    cluster_tolerance_m_ = declare_parameter<double>("cluster_tolerance_m", 0.25);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 20);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 20000);
    min_human_height_m_ = declare_parameter<double>("min_human_height_m", 0.8);
    max_human_height_m_ = declare_parameter<double>("max_human_height_m", 2.5);
    min_human_width_m_ = declare_parameter<double>("min_human_width_m", 0.15);
    max_human_width_m_ = declare_parameter<double>("max_human_width_m", 1.4);
    max_human_depth_m_ = declare_parameter<double>("max_human_depth_m", 1.2);
    enable_fail_safe_ = declare_parameter<bool>("enable_fail_safe", true);
    const auto ground_coefficients = declare_parameter<std::vector<double>>(
      "ground_coefficients", {0.0, 0.0, 1.0, 0.0});

    validate_parameters(ground_coefficients);
    ground_normal_ = Eigen::Vector3f(
      static_cast<float>(ground_coefficients[0]),
      static_cast<float>(ground_coefficients[1]),
      static_cast<float>(ground_coefficients[2]));
    const auto normal_norm = ground_normal_.norm();
    ground_normal_ /= normal_norm;
    ground_offset_m_ = static_cast<float>(ground_coefficients[3] / normal_norm);
    make_ground_basis();

    detections_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      detections_topic, 10);
    candidate_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      candidate_points_topic, rclcpp::SensorDataQoS());
    human_candidate_detected_publisher_ = create_publisher<std_msgs::msg::Bool>(
      human_candidate_detected_topic, 10);
    inference_healthy_publisher_ = create_publisher<std_msgs::msg::Bool>(
      inference_healthy_topic, 10);
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        pointcloud_callback(message);
      });

    RCLCPP_INFO(
      get_logger(), "深度点群の人体形状候補検出を開始: input=%s", input_topic.c_str());
  }

private:
  void validate_parameters(const std::vector<double> & ground_coefficients) const
  {
    if (voxel_size_m_ <= 0.0 || cluster_tolerance_m_ <= 0.0) {
      throw std::invalid_argument("voxel_size_mとcluster_tolerance_mは正数が必要です");
    }
    if (min_range_m_ < 0.0 || max_range_m_ <= min_range_m_) {
      throw std::invalid_argument("rangeの範囲が不正です");
    }
    if (min_ground_clearance_m_ < 0.0 || max_ground_height_m_ <= min_ground_clearance_m_) {
      throw std::invalid_argument("ground_heightの範囲が不正です");
    }
    if (min_cluster_points_ < 1 || max_cluster_points_ < min_cluster_points_) {
      throw std::invalid_argument("cluster_pointsの範囲が不正です");
    }
    if (min_human_height_m_ <= 0.0 || max_human_height_m_ <= min_human_height_m_) {
      throw std::invalid_argument("human_heightの範囲が不正です");
    }
    if (min_human_width_m_ <= 0.0 || max_human_width_m_ <= min_human_width_m_ ||
      max_human_depth_m_ <= 0.0)
    {
      throw std::invalid_argument("human_widthまたはhuman_depthの範囲が不正です");
    }
    if (ground_coefficients.size() != 4) {
      throw std::invalid_argument("ground_coefficientsには4要素が必要です");
    }
    const Eigen::Vector3d normal(
      ground_coefficients[0], ground_coefficients[1], ground_coefficients[2]);
    if (normal.norm() <= 1.0e-6) {
      throw std::invalid_argument("ground_coefficientsの法線が不正です");
    }
  }

  void make_ground_basis()
  {
    const Eigen::Vector3f reference =
      std::abs(ground_normal_.z()) < 0.9F ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    ground_axis_u_ = ground_normal_.cross(reference).normalized();
    ground_axis_v_ = ground_normal_.cross(ground_axis_u_).normalized();
  }

  static bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & message)
  {
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : message.fields) {
      has_x = has_x || field.name == "x";
      has_y = has_y || field.name == "y";
      has_z = has_z || field.name == "z";
    }
    return has_x && has_y && has_z;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    if (!has_xyz_fields(*message)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "PointCloud2にx、y、z fieldが必要です");
      publish_health(false);
      return;
    }

    point_cloud_type input_cloud;
    try {
      pcl::fromROSMsg(*message, input_cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "PointCloud2変換失敗: %s", error.what());
      publish_health(false);
      return;
    }

    auto region_cloud = std::make_shared<point_cloud_type>();
    region_cloud->reserve(input_cloud.size());
    for (const auto & point : input_cloud) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      const Eigen::Vector3f position(point.x, point.y, point.z);
      const auto range_m = position.norm();
      const auto ground_height_m = ground_normal_.dot(position) + ground_offset_m_;
      if (range_m < min_range_m_ || range_m > max_range_m_ ||
        ground_height_m < min_ground_clearance_m_ ||
        ground_height_m > max_ground_height_m_)
      {
        continue;
      }
      region_cloud->push_back(point);
    }

    auto filtered_cloud = std::make_shared<point_cloud_type>();
    pcl::VoxelGrid<point_type> voxel_filter;
    voxel_filter.setInputCloud(region_cloud);
    const auto voxel_size = static_cast<float>(voxel_size_m_);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*filtered_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    if (!filtered_cloud->empty()) {
      auto search_tree = std::make_shared<pcl::search::KdTree<point_type>>();
      search_tree->setInputCloud(filtered_cloud);
      pcl::EuclideanClusterExtraction<point_type> clustering;
      clustering.setClusterTolerance(static_cast<float>(cluster_tolerance_m_));
      clustering.setMinClusterSize(min_cluster_points_);
      clustering.setMaxClusterSize(max_cluster_points_);
      clustering.setSearchMethod(search_tree);
      clustering.setInputCloud(filtered_cloud);
      clustering.extract(cluster_indices);
    }
    publish_candidates(*message, filtered_cloud, cluster_indices);
  }

  void publish_candidates(
    const sensor_msgs::msg::PointCloud2 & input_message,
    const point_cloud_type::Ptr & filtered_cloud,
    const std::vector<pcl::PointIndices> & cluster_indices)
  {
    vision_msgs::msg::Detection3DArray detections_message;
    detections_message.header = input_message.header;
    auto candidate_cloud = std::make_shared<point_cloud_type>();
    std::size_t detection_idx = 0;

    for (const auto & indices : cluster_indices) {
      Eigen::Vector4f min_bound;
      Eigen::Vector4f max_bound;
      pcl::getMinMax3D(*filtered_cloud, indices.indices, min_bound, max_bound);
      float min_height = std::numeric_limits<float>::max();
      float max_height = std::numeric_limits<float>::lowest();
      float min_u = std::numeric_limits<float>::max();
      float max_u = std::numeric_limits<float>::lowest();
      float min_v = std::numeric_limits<float>::max();
      float max_v = std::numeric_limits<float>::lowest();
      for (const auto point_idx : indices.indices) {
        const auto & point = (*filtered_cloud)[static_cast<std::size_t>(point_idx)];
        const Eigen::Vector3f position(point.x, point.y, point.z);
        const auto height = ground_normal_.dot(position) + ground_offset_m_;
        const auto u = ground_axis_u_.dot(position);
        const auto v = ground_axis_v_.dot(position);
        min_height = std::min(min_height, height);
        max_height = std::max(max_height, height);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
      }
      const auto height_m = max_height - min_height;
      const auto width_m = std::min(max_u - min_u, max_v - min_v);
      const auto depth_m = std::max(max_u - min_u, max_v - min_v);
      if (height_m < min_human_height_m_ || height_m > max_human_height_m_ ||
        width_m < min_human_width_m_ || width_m > max_human_width_m_ ||
        depth_m > max_human_depth_m_)
      {
        continue;
      }

      vision_msgs::msg::Detection3D detection;
      detection.header = input_message.header;
      detection.id = std::to_string(detection_idx++);
      detection.bbox.center.position.x = (min_bound.x() + max_bound.x()) * 0.5F;
      detection.bbox.center.position.y = (min_bound.y() + max_bound.y()) * 0.5F;
      detection.bbox.center.position.z = (min_bound.z() + max_bound.z()) * 0.5F;
      detection.bbox.center.orientation.w = 1.0;
      detection.bbox.size.x = max_bound.x() - min_bound.x();
      detection.bbox.size.y = max_bound.y() - min_bound.y();
      detection.bbox.size.z = max_bound.z() - min_bound.z();
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = "human_candidate";
      hypothesis.hypothesis.score = 1.0;
      detection.results.push_back(hypothesis);
      detections_message.detections.push_back(detection);

      for (const auto point_idx : indices.indices) {
        candidate_cloud->push_back((*filtered_cloud)[static_cast<std::size_t>(point_idx)]);
      }
    }

    sensor_msgs::msg::PointCloud2 candidate_points_message;
    pcl::toROSMsg(*candidate_cloud, candidate_points_message);
    candidate_points_message.header = input_message.header;
    candidate_points_publisher_->publish(candidate_points_message);
    detections_publisher_->publish(detections_message);
    std_msgs::msg::Bool detected_message;
    detected_message.data = !detections_message.detections.empty();
    human_candidate_detected_publisher_->publish(detected_message);
    publish_health(true);
  }

  void publish_health(bool is_healthy)
  {
    std_msgs::msg::Bool health_message;
    health_message.data = is_healthy;
    inference_healthy_publisher_->publish(health_message);
    if (!is_healthy && enable_fail_safe_) {
      std_msgs::msg::Bool detected_message;
      detected_message.data = true;
      human_candidate_detected_publisher_->publish(detected_message);
    }
  }

  double voxel_size_m_;
  double min_range_m_;
  double max_range_m_;
  double min_ground_clearance_m_;
  double max_ground_height_m_;
  double cluster_tolerance_m_;
  int min_cluster_points_;
  int max_cluster_points_;
  double min_human_height_m_;
  double max_human_height_m_;
  double min_human_width_m_;
  double max_human_width_m_;
  double max_human_depth_m_;
  bool enable_fail_safe_;
  Eigen::Vector3f ground_normal_;
  Eigen::Vector3f ground_axis_u_;
  Eigen::Vector3f ground_axis_v_;
  float ground_offset_m_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_points_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_candidate_detected_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inference_healthy_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_human_candidate_detector_node>());
  rclcpp::shutdown();
  return 0;
}
