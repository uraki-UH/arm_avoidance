#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/people/person_classifier.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

class rgbd_pointcloud_person_detector_node : public rclcpp::Node
{
public:
  using point_type = pcl::PointXYZRGBA;
  using point_cloud_type = pcl::PointCloud<point_type>;
  using person_cluster_type = pcl::people::PersonCluster<point_type>;

  rgbd_pointcloud_person_detector_node()
  : Node("rgbd_pointcloud_person_detector_node")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/camera/depth/color/points");
    const auto camera_info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    const auto detections_topic = declare_parameter<std::string>(
      "detections_3d_topic", "/perception/person/detections_3d");
    const auto person_points_topic = declare_parameter<std::string>(
      "person_points_topic", "/perception/person/points");
    const auto person_detected_topic = declare_parameter<std::string>(
      "person_detected_topic", "/perception/person/is_detected");
    const auto inference_healthy_topic = declare_parameter<std::string>(
      "inference_healthy_topic", "/perception/person/is_inference_healthy");
    svm_file_ = declare_parameter<std::string>(
      "svm_file", "/opt/pcl_people/trainedLinearSVMForPeopleDetectionWithHOG.yaml");
    person_confidence_th_ = declare_parameter<double>("person_confidence_th", -1.5);
    const auto voxel_size_m = declare_parameter<double>("voxel_size_m", 0.06);
    const auto min_person_height_m = declare_parameter<double>("min_person_height_m", 1.0);
    const auto max_person_height_m = declare_parameter<double>("max_person_height_m", 2.4);
    const auto min_person_width_m = declare_parameter<double>("min_person_width_m", 0.1);
    const auto max_person_width_m = declare_parameter<double>("max_person_width_m", 1.5);
    const auto min_fov_m = declare_parameter<double>("min_fov_m", 0.3);
    const auto max_fov_m = declare_parameter<double>("max_fov_m", 8.0);
    const auto min_head_dist_m = declare_parameter<double>("min_head_dist_m", 0.3);
    const auto sampling_factor = declare_parameter<int>("sampling_factor", 1);
    const auto enable_sensor_portrait = declare_parameter<bool>(
      "enable_sensor_portrait", false);
    const auto enable_head_centroid = declare_parameter<bool>(
      "enable_head_centroid", false);
    enable_fail_safe_ = declare_parameter<bool>("enable_fail_safe", true);
    const auto ground_coefficients = declare_parameter<std::vector<double>>(
      "ground_coefficients", {0.0, 1.0, 0.0, -1.0});

    validate_parameters(
      voxel_size_m, min_person_height_m, max_person_height_m,
      min_person_width_m, max_person_width_m, min_fov_m, max_fov_m,
      min_head_dist_m, sampling_factor, ground_coefficients);
    if (!std::filesystem::is_regular_file(svm_file_)) {
      throw std::runtime_error("PCL人物検出SVMがありません: " + svm_file_);
    }

    pcl::people::PersonClassifier<pcl::RGB> classifier;
    if (!classifier.loadSVMFromFile(svm_file_)) {
      throw std::runtime_error("PCL人物検出SVMを読み込めません: " + svm_file_);
    }
    people_detector_.setClassifier(classifier);
    people_detector_.setVoxelSize(static_cast<float>(voxel_size_m));
    people_detector_.setPersonClusterLimits(
      static_cast<float>(min_person_height_m), static_cast<float>(max_person_height_m),
      static_cast<float>(min_person_width_m), static_cast<float>(max_person_width_m));
    people_detector_.setFOV(static_cast<float>(min_fov_m), static_cast<float>(max_fov_m));
    people_detector_.setMinimumDistanceBetweenHeads(static_cast<float>(min_head_dist_m));
    people_detector_.setSamplingFactor(sampling_factor);
    people_detector_.setSensorPortraitOrientation(enable_sensor_portrait);
    people_detector_.setHeadCentroid(enable_head_centroid);
    ground_coefficients_.resize(4);
    for (std::size_t idx = 0; idx < ground_coefficients.size(); ++idx) {
      ground_coefficients_(idx) = static_cast<float>(ground_coefficients[idx]);
    }
    people_detector_.setGround(ground_coefficients_);

    detections_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      detections_topic, 10);
    person_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      person_points_topic, rclcpp::SensorDataQoS());
    person_detected_publisher_ = create_publisher<std_msgs::msg::Bool>(
      person_detected_topic, 10);
    inference_healthy_publisher_ = create_publisher<std_msgs::msg::Bool>(
      inference_healthy_topic, 10);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_callback(message);
      });
    pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        pointcloud_callback(message);
      });

    RCLCPP_INFO(
      get_logger(), "PCL RGB-D人物検出を開始: input=%s camera_info=%s svm=%s",
      input_topic.c_str(), camera_info_topic.c_str(), svm_file_.c_str());
  }

private:
  static void validate_parameters(
    double voxel_size_m, double min_person_height_m, double max_person_height_m,
    double min_person_width_m, double max_person_width_m, double min_fov_m,
    double max_fov_m, double min_head_dist_m, int sampling_factor,
    const std::vector<double> & ground_coefficients)
  {
    if (voxel_size_m <= 0.0) {
      throw std::invalid_argument("voxel_size_mは正数が必要です");
    }
    if (min_person_height_m <= 0.0 || max_person_height_m <= min_person_height_m) {
      throw std::invalid_argument("person_heightの範囲が不正です");
    }
    if (min_person_width_m <= 0.0 || max_person_width_m <= min_person_width_m) {
      throw std::invalid_argument("person_widthの範囲が不正です");
    }
    if (min_fov_m < 0.0 || max_fov_m <= min_fov_m) {
      throw std::invalid_argument("fovの範囲が不正です");
    }
    if (min_head_dist_m <= 0.0 || sampling_factor < 1) {
      throw std::invalid_argument("head_distまたはsampling_factorが不正です");
    }
    if (ground_coefficients.size() != 4) {
      throw std::invalid_argument("ground_coefficientsには4要素が必要です");
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
  {
    Eigen::Matrix3f intrinsics;
    intrinsics <<
      static_cast<float>(message->k[0]), static_cast<float>(message->k[1]),
      static_cast<float>(message->k[2]), static_cast<float>(message->k[3]),
      static_cast<float>(message->k[4]), static_cast<float>(message->k[5]),
      static_cast<float>(message->k[6]), static_cast<float>(message->k[7]),
      static_cast<float>(message->k[8]);
    if (intrinsics(0, 0) <= 0.0F || intrinsics(1, 1) <= 0.0F) {
      RCLCPP_ERROR(get_logger(), "CameraInfoの焦点距離が不正です");
      publish_health(false);
      return;
    }
    people_detector_.setIntrinsics(intrinsics);
    has_camera_info_ = true;
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    if (!has_camera_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "CameraInfo待ちのためRGB-D人物検出を保留");
      publish_health(false);
      return;
    }
    if (message->height <= 1 || !has_rgb_field(*message)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "organized RGB/RGBA PointCloud2が必要です: height=%u", message->height);
      publish_health(false);
      return;
    }

    auto cloud = std::make_shared<point_cloud_type>();
    try {
      pcl::fromROSMsg(*message, *cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "PointCloud2変換失敗: %s", error.what());
      publish_health(false);
      return;
    }
    if (!cloud->isOrganized()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "PCL変換後の点群がorganizedではありません");
      publish_health(false);
      return;
    }

    people_detector_.setInputCloud(cloud);
    people_detector_.setGround(ground_coefficients_);
    std::vector<person_cluster_type> clusters;
    if (!people_detector_.compute(clusters)) {
      RCLCPP_ERROR(get_logger(), "PCL RGB-D人物検出compute失敗");
      publish_health(false);
      return;
    }
    ground_coefficients_ = people_detector_.getGround();
    publish_detections(*message, clusters);
  }

  static bool has_rgb_field(const sensor_msgs::msg::PointCloud2 & message)
  {
    return std::any_of(
      message.fields.begin(), message.fields.end(),
      [](const sensor_msgs::msg::PointField & field) {
        return field.name == "rgb" || field.name == "rgba";
      });
  }

  void publish_detections(
    const sensor_msgs::msg::PointCloud2 & input_message,
    std::vector<person_cluster_type> & clusters)
  {
    vision_msgs::msg::Detection3DArray detections_message;
    detections_message.header = input_message.header;
    auto person_cloud = std::make_shared<point_cloud_type>();
    const auto no_ground_cloud = people_detector_.getNoGroundCloud();
    std::size_t detection_idx = 0;

    for (auto & cluster : clusters) {
      const auto raw_confidence = static_cast<double>(cluster.getPersonConfidence());
      if (raw_confidence <= person_confidence_th_) {
        continue;
      }
      const Eigen::Vector3f min_bound = cluster.getMin();
      const Eigen::Vector3f max_bound = cluster.getMax();
      const Eigen::Vector3f center = (min_bound + max_bound) * 0.5F;
      const Eigen::Vector3f size = max_bound - min_bound;

      vision_msgs::msg::Detection3D detection;
      detection.header = input_message.header;
      detection.id = std::to_string(detection_idx++);
      detection.bbox.center.position.x = center.x();
      detection.bbox.center.position.y = center.y();
      detection.bbox.center.position.z = center.z();
      detection.bbox.center.orientation.w = 1.0;
      detection.bbox.size.x = size.x();
      detection.bbox.size.y = size.y();
      detection.bbox.size.z = size.z();
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = "person";
      hypothesis.hypothesis.score = 1.0 / (1.0 + std::exp(-raw_confidence));
      detection.results.push_back(hypothesis);
      detections_message.detections.push_back(detection);

      for (const auto point_idx : cluster.getIndices().indices) {
        if (point_idx >= 0 && static_cast<std::size_t>(point_idx) < no_ground_cloud->size()) {
          person_cloud->push_back((*no_ground_cloud)[static_cast<std::size_t>(point_idx)]);
        }
      }
    }

    sensor_msgs::msg::PointCloud2 person_points_message;
    pcl::toROSMsg(*person_cloud, person_points_message);
    person_points_message.header = input_message.header;
    person_points_publisher_->publish(person_points_message);
    detections_publisher_->publish(detections_message);
    std_msgs::msg::Bool detected_message;
    detected_message.data = !detections_message.detections.empty();
    person_detected_publisher_->publish(detected_message);
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
      person_detected_publisher_->publish(detected_message);
    }
  }

  std::string svm_file_;
  double person_confidence_th_;
  bool enable_fail_safe_;
  bool has_camera_info_{false};
  Eigen::VectorXf ground_coefficients_;
  pcl::people::GroundBasedPeopleDetectionApp<point_type> people_detector_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr person_points_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr person_detected_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inference_healthy_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rgbd_pointcloud_person_detector_node>());
  rclcpp::shutdown();
  return 0;
}
