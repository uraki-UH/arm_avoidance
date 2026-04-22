#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include "core_safety/spatial/index_voxel_grid.hpp"

using namespace std::chrono_literals;

class VoxelStatusTestPublisher : public rclcpp::Node {
public:
  VoxelStatusTestPublisher() : Node("voxel_status_test_publisher") {
    this->declare_parameter<std::vector<int64_t>>("occupied_voxels", std::vector<int64_t>{});
    this->declare_parameter<std::vector<int64_t>>("danger_voxels", std::vector<int64_t>{});
    this->declare_parameter<std::string>("scenario", "static");
    this->declare_parameter<double>("scenario_period_s", 1.0);
    this->declare_parameter<std::vector<double>>("sphere_center_world_cm", std::vector<double>{20.0, 20.0, 30.0});
    this->declare_parameter<double>("sphere_radius_cm", 10.0);
    this->declare_parameter<double>("sphere_danger_margin_cm", 3.0);
    this->declare_parameter<double>("sphere_orbit_radius_cm", 0.0);
    this->declare_parameter<double>("sphere_orbit_period_s", 0.0);
    this->declare_parameter<double>("sphere_orbit_phase_rad", 0.0);
    this->declare_parameter<double>("voxel_size", 0.02);
    this->declare_parameter<double>("publish_hz", 1.0);
    this->declare_parameter<bool>("publish_once", false);

    refreshParameters();

    occupied_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
        "/occupied_voxels", rclcpp::SystemDefaultsQoS());
    danger_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>(
        "/danger_voxels", rclcpp::SystemDefaultsQoS());

    publishCurrentScenario();

    if (!publish_once_) {
      const double hz = std::max(0.1, scenario_period_s_ > 0.0 ? (1.0 / scenario_period_s_) : publish_hz_);
      publish_interval_s_ = 1.0 / hz;
      timer_ = this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>(1.0 / hz)),
          std::bind(&VoxelStatusTestPublisher::publishCurrentScenario, this));
    } else {
      publish_interval_s_ = 0.0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Voxel status test publisher started: scenario=%s occ=%zu danger=%zu period=%.2fs once=%s",
                scenario_.c_str(), occupied_voxels_.size(), danger_voxels_.size(),
                scenario_period_s_, publish_once_ ? "true" : "false");
  }

private:
  void refreshParameters() {
    occupied_voxels_ = this->get_parameter("occupied_voxels").as_integer_array();
    danger_voxels_ = this->get_parameter("danger_voxels").as_integer_array();
    scenario_ = this->get_parameter("scenario").as_string();
    scenario_period_s_ = this->get_parameter("scenario_period_s").as_double();
    sphere_center_world_cm_ = this->get_parameter("sphere_center_world_cm").as_double_array();
    sphere_radius_cm_ = this->get_parameter("sphere_radius_cm").as_double();
    sphere_danger_margin_cm_ = this->get_parameter("sphere_danger_margin_cm").as_double();
    sphere_orbit_radius_cm_ = this->get_parameter("sphere_orbit_radius_cm").as_double();
    sphere_orbit_period_s_ = this->get_parameter("sphere_orbit_period_s").as_double();
    sphere_orbit_phase_rad_ = this->get_parameter("sphere_orbit_phase_rad").as_double();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    publish_hz_ = this->get_parameter("publish_hz").as_double();
    publish_once_ = this->get_parameter("publish_once").as_bool();
  }

  static std_msgs::msg::Int64MultiArray makeMsg(const std::vector<int64_t> &values) {
    std_msgs::msg::Int64MultiArray msg;
    msg.data.reserve(values.size());
    for (int64_t v : values) {
      msg.data.push_back(v);
    }
    return msg;
  }

  void publishCurrentScenario() {
    refreshParameters();
    if (scenario_ == "static") {
      occupied_pub_->publish(makeMsg(occupied_voxels_));
      danger_pub_->publish(makeMsg(danger_voxels_));
      RCLCPP_INFO(this->get_logger(),
                  "Published voxel test input: occupied=%zu danger=%zu",
                  occupied_voxels_.size(), danger_voxels_.size());
      return;
    }

    const auto pair = scenarioForTick(tick_++);
    occupied_pub_->publish(makeMsg(pair.first));
    danger_pub_->publish(makeMsg(pair.second));
    RCLCPP_INFO(this->get_logger(),
                "Published voxel test scenario '%s' step=%zu occ=%zu danger=%zu",
                scenario_.c_str(), tick_, pair.first.size(), pair.second.size());
  }

  std::pair<std::vector<int64_t>, std::vector<int64_t>> scenarioForTick(std::size_t tick) const {
    const std::vector<int64_t> base_a{12345, 12346, 12347, 12348, 12349, 12350};
    const std::vector<int64_t> base_b{22345, 22346, 22347, 22348, 22349, 22350};
    const std::vector<int64_t> base_c{32345, 32346, 32347, 32348, 32349, 32350};

    auto shift = [tick](std::vector<int64_t> values, int64_t delta) {
      for (auto &v : values) v += static_cast<int64_t>(tick) * delta;
      return values;
    };

    if (scenario_ == "flip") {
      if (tick % 2 == 0) return {shift(base_a, 0), shift(std::vector<int64_t>{12348, 12349}, 0)};
      return {shift(base_b, 0), shift(std::vector<int64_t>{22348, 22349, 22350}, 0)};
    }

    if (scenario_ == "sweep") {
      auto occ = shift(base_a, 7);
      auto dan = shift(base_c, 7);
      return {occ, dan};
    }

    if (scenario_ == "dense") {
      auto occ = shift(base_a, 0);
      auto dan = shift(base_a, 0);
      dan.push_back(12351);
      dan.push_back(12352);
      return {occ, dan};
    }

    if (scenario_ == "sphere") {
      return generateSphereScenario(tick);
    }

    return {occupied_voxels_, danger_voxels_};
  }

  Eigen::Vector3d sphereCenterWorldMForTick(std::size_t tick) const {
    if (sphere_center_world_cm_.size() != 3) {
      return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d base_center_world_m(
        sphere_center_world_cm_[0] / 100.0,
        sphere_center_world_cm_[1] / 100.0,
        sphere_center_world_cm_[2] / 100.0);

    if (sphere_orbit_radius_cm_ <= 0.0 || sphere_orbit_period_s_ <= 0.0 || publish_interval_s_ <= 0.0) {
      return base_center_world_m;
    }

    const double elapsed_s = static_cast<double>(tick) * publish_interval_s_;
    const double theta = (2.0 * M_PI * std::fmod(elapsed_s, sphere_orbit_period_s_) / sphere_orbit_period_s_) +
                         sphere_orbit_phase_rad_;
    const double orbit_radius_m = sphere_orbit_radius_cm_ / 100.0;

    return Eigen::Vector3d(
        orbit_radius_m * std::cos(theta),
        orbit_radius_m * std::sin(theta),
        base_center_world_m.z());
  }

  std::pair<std::vector<int64_t>, std::vector<int64_t>> generateSphereScenario(std::size_t tick) const {
    if (sphere_center_world_cm_.size() != 3 || voxel_size_ <= 0.0) {
      return {occupied_voxels_, danger_voxels_};
    }

    const Eigen::Vector3d center_world_m = sphereCenterWorldMForTick(tick);
    const Eigen::Vector3i center_idx = GNG::Analysis::IndexVoxelGrid::getIndex(
        center_world_m.cast<float>(), static_cast<float>(voxel_size_));
    const double r_occ = std::max(0.0, sphere_radius_cm_ / 100.0);
    const double r_dan = std::max(r_occ, r_occ + std::max(0.0, sphere_danger_margin_cm_ / 100.0));
    const int r_max = static_cast<int>(std::ceil(r_dan / voxel_size_));

    std::vector<int64_t> occupied;
    std::vector<int64_t> danger;
    occupied.reserve(static_cast<std::size_t>((2 * r_max + 1) * (2 * r_max + 1) * (2 * r_max + 1)));
    danger.reserve(occupied.capacity());

    const double occ_sq = r_occ * r_occ;
    const double dan_sq = r_dan * r_dan;

    for (int dx = -r_max; dx <= r_max; ++dx) {
      for (int dy = -r_max; dy <= r_max; ++dy) {
        for (int dz = -r_max; dz <= r_max; ++dz) {
          const Eigen::Vector3i idx(center_idx.x() + dx,
                                     center_idx.y() + dy,
                                     center_idx.z() + dz);
          const Eigen::Vector3d voxel_center_world =
              ((idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size_);
          const double dist_sq = (voxel_center_world - center_world_m).squaredNorm();
          const int64_t flat = static_cast<int64_t>(GNG::Analysis::IndexVoxelGrid::getFlatVoxelId(idx));

          if (dist_sq <= occ_sq) {
            occupied.push_back(flat);
          } else if (dist_sq <= dan_sq) {
            danger.push_back(flat);
          }
        }
      }
    }

    return {occupied, danger};
  }

  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr danger_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<int64_t> occupied_voxels_;
  std::vector<int64_t> danger_voxels_;
    std::vector<double> sphere_center_world_cm_;
    std::string scenario_ = "static";
    double scenario_period_s_ = 1.0;
    double sphere_orbit_radius_cm_ = 0.0;
    double sphere_orbit_period_s_ = 0.0;
    double sphere_orbit_phase_rad_ = 0.0;
    double sphere_radius_cm_ = 10.0;
    double sphere_danger_margin_cm_ = 3.0;
    double voxel_size_ = 0.02;
    double publish_hz_ = 1.0;
    double publish_interval_s_ = 1.0;
    bool publish_once_ = false;
    std::size_t tick_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelStatusTestPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
