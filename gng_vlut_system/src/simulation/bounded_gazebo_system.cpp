// ODEの有限トルクモータによる位置追従。速度・トルク上限の正本はURDF
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robot_sim {
class bounded_gazebo_system : public gazebo_ros2_control::GazeboSystemInterface {
  struct joint_data {
    gazebo::physics::JointPtr joint;
    std::string name;
    double position = 0, velocity = 0, effort = 0, command = 0;
    double max_effort = 0, max_velocity = 0, min_position = 0, max_position = 0;
    double position_gain = 20, multiplier = 1;
    int mimic_idx = -1;
  };
  std::vector<joint_data> joints_;

public:
  bool initSim(rclcpp::Node::SharedPtr & node, gazebo::physics::ModelPtr model,
               const hardware_interface::HardwareInfo & info, sdf::ElementPtr) override {
    nh_ = node;
    if (model->GetWorld()->Physics()->GetType() != "ode") {
      RCLCPP_ERROR(nh_->get_logger(), "有限トルクモータにはODEが必要です");
      return false;
    }
    joints_.reserve(info.joints.size());
    for (const auto & item : info.joints) {
      joint_data data;
      data.name = item.name;
      data.joint = model->GetJoint(item.name);
      if (!data.joint || data.joint->DOF() != 1) return false;
      const double motor_limit_scale = std::stod(item.parameters.at("motor_limit_scale"));
      if (!std::isfinite(motor_limit_scale) || motor_limit_scale <= 0 || motor_limit_scale > 1) return false;
      // 数値積分と反力測定のずれに対するURDF上限内の駆動余裕
      data.max_effort = data.joint->GetEffortLimit(0) * motor_limit_scale;
      data.max_velocity = data.joint->GetVelocityLimit(0) * motor_limit_scale;
      data.min_position = data.joint->LowerLimit(0);
      data.max_position = data.joint->UpperLimit(0);
      if (item.parameters.count("position_gain"))
        data.position_gain = std::stod(item.parameters.at("position_gain"));
      if (!(std::isfinite(data.max_effort) && data.max_effort > 0 &&
            std::isfinite(data.max_velocity) && data.max_velocity > 0 &&
            std::isfinite(data.position_gain) && data.position_gain > 0)) {
        RCLCPP_ERROR(nh_->get_logger(), "関節%sの上限・ゲインが不正です", data.name.c_str());
        return false;
      }
      // 一時停止中の生成姿勢を初期目標とする、位置・速度の直接設定なし
      data.position = data.command = data.joint->Position(0);
      data.joint->SetProvideFeedback(true);
      // 関節ストッパ離脱時の過大なモータ力を防ぐODE係数
      data.joint->SetParam("fudge_factor", 0, 0.0);
      // 可動端の重複拘束に対する微小コンプライアンス
      data.joint->SetParam("cfm", 0, 1e-8);
      data.joint->SetParam("stop_cfm", 0, 1e-8);
      // 位置・速度の直接設定を使わない、物理ソルバー内の有限トルク拘束
      if (!data.joint->SetParam("fmax", 0, data.max_effort) ||
          !data.joint->SetParam("vel", 0, 0.0)) return false;
      joints_.push_back(data);
    }
    for (std::size_t idx = 0; idx < info.joints.size(); ++idx) {
      const auto & params = info.joints[idx].parameters;
      if (!params.count("mimic")) continue;
      auto parent = std::find_if(joints_.begin(), joints_.end(), [&](const auto & item) {
        return item.name == params.at("mimic");
      });
      if (parent == joints_.end() || parent->name == joints_[idx].name) return false;
      joints_[idx].mimic_idx = std::distance(joints_.begin(), parent);
      if (params.count("multiplier")) joints_[idx].multiplier = std::stod(params.at("multiplier"));
    }
    RCLCPP_INFO(nh_->get_logger(), "ODE有限トルクモータ: %zu関節、URDF速度・トルク上限", joints_.size());
    return true;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (auto & data : joints_) {
      interfaces.emplace_back(data.name, "position", &data.position);
      interfaces.emplace_back(data.name, "velocity", &data.velocity);
      interfaces.emplace_back(data.name, "effort", &data.effort);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (auto & data : joints_)
      if (data.mimic_idx < 0) interfaces.emplace_back(data.name, "position", &data.command);
    return interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    for (auto & data : joints_) {
      data.position = data.joint->Position(0);
      data.velocity = data.joint->GetVelocity(0);
      // 子リンク座標系の関節反力トルクを回転軸へ射影。ストッパ反力を含む測定値
      const auto axis = data.joint->GetChild()->WorldPose().Rot().RotateVectorReverse(data.joint->GlobalAxis(0));
      data.effort = -data.joint->GetForceTorque(0).body2Torque.Dot(axis);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    for (auto & data : joints_) {
      double target = data.mimic_idx < 0 ? data.command :
        joints_[data.mimic_idx].command * data.multiplier;
      double velocity = 0;
      if (std::isfinite(target)) {
        target = std::clamp(target, data.min_position, data.max_position);
        velocity = std::clamp(data.position_gain * (target - data.joint->Position(0)),
                              -data.max_velocity, data.max_velocity);
      }
      data.joint->SetParam("fmax", 0, data.max_effort);
      data.joint->SetParam("vel", 0, velocity);
    }
    return hardware_interface::return_type::OK;
  }
};
}  // robot_sim名前空間
PLUGINLIB_EXPORT_CLASS(robot_sim::bounded_gazebo_system, gazebo_ros2_control::GazeboSystemInterface)
