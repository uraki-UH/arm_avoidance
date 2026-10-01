// ODEの有限トルクモータによる位置追従。速度・トルク上限の正本はURDF
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gazebo_stop_latch.hpp"

namespace robot_sim {
class bounded_gazebo_system : public gazebo_ros2_control::GazeboSystemInterface {
  struct joint_data {
    gazebo::physics::JointPtr joint;
    std::string name;
    double position = 0, velocity = 0, effort = 0, command = 0;
    double max_effort = 0, max_velocity = 0, min_position = 0, max_position = 0;
    double position_gain = 20, multiplier = 1;
    double hold_position = 0;
    bool is_command_active = false;
    bool is_prismatic = false;
    bool has_pending_hold_capture = false;
    int mimic_idx = -1;
  };
  std::vector<joint_data> joints_;
  std::mutex stop_mutex_;
  gazebo_stop_latch stop_latch_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_latch_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr stop_status_timer_;

  static double wall_sec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool has_active_commands() const {
    return std::any_of(joints_.begin(), joints_.end(), [](const auto & data) {
      return data.mimic_idx < 0 && data.is_command_active;
    });
  }

  bool has_start_commands(const std::vector<std::string> & start_interfaces) const {
    return std::any_of(joints_.begin(), joints_.end(), [&](const auto & data) {
      return data.mimic_idx < 0 && std::find(start_interfaces.begin(), start_interfaces.end(),
        data.name + "/position") != start_interfaces.end();
    });
  }

  void publish_stop_status() {
    std_msgs::msg::Bool latch_message;
    std_msgs::msg::String status_message;
    std::string state;
    bool is_stop_applied, is_stopped, has_active;
    double age_sec, max_velocity_rad_sec, max_linear_velocity_m_sec;
    {
      std::lock_guard<std::mutex> lock(stop_mutex_);
      const double now_wall_sec = wall_sec();
      age_sec = stop_latch_.state_age_sec(now_wall_sec);
      max_velocity_rad_sec = stop_latch_.max_velocity_rad_sec();
      max_linear_velocity_m_sec = stop_latch_.max_linear_velocity_m_sec();
      latch_message.data = stop_latch_.is_stop_latched();
      state = stop_latch_.state(now_wall_sec);
      is_stop_applied = stop_latch_.is_stop_applied();
      is_stopped = stop_latch_.is_stopped(now_wall_sec);
      has_active = has_active_commands();
    }
    // 物理更新の排他区間外での診断文字列生成
    nlohmann::json status = {
      {"state", state}, {"is_stop_latched", latch_message.data},
      {"is_stop_applied", is_stop_applied}, {"is_stopped", is_stopped},
      {"has_active_commands", has_active},
      {"max_velocity_rad_sec", std::isfinite(max_velocity_rad_sec)
        ? nlohmann::json(max_velocity_rad_sec) : nlohmann::json(nullptr)},
      {"max_linear_velocity_m_sec", std::isfinite(max_linear_velocity_m_sec)
        ? nlohmann::json(max_linear_velocity_m_sec) : nlohmann::json(nullptr)},
      {"state_age_sec", std::isfinite(age_sec) ? nlohmann::json(age_sec) : nlohmann::json(nullptr)}
    };
    status_message.data = status.dump();
    stop_latch_pub_->publish(latch_message);
    stop_status_pub_->publish(status_message);
  }

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
      data.is_prismatic = data.joint->HasType(gazebo::physics::Base::SLIDER_JOINT);
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
      // モデル生成時の一時停止中に限る初期姿勢の設定。駆動開始後の位置更新は有限トルクモータのみ
      for (const auto & state : item.state_interfaces) {
        if (state.name != "position" || state.initial_value.empty()) continue;
        const double initial_position = std::stod(state.initial_value);
        if (!model->GetWorld()->IsPaused() || !std::isfinite(initial_position) ||
            initial_position < data.min_position || initial_position > data.max_position) {
          RCLCPP_ERROR(nh_->get_logger(), "関節%sの初期姿勢または物理停止状態が不正です", data.name.c_str());
          return false;
        }
        if (!data.joint->SetPosition(0, initial_position)) return false;
      }
      data.position = data.command = data.hold_position = data.joint->Position(0);
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
    const auto stop_qos = rclcpp::QoS(1).reliable().transient_local();
    stop_latch_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("safety/is_stop_latched", stop_qos);
    stop_status_pub_ = nh_->create_publisher<std_msgs::msg::String>("safety/status", stop_qos);
    stop_service_ = nh_->create_service<std_srvs::srv::Trigger>("safety/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        {
          std::lock_guard<std::mutex> lock(stop_mutex_);
          stop_latch_.request_stop();
        }
        response->success = true;
        response->message = "停止要求の受付。実測確認はsafety/status";
        publish_stop_status();
      });
    reset_service_ = nh_->create_service<std_srvs::srv::Trigger>("safety/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        {
          std::lock_guard<std::mutex> lock(stop_mutex_);
          response->success = stop_latch_.reset(wall_sec(), has_active_commands());
        }
        response->message = response->success
          ? "停止ラッチの解除。再開にはcontrollerの明示的なactivateが必要"
          : "解除拒否。全位置controllerのdeactivateと新鮮な停止実測が必要";
        publish_stop_status();
      });
    stop_status_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(50),
      [this]() { publish_stop_status(); });
    publish_stop_status();
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

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> &) override {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    if (stop_latch_.is_stop_latched() && has_start_commands(start_interfaces)) {
      RCLCPP_WARN(nh_->get_logger(), "停止ラッチ中のcontroller activateを拒否");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    const bool has_rejected_start = stop_latch_.is_stop_latched() && has_start_commands(start_interfaces);
    for (auto & data : joints_) {
      if (data.mimic_idx >= 0) continue;
      const std::string command_name = data.name + "/position";
      const bool has_stop_interface =
        std::find(stop_interfaces.begin(), stop_interfaces.end(), command_name) != stop_interfaces.end();
      const bool has_start_interface =
        std::find(start_interfaces.begin(), start_interfaces.end(), command_name) != start_interfaces.end();
      // prepare後の停止競合でも、manager側のactivation継続を想定した解除禁止
      data.is_command_active = is_command_active_after_switch(
        data.is_command_active, has_start_interface, has_stop_interface);
      if (has_stop_interface) {
        data.has_pending_hold_capture = true;
        for (auto & child : joints_) {
          if (child.mimic_idx >= 0 && joints_[child.mimic_idx].name == data.name)
            child.has_pending_hold_capture = true;
        }
      }
      if (has_start_interface && !has_rejected_start) {
        // 再activation時の旧目標破棄。実測姿勢からの明示再開
        data.command = data.hold_position = data.position;
        data.has_pending_hold_capture = false;
      }
    }
    return has_rejected_start ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration &) override {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    double max_velocity_rad_sec = 0;
    double max_linear_velocity_m_sec = 0;
    bool has_finite_state = !joints_.empty();
    for (auto & data : joints_) {
      data.position = data.joint->Position(0);
      data.velocity = data.joint->GetVelocity(0);
      // 子リンク座標系の関節反力の軸方向成分。直動は力 [N]、回転はトルク [N m]
      const auto axis = data.joint->GetChild()->WorldPose().Rot().RotateVectorReverse(data.joint->GlobalAxis(0));
      const auto reaction = data.joint->GetForceTorque(0);
      data.effort = -(data.is_prismatic ? reaction.body2Force : reaction.body2Torque).Dot(axis);
      has_finite_state = has_finite_state && std::isfinite(data.position) && std::isfinite(data.velocity);
      if (data.is_prismatic) {
        max_linear_velocity_m_sec = std::max(max_linear_velocity_m_sec, std::abs(data.velocity));
      } else {
        max_velocity_rad_sec = std::max(max_velocity_rad_sec, std::abs(data.velocity));
      }
    }
    stop_latch_.observe(time.seconds(), wall_sec(), max_velocity_rad_sec, has_finite_state,
                       max_linear_velocity_m_sec);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    bool can_apply_stop = false;
    if (stop_latch_.is_stop_latched() && !stop_latch_.is_stop_applied()) {
      // 全関節の停止位置取得。ROS callbackからのGazebo API操作なし
      std::vector<double> stop_positions;
      bool has_finite_positions = true;
      for (const auto & data : joints_) {
        stop_positions.push_back(data.joint->Position(0));
        has_finite_positions = has_finite_positions && std::isfinite(stop_positions.back());
      }
      if (has_finite_positions) {
        for (std::size_t idx = 0; idx < joints_.size(); ++idx)
          joints_[idx].hold_position = stop_positions[idx];
        can_apply_stop = true;
      }
    }
    bool has_output_success = true;
    for (auto & data : joints_) {
      const bool is_active = data.mimic_idx < 0 ? data.is_command_active :
        joints_[data.mimic_idx].is_command_active;
      if (!stop_latch_.is_stop_latched() && !is_active && data.has_pending_hold_capture) {
        data.hold_position = data.joint->Position(0);
        data.has_pending_hold_capture = false;
      }
      // ラッチ中・controller非active中の指令遮断。mimic関節も個別停止位置で保持
      double target = data.hold_position;
      if (!stop_latch_.is_stop_latched() && is_active) {
        target = data.mimic_idx < 0 ? data.command : joints_[data.mimic_idx].command * data.multiplier;
      } else if (stop_latch_.is_stop_latched() && !stop_latch_.is_stop_applied() && !can_apply_stop) {
        target = std::numeric_limits<double>::quiet_NaN();
      }
      double velocity = 0;
      const double current_position = data.joint->Position(0);
      if (std::isfinite(target) && std::isfinite(current_position)) {
        target = std::clamp(target, data.min_position, data.max_position);
        velocity = std::clamp(data.position_gain * (target - current_position),
                              -data.max_velocity, data.max_velocity);
      }
      const bool has_effort_output = data.joint->SetParam("fmax", 0, data.max_effort);
      const bool has_velocity_output = data.joint->SetParam("vel", 0, velocity);
      has_output_success = has_output_success && has_effort_output && has_velocity_output;
    }
    if (!has_output_success) {
      stop_latch_.mark_stop_unapplied();
      return hardware_interface::return_type::ERROR;
    }
    if (can_apply_stop) stop_latch_.mark_stop_applied();
    return hardware_interface::return_type::OK;
  }
};
}  // robot_sim名前空間
PLUGINLIB_EXPORT_CLASS(robot_sim::bounded_gazebo_system, gazebo_ros2_control::GazeboSystemInterface)
