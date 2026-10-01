#pragma once

#include <cmath>
#include <limits>
#include <string>

namespace robot_sim {

// perform失敗後もactivationを継続するcontroller_managerへの保守的な指令権追跡
constexpr bool is_command_active_after_switch(
    bool is_active, bool has_start_interface, bool has_stop_interface) {
  return has_start_interface || (is_active && !has_stop_interface);
}

// Gazebo専用の停止要求・実測確認。外側のmutexによる呼出し直列化
class gazebo_stop_latch {
public:
  static constexpr double max_stop_velocity_th = 0.01;
  // 直動関節の停止判定速度 [m/s]
  static constexpr double max_stop_linear_velocity_th = 0.001;
  static constexpr double min_stop_confirm_sec = 0.25;
  static constexpr double max_state_age_sec = 0.5;

  void request_stop() {
    if (is_stop_latched_) return;
    is_stop_latched_ = true;
    is_stop_applied_ = false;
    low_velocity_since_sim_sec_ = invalid_time();
  }

  void mark_stop_applied() {
    if (!is_stop_latched_ || is_stop_applied_) return;
    is_stop_applied_ = true;
    low_velocity_since_sim_sec_ = invalid_time();
  }

  void mark_stop_unapplied() {
    is_stop_applied_ = false;
    low_velocity_since_sim_sec_ = invalid_time();
  }

  void observe(double sim_sec, double wall_sec, double max_velocity_rad_sec,
               bool has_finite_state, double max_linear_velocity_m_sec = 0.0) {
    const bool has_fresh_previous = std::isfinite(last_read_wall_sec_) &&
      wall_sec >= last_read_wall_sec_ && wall_sec - last_read_wall_sec_ <= max_state_age_sec;
    const bool has_advanced = !std::isfinite(last_read_sim_sec_) || sim_sec > last_read_sim_sec_;
    has_finite_state_ = has_finite_state && std::isfinite(sim_sec) && std::isfinite(wall_sec) &&
      std::isfinite(max_velocity_rad_sec) && max_velocity_rad_sec >= 0 && has_advanced &&
      std::isfinite(max_linear_velocity_m_sec) && max_linear_velocity_m_sec >= 0;
    max_velocity_rad_sec_ = has_finite_state_ ? max_velocity_rad_sec : invalid_time();
    max_linear_velocity_m_sec_ = has_finite_state_ ? max_linear_velocity_m_sec : invalid_time();
    const bool is_low_velocity = max_velocity_rad_sec <= max_stop_velocity_th &&
      max_linear_velocity_m_sec <= max_stop_linear_velocity_th;
    if (!has_fresh_previous || !has_finite_state_ || !is_stop_applied_ ||
        !is_low_velocity) {
      low_velocity_since_sim_sec_ = invalid_time();
    }
    if (is_stop_applied_ && has_finite_state_ && is_low_velocity &&
        !std::isfinite(low_velocity_since_sim_sec_)) {
      low_velocity_since_sim_sec_ = sim_sec;
    }
    last_read_sim_sec_ = sim_sec;
    last_read_wall_sec_ = wall_sec;
  }

  double state_age_sec(double wall_sec) const {
    return std::isfinite(last_read_wall_sec_) && wall_sec >= last_read_wall_sec_
      ? wall_sec - last_read_wall_sec_ : std::numeric_limits<double>::infinity();
  }

  bool is_stopped(double wall_sec) const {
    return is_stop_latched_ && is_stop_applied_ && has_finite_state_ &&
      state_age_sec(wall_sec) <= max_state_age_sec && std::isfinite(low_velocity_since_sim_sec_) &&
      last_read_sim_sec_ - low_velocity_since_sim_sec_ >= min_stop_confirm_sec;
  }

  bool reset(double wall_sec, bool has_active_commands) {
    if (has_active_commands || !is_stopped(wall_sec)) return false;
    is_stop_latched_ = false;
    is_stop_applied_ = false;
    low_velocity_since_sim_sec_ = invalid_time();
    return true;
  }

  std::string state(double wall_sec) const {
    if (!is_stop_latched_) return "running";
    if (!is_stop_applied_) return "stop_requested";
    return is_stopped(wall_sec) ? "stopped" : "stop_unconfirmed";
  }

  bool is_stop_latched() const { return is_stop_latched_; }
  bool is_stop_applied() const { return is_stop_applied_; }
  double max_velocity_rad_sec() const { return max_velocity_rad_sec_; }
  double max_linear_velocity_m_sec() const { return max_linear_velocity_m_sec_; }

private:
  static double invalid_time() { return std::numeric_limits<double>::quiet_NaN(); }
  bool is_stop_latched_ = false;
  bool is_stop_applied_ = false;
  bool has_finite_state_ = false;
  double last_read_wall_sec_ = invalid_time();
  double last_read_sim_sec_ = invalid_time();
  double low_velocity_since_sim_sec_ = invalid_time();
  double max_velocity_rad_sec_ = invalid_time();
  double max_linear_velocity_m_sec_ = invalid_time();
};

}  // robot_sim名前空間
