#include <gtest/gtest.h>

#include <limits>
#include "simulation/gazebo_stop_latch.hpp"

namespace {
using robot_sim::gazebo_stop_latch;

void confirm_stop(gazebo_stop_latch & latch) {
  latch.request_stop();
  latch.mark_stop_applied();
  latch.observe(1.0, 10.0, 0.0, true);
  latch.observe(1.125, 10.125, 0.0, true);
  latch.observe(1.25, 10.25, 0.0, true);
}

TEST(gazebo_stop_latch, request_is_not_measured_stop) {
  gazebo_stop_latch latch;
  EXPECT_EQ(latch.state(10.0), "running");
  latch.request_stop();
  latch.observe(1.0, 10.0, 0.0, true);
  latch.observe(1.5, 10.5, 0.0, true);
  EXPECT_TRUE(latch.is_stop_latched());
  EXPECT_FALSE(latch.is_stop_applied());
  EXPECT_FALSE(latch.is_stopped(10.5));
  EXPECT_EQ(latch.state(10.5), "stop_requested");
}

TEST(gazebo_stop_latch, confirmation_requires_continuous_sim_time) {
  gazebo_stop_latch latch;
  latch.request_stop();
  latch.mark_stop_applied();
  latch.observe(1.0, 10.0, 0.0, true);
  latch.observe(1.2, 10.2, 0.0, true);
  EXPECT_FALSE(latch.is_stopped(10.2));
  latch.observe(1.25, 10.25, 0.0, true);
  EXPECT_TRUE(latch.is_stopped(10.25));
  EXPECT_EQ(latch.state(10.25), "stopped");
}

TEST(gazebo_stop_latch, repeated_request_preserves_confirmed_hold) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  latch.request_stop();
  EXPECT_TRUE(latch.is_stop_applied());
  EXPECT_TRUE(latch.is_stopped(10.25));
}

TEST(gazebo_stop_latch, stale_or_repeated_physics_samples_are_not_stopped) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  EXPECT_FALSE(latch.is_stopped(10.751));
  EXPECT_EQ(latch.state(10.751), "stop_unconfirmed");
  latch.observe(1.25, 10.3, 0.0, true);
  EXPECT_FALSE(latch.is_stopped(10.3));
  latch.observe(1.5, 10.5, 0.0, true);
  EXPECT_FALSE(latch.is_stopped(10.5));
}

TEST(gazebo_stop_latch, wall_gap_and_sim_rewind_restart_confirmation) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  latch.observe(1.3, 11.0, 0.0, true);
  EXPECT_FALSE(latch.is_stopped(11.0));
  latch.observe(1.55, 11.25, 0.0, true);
  EXPECT_TRUE(latch.is_stopped(11.25));
  latch.observe(0.0, 11.3, 0.0, true);
  EXPECT_FALSE(latch.is_stopped(11.3));
}

TEST(gazebo_stop_latch, renewed_motion_and_invalid_state_clear_confirmation) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  latch.observe(1.3, 10.3, 0.010001, true);
  EXPECT_FALSE(latch.is_stopped(10.3));
  latch.observe(1.4, 10.4, 0.0, false);
  EXPECT_FALSE(latch.is_stopped(10.4));
  latch.observe(1.5, 10.5, std::numeric_limits<double>::quiet_NaN(), true);
  EXPECT_FALSE(latch.is_stopped(10.5));
}

TEST(gazebo_stop_latch, reset_requires_inactive_controller_and_fresh_stop) {
  gazebo_stop_latch latch;
  EXPECT_FALSE(latch.reset(0.0, false));
  confirm_stop(latch);
  EXPECT_FALSE(latch.reset(10.25, true));
  EXPECT_FALSE(latch.reset(11.0, false));
  EXPECT_TRUE(latch.is_stop_latched());
  latch.observe(1.3, 11.0, 0.0, true);
  latch.observe(1.55, 11.25, 0.0, true);
  EXPECT_TRUE(latch.reset(11.25, false));
  EXPECT_FALSE(latch.is_stop_latched());
  EXPECT_FALSE(latch.is_stop_applied());
  EXPECT_EQ(latch.state(11.25), "running");
}

TEST(gazebo_stop_latch, rejected_motor_write_removes_stop_confirmation) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  latch.mark_stop_unapplied();
  EXPECT_TRUE(latch.is_stop_latched());
  EXPECT_FALSE(latch.is_stop_applied());
  EXPECT_FALSE(latch.is_stopped(10.25));
  EXPECT_FALSE(latch.reset(10.25, false));
}

TEST(gazebo_stop_latch, activation_after_stop_race_prevents_reset_until_deactivation) {
  gazebo_stop_latch latch;
  confirm_stop(latch);
  // prepare通過後のstopと、perform失敗後にも残るactivationの再現
  bool is_active = robot_sim::is_command_active_after_switch(false, true, false);
  EXPECT_TRUE(is_active);
  EXPECT_FALSE(latch.reset(10.25, is_active));
  // 同時stop/startの最終状態もactive扱い
  is_active = robot_sim::is_command_active_after_switch(is_active, true, true);
  EXPECT_TRUE(is_active);
  EXPECT_FALSE(latch.reset(10.25, is_active));
  is_active = robot_sim::is_command_active_after_switch(is_active, false, true);
  EXPECT_FALSE(is_active);
  EXPECT_TRUE(latch.reset(10.25, is_active));
}
}  // 無名名前空間
