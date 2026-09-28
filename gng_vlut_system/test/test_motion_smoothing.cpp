#include <gtest/gtest.h>
#include "control/motion_smoothing.hpp"

// 関節変位の比率を保持した速度制限と停止姿勢の確認
TEST(motion_smoothing, velocity_bound_and_proportions) {
  Eigen::VectorXf delta(3);
  delta << 1.0f, -0.5f, 0.0f;
  const float scale = motion_smoothing::calculate_velocity_scale(delta, 0.6f, 0.02f);
  const Eigen::VectorXf command_delta = delta * scale;
  EXPECT_NEAR(command_delta[0], 0.012f, 1e-7f);
  EXPECT_NEAR(command_delta[1], -0.006f, 1e-7f);
  EXPECT_FLOAT_EQ(command_delta[2], 0.0f);
  EXPECT_FLOAT_EQ(motion_smoothing::calculate_velocity_scale(delta, 100.0f, 1.0f), 1.0f);
  delta.setZero();
  EXPECT_FLOAT_EQ(motion_smoothing::calculate_velocity_scale(delta, 0.6f, 0.02f), 1.0f);
}
