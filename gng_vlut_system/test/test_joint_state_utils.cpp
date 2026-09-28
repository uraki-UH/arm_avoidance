#include <gtest/gtest.h>
#include <limits>
#include "core/control/joint_state_utils.hpp"

// 配列長不一致・重複名に対する既存の解釈の維持
TEST(joint_state_utils, duplicate_and_missing_positions) {
  sensor_msgs::msg::JointState message;
  message.name = {"left", "right", "left", "missing"};
  message.position = {1.0, 2.0, 3.0};
  const auto positions = joint_state_utils::build_position_map(message);
  ASSERT_EQ(positions.size(), 2U);
  EXPECT_DOUBLE_EQ(positions.at("left"), 3.0);
  EXPECT_DOUBLE_EQ(positions.at("right"), 2.0);
  EXPECT_EQ(positions.count("missing"), 0U);
}

// ノードごとの関節順序を保持した部分指令と初期値補完
TEST(joint_state_utils, requested_order_and_partial_target) {
  sensor_msgs::msg::JointState state, target;
  state.name = {"right", "left"};
  state.position = {0.25, 0.5};
  target.name = {"left", "new"};
  target.position = {-0.75, 1.0};
  const auto positions = joint_state_utils::positions_in_order(
      state, target, {"new", "right", "left", "unknown"});
  ASSERT_EQ(positions.current.size(), 4);
  EXPECT_FLOAT_EQ(positions.current[0], 0.0f);
  EXPECT_FLOAT_EQ(positions.target[0], 1.0f);
  EXPECT_FLOAT_EQ(positions.current[1], 0.25f);
  EXPECT_FLOAT_EQ(positions.target[1], 0.25f);
  EXPECT_FLOAT_EQ(positions.current[2], 0.5f);
  EXPECT_FLOAT_EQ(positions.target[2], -0.75f);
  EXPECT_FLOAT_EQ(positions.target[3], 0.0f);
}

// 空入力と非有限値に対する既存契約。入力検証の追加は対象外
TEST(joint_state_utils, empty_and_nonfinite_values) {
  sensor_msgs::msg::JointState message;
  EXPECT_TRUE(joint_state_utils::build_position_map(message).empty());
  EXPECT_EQ(joint_state_utils::positions_in_order(message, message, {}).current.size(), 0);
  message.name = {"joint"};
  message.position = {std::numeric_limits<double>::quiet_NaN()};
  const auto positions = joint_state_utils::positions_in_order(message, message, {"joint"});
  EXPECT_TRUE(std::isnan(positions.current[0]));
  EXPECT_TRUE(std::isnan(positions.target[0]));
}
