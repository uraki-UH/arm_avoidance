#include <gtest/gtest.h>
#include <limits>
#include "core/control/joint_command_mux.hpp"

using robot_sim::control::joint_command_mux;

TEST(joint_command_mux, partial_updates_preserve_other_joints) {
  joint_command_mux mux({"arm", "gripper"});
  ASSERT_TRUE(mux.set_claim("manual", {{}, 50, true, true, 0.0}));
  ASSERT_TRUE(mux.set_command("manual", {"arm", "gripper"}, {1.2, 0.1}, 0.0));
  ASSERT_TRUE(mux.set_command("manual", {"gripper"}, {0.3}, 1.0));
  const auto result = mux.resolve(2.0);
  EXPECT_DOUBLE_EQ(result.active.at("arm"), 1.2);
  EXPECT_DOUBLE_EQ(result.active.at("gripper"), 0.3);
}

TEST(joint_command_mux, gripper_override_does_not_claim_arm) {
  joint_command_mux mux({"arm", "gripper"});
  mux.set_claim("leader", {{}, 100, true, true, 0.5});
  mux.set_claim("grip", {{"gripper"}, 200, true, true, 0.0});
  mux.set_command("leader", {"arm", "gripper"}, {1.2, 0.1}, 0.0);
  mux.set_command("grip", {"gripper"}, {0.3}, 0.0);
  const auto result = mux.resolve(0.2);
  EXPECT_DOUBLE_EQ(result.active.at("arm"), 1.2);
  EXPECT_DOUBLE_EQ(result.active.at("gripper"), 0.3);
  EXPECT_FALSE(mux.set_command("grip", {"arm"}, {2.0}, 0.2));
}

TEST(joint_command_mux, expiry_is_per_joint_and_latched_goal_is_separate) {
  joint_command_mux mux;
  mux.set_claim("stream", {{}, 100, true, true, 0.5});
  mux.set_claim("grip", {{"gripper"}, 200, true, true, 0.0});
  mux.set_command("stream", {"arm", "neck"}, {1.0, 0.1}, 0.0);
  mux.set_command("grip", {"gripper"}, {0.3}, 0.0);
  mux.resolve(0.1);
  mux.set_command("stream", {"neck"}, {0.2}, 0.4);
  const auto result = mux.resolve(0.6);
  EXPECT_EQ(result.active.count("arm"), 0u);
  EXPECT_DOUBLE_EQ(result.held.at("arm"), 1.0);
  EXPECT_DOUBLE_EQ(result.active.at("neck"), 0.2);
  EXPECT_DOUBLE_EQ(result.active.at("gripper"), 0.3);
}

TEST(joint_command_mux, malformed_messages_are_atomic_and_never_add_zero_joints) {
  joint_command_mux mux({"arm", "gripper"});
  mux.set_claim("source", {});
  mux.set_command("source", {"arm"}, {1.0}, 0.0);
  EXPECT_FALSE(mux.set_command("source", {"arm", "gripper"}, {2.0}, 1.0));
  EXPECT_FALSE(mux.set_command("source", {"arm", "arm"}, {2.0, 3.0}, 1.0));
  EXPECT_FALSE(mux.set_command("source", {"unknown"}, {0.0}, 1.0));
  EXPECT_FALSE(mux.set_command("source", {"arm", "gripper"},
                               {2.0, std::numeric_limits<double>::quiet_NaN()}, 1.0));
  const auto result = mux.resolve(1.1);
  EXPECT_EQ(result.held.size(), 1u);
  EXPECT_DOUBLE_EQ(result.active.at("arm"), 1.0);
}

TEST(joint_command_mux, disabled_source_cannot_replay_old_goal) {
  joint_command_mux mux;
  joint_command_mux::claim settings;
  mux.set_claim("source", settings);
  mux.set_command("source", {"arm"}, {1.0}, 0.0);
  mux.resolve(0.1);
  settings.enable_source = false;
  mux.set_claim("source", settings);
  EXPECT_TRUE(mux.resolve(0.2).active.empty());
  settings.enable_source = true;
  mux.set_claim("source", settings);
  EXPECT_TRUE(mux.resolve(0.3).active.empty());
  mux.set_command("source", {"arm"}, {2.0}, 0.4);
  EXPECT_DOUBLE_EQ(mux.resolve(0.4).active.at("arm"), 2.0);
}

TEST(joint_command_mux, parent_and_mimic_share_one_owner) {
  joint_command_mux mux({"grip", "mimic"});
  ASSERT_TRUE(mux.set_alias("mimic", {"grip", -1.0, 0.0}));
  mux.set_claim("leader", {{"grip", "mimic"}, 100, true, true, 0.5});
  mux.set_claim("grip_source", {{"grip"}, 200, true, true, 0.0});
  ASSERT_TRUE(mux.set_command("leader", {"grip", "mimic"}, {0.1, -0.1}, 0.0));
  ASSERT_TRUE(mux.set_command("grip_source", {"grip"}, {0.4}, 0.0));
  const auto result = mux.resolve(0.1);
  ASSERT_EQ(result.active.size(), 1u);
  EXPECT_DOUBLE_EQ(result.active.at("grip"), 0.4);
  EXPECT_FALSE(mux.set_command("leader", {"grip", "mimic"}, {0.1, -0.2}, 0.1));
}

TEST(joint_command_mux, scope_reduction_removes_previous_ownership) {
  joint_command_mux mux;
  mux.set_claim("source", {});
  mux.set_command("source", {"arm", "grip"}, {1.0, 0.4}, 0.0);
  mux.set_claim("source", {{"grip"}, 0, true, true, 0.0});
  const auto result = mux.resolve(0.1, false);
  EXPECT_EQ(result.active.count("arm"), 0u);
  EXPECT_EQ(result.held.count("arm"), 0u);
}

TEST(joint_command_mux, deterministic_ties_and_legacy_exclusive_precedence) {
  joint_command_mux mux;
  mux.set_claim("b", {{}, 10, true, true, 0.0});
  mux.set_claim("a", {{}, 10, true, true, 0.0});
  mux.set_claim("shared", {{}, 1000, false, true, 0.0});
  mux.set_command("b", {"arm"}, {2.0}, 0.0);
  mux.set_command("a", {"arm"}, {1.0}, 0.0);
  mux.set_command("shared", {"arm"}, {3.0}, 0.0);
  EXPECT_DOUBLE_EQ(mux.resolve(0.1).active.at("arm"), 1.0);
}

TEST(joint_command_mux, release_keeps_source_ready_for_next_goal) {
  joint_command_mux mux;
  mux.set_claim("grip", {});
  mux.set_command("grip", {"gripper"}, {0.2}, 0.0);
  ASSERT_TRUE(mux.clear_command("grip"));
  EXPECT_TRUE(mux.resolve(0.1).active.empty());
  ASSERT_TRUE(mux.set_command("grip", {"gripper"}, {0.4}, 0.2));
  EXPECT_DOUBLE_EQ(mux.resolve(0.2).active.at("gripper"), 0.4);
}
