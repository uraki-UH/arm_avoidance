#include <gtest/gtest.h>
#include "core/tasks/goal_task.hpp"

namespace {
using namespace robot_sim::tasks;

TEST(goal_tasks, default_selection_preserves_fallback_conditions) {
  auto pipeline = make_goal_task_pipeline({"requested_goal", "safe_retreat"});
  const std::vector<int> requested{3, 4}, safe{5}, empty;
  auto request = pipeline.select({requested, safe, true, true});
  ASSERT_TRUE(request);
  EXPECT_EQ(request->goal_ids, requested);
  EXPECT_EQ(request->label, "GoalPlanning");
  request = pipeline.select({empty, safe, true, true});
  ASSERT_TRUE(request);
  EXPECT_EQ(request->goal_ids, safe);
  EXPECT_FALSE(pipeline.select({empty, safe, false, true}));
  EXPECT_FALSE(pipeline.select({empty, safe, true, false}));
  EXPECT_FALSE(pipeline.select({empty, empty, true, true}));
  auto requested_only = make_goal_task_pipeline({"requested_goal"});
  EXPECT_FALSE(requested_only.select({empty, safe, true, true}));
  EXPECT_THROW(make_goal_task_pipeline({"unknown"}), std::invalid_argument);
}

class custom_goal_task final : public goal_task {
public:
  std::optional<goal_task_request> select(const goal_task_context &) const override {
    return goal_task_request{{42}, "custom"};
  }
};

TEST(goal_tasks, custom_component_substitution_and_order) {
  std::vector<std::unique_ptr<goal_task>> components;
  components.push_back(std::make_unique<requested_goal_task>());
  components.push_back(std::make_unique<custom_goal_task>());
  goal_task_pipeline pipeline(std::move(components));
  const std::vector<int> requested{3}, empty;
  EXPECT_EQ(pipeline.select({requested, empty})->goal_ids, requested);
  EXPECT_EQ(pipeline.select({empty, empty})->goal_ids, std::vector<int>({42}));
  EXPECT_FALSE(make_goal_task_pipeline({}).select({requested, empty}));
}
}  // 無名名前空間の終端
