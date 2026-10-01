#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robot_sim::tasks {

struct goal_task_context {
  const std::vector<int> &requested_goals;
  const std::vector<int> &safe_goals;
  bool is_start_unsafe = false;
  bool allow_safe_goal_fallback = false;
};

struct goal_task_request {
  std::vector<int> goal_ids;
  std::string label;
  bool is_retreat = false;
};

// 目標選択のみを担当するタスク部品。実行状態と安全監視は所有しない境界
class goal_task {
public:
  virtual ~goal_task() = default;
  virtual std::optional<goal_task_request> select(
      const goal_task_context &context) const = 0;
};

class requested_goal_task final : public goal_task {
public:
  std::optional<goal_task_request> select(const goal_task_context &context) const override {
    if (context.requested_goals.empty()) return std::nullopt;
    return goal_task_request{context.requested_goals, "GoalPlanning"};
  }
};

class safe_retreat_task final : public goal_task {
public:
  std::optional<goal_task_request> select(const goal_task_context &context) const override {
    if (!context.requested_goals.empty() || !context.allow_safe_goal_fallback ||
        !context.is_start_unsafe || context.safe_goals.empty()) return std::nullopt;
    return goal_task_request{context.safe_goals, "Avoidance", true};
  }
};

class goal_task_pipeline {
public:
  explicit goal_task_pipeline(std::vector<std::unique_ptr<goal_task>> components)
      : components_(std::move(components)) {
    for (const auto &component : components_) {
      if (!component) throw std::invalid_argument("Null goal task component");
    }
  }

  std::optional<goal_task_request> select(const goal_task_context &context) const {
    for (const auto &component : components_) {
      auto request = component->select(context);
      if (request) return request;
    }
    return std::nullopt;
  }

private:
  std::vector<std::unique_ptr<goal_task>> components_;
};

inline goal_task_pipeline make_goal_task_pipeline(const std::vector<std::string> &names) {
  std::vector<std::unique_ptr<goal_task>> components;
  for (const auto &name : names) {
    if (name == "requested_goal") {
      components.push_back(std::make_unique<requested_goal_task>());
    } else if (name == "safe_retreat") {
      components.push_back(std::make_unique<safe_retreat_task>());
    } else {
      throw std::invalid_argument("Unknown goal task: " + name);
    }
  }
  return goal_task_pipeline(std::move(components));
}

}  // 名前空間robot_sim::tasksの終端
