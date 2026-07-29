#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gng_control_msgs/msg/evaluation_metrics.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric.hpp>
#include <gng_control_msgs/msg/grasp_candidate_metric_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_sim::common {

namespace detail {

struct MetricDefinition {
  std::string metric_id;
  std::string label;
  std::string unit;
  uint8_t value_type = gng_control_msgs::msg::EvaluationMetrics::VALUE_FLOAT;
  std::string scope_type;
  std::string group_name;
  std::string description;
  bool visible = true;
  bool enabled = true;
  float default_weight = 0.0f;
};

inline uint8_t valueTypeFromName(const std::string &name) {
  if (name == "int") {
    return gng_control_msgs::msg::EvaluationMetrics::VALUE_INT;
  }
  if (name == "bool") {
    return gng_control_msgs::msg::EvaluationMetrics::VALUE_BOOL;
  }
  if (name == "string") {
    return gng_control_msgs::msg::EvaluationMetrics::VALUE_STRING;
  }
  if (name == "float_array") {
    return gng_control_msgs::msg::EvaluationMetrics::VALUE_FLOAT_ARRAY;
  }
  return gng_control_msgs::msg::EvaluationMetrics::VALUE_FLOAT;
}

inline MetricDefinition makeMetricDefinition(
    const std::string &metric_id, const std::string &label,
    const std::string &value_type, const std::string &scope_type,
    const std::string &group_name, const std::string &description = "",
    const std::string &unit = "", bool visible = true, bool enabled = true,
    float default_weight = 0.0f) {
  return MetricDefinition{
      metric_id, label, unit, valueTypeFromName(value_type), scope_type,
      group_name, description, visible, enabled, default_weight};
}

inline void appendDefinition(gng_control_msgs::msg::EvaluationMetrics &out,
                             const MetricDefinition &def) {
  out.metric_ids.push_back(def.metric_id);
  out.labels.push_back(def.label);
  out.units.push_back(def.unit);
  out.value_types.push_back(def.value_type);
  out.metric_scope_types.push_back(def.scope_type);
  out.group_names.push_back(def.group_name);
  out.descriptions.push_back(def.description);
  out.visible.push_back(def.visible);
  out.enabled.push_back(def.enabled);
  out.default_weights.push_back(def.default_weight);
}

inline void appendFloatValue(gng_control_msgs::msg::EvaluationMetrics &out,
                             const std::string &metric_id, float value,
                             bool valid = true) {
  out.sample_metric_ids.push_back(metric_id);
  out.sample_metric_valid.push_back(valid);
  out.sample_metric_scalar_values.push_back(static_cast<double>(value));
  out.sample_metric_array_offsets.push_back(
      static_cast<uint32_t>(out.sample_metric_array_values.size()));
}

inline void appendFloatArrayValue(gng_control_msgs::msg::EvaluationMetrics &out,
                                  const std::string &metric_id,
                                  const std::vector<float> &values,
                                  bool valid = true) {
  out.sample_metric_ids.push_back(metric_id);
  out.sample_metric_valid.push_back(valid);
  out.sample_metric_scalar_values.push_back(0.0);
  out.sample_metric_array_offsets.push_back(
      static_cast<uint32_t>(out.sample_metric_array_values.size()));
  for (float value : values) {
    out.sample_metric_array_values.push_back(static_cast<double>(value));
  }
}

} // namespace detail

inline gng_control_msgs::msg::EvaluationMetrics buildCandidateEvaluationMetrics(
    const rclcpp::Time &stamp, const std::string &profile_name,
    const std::string &scope_type, const std::string &source_topic,
    const gng_control_msgs::msg::GraspCandidateMetricArray &candidates,
    const std::string &schema_id = "grasp_candidate_metrics",
    uint32_t schema_revision = 1) {
  gng_control_msgs::msg::EvaluationMetrics out;
  out.header.stamp = stamp;
  out.profile_name = profile_name;
  out.schema_id = schema_id;
  out.schema_revision = schema_revision;
  out.scope_type = scope_type;
  out.source_topic = source_topic;

  std::unordered_map<std::string, detail::MetricDefinition> defs;
  auto add_def = [&](const detail::MetricDefinition &def) {
    defs.emplace(def.metric_id, def);
  };

  add_def(detail::makeMetricDefinition("position_manipulability", "Position manipulability", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("rotation_manipulability", "Rotation manipulability", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("manipulability_condition_number", "Manipulability condition number", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("min_singular_value", "Min singular value", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("joint_limit_margin_min", "Joint limit margin min", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("joint_limit_margin_mean", "Joint limit margin mean", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("self_collision_margin", "Self collision margin", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("environment_collision_margin", "Environment collision margin", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("gripper_width", "Gripper width", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("grasp_region_score", "Grasp region score", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("estimated_energy", "Estimated energy", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("estimated_duration", "Estimated duration", "float", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("path_position_manipulability", "Path position manipulability", "float_array", scope_type, "candidate"));
  add_def(detail::makeMetricDefinition("path_rotation_manipulability", "Path rotation manipulability", "float_array", scope_type, "candidate"));

  for (const auto &candidate : candidates.candidates) {
    if (!candidate.feasible) {
      continue;
    }
    for (std::size_t i = 0; i < candidate.metric_names.size(); ++i) {
      const std::string &metric_id = candidate.metric_names[i];
      if (metric_id.empty() || metric_id == "selected" || defs.count(metric_id)) {
        continue;
      }
      add_def(detail::makeMetricDefinition(metric_id, metric_id, "float",
                                           scope_type, "candidate"));
    }
  }

  std::vector<detail::MetricDefinition> sorted_defs;
  sorted_defs.reserve(defs.size());
  for (const auto &[_, def] : defs) {
    sorted_defs.push_back(def);
  }
  std::sort(sorted_defs.begin(), sorted_defs.end(), [](const auto &a, const auto &b) {
    return a.metric_id < b.metric_id;
  });
  for (const auto &def : sorted_defs) {
    detail::appendDefinition(out, def);
  }

  out.sample_metric_offsets.reserve(candidates.candidates.size() + 1);
  out.sample_metric_offsets.push_back(0);
  for (const auto &candidate : candidates.candidates) {
    if (!candidate.feasible) {
      continue;
    }
    out.sample_scope_ids.push_back(std::to_string(candidate.goal_node_id));

    detail::appendFloatValue(out, "position_manipulability", candidate.position_manipulability, std::isfinite(candidate.position_manipulability));
    detail::appendFloatValue(out, "rotation_manipulability", candidate.rotation_manipulability, std::isfinite(candidate.rotation_manipulability));
    detail::appendFloatValue(out, "manipulability_condition_number", candidate.manipulability_condition_number, std::isfinite(candidate.manipulability_condition_number));
    detail::appendFloatValue(out, "min_singular_value", candidate.min_singular_value, std::isfinite(candidate.min_singular_value));
    detail::appendFloatValue(out, "joint_limit_margin_min", candidate.joint_limit_margin_min, std::isfinite(candidate.joint_limit_margin_min));
    detail::appendFloatValue(out, "joint_limit_margin_mean", candidate.joint_limit_margin_mean, std::isfinite(candidate.joint_limit_margin_mean));
    detail::appendFloatValue(out, "self_collision_margin", candidate.self_collision_margin, std::isfinite(candidate.self_collision_margin));
    detail::appendFloatValue(out, "environment_collision_margin", candidate.environment_collision_margin, std::isfinite(candidate.environment_collision_margin));
    detail::appendFloatValue(out, "gripper_width", candidate.gripper_width, std::isfinite(candidate.gripper_width));
    detail::appendFloatValue(out, "grasp_region_score", candidate.grasp_region_score, std::isfinite(candidate.grasp_region_score));
    detail::appendFloatValue(out, "estimated_energy", candidate.estimated_energy, std::isfinite(candidate.estimated_energy));
    detail::appendFloatValue(out, "estimated_duration", candidate.estimated_duration, std::isfinite(candidate.estimated_duration));
    detail::appendFloatArrayValue(out, "path_position_manipulability", candidate.path_position_manipulability);
    detail::appendFloatArrayValue(out, "path_rotation_manipulability", candidate.path_rotation_manipulability);

    for (std::size_t i = 0; i < candidate.metric_names.size(); ++i) {
      if (i >= candidate.metric_values.size()) {
        continue;
      }
      const std::string &metric_id = candidate.metric_names[i];
      if (metric_id.empty() || metric_id == "selected") {
        continue;
      }
      detail::appendFloatValue(out, metric_id, candidate.metric_values[i],
                               std::isfinite(candidate.metric_values[i]));
    }

    out.sample_metric_offsets.push_back(
        static_cast<uint32_t>(out.sample_metric_ids.size()));
  }
  out.sample_metric_array_offsets.push_back(
      static_cast<uint32_t>(out.sample_metric_array_values.size()));

  return out;
}

} // namespace robot_sim::common
