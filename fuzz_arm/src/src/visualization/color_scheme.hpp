#pragma once

#include "visualization/color.hpp"
#include "visualization/node_attributes.hpp"
#include <algorithm>
#include <vector>

namespace robot_sim {
namespace visualization {

/**
 * @brief Color assignment rule based on node attributes
 */
struct ColorRule {
  NodeAttribute required_attributes;  ///< Attributes that must be present
  NodeAttribute forbidden_attributes; ///< Attributes that must NOT be present
  Color color;                        ///< Color to assign
  int priority; ///< Higher priority rules are checked first

  ColorRule(NodeAttribute required, NodeAttribute forbidden, const Color &c,
            int prio)
      : required_attributes(required), forbidden_attributes(forbidden),
        color(c), priority(prio) {}
};

/**
 * @brief Parameters for dynamic color determination
 */
struct DynamicColorParams {
  NodeAttribute attributes = NodeAttribute::NONE;
  float danger_level = 0.0f;
  float danger_velocity = 0.0f;
  bool show_trends = true;
  bool is_colliding = false;
  bool is_target = false;
  bool is_danger_status = false;
  bool is_topology_mode = false;
};

class ColorScheme {
public:
  ColorScheme() { initializeDefaultRules(); }

  void addRule(const ColorRule &rule) {
    rules_.push_back(rule);
    sortRules();
  }

  void addRule(NodeAttribute required, NodeAttribute forbidden,
               const Color &color, int priority) {
    addRule(ColorRule(required, forbidden, color, priority));
  }

  Color getColor(NodeAttribute attributes) const {
    for (const auto &rule : rules_) {
      if (matchesRule(attributes, rule)) {
        return rule.color;
      }
    }
    return default_color_;
  }

  Color getColor(const DynamicColorParams &params) const {
    // 1. Collision (Highest priority)
    if (params.is_colliding) {
      return Color::Red();
    }

    // 2. Target
    if (params.is_target) {
      return Color::Red();
    }

    // 3. Evaluate Rule-based Colors (including Topology and Danger)
    Color c = getColor(params.attributes);

    // 4. Alpha Calculation
    float alpha_base = 0.15f;

    // Force higher visibility for specific attributes
    if (params.is_topology_mode &&
        hasAnyAttribute(params.attributes,
                        NodeAttribute::MAINLAND | NodeAttribute::ISLAND)) {
      alpha_base = 0.8f;
    } else if (hasAttribute(params.attributes, NodeAttribute::DANGER)) {
      alpha_base = 0.8f; // Clear danger representation
    } else if (hasAttribute(params.attributes, NodeAttribute::SURFACE)) {
      alpha_base = 0.9f; // Surface nodes should be solid
    }

    float alpha_danger = std::min(1.0f, params.danger_level * 10.0f);
    c.a = std::max(alpha_base, alpha_danger);

    if (params.is_colliding || params.is_target) {
      c.a = 1.0f;
    }

    return c;
  }

  /**
   * @brief Set default color (used when no rules match)
   */
  void setDefaultColor(const Color &color) { default_color_ = color; }

  /**
   * @brief Clear all rules
   */
  void clearRules() { rules_.clear(); }

  /**
   * @brief Get number of rules
   */
  size_t getRuleCount() const { return rules_.size(); }

private:
  std::vector<ColorRule> rules_;
  Color default_color_ = Color::White();

  /**
   * @brief Initialize default color rules for GNG visualization
   */
  void initializeDefaultRules() {
    // Highest priority: Target and selection
    addRule(NodeAttribute::TARGET, NodeAttribute::NONE, Color::Red(), 100);

    addRule(NodeAttribute::SELECTED, NodeAttribute::NONE, Color::Purple(), 90);

    addRule(NodeAttribute::CANDIDATE, NodeAttribute::NONE, Color::Brown(), 85);

    // Topology and Reachability (High priority to highlight structure)
    addRule(NodeAttribute::ISLAND, NodeAttribute::NONE,
            Color::fromRGB(255, 0, 255), 75); // Magenta/Purple

    addRule(NodeAttribute::MAINLAND, NodeAttribute::NONE,
            Color::fromRGB(0, 180, 255), 70); // Cyan/SkyBlue (Mainland)

    addRule(NodeAttribute::DANGER, NodeAttribute::NONE,
            Color::fromRGB(255, 255, 0), 65); // Bright Yellow

    // Collision status
    addRule(NodeAttribute::COLLIDING, NodeAttribute::NONE, Color::Red(), 80);

    // [MODIFIED] Surface nodes no longer override color,
    // they only affect alpha (handled in getColor)

    // Path planning
    addRule(NodeAttribute::ON_PLANNED_PATH, NodeAttribute::NONE, Color::Cyan(),
            45);

    // Active nodes (Safe)
    addRule(NodeAttribute::ACTIVE, NodeAttribute::NONE,
            Color::fromRGB(150, 255, 0), 50); // Yellow-Green

    // Inactive nodes
    addRule(NodeAttribute::INACTIVE, NodeAttribute::NONE, Color::Gray(), 40);

    // Manipulability
    addRule(NodeAttribute::HIGH_MANIPULABILITY, NodeAttribute::NONE,
            Color::fromRGB(0, 255, 128), 35); // Bright green

    addRule(NodeAttribute::LOW_MANIPULABILITY, NodeAttribute::NONE,
            Color::fromRGB(255, 128, 0), 30); // Orange
  }

  /**
   * @brief Check if attributes match a rule
   */
  bool matchesRule(NodeAttribute attributes, const ColorRule &rule) const {
    // Check if all required attributes are present
    bool has_required = hasAllAttributes(attributes, rule.required_attributes);

    // Check if any forbidden attributes are present
    bool has_forbidden = hasAnyAttribute(attributes, rule.forbidden_attributes);

    return has_required && !has_forbidden;
  }

  /**
   * @brief Sort rules by priority (highest first)
   */
  void sortRules() {
    std::sort(rules_.begin(), rules_.end(),
              [](const ColorRule &a, const ColorRule &b) {
                return a.priority > b.priority;
              });
  }
};

} // namespace visualization
} // namespace robot_sim
