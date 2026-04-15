#pragma once

#include <cstdint>

namespace robot_sim {
namespace visualization {

/**
 * @brief Node attributes using bit flags for flexible combinations
 *
 * Multiple attributes can be combined using bitwise OR.
 * Example: NodeAttribute::ACTIVE | NodeAttribute::SURFACE
 */
enum class NodeAttribute : uint32_t {
  NONE = 0, ///< No special attributes

  // Activity status
  ACTIVE = 1 << 0,   ///< Node is active in GNG
  INACTIVE = 1 << 1, ///< Node is inactive in GNG

  // Topology
  SURFACE = 1 << 2,  ///< Node is on the surface
  BOUNDARY = 1 << 3, ///< Node is on a boundary

  // Collision status
  COLLIDING = 1 << 4,      ///< Node is in collision
  COLLISION_FREE = 1 << 5, ///< Node is verified collision-free

  // Selection/Target
  TARGET = 1 << 6,    ///< Node is the current target
  SELECTED = 1 << 7,  ///< Node is selected by user
  CANDIDATE = 1 << 8, ///< Node is a path planning candidate

  // Quality metrics
  HIGH_MANIPULABILITY = 1 << 9, ///< High manipulability measure
  LOW_MANIPULABILITY = 1 << 10, ///< Low manipulability measure

  // Path planning
  ON_PLANNED_PATH = 1 << 11, ///< Node is on the planned path
  VISITED = 1 << 12,         ///< Node has been visited

  // Topology status
  MAINLAND = 1 << 13, ///< Node is on the mainland (largest safe component)
  ISLAND = 1 << 14,   ///< Node is isolated from mainland (trapped network)

  // Reserved for future use
  DANGER = 1 << 15, ///< Node is in a dangerous area
  CUSTOM_1 = 1 << 16,
  CUSTOM_2 = 1 << 17,
  CUSTOM_3 = 1 << 18,
};

/**
 * @brief Bitwise OR operator for combining attributes
 */
inline NodeAttribute operator|(NodeAttribute a, NodeAttribute b) {
  return static_cast<NodeAttribute>(static_cast<uint32_t>(a) |
                                    static_cast<uint32_t>(b));
}

/**
 * @brief Bitwise AND operator for checking attributes
 */
inline NodeAttribute operator&(NodeAttribute a, NodeAttribute b) {
  return static_cast<NodeAttribute>(static_cast<uint32_t>(a) &
                                    static_cast<uint32_t>(b));
}

/**
 * @brief Bitwise OR assignment operator
 */
inline NodeAttribute &operator|=(NodeAttribute &a, NodeAttribute b) {
  a = a | b;
  return a;
}

/**
 * @brief Bitwise AND assignment operator
 */
inline NodeAttribute &operator&=(NodeAttribute &a, NodeAttribute b) {
  a = a & b;
  return a;
}

/**
 * @brief Check if attribute set has a specific attribute
 * @param attrs Attribute set to check
 * @param check Attribute to look for
 * @return true if attrs contains check
 */
inline bool hasAttribute(NodeAttribute attrs, NodeAttribute check) {
  return (static_cast<uint32_t>(attrs) & static_cast<uint32_t>(check)) != 0;
}

/**
 * @brief Check if attribute set has ALL specified attributes
 * @param attrs Attribute set to check
 * @param required Required attributes (can be combined with |)
 * @return true if attrs contains all required attributes
 */
inline bool hasAllAttributes(NodeAttribute attrs, NodeAttribute required) {
  uint32_t attrs_val = static_cast<uint32_t>(attrs);
  uint32_t required_val = static_cast<uint32_t>(required);
  return (attrs_val & required_val) == required_val;
}

/**
 * @brief Check if attribute set has ANY of the specified attributes
 * @param attrs Attribute set to check
 * @param any_of Attributes to check for (can be combined with |)
 * @return true if attrs contains at least one of the specified attributes
 */
inline bool hasAnyAttribute(NodeAttribute attrs, NodeAttribute any_of) {
  return (static_cast<uint32_t>(attrs) & static_cast<uint32_t>(any_of)) != 0;
}

/**
 * @brief Remove an attribute from an attribute set
 * @param attrs Attribute set to modify
 * @param to_remove Attribute to remove
 * @return Modified attribute set
 */
inline NodeAttribute removeAttribute(NodeAttribute attrs,
                                     NodeAttribute to_remove) {
  return static_cast<NodeAttribute>(static_cast<uint32_t>(attrs) &
                                    ~static_cast<uint32_t>(to_remove));
}

} // namespace visualization
} // namespace robot_sim
