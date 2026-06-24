#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <optional>
#include <utility>

#include <Eigen/Dense>

namespace simulation {

template <typename Enum, std::size_t... I>
constexpr std::array<Enum, sizeof...(I)> makeEnumRangeArray(
    std::index_sequence<I...>) {
  return {static_cast<Enum>(I)...};
}

template <typename Enum, std::size_t N>
constexpr std::array<Enum, N> makeEnumRangeArray() {
  return makeEnumRangeArray<Enum>(std::make_index_sequence<N>{});
}

/**
 * @brief Simple data structures for GNG visualization.
 * Rescued from legacy simulation module.
 */
struct VisualNode {
  int id;
  Eigen::Vector3f position;
  float level;
  bool active = true;
  std::uint32_t state_mask = 0;

  enum class State : std::uint8_t {
    kMainland = 0,
    kIsland = 1,
    kSurface = 2,
    kActiveSurface = 3,
    kPath = 4,
    kInfluence = 5,
    kDanger = 6,
    kCollision = 7,
  };

  static constexpr std::uint32_t bit(State state) {
    return 1u << static_cast<std::uint32_t>(state);
  }

  void setState(State state, bool enabled = true) {
    const auto flag = bit(state);
    if (enabled) {
      state_mask |= flag;
    } else {
      state_mask &= ~flag;
    }
  }

  bool hasState(State state) const { return (state_mask & bit(state)) != 0; }
};

struct VisualEdge {
  int node1_id;
  int node2_id;
  float level;
  bool is_path = false;
};

class VisualStyleResolver {
public:
  static constexpr auto kAllStates =
      makeEnumRangeArray<VisualNode::State,
                         static_cast<std::size_t>(VisualNode::State::kCount)>();

  static std::optional<VisualNode::State> priorityOf(const VisualNode &node) {
    if (!node.active || node.state_mask == 0) {
      return std::nullopt;
    }
    const std::uint32_t highest_bit = 31u - std::countl_zero(node.state_mask);
    return static_cast<VisualNode::State>(highest_bit);
  }
};

} // namespace simulation
