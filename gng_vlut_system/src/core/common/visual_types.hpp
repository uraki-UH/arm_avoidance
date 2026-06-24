#pragma once

#include <cstdint>
#include <bit>
#include <optional>
#include <Eigen/Dense>

namespace simulation {

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
    }};

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

    bool hasState(State state) const {
        return (state_mask & bit(state)) != 0;
    }
};

struct VisualEdge {
    int node1_id;
    int node2_id;
    float level;
    bool is_path = false;
};

class VisualStyleResolver {
public:
    static std::optional<VisualNode::State> priorityOf(const VisualNode &node) {
        if (!node.active) {
            return std::nullopt;
        }
        if (node.state_mask == 0) {
            return std::nullopt;
        }
        const std::uint32_t highest_bit = 31u - std::countl_zero(node.state_mask);
        return static_cast<VisualNode::State>(highest_bit);
    }
};

} // namespace simulation
