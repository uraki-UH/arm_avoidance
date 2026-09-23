#include <fuzzrobo/libgng/api.h>

#include <array>
#include <cstdint>
#include <iostream>

namespace {

bool keyLess(const GngNodeKey &first, const GngNodeKey &second) {
    return first.id < second.id ||
           (first.id == second.id && first.frame < second.frame);
}

bool validateDelta(const GngMapDelta &delta) {
    for (uint32_t index = 0; index < delta.node_delta_count; ++index) {
        const uint8_t operation = delta.node_deltas[index].operation;
        if (operation < GNG_DELTA_ADD || operation > GNG_DELTA_REMOVE) return false;
    }
    for (uint32_t index = 0; index < delta.edge_delta_count; ++index) {
        const GngEdgeDelta &edge = delta.edge_deltas[index];
        if ((edge.operation != GNG_DELTA_ADD && edge.operation != GNG_DELTA_REMOVE) ||
            !keyLess(edge.first, edge.second)) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    static_assert(sizeof(GngNodeKey) == 8);
    static_assert(sizeof(GngNodeDelta) == 12);
    static_assert(sizeof(GngEdgeDelta) == 20);
    static_assert(sizeof(GngMapDelta) == 32);

    if (gng_init() != SUCCESS || gng_getTopologicalMapDelta() != nullptr) return 1;

    constexpr std::array<float, 24> points{
        -0.3f, -0.3f, 0.0f, -0.3f, 0.3f, 0.0f,
         0.3f, -0.3f, 0.0f,  0.3f, 0.3f, 0.0f,
        -0.2f, -0.2f, 0.1f, -0.2f, 0.2f, 0.1f,
         0.2f, -0.2f, 0.1f,  0.2f, 0.2f, 0.1f,
    };
    LiDAR_Config config;
    config.point_step = sizeof(float) * 3;

    gng_setMapDeltaCapture(1);
    gng_setPointCloud(
        reinterpret_cast<const uint8_t *>(points.data()), points.size() / 3, &config);
    gng_exec();
    const GngMapDelta *delta = gng_getTopologicalMapDelta();
    if (delta == nullptr || delta->version != 1U ||
        delta->node_delta_count == 0U || !validateDelta(*delta)) {
        return 2;
    }

    std::cout << "node_deltas=" << delta->node_delta_count
              << " edge_deltas=" << delta->edge_delta_count
              << " api_test=passed\n";
    gng_setMapDeltaCapture(0);
    return gng_getTopologicalMapDelta() == nullptr ? 0 : 3;
}
