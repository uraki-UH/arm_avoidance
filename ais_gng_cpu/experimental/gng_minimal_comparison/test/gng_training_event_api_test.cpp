#include <fuzzrobo/libgng/api.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

int main() {
    if (gng_init() != SUCCESS) {
        std::cerr << "gng_init failed\n";
        return 1;
    }

    constexpr std::array<float, 24> points{
        -1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        -0.8f, -0.8f, 0.1f,
        -0.8f, 0.8f, 0.1f,
        0.8f, -0.8f, 0.1f,
        0.8f, 0.8f, 0.1f,
    };
    LiDAR_Config config;
    config.point_step = sizeof(float) * 3;
    gng_setPointCloud(
        reinterpret_cast<const uint8_t *>(points.data()),
        static_cast<uint32_t>(points.size() / 3),
        &config);

    uint32_t event_num = 0;
    gng_setTrainingEventCapture(0);
    gng_exec();
    if (gng_getTrainingEvents(&event_num) != nullptr || event_num != 0) {
        std::cerr << "disabled capture returned events\n";
        return 2;
    }

    gng_setTrainingEventCapture(1);
    gng_exec();
    const auto *events = gng_getTrainingEvents(&event_num);
    if (events == nullptr || event_num == 0) {
        std::cerr << "enabled capture returned no events\n";
        return 3;
    }
    const uint32_t captured_event_num = event_num;
    for (uint32_t event_idx = 0; event_idx < event_num; ++event_idx) {
        if (events[event_idx].winner_rank != 1) {
            std::cerr << "unexpected first winner rank\n";
            return 4;
        }
    }

    gng_setTrainingEventMaxWinnerRank(2);
    gng_exec();
    events = gng_getTrainingEvents(&event_num);
    bool has_second_winner = false;
    for (uint32_t event_idx = 0; event_idx < event_num; ++event_idx) {
        if (events[event_idx].winner_rank == 2) {
            has_second_winner = true;
        } else if (events[event_idx].winner_rank != 1) {
            std::cerr << "unexpected winner rank\n";
            return 5;
        }
    }
    if (!has_second_winner) {
        std::cerr << "second winner was not captured\n";
        return 6;
    }
    for (uint32_t event_idx = 0; event_idx < event_num; ++event_idx) {
        const auto &event = events[event_idx];
        if (!std::isfinite(event.residual.x) ||
            !std::isfinite(event.residual.y) ||
            !std::isfinite(event.residual.z)) {
            std::cerr << "non-finite residual\n";
            return 7;
        }
    }

    gng_setTrainingEventCapture(0);
    gng_exec();
    if (gng_getTrainingEvents(&event_num) != nullptr || event_num != 0) {
        std::cerr << "disabled capture retained events\n";
        return 8;
    }

    std::cout << "captured_event_num=" << captured_event_num << " api_test=passed\n";
    return 0;
}
