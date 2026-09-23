#pragma once
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

// 一時コピー専用の計測。詳細タイマーの無効化による計測負荷の比較。
namespace bsp_profile {
using clock = std::chrono::steady_clock;
inline const bool enable_detail = [] {
    const char *value = std::getenv("GNG_PROFILE_DETAIL");
    return value != nullptr && value[0] == '1';
}();
inline std::array<double, 20> time_ns{};
inline std::array<uint64_t, 20> num_calls{};
inline uint64_t num_node_adds = 0;
inline uint64_t num_free_slot_checks = 0;
inline uint64_t num_add_reject_cap = 0;
inline void add(int idx, clock::time_point start) {
    time_ns[idx] += std::chrono::duration<double, std::nano>(clock::now() - start).count();
    ++num_calls[idx];
}
struct scope {
    int idx;
    clock::time_point start{};
    explicit scope(int value) : idx(value) {
        if (enable_detail) {start = clock::now();}
    }
    ~scope() {if (enable_detail) {add(idx, start);}}
};
inline void reset() {
    time_ns.fill(0);
    num_calls.fill(0);
    num_node_adds = num_free_slot_checks = num_add_reject_cap = 0;
}
inline void print(const std::array<double, 6> &stages, int num_input, int num_voxel, int num_attention) {
    std::fprintf(stderr, "BSP_PROFILE {\"time_ms\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", time_ns[idx] / 1e6);
    }
    std::fprintf(stderr, "],\"num_calls\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "", static_cast<unsigned long long>(num_calls[idx]));
    }
    std::fprintf(stderr, "],\"stage_ms\":[");
    for (int idx = 0; idx < 6; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", stages[idx]);
    }
    std::fprintf(stderr, "],\"num_input\":%d,\"num_voxel\":%d,\"num_attention\":%d,"
        "\"num_node_adds\":%llu,\"num_free_slot_checks\":%llu,\"num_add_reject_cap\":%llu}\n",
        num_input, num_voxel, num_attention,
        static_cast<unsigned long long>(num_node_adds),
        static_cast<unsigned long long>(num_free_slot_checks),
        static_cast<unsigned long long>(num_add_reject_cap));
}
}
