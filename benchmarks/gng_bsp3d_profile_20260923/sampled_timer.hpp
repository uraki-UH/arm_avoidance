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
inline const uint64_t sample_stride = [] {
    const char *value = std::getenv("GNG_PROFILE_SAMPLE_STRIDE");
    return value != nullptr ? std::max<uint64_t>(1, std::strtoull(value, nullptr, 10)) : 1;
}();
inline std::array<double, 20> time_ns{};
inline std::array<uint64_t, 20> num_calls{};
inline std::array<uint64_t, 20> num_samples{};
inline uint64_t frame_seq = 0;
inline uint64_t num_node_adds = 0;
inline uint64_t num_free_slot_checks = 0;
inline uint64_t num_add_reject_cap = 0;
inline void add(int idx, clock::time_point start) {
    time_ns[idx] += std::chrono::duration<double, std::nano>(clock::now() - start).count();
    ++num_samples[idx];
}
struct scope {
    int idx;
    clock::time_point start{};
    bool is_sampled = false;
    explicit scope(int value) : idx(value) {
        if (!enable_detail) {return;}
        ++num_calls[idx];
        // 高頻度の探索・接続・移動だけの間引き計時。GNGの入力点・処理回数への影響なし。
        const bool has_high_frequency = idx <= 2 || idx == 4 || idx == 7 || idx == 9 || idx == 10 || idx == 19;
        is_sampled = !has_high_frequency || (num_calls[idx] + frame_seq) % sample_stride == 0;
        if (is_sampled) {start = clock::now();}
    }
    ~scope() {if (is_sampled) {add(idx, start);}}
};
inline void reset() {
    time_ns.fill(0);
    num_calls.fill(0);
    num_samples.fill(0);
    ++frame_seq;
    num_node_adds = num_free_slot_checks = num_add_reject_cap = 0;
}
inline void print(const std::array<double, 6> &stages, int num_input, int num_voxel, int num_attention) {
    std::fprintf(stderr, "BSP_PROFILE {\"time_ms\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", (num_samples[idx] ? time_ns[idx] * num_calls[idx] / num_samples[idx] : 0) / 1e6);
    }
    std::fprintf(stderr, "],\"num_calls\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "", static_cast<unsigned long long>(num_calls[idx]));
    }
    std::fprintf(stderr, "],\"num_samples\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "", static_cast<unsigned long long>(num_samples[idx]));
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
