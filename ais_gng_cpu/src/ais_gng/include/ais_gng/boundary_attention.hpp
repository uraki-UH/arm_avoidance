#pragma once

#include <fuzzrobo/libgng/api.h>
#include <array>
#include <cmath>
#include <string>

#if allow_external_sampler_build
#include <fuzzrobo/libgng/builtin_sampling.hpp>
#endif

namespace fuzzrobo::boundary_attention {
using point = std::array<float, 3>;

// 時刻逆行・同一入力再処理・座標系変更・受信停止時の旧境界の除外。
inline bool can_reuse(const std::string &old_frame, double old_sec,
        const std::string &frame, double sec, double elapsed_sec, double timeout_sec) {
    return !frame.empty() && frame == old_frame && std::isfinite(old_sec) &&
        std::isfinite(sec) && std::isfinite(elapsed_sec) && old_sec > 0 && sec > old_sec &&
        sec - old_sec <= timeout_sec && elapsed_sec >= 0 && elapsed_sec <= timeout_sec;
}
}  // 名前空間fuzzrobo::boundary_attention
