#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gng_observation {

// センサから実測点へ向かうレイの整数角度。yawは循環、pitchは非循環。
struct ray_angles {
    uint16_t yaw = 0;
    uint16_t pitch = 0;
    bool has_yaw = false;
};

inline bool can_quantize_ray(double x, double y, double z, ray_angles &result) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || (x == 0 && y == 0 && z == 0)) {return false;}
    constexpr double pi = 3.14159265358979323846;
    const double horizontal = std::hypot(x, y);
    result.has_yaw = horizontal != 0;
    const double yaw = (std::atan2(y, x) + pi) * (65536.0 / (2 * pi));
    const double pitch = (std::atan2(z, horizontal) + pi / 2) * (65536.0 / pi);
    result.yaw = static_cast<uint16_t>(std::clamp(std::floor(yaw), 0.0, 65535.0));
    result.pitch = static_cast<uint16_t>(std::clamp(std::floor(pitch), 0.0, 65535.0));
    return true;
}

// 端点は包含ビン番号。実角度の下端はmin、上端はmax+1の境界で外側への丸め。
// yawのmin/maxは循環区間の始端/終端。離れた観測の間も含む単一区間。
struct angle_range {
    uint16_t min_yaw = 0;
    uint16_t max_yaw = 0;
    uint16_t min_pitch = 0;
    uint16_t max_pitch = 0;
    bool has_support = false;
    bool has_yaw = false;

    static uint32_t forward_span(uint16_t begin, uint16_t end) {
        return (static_cast<uint32_t>(end) + 65536U - begin) & 65535U;
    }

    void clear() {*this = {};}

    bool contains(const ray_angles &value) const {
        if (!has_support || value.pitch < min_pitch || value.pitch > max_pitch) {return false;}
        return !value.has_yaw || (has_yaw && forward_span(min_yaw, value.yaw) <= forward_span(min_yaw, max_yaw));
    }

    void add(const ray_angles &value) {
        if (!has_support) {
            min_pitch = max_pitch = value.pitch;
            has_support = true;
        } else {
            min_pitch = std::min(min_pitch, value.pitch);
            max_pitch = std::max(max_pitch, value.pitch);
        }
        // 極方向のyawは未定義。既存のyaw範囲への影響なし。
        if (!value.has_yaw) {return;}
        if (!has_yaw) {
            min_yaw = max_yaw = value.yaw;
            has_yaw = true;
        } else if (forward_span(min_yaw, value.yaw) > forward_span(min_yaw, max_yaw)) {
            // 既存区間を包含したまま拡張量の小さい側を更新。同値時は終端側。
            if (forward_span(max_yaw, value.yaw) <= forward_span(value.yaw, min_yaw)) {
                max_yaw = value.yaw;
            } else {
                min_yaw = value.yaw;
            }
        }
    }

    bool add_ray(double x, double y, double z) {
        ray_angles value;
        if (!can_quantize_ray(x, y, z, value)) {return false;}
        add(value);
        return true;
    }
};

static_assert(sizeof(angle_range) == 10, "angle range layout changed");

}
