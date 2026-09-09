#include <fuzzrobo/libgng/observation_angle_range.hpp>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

using gng_observation::angle_range;
using gng_observation::ray_angles;

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    angle_range range;
    require(sizeof(range) == 10, "unexpected memory size");
    require(!range.add_ray(0, 0, 0), "zero ray accepted");
    require(!range.add_ray(std::numeric_limits<double>::infinity(), 0, 0), "infinite ray accepted");
    require(!range.add_ray(0, std::numeric_limits<double>::quiet_NaN(), 0), "NaN ray accepted");
    require(!range.has_support, "invalid ray changed support");
    range.add_ray(0, 0, 1);
    require(range.has_support && !range.has_yaw && range.min_pitch == 65535, "north pole handling");
    range.add_ray(0, 0, -1);
    require(!range.has_yaw && range.min_pitch == 0 && range.max_pitch == 65535, "south pole handling");
    range.clear();
    require(!range.has_support && !range.has_yaw, "clear failed");

    constexpr double pi = 3.14159265358979323846;
    for (const double yaw_deg : {179.0, -179.0}) {
        range.add_ray(std::cos(yaw_deg * pi / 180), std::sin(yaw_deg * pi / 180), 0);
    }
    require(range.min_yaw > range.max_yaw, "missing wrap");
    require(angle_range::forward_span(range.min_yaw, range.max_yaw) < 366, "wrap expanded to long arc");
    ray_angles front;
    gng_observation::can_quantize_ray(1, 0, 0, front);
    require(!range.contains(front), "opposite direction included");

    // 全yawビンを単調に追加した場合の全周表現と重複更新。
    range.clear();
    for (uint32_t idx = 0; idx < 65536; ++idx) {range.add({static_cast<uint16_t>(idx), 32768, true});}
    require(angle_range::forward_span(range.min_yaw, range.max_yaw) == 65535, "full yaw not representable");
    for (uint32_t idx = 0; idx < 65536; ++idx) {require(range.contains({static_cast<uint16_t>(idx), 32768, true}), "full yaw missing bin");}

    // 任意順の循環区間拡張で、既に取り込んだ方向が失われないことの検査。
    std::mt19937 generator(42);
    for (uint32_t iter = 0; iter < 100; ++iter) {
        range.clear();
        std::vector<ray_angles> rays;
        for (uint32_t idx = 0; idx < 200; ++idx) {
            rays.push_back({static_cast<uint16_t>(generator()), static_cast<uint16_t>(generator()), true});
            range.add(rays.back());
            for (const auto &ray : rays) {require(range.contains(ray), "previous ray excluded");}
        }
    }

    // 連続角度の外側丸め。各ビン境界の直前・直後と両極近傍を含む検査。
    for (uint32_t idx = 0; idx < 65536; ++idx) {
        for (const double fraction : {0.000001, 0.5, 0.999999}) {
            const double yaw = -pi + (idx + fraction) * (2 * pi / 65536);
            const double pitch = -pi / 2 + (idx + fraction) * (pi / 65536);
            ray_angles value;
            require(gng_observation::can_quantize_ray(std::cos(pitch) * std::cos(yaw),
                std::cos(pitch) * std::sin(yaw), std::sin(pitch), value), "valid ray rejected");
            require(value.yaw == idx && value.pitch == idx, "outward bin mapping failed");
        }
    }
    std::cout << "observation_angle_range_test=passed bytes_per_node=" << sizeof(range) << '\n';
}
