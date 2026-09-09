#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace fuzzrobo::boundary_evidence {

constexpr uint8_t occlusion = 1, free_space = 2, field_of_view = 4;
constexpr double pi = 3.14159265358979323846;
struct point { double x = 0, y = 0, z = 0; };
inline point operator-(point a, point b) {return {a.x-b.x, a.y-b.y, a.z-b.z};}
inline point operator*(point a, double scale) {return {a.x*scale, a.y*scale, a.z*scale};}
inline double dot(point a, point b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
inline double norm(point a) {return std::sqrt(dot(a, a));}
inline bool is_finite(point a) {return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);}

// 候補近傍の実測レイと局所接平面の比較。未観測セルの補間なし。
class classifier {
public:
    double min_range_gap_th = 0.03;
    double max_anchor_dist = 0.05;
    double min_abs_cos = 0.2;

    void clear() {is_ready_ = false; samples_.clear();}

    bool camera(uint32_t width, uint32_t height, double fx, double fy, double cx, double cy) {
        clear();
        if (!width || !height || !std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
            !std::isfinite(cx) || !std::isfinite(cy)) {return false;}
        width_ = width; height_ = height; fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
        is_camera_ = true; is_periodic_ = false; is_ready_ = true;
        return true;
    }

    // センサ座標はx前方・y左方・z上方。yawは連続区間指定、全周のみ周期接続。
    bool lidar(double min_yaw_deg, double max_yaw_deg, double min_pitch_deg, double max_pitch_deg,
        double yaw_step_deg, double pitch_step_deg) {
        clear();
        for (double value : {min_yaw_deg, max_yaw_deg, min_pitch_deg, max_pitch_deg, yaw_step_deg, pitch_step_deg}) {
            if (!std::isfinite(value)) {return false;}
        }
        const double yaw_span = max_yaw_deg-min_yaw_deg, pitch_span = max_pitch_deg-min_pitch_deg;
        if (min_yaw_deg < -180 || min_yaw_deg > 180 || yaw_span <= 0 || yaw_span > 360 ||
            pitch_span <= 0 || min_pitch_deg < -90 || max_pitch_deg > 90 ||
            yaw_step_deg <= 0 || pitch_step_deg <= 0) {return false;}
        const double columns = std::ceil(yaw_span/yaw_step_deg), rows = std::ceil(pitch_span/pitch_step_deg);
        if (columns > 100000 || rows > 100000) {return false;}
        width_ = static_cast<uint32_t>(columns); height_ = static_cast<uint32_t>(rows);
        min_yaw_ = min_yaw_deg*pi/180; yaw_span_ = yaw_span*pi/180;
        min_pitch_ = min_pitch_deg*pi/180; max_pitch_ = max_pitch_deg*pi/180;
        yaw_step_ = yaw_step_deg*pi/180; pitch_step_ = pitch_step_deg*pi/180;
        is_camera_ = false; is_periodic_ = std::abs(yaw_span-360) < 1e-9; is_ready_ = true;
        return true;
    }

    void add(point sample) {
        int column, row;
        if (!project(sample, column, row)) {return;}
        const auto idx = key(column, row);
        auto found = samples_.find(idx);
        // 同一セルの複数リターンは最短距離。奥の点による手前遮蔽物の透過扱いを回避。
        if (found == samples_.end() || dot(sample, sample) < dot(found->second, found->second)) {samples_[idx] = sample;}
    }

    uint8_t classify(point node, point normal) const {
        int column, row;
        if (!project(node, column, row)) {return 0;}
        const auto anchor = samples_.find(key(column, row));
        if (anchor == samples_.end() || norm(anchor->second-node) > max_anchor_dist) {return 0;}
        uint8_t result = 0;
        if (row == 0 || row == static_cast<int>(height_)-1 ||
            (!is_periodic_ && (column == 0 || column == static_cast<int>(width_)-1))) {result |= field_of_view;}
        const double normal_norm = norm(normal), anchor_norm = norm(anchor->second);
        if (!is_finite(normal) || normal_norm < 1e-9 || anchor_norm < 1e-9) {return result;}
        normal = normal*(1/normal_norm);
        if (std::abs(dot(normal, anchor->second)*(1/anchor_norm)) < min_abs_cos) {return result;}
        const double plane = dot(normal, anchor->second);
        uint8_t ray_evidence = 0;
        std::array<point, 8> support{};
        uint32_t num_support = 0;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (!dx && !dy) {continue;}
                int x = column+dx, y = row+dy;
                if (is_periodic_) {x = (x+static_cast<int>(width_))%static_cast<int>(width_);}
                if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_)) {continue;}
                const auto measured = samples_.find(key(x, y));
                if (measured == samples_.end()) {continue;}
                const double measured_range = norm(measured->second);
                const point ray = measured->second*(1/measured_range);
                const double cos = dot(normal, ray);
                if (std::abs(cos) < min_abs_cos) {continue;}
                const double expected_range = plane/cos;
                if (expected_range <= 0) {continue;}
                const double residual = measured_range-expected_range;
                if (residual > min_range_gap_th) {ray_evidence |= free_space;}
                else if (residual < -min_range_gap_th) {ray_evidence |= occlusion;}
                else {support[num_support++] = measured->second-anchor->second;}
            }
        }
        // 少なくとも非平行な2方向の実測支持。走査線1本や不安定法線による反証を抑制。
        bool has_surface_support = false;
        for (uint32_t i = 0; i < num_support; ++i) {
            for (uint32_t j = i+1; j < num_support; ++j) {
                const double lengths = norm(support[i])*norm(support[j]);
                if (lengths > 1e-12 && std::abs(dot(support[i], support[j])/lengths) < 0.9) {has_surface_support = true;}
            }
        }
        return result | (has_surface_support ? ray_evidence : 0);
    }

private:
    bool project(point sample, int &column, int &row) const {
        if (!is_ready_ || !is_finite(sample) || norm(sample) < 1e-9) {return false;}
        double x, y;
        if (is_camera_) {
            if (sample.z <= 0) {return false;}
            x = fx_*sample.x/sample.z+cx_+0.5;
            y = fy_*sample.y/sample.z+cy_+0.5;
        } else {
            double yaw = std::fmod(std::atan2(sample.y, sample.x)-min_yaw_+4*pi, 2*pi);
            const double pitch = std::atan2(sample.z, std::hypot(sample.x, sample.y));
            if (yaw >= yaw_span_ || pitch < min_pitch_ || pitch >= max_pitch_) {return false;}
            x = yaw/yaw_step_; y = (pitch-min_pitch_)/pitch_step_;
        }
        if (x < 0 || y < 0 || x >= width_ || y >= height_) {return false;}
        column = static_cast<int>(x); row = static_cast<int>(y);
        return true;
    }
    uint64_t key(int column, int row) const {return static_cast<uint64_t>(row)*width_+column;}
    bool is_ready_ = false, is_camera_ = false, is_periodic_ = false;
    uint32_t width_ = 0, height_ = 0;
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    double min_yaw_ = 0, yaw_span_ = 0, min_pitch_ = 0, max_pitch_ = 0, yaw_step_ = 0, pitch_step_ = 0;
    std::unordered_map<uint64_t, point> samples_;
};
}
