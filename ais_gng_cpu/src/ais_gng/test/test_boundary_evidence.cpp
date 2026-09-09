#include "ais_gng/boundary_evidence.hpp"
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace fuzzrobo::boundary_evidence;
void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}

int main() {
    classifier model;
    const point normal{0, 0, 1};
    const auto camera_scene = [&](int mode) {
        model.camera(7, 7, 20, 20, 3, 3);
        for (int y = 0; y < 7; ++y) {
            for (int x = 0; x < 7; ++x) {
                if (mode == 3 && x > 3) {continue;}
                if (mode == 4 && y != 3) {continue;}
                double depth = 2;
                if (x > 3 && mode == 1) {depth = 3;}
                if (x > 3 && (mode == 2 || mode == 4)) {depth = 1;}
                model.add({(x-3)*depth/20, (y-3)*depth/20, depth});
            }
        }
    };
    camera_scene(0);
    require(model.classify({0, 0, 2}, normal) == 0, "連続面の誤判定");
    require(model.classify({-0.3, 0, 2}, normal) == field_of_view, "視野端の欠落");
    require(model.classify({-0.4, 0, 2}, normal) == 0, "視野外ノードの確定扱い");
    require(model.classify({0, 0, 2.2}, normal) == 0, "実測点から離れたノードの誤判定");
    camera_scene(1);
    require(model.classify({0, 0, 2}, normal) == free_space, "背景までの実測レイの欠落");
    require(model.classify({0, -0.3, 2}, normal) == (free_space|field_of_view), "視野端と自由空間の併存欠落");
    require(model.classify({0, 0, 2}, {}) == 0, "法線欠落時の反証");
    camera_scene(2);
    require(model.classify({0, 0, 2}, normal) == occlusion, "手前の遮蔽物の欠落");
    camera_scene(3);
    require(model.classify({0, 0, 2}, normal) == 0, "欠測を自由空間扱い");
    camera_scene(4);
    require(model.classify({0, 0, 2}, normal) == 0, "単一走査線による確定扱い");
    model.clear();
    require(model.classify({0, 0, 2}, normal) == 0, "前フレームの証拠残留");
    // 傾斜面の距離変化は段差として扱わない局所平面補正。
    model.camera(7, 7, 20, 20, 3, 3);
    for (int y = 0; y < 7; ++y) {
        for (int x = 0; x < 7; ++x) {
            const double rx = (x-3)/20.0, ry = (y-3)/20.0, depth = 2/(1-2*rx);
            model.add({rx*depth, ry*depth, depth});
        }
    }
    require(model.classify({0, 0, 2}, {-2, 0, 1}) == 0, "傾斜面の誤反証");
    model.add({std::numeric_limits<double>::quiet_NaN(), 0, 2});
    require(model.classify({0, 0, 2}, {-2, 0, 1}) == 0, "無効レイの混入");
    // 全周の継ぎ目は視野端ではなく、垂直視野端のみを判定。
    require(model.lidar(-180, 180, -30, 30, 1, 1), "LiDAR設定の拒否");
    const auto sample = [](double yaw, double pitch) {
        yaw *= pi/180; pitch *= pi/180;
        return point{2*std::cos(pitch)*std::cos(yaw), 2*std::cos(pitch)*std::sin(yaw), 2*std::sin(pitch)};
    };
    const auto seam = sample(-179.5, 0.5), top = sample(0.5, 29.5);
    model.add(seam); model.add(top);
    require(model.classify(seam, {}) == 0, "全周継ぎ目の誤判定");
    require(model.classify(top, {}) == field_of_view, "LiDAR垂直視野端の欠落");
    require(!model.lidar(-180, 180, -30, 30, 0, 1), "無効な走査間隔の受付");
    require(model.classify(top, {}) == 0, "無効視野設定後の証拠残留");
    require(model.lidar(170, 190, -30, 30, 1, 1), "yaw折返しをまたぐ部分視野の拒否");
    const auto edge = sample(-170.5, 0.5);
    model.add(edge);
    require(model.classify(edge, {}) == field_of_view, "部分視野端の欠落");

    // 20,000入力・5,000候補の合成負荷。上限保証ではない単発測定。
    const auto start = std::chrono::steady_clock::now();
    model.camera(200, 100, 200, 200, 100, 50);
    for (int y = 0; y < 100; ++y) {
        for (int x = 0; x < 200; ++x) {model.add({(x-100)/100.0, (y-50)/100.0, 2});}
    }
    uint32_t num_evidence = 0;
    for (int idx = 0; idx < 5000; ++idx) {
        num_evidence += model.classify({(idx%200-100)/100.0, (idx/200-50)/100.0, 2}, normal) != 0;
    }
    std::cout << "boundary_evidence=passed ms=" << std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-start).count() << " evidence=" << num_evidence << '\n';
}
