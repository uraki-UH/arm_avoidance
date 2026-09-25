#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/observation_api.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    gng_setParameter("node.num_max", 0, 1024);
    gng_setParameter("input.point_cloud_num", 0, 5000);
    gng_setParameter("node.learning_num", 0, 1000);
    require(gng_init() == SUCCESS, "初期化失敗");
    gng_setParameter("node.covariance_enabled", 0, 1);
    gng_setParameter("node.enable_observation_support", 0, 1);
    gng_setTrainingEventCapture(1);
    std::vector<float> points;
    for (int x = 0; x < 25; ++x) for (int y = 0; y < 25; ++y) {
        points.insert(points.end(), {0.5f + x * 0.03f, -0.4f + y * 0.03f, 0.2f});
    }
    LiDAR_Config config;
    config.point_step = 12;
    const auto submit = [&] {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size() / 3, &config);
    };
    const auto run = [&] {
        gng_exec();
        uint32_t num = 0;
        gng_getTrainingEvents(&num);
        return num;
    };
    for (int idx = 0; idx < 5; ++idx) {submit(); run();}
    submit();
    const auto baseline = run();
    require(baseline > 950 && baseline <= 1000, "通常学習のイベント数不整合");
    submit();
    const uint32_t ids[] = {200, 201, 200};
    const auto count_samples = [] {
        const auto map = gng_getTopologicalMap();
        uint64_t num = 0;
        for (uint32_t idx = 0; idx < map.node_num; ++idx) {
            num += gng_get_node_statistics(map.nodes[idx].id).winner_point_count;
        }
        return num;
    };
    const auto count_before = count_samples();
    gng_observation_input observation;
    observation.has_origin = 1;
    require(gng_set_observation_input(&observation), "観測原点設定失敗");
    require(gng_set_priority_input(ids, 3, 0.5f), "重点設定失敗");
    const auto focused = run();
    require(focused > 450 && focused <= 500, "重点更新が観測統計へ混入");
    require(count_samples() <= count_before + 500, "重点更新が共分散件数へ混入");
    const auto observation_frame = gng_get_observation_frame();
    require(observation_frame.ray_num + observation_frame.pixel_hit_num == focused, "重点更新が観測方向件数へ混入");
    submit();
    require(run() > 950, "重点指定が次フレームへ残存");
    submit();
    require(gng_set_priority_input(ids, 3, 0.5f), "再設定失敗");
    submit();
    require(run() > 950, "入力置換時に旧添字が残存");
    submit();
    require(gng_set_priority_input(ids, 3, 0.5f), "再設定失敗");
    const uint32_t invalid = 999999;
    require(!gng_set_priority_input(&invalid, 1, 0.5f), "不正添字を受理");
    require(run() > 950, "不正指定後も重点設定が残存");
    for (float ratio : {0.f, 1.f, -0.5f, std::numeric_limits<float>::quiet_NaN()}) {
        require(!gng_set_priority_input(ids, 3, ratio), "不正配分率を受理");
    }
    require(!gng_set_priority_input(nullptr, 1, 0.5f), "null配列を受理");
    require(gng_set_priority_input(nullptr, 0, 0), "明示解除失敗");
    submit();
    gng_set_priority_input(ids, 3, 0.5f);
    run();
    require(run() > 950, "同じ入力の再実行時に重点指定が残存");
    submit();
    const uint32_t weighted_ids[] = {201, 200};
    const float weights[] = {.1f, .9f};
    require(gng_set_weighted_priority_input(weighted_ids, weights, 2, .7f), "重み付き指定失敗");
    const auto weighted = run();
    require(weighted > 250 && weighted <= 301, "重み付き指定の通常枠不整合");
    require(run() > 950, "重み付き指定が次回へ残存");
    submit();
    const float invalid_weights[] = {1, NAN};
    require(!gng_set_weighted_priority_input(weighted_ids, invalid_weights, 2, .7f), "非有限重みを受理");
    require(!gng_set_weighted_priority_input(weighted_ids, nullptr, 2, .7f), "null重みを受理");
    const uint32_t duplicate_ids[] = {200, 200};
    require(!gng_set_weighted_priority_input(duplicate_ids, weights, 2, .7f), "重複添字を受理");
    require(run() > 950, "不正重み指定後の通常枠不整合");
    std::cout << "通常イベント=" << baseline << " 重点50%時の通常イベント=" << focused << '\n';
}
