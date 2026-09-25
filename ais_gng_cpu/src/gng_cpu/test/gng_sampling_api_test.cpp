#include <fuzzrobo/libgng/api.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}
int main() {
    gng_setParameter("node.num_max", 0, 1024);
    gng_setParameter("input.point_cloud_num", 0, 5000);
    gng_setParameter("node.learning_num", 0, 1000);
    require(gng_init() == SUCCESS, "初期化");
    gng_setTrainingEventCapture(1);
    std::vector<float> points;
    for (int x = 0; x < 25; ++x) for (int y = 0; y < 25; ++y) {
        points.insert(points.end(), {.5f + x * .03f, -.4f + y * .03f, .2f});
    }
    LiDAR_Config input; input.point_step = 12;
    const auto submit = [&] {gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size() / 3, &input);};
    for (int iter = 0; iter < 5; ++iter) {submit(); gng_exec();}
    gng_sampling_rule rule;
    rule.id = 1234; rule.ratio = .5;
    rule.cell_score = [](const gng_sampling_cell &, const void *) {return gng_sampling_score{1, 0};};
    submit(); require(gng_set_sampling_rules(&rule, 1), "共通規則設定"); gng_exec();
    require(gng_get_sampling_stats().num_priority_samples == 500, "総学習回数内の重点枠");
    uint32_t num_events = 0;
    gng_getTrainingEvents(&num_events);
    require(num_events > 450 && num_events <= 500, "重点反復の観測統計への混入防止");
    uint32_t num_points = 0;
    const auto *ids = gng_get_sampling_points(rule.id, &num_points);
    require(ids && num_points == points.size() / 3, "共通候補の公開");
    for (uint32_t idx = 1; idx < num_points; ++idx) {require(ids[idx - 1] < ids[idx], "元番号の重複なし");}
    gng_exec(); require(gng_get_sampling_stats().num_priority_samples == 0, "再実行時の指定失効");
    submit(); require(gng_set_sampling_rules(&rule, 1), "再設定"); submit(); gng_exec();
    require(gng_get_sampling_stats().num_priority_samples == 0, "入力置換時の失効");
    gng_sampling_rule duplicate[] = {rule, rule};
    require(!gng_set_sampling_rules(duplicate, 2), "重複IDの拒否");
    require(!gng_set_sampling_rules(nullptr, 1), "null規則の拒否");
    for (double ratio : std::vector<double>{-1.0, 1.0, INFINITY, NAN}) {
        rule.ratio = ratio; require(!gng_set_sampling_rules(&rule, 1), "不正配分率の拒否");
    }
    submit(); rule.ratio = .5; require(gng_set_sampling_rules(&rule, 1), "旧APIとの混合");
    const uint32_t raw_idx = 200;
    require(!gng_set_priority_input(&raw_idx, 1, .5), "旧APIとの合計配分超過の拒否");
    require(gng_set_priority_input(&raw_idx, 1, .2), "旧APIとの配分維持");
    gng_exec(); require(gng_get_sampling_stats().num_priority_samples >= 700, "混合配分の保持");
    submit(); rule.ratio = 0; require(gng_set_sampling_rules(&rule, 1), "無効規則の設定"); gng_exec();
    require(gng_get_sampling_stats().num_cell_evaluations == 0, "無効規則の評価省略");
    std::cout << "gng_sampling_api_test=passed\n";
}
