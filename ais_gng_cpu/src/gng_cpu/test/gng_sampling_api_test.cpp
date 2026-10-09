#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/observation_api.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <utility>

void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}

void test_registered_input(const std::vector<float> &points, const LiDAR_Config &input) {
    const uint32_t input_num = points.size() / 3;
    const auto submit = [&] {gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), input_num, &input);};
    gng_input_grid grid;
    require(gng_get_input_grid(&grid) && grid.enable_downsampling, "登録格子の公開");
    require(!gng_get_input_grid(nullptr), "null格子出力の拒否");
    submit();
    uint32_t transformed_num = 0;
    const auto *xyz = gng_getAffineTransformedInputPointCloud(&transformed_num);
    require(transformed_num == input_num, "変換済み元点数の保持");
    std::vector<uint64_t> registered;
    for (uint32_t idx = 0; idx < transformed_num; ++idx) {
        uint32_t key[3]; bool is_inside = true;
        for (uint32_t axis = 0; axis < 3; ++axis) {
            const float value = xyz[3 * idx + axis];
            is_inside = is_inside && value >= grid.min_pos[axis] && value <= grid.max_pos[axis];
            key[axis] = static_cast<uint32_t>((value - grid.min_pos[axis]) * (1.f / grid.size));
        }
        const auto cell_idx = key[0] + grid.num_cells[0] * (key[1] + grid.num_cells[1] * key[2]);
        if (is_inside && cell_idx < grid.num_cells[0] * grid.num_cells[1] * grid.num_cells[2]) {
            registered.push_back((uint64_t{cell_idx} << 32) | idx);
        }
    }
    std::sort(registered.begin(), registered.end());
    struct capture {mutable std::vector<std::pair<uint32_t, uint32_t>> cells;};
    capture observations;
    gng_sampling_rule rule; rule.id = 900; rule.ratio = .5; rule.data = &observations;
    rule.enable_nearest = 0;
    rule.cell_score = [](const gng_sampling_cell &cell, const void *data) {
        static_cast<const capture *>(data)->cells.emplace_back(cell.idx, cell.num_points);
        return gng_sampling_score{1, 0};
    };
    require(gng_set_sampling_rules(&rule, 1), "通常登録の比較規則"); gng_exec();
    const auto expected_cells = observations.cells;
    require(!expected_cells.empty(), "通常登録の占有セル");
    uint32_t num_candidates = 0;
    const auto *candidates = gng_get_sampling_points(rule.id, &num_candidates);
    const std::vector<uint32_t> expected_candidates(candidates, candidates + num_candidates);
    submit(); observations.cells.clear();
    require(gng_set_registered_input(&grid, registered.data(), registered.size(), input_num), "外部登録の投入");
    require(gng_set_sampling_rules(&rule, 1), "外部登録と既存サンプラーの併用"); gng_exec();
    require(observations.cells == expected_cells, "登録セル・元点数の一致");
    candidates = gng_get_sampling_points(rule.id, &num_candidates);
    require(num_candidates == expected_candidates.size() &&
        std::equal(expected_candidates.begin(), expected_candidates.end(), candidates), "学習候補元番号の一致");
    require(gng_setParameter("node.enable_observation_support", 0, 1), "観測支持の失効試験設定");
    const auto reject = [&](const gng_input_grid *spec, const std::vector<uint64_t> &entries, uint32_t num) {
        submit();
        require(gng_set_sampling_rules(&rule, 1), "不正登録前の借用サンプラー指定");
        gng_observation_input observation; observation.has_origin = 1;
        require(gng_set_observation_input(&observation), "不正登録前の観測指定");
        const auto frame = gng_getTopologicalMap().frame_number;
        require(!gng_set_registered_input(spec, entries.data(), entries.size(), num), "不正登録の拒否");
        gng_exec(); require(gng_getTopologicalMap().frame_number == frame, "不正登録後の学習抑止");
        observations.cells.clear();
        require(gng_set_registered_input(&grid, registered.data(), registered.size(), input_num), "同一入力での登録修復");
        gng_exec();
        require(observations.cells.empty(), "保留後の借用サンプラー失効");
        require(!gng_get_observation_frame().has_origin, "保留後の観測指定失効");
    };
    reject(nullptr, registered, input_num);
    auto changed = grid; changed.size *= 2; reject(&changed, registered, input_num);
    changed = grid; changed.min_pos[0] += .1f; reject(&changed, registered, input_num);
    changed = grid; ++changed.num_cells[0]; reject(&changed, registered, input_num);
    reject(&grid, registered, input_num + 1);
    auto duplicate = registered; duplicate[1] = duplicate[0]; reject(&grid, duplicate, input_num);
    auto reversed = registered; std::reverse(reversed.begin(), reversed.end()); reject(&grid, reversed, input_num);
    auto outside = registered; outside.back() = UINT64_MAX; reject(&grid, outside, input_num);
    submit(); require(!gng_set_registered_input(&grid, nullptr, 1, input_num), "null登録配列の拒否");
    require(gng_set_registered_input(&grid, registered.data(), registered.size(), input_num), "有効登録による復帰");
    gng_exec();
    submit(); require(gng_set_registered_input(&grid, nullptr, 0, input_num), "占有セルなしの有効入力");
    gng_exec(); uint32_t num_labels = 0; const auto *labels = gng_getDownSampling(&num_labels);
    for (uint32_t idx = 0; idx < num_labels; ++idx) {require(labels[idx] == 0, "空登録での元点学習なし");}
    submit(); observations.cells.clear();
    require(gng_set_sampling_rules(&rule, 1), "新入力による外部登録の失効"); gng_exec();
    require(observations.cells == expected_cells, "新入力での通常登録復帰");
}

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
    test_registered_input(points, input);
    std::cout << "gng_sampling_api_test=passed\n";
}
