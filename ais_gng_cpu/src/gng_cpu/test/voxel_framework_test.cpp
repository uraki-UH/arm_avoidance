#include <fuzzrobo/libgng/api.h>
#include <fuzzrobo/libgng/voxel_framework.hpp>
#include <iostream>
#include <type_traits>
#include <vector>

namespace framework = fuzzrobo::voxel_framework;
void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}

// 不要経路を定義しないポリシー。実行時分岐では成立しないコンパイル検証。
struct baseline_only {
    int baseline(int value) const {return value + 1;}
    void collect(int) = delete;
    void fuzzy(int) = delete;
};
using disabled = framework::pipeline<framework::features<false>, baseline_only>;
static_assert(std::is_empty_v<disabled>);
struct disabled_owner : disabled {uint64_t value;};
static_assert(sizeof(disabled_owner) == sizeof(uint64_t));

struct cell {uint32_t key; double points, nodes;};
struct policy {
    struct attributes_type {double support, shortage;};
    using key_type = uint32_t;
    using key_hash = std::hash<key_type>;
    uint32_t num_collections = 0, num_updates = 0, num_evaluations = 0, num_fuzzy = 0;
    double baseline(const cell &input) const {return input.points;}
    attributes_type collect(const cell &input) {
        ++num_collections;
        return {input.points, input.points / (1 + input.nodes)};
    }
    key_type key(const cell &input) const {return input.key;}
    void update(attributes_type &value, const attributes_type *previous, double elapsed_sec) {
        ++num_updates;
        if (previous) {value.shortage = (previous->shortage + value.shortage) / (1 + elapsed_sec);}
    }
    double evaluate(const attributes_type &value) {++num_evaluations; return value.shortage;}
    double fuzzy(const attributes_type &value) {++num_fuzzy; return value.shortage / (1 + value.shortage);}
};
using stateless = framework::pipeline<framework::features<true>, policy>;
using fuzzy_only = framework::pipeline<framework::features<true, true>, policy>;
using full = framework::pipeline<framework::features<true, true, true>, policy>;
static_assert(std::is_empty_v<stateless> && std::is_empty_v<fuzzy_only>);
static_assert(!std::is_empty_v<full>);

// 実GNG APIを通す拡張評価例。実運用の密度評価式ではない。
struct gng_policy {
    using attributes_type = double;
    using key_type = int64_t;
    using key_hash = std::hash<key_type>;
    double baseline(const gng_sampling_cell &) const {return 1;}
    double collect(const gng_sampling_cell &input) const {return input.num_points;}
    key_type key(const gng_sampling_cell &input) const {
        return std::llround(input.min_pos[0] * 1000000);
    }
    void update(double &value, const double *previous, double) const {if (previous) {value += *previous;}}
    double evaluate(double value) const {return value;}
    double fuzzy(double value) const {return value / (1 + value);}
};
struct gng_context {
    framework::pipeline<framework::build_features, gng_policy> pipeline;
    gng_policy rules;
};

void check_gng() {
    require(gng_setParameter("node.num_max", 0, 1024), "GNG設定");
    gng_setParameter("input.point_cloud_num", 0, 1000);
    gng_setParameter("node.learning_num", 0, 1000);
    require(gng_init() == SUCCESS, "GNG初期化");
    std::vector<float> points;
    for (int idx = 0; idx < 200; ++idx) {points.insert(points.end(), {.5f + idx * .01f, 0.f, .2f});}
    LiDAR_Config input; input.point_step = 12;
    gng_context context;
    gng_sampling_rule rule;
    rule.id = 77; rule.ratio = .5; rule.data = &context;
    rule.cell_score = [](const gng_sampling_cell &cell, const void *data) {
        auto &context = *static_cast<gng_context *>(const_cast<void *>(data));
        const double score = context.pipeline.evaluate(cell, context.rules);
        return gng_sampling_score{score, 0};
    };
    for (uint32_t frame = 0; frame < 4; ++frame) {
        context.pipeline.begin_frame(1, frame + 1, 1000);
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size()/3, &input);
        require(gng_set_sampling_rules(&rule, 1), "基盤規則の登録");
        gng_exec(); context.pipeline.end_frame();
        const auto stats = gng_get_sampling_stats();
        require(!stats.has_invalid_score && stats.num_priority_samples == 500, "実GNGの重点配分");
        uint32_t num = 0;
        gng_get_sampling_points(77, &num);
        require(num == 200 && gng_getTopologicalMap().node_num > 0, "元点集合と学習出力");
    }
    require((context.pipeline.num_history_cells() > 0) == framework::build_features::enable_history, "構成に応じた実GNG履歴");
}

int main() {
    baseline_only baseline;
    disabled off;
    require(off.evaluate(3, baseline) == 4, "無拡張経路");
    policy rules;
    stateless simple;
    require(simple.evaluate(cell{1, 10, 1}, rules) == 5, "属性集計と通常評価");
    require(rules.num_collections == 1 && rules.num_updates == 0 && rules.num_fuzzy == 0, "不要処理の不実行");
    fuzzy_only fuzzy;
    require(std::abs(fuzzy.evaluate(cell{1, 10, 1}, rules) - 5.0/6) < 1e-12, "ファジィ経路");
    full history;
    history.begin_frame(1, 1, 10); history.evaluate(cell{1, 10, 1}, rules); history.end_frame();
    history.begin_frame(1, 2, 10);
    require(std::abs(history.evaluate(cell{1, 6, 1}, rules) - .8) < 1e-12, "同じ空間キーの履歴更新");
    history.end_frame();
    history.begin_frame(2, 3, 10);
    require(history.num_history_cells() == 0, "座標定義の変更による履歴失効");
    history.evaluate(cell{2, 6, 1}, rules); history.end_frame();
    history.begin_frame(2, 2, 10);
    require(history.num_history_cells() == 0, "時刻逆行による履歴失効");
    history.evaluate(cell{2, 6, 1}, rules); history.end_frame();
    history.begin_frame(2, 3, 10); history.end_frame();
    require(history.num_history_cells() == 0, "未観測セルの履歴削除");
    history.begin_frame(2, 4, 1); history.evaluate(cell{1, 1, 0}, rules);
    bool has_error = false;
    try {history.evaluate(cell{2, 1, 0}, rules);} catch (const std::length_error &) {has_error = true;}
    require(has_error && history.num_history_cells() == 0, "容量上限と部分更新の破棄");
    history.begin_frame(2, 5, 10); history.evaluate(cell{1, 1, 0}, rules); has_error = false;
    try {history.evaluate(cell{1, 1, 0}, rules);} catch (const std::logic_error &) {has_error = true;}
    require(has_error && history.num_history_cells() == 0, "重複キーの拒否");
    check_gng();
    std::cout << "voxel_framework_test=passed attributes=" << framework::build_features::enable_attributes
        << " fuzzy=" << framework::build_features::enable_fuzzy << " history=" << framework::build_features::enable_history
        << " empty_size=" << sizeof(disabled) << " owner_size=" << sizeof(disabled_owner)
        << " history_size=" << sizeof(full) << '\n';
}
