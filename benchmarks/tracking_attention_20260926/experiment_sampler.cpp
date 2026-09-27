#include <fuzzrobo/libgng/api.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// 比較専用の外部評価器。本番ライブラリ・ROS設定への組込みなし。
namespace {
std::vector<uint32_t> generations;
int mode = 0;
double min_dist_sq = .0025;
double max_dist_sq = .04;
struct cell_record {std::array<double,3> key; uint32_t count;};
std::vector<cell_record> previous_cells, current_cells;
std::size_t previous_pos = 0;
struct attention_cell {std::array<double,3> key; uint32_t age;};
std::vector<attention_cell> previous_attention, current_attention;
std::size_t attention_pos = 0;
uint32_t hold_frames = 0;
double retained_mass = 0, total_mass = 0;

void carry_attention() {
    const auto &cell = previous_attention[attention_pos++];
    if (cell.age < hold_frames) {current_attention.push_back({cell.key, cell.age+1});}
}

gng_sampling_score score(const gng_sampling_cell &cell, const void *) {
    const bool is_nonplane = cell.node_id < generations.size() &&
        generations[cell.node_id] != UINT32_MAX && generations[cell.node_id] == cell.node_frame;
    if (mode == 8) {
        if (!cell.num_points) {return {};}
        if (!hold_frames) {return {is_nonplane ? 1. : 0., 0};}
        if (!cell.has_volume) {return {};}
        const std::array<double,3> key{cell.min_pos[2], cell.min_pos[1], cell.min_pos[0]};
        while (attention_pos < previous_attention.size() && previous_attention[attention_pos].key < key) {
            carry_attention();
        }
        uint32_t age = hold_frames+1;
        if (attention_pos < previous_attention.size() && previous_attention[attention_pos].key == key) {
            age = previous_attention[attention_pos++].age+1;
        }
        if (is_nonplane) {age = 0;}
        if (age > hold_frames) {return {};}
        current_attention.push_back({key, age});
        const double weight = 1.-double(age)/(hold_frames+1);
        total_mass += weight*cell.num_points;
        if (age) {retained_mass += weight*cell.num_points;}
        // 入力点のあるセルだけの抽選。未観測セルは期限付きメタ情報のみ。
        return {weight, 0};
    }
    if (mode >= 5) {
        if (!cell.has_volume || !cell.num_points) {return {};}
        // 既存voxelの整列順。XYZ元点走査・追加ソート・ハッシュなし。
        const std::array<double,3> key{cell.min_pos[2], cell.min_pos[1], cell.min_pos[0]};
        current_cells.push_back({key, cell.num_points});
        while (previous_pos < previous_cells.size() && previous_cells[previous_pos].key < key) {++previous_pos;}
        if (previous_cells.empty() || (mode == 7 && !is_nonplane)) {return {};}
        const double count = previous_pos < previous_cells.size() && previous_cells[previous_pos].key == key ?
            previous_cells[previous_pos].count : 0;
        const double change = std::abs(double(cell.num_points)-count) / std::max(double(cell.num_points), count);
        return {mode == 6 ? change/cell.num_points : change, 0};
    }
    if (mode != 4 && !is_nonplane) {return {};}
    if (!cell.num_points) {return {};}
    if (mode == 1) {return {1, 0};}
    if (mode == 2) {return {1. / cell.num_points, 0};}
    if (!std::isfinite(cell.nearest_dist_sq)) {return {};}
    const double error = std::clamp((cell.nearest_dist_sq - min_dist_sq) / (max_dist_sq - min_dist_sq), 0., 1.);
    return {error, 0};
}
}

extern "C" int configure_retention(uint32_t frames) {
    if (frames > 1000) {return 0;}
    hold_frames = frames;
    previous_attention.clear(); current_attention.clear(); attention_pos = 0;
    retained_mass = total_mass = 0;
    return 1;
}

extern "C" double retained_mass_ratio() {return total_mass ? retained_mass/total_mass : 0.;}

extern "C" int configure_experiment(uint8_t (*register_rules)(const gng_sampling_rule *, uint32_t),
        const gng_sampling_node_ref *refs, uint32_t num_refs, int selected_mode, double ratio,
        double min_dist, double max_dist) {
    if (!register_rules || !std::isfinite(ratio) || ratio <= 0 || ratio >= 1 ||
        !std::isfinite(min_dist) || !std::isfinite(max_dist) || min_dist < 0 || max_dist <= min_dist ||
        selected_mode < 1 || selected_mode > 8 || (num_refs && !refs)) {return 0;}
    mode = selected_mode; min_dist_sq = min_dist*min_dist; max_dist_sq = max_dist*max_dist;
    if (mode == 8) {
        while (attention_pos < previous_attention.size()) {carry_attention();}
        previous_attention.swap(current_attention); current_attention.clear(); attention_pos = 0;
        retained_mass = total_mass = 0;
    } else if (mode >= 5) {previous_cells.swap(current_cells); current_cells.clear(); previous_pos = 0;}
    uint32_t num = 0;
    for (uint32_t idx = 0; idx < num_refs; ++idx) {
        if (refs[idx].id > 1000000) {return 0;}
        num = std::max(num, refs[idx].id+1);
    }
    generations.assign(num, UINT32_MAX);
    for (uint32_t idx = 0; idx < num_refs; ++idx) {generations[refs[idx].id] = refs[idx].frame;}
    gng_sampling_rule rule;
    rule.id = 10; rule.ratio = ratio; rule.cell_score = score;
    rule.enable_cell_bounds = mode >= 5 && (mode != 8 || hold_frames);
    rule.enable_nearest = mode < 5 || mode >= 7;
    return register_rules(&rule, 1);
}

#ifdef EXPERIMENT_TEST
#include <cassert>
gng_sampling_rule test_rule;
uint8_t capture_rule(const gng_sampling_rule *rules, uint32_t count) {assert(count == 1); test_rule = *rules; return 1;}
int main() {
    const auto setup = [] {assert(configure_experiment(capture_rule, nullptr, 0, 5, .25, .05, .2));};
    gng_sampling_cell cell; cell.has_volume = 1; cell.num_points = 10;
    setup(); assert(test_rule.cell_score(cell, nullptr).weight == 0);
    setup(); assert(test_rule.cell_score(cell, nullptr).weight == 0);
    setup(); cell.num_points = 20; assert(test_rule.cell_score(cell, nullptr).weight == .5);
    cell.min_pos[0] = .1; assert(test_rule.cell_score(cell, nullptr).weight == 1);
    setup(); cell.min_pos[0] = 0; assert(test_rule.cell_score(cell, nullptr).weight == 0);
    cell.min_pos[0] = .1; assert(test_rule.cell_score(cell, nullptr).weight == 0);
    const gng_sampling_node_ref ref{4, 9};
    const auto attention_setup = [&](bool has_label) {
        assert(configure_experiment(capture_rule, has_label ? &ref : nullptr, has_label ? 1 : 0, 8, .5, .05, .2));
    };
    for (const uint32_t frames : {0U, 2U, 4U}) {
        assert(configure_retention(frames));
        cell = {}; cell.has_volume = 1; cell.num_points = 10; cell.node_id = 4; cell.node_frame = 9;
        attention_setup(true); assert(test_rule.cell_score(cell, nullptr).weight == 1);
        for (uint32_t age = 1; age <= frames+1; ++age) {
            attention_setup(false);
            assert(std::abs(test_rule.cell_score(cell, nullptr).weight-std::max(0., 1.-double(age)/(frames+1))) < 1e-12);
        }
        attention_setup(true); assert(test_rule.cell_score(cell, nullptr).weight == 1);
    }
    assert(configure_retention(2));
    attention_setup(true); assert(test_rule.cell_score(cell, nullptr).weight == 1);
    attention_setup(false); cell.num_points = 0; assert(test_rule.cell_score(cell, nullptr).weight == 0);
    attention_setup(false); cell.num_points = 10;
    assert(std::abs(test_rule.cell_score(cell, nullptr).weight-1./3) < 1e-12);
    attention_setup(false); assert(test_rule.cell_score(cell, nullptr).weight == 0);
    // 世代不一致・負座標・先頭と末尾の未観測セルの期限切れ。
    assert(configure_retention(2));
    attention_setup(true); cell.node_frame = 10; assert(test_rule.cell_score(cell, nullptr).weight == 0);
    cell.node_frame = 9; cell.min_pos[0] = -1; assert(test_rule.cell_score(cell, nullptr).weight == 1);
    cell.min_pos[0] = 1; assert(test_rule.cell_score(cell, nullptr).weight == 1);
    attention_setup(false); cell.min_pos[0] = 0; assert(test_rule.cell_score(cell, nullptr).weight == 0);
    attention_setup(false); cell.min_pos[0] = -1;
    assert(std::abs(test_rule.cell_score(cell, nullptr).weight-1./3) < 1e-12);
    cell.min_pos[0] = 1; assert(std::abs(test_rule.cell_score(cell, nullptr).weight-1./3) < 1e-12);
    attention_setup(false); attention_setup(false); attention_setup(false);
    assert(previous_attention.empty() && current_attention.empty());
}
#endif
