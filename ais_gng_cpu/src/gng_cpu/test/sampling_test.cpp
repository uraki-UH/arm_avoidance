#include "cpu/sampling.hpp"
#include <iostream>
#include <stdexcept>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    GridConfig grid_config{};
    grid_config.unit = 1;
    grid_config.x_min = grid_config.y_min = grid_config.z_min = -4;
    grid_config.x_max = grid_config.y_max = grid_config.z_max = 4;
    require(grid_config.init(grid_config), "グリッド初期化");
    OtherConfig config{};
    config.point_cloud_num = 32; config.voxel_grid_unit = 1;
    VoxelGrid grid;
    grid.init(&grid_config, &config);
    std::vector<Vec3f> points{{.1f, .1f, .1f}, {.2f, .2f, .2f}, {.3f, .3f, .3f}, {1.1f, .1f, .1f}, {9, 0, 0}};
    std::vector<uint8_t> labels(32);
    grid.applyFilter(points, points.size(), labels);
    std::vector<Node> nodes(1);
    nodes[0].id = 0; nodes[0].frame = 17; nodes[0].label = 3; nodes[0].pos = points[0];
    gng_sampling::frame_sampler sampler;
    gng_sampling_rule uniform;
    uniform.id = 7; uniform.ratio = .5;
    uniform.cell_score = [](const gng_sampling_cell &cell, const void *) {
        require(!cell.has_node_count, "未要求のノード数集計なし");
        return gng_sampling_score{1, 0};
    };
    sampler.rules = {uniform};
    sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.num_cells == 2 && sampler.entries.size() == 2, "セル共有と範囲外除外");
    require(sampler.entries[0].mass == 3 && sampler.entries[1].mass == 1, "元点均等抽選の質量");
    std::mt19937 random(41);
    uint32_t first_cell = 0;
    for (uint32_t iter = 0; iter < 100000; ++iter) {first_cell += sampler.sample(random, &grid) < 3;}
    require(std::abs(first_cell / 100000.0 - .75) < .008, "点数差による抽選分布");
    sampler.finish_input();
    require(sampler.points_for(7, grid) == std::vector<uint32_t>({0, 1, 2, 3}), "確認用点群の展開");
    sampler.reset_input();
    require(sampler.points_for(7, grid).empty(), "確認用点群の失効");

    auto cell_uniform = uniform;
    cell_uniform.cell_score = [](const gng_sampling_cell &cell, const void *) {
        return gng_sampling_score{1.0 / cell.num_points, 0};
    };
    sampler.rules = {cell_uniform};
    sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.entries[0].mass == 1 && sampler.entries[1].mass == 1, "セル均等と元点均等の明示的区別");

    // 拡張点検証用の属性表。非平面や小平面の実運用判定ではない。
    auto attribute_rule = uniform;
    attribute_rule.id = 21; attribute_rule.ratio = .25;
    attribute_rule.cell_score = [](const gng_sampling_cell &cell, const void *) {
        return gng_sampling_score{cell.node_id == 0 && cell.node_frame == 17 && cell.node_label == 3 ? 2.0 : 0.0, 0};
    };
    auto density_rule = uniform;
    density_rule.id = 82; density_rule.ratio = .25; density_rule.enable_node_counts = true;
    density_rule.cell_score = [](const gng_sampling_cell &cell, const void *) {
        require(cell.has_node_count && cell.has_volume, "要求時だけの実セル内ノード数");
        require(cell.num_nodes == (cell.idx == 0 ? 1U : 0U), "入力セルと同じ範囲でのノード集計");
        return gng_sampling_score{cell.num_nodes == 0 ? 1.0 : 0.0, 0};
    };
    sampler.matches = {{0, .03f}, {UINT32_MAX, 1.f}};
    sampler.rules = {attribute_rule, density_rule};
    sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.num_point_evaluations == 0 && sampler.entries.size() == 2 && sampler.ratio == .5,
        "属性・密度条件の追加による全元点走査なし");
    first_cell = 0;
    for (uint32_t iter = 0; iter < 100000; ++iter) {first_cell += sampler.sample(random, &grid) < 3;}
    require(std::abs(first_cell / 100000.0 - .5) < .008, "群別配分の正規化");

    auto refined = uniform;
    refined.cell_score = [](const gng_sampling_cell &cell, const void *) {
        return gng_sampling_score{cell.idx == 0 ? 1.0 : 0.0, 1};
    };
    refined.point_score = [](const float *point, const void *) {return point[0] > .15 ? 2.0 : 0.0;};
    sampler.rules = {refined};
    sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.num_point_evaluations == 3 && sampler.entries.size() == 2, "関係セルだけの詳細判定");
    require(sampler.entries[0].raw_idx == 1 && sampler.entries[1].raw_idx == 2, "元点判定の保持");
    refined.point_score = [](const float *, const void *) {return -1.0;};
    sampler.rules = {refined}; sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.has_invalid_score && sampler.ratio == 0 && sampler.entries.empty(), "負の評価値の除外");
    refined.cell_score = [](const gng_sampling_cell &, const void *) -> gng_sampling_score {throw std::runtime_error("試験例外");};
    sampler.rules = {refined}; sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.has_invalid_score && sampler.ratio == 0, "評価例外からの通常学習への復帰");
    refined.cell_score = [](const gng_sampling_cell &, const void *) {return gng_sampling_score{NAN, 0};};
    sampler.rules = {refined}; sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.has_invalid_score && sampler.entries.empty(), "非有限スコアの拒否");
    sampler.reset_input(); sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.stats.num_cells == 0 && sampler.entries.empty(), "全規則OFFの追加走査なし");

    for (const double ratio : {0.0, .2, .5, .7}) {
        int actual_iter = 0, expected_iter = 0;
        for (int iter = 0; iter < 4000; ++iter) {
            auto expected = gng_sampling::source::unknown;
            if (int((iter + 1) * ratio) > int(iter * ratio)) {expected = gng_sampling::source::priority;}
            else if (++expected_iter == 5) {expected = gng_sampling::source::regular; expected_iter = 0;}
            require(gng_sampling::select_source(iter, actual_iter, ratio, 10, 5) == expected, "既存配分順・学習回数の保持");
        }
    }
    grid.enable_voxel_downsampling = false;
    grid.applyFilter(points, points.size(), labels);
    sampler.rules = {uniform}; sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.entries.size() == 4 && sampler.stats.num_cells == 4, "voxel OFFの一対一入力対応");
    points.clear(); grid.applyFilter(points, 0, labels);
    sampler.build(&grid, &points, nodes, {}, {}, 0);
    require(sampler.ratio == 0 && sampler.entries.empty(), "空入力・候補消失時の配分返却");
    std::cout << "sampling_test=passed\n";
}
