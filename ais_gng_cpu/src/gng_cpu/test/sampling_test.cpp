#include "cpu/sampling.hpp"
#include <iostream>
#include <stdexcept>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

void test_tracking_cells() {
    using cells_type = fuzzrobo::builtin_sampling::tracking_cells;
    cells_type cells;
    gng_tracking_sampling_input settings;
    settings.ratio = .25; settings.min_points = 2; settings.min_nonplane_nodes = 3;
    require(cells_type::is_valid(settings), "粗いセルの既定設定");
    for (double size : {0., -1., double(NAN), double(INFINITY)}) {
        auto invalid = settings; invalid.cell_size = size;
        require(!cells_type::is_valid(invalid), "不正なセル幅の拒否");
    }
    const float anchor[] = {.1f, .1f, .1f}, moved[] = {.7f, .1f, .1f}, far[] = {2.7f, .1f, .1f};
    cells.reset(settings);
    for (int idx = 0; idx < 3; ++idx) {cells.add_node(anchor, true);}
    for (int idx = 0; idx < 20; ++idx) {cells.add_point(moved); cells.add_point(far);}
    cells.evaluate();
    const auto rule = cells.sampling_rule();
    require(rule.point_score(moved, &cells) > 0 && rule.point_score(far, &cells) == 0,
        "入力がない旧位置の非平面支持とノード0の移動先の選別");
    require(cells.cells[1].num_points == 20 && cells.cells[1].num_nodes == 0,
        "点群とノードの独立集計");
    const float shifted[] = {.35f, .1f, .1f};
    cells.reset(settings);
    for (int idx = 0; idx < 3; ++idx) {cells.add_node(anchor, true);}
    for (int idx = 0; idx < 20; ++idx) {cells.add_point(shifted);}
    cells.evaluate();
    require(cells.sampling_rule().point_score(shifted, &cells) > 0,
        "ノード不足なしでも同一セル内の重心ずれによる重点化");
    cells.reset(settings);
    for (int idx = 0; idx < 3; ++idx) {cells.add_node(anchor, true);}
    for (int idx = 0; idx < 20; ++idx) {cells.add_point(anchor);}
    cells.evaluate();
    require(cells.sampling_rule().point_score(anchor, &cells) == 0, "密度と重心が十分なセルの抑制");
    cells.reset(settings); cells.add_node(anchor, true);
    for (int idx = 0; idx < 20; ++idx) {cells.add_point(moved);}
    cells.evaluate();
    require(cells.sampling_rule().point_score(moved, &cells) == 0, "散発的な非平面支持の除外");

    // 入力voxelより粗いセルが小さい場合も含む、境界を跨ぐ元点の厳密な抽選。
    GridConfig bounds{}; bounds.unit = 1;
    bounds.x_min = bounds.y_min = bounds.z_min = -4;
    bounds.x_max = bounds.y_max = bounds.z_max = 4;
    require(bounds.init(bounds), "境界試験の入力範囲");
    OtherConfig config{}; config.point_cloud_num = 64; config.voxel_grid_unit = 1;
    VoxelGrid grid; grid.init(&bounds, &config);
    const std::vector<Vec3f> source{{.1f,.1f,.1f}, {.7f,.1f,.1f}, {.7f,.1f,.1f},
        {2.7f,.1f,.1f}, {2.7f,.1f,.1f}, {-3.1f,.1f,.1f}, {9,0,0}};
    for (bool enable_voxels : {true, false}) {
        auto points = source; std::vector<uint8_t> labels(64);
        grid.enable_voxel_downsampling = enable_voxels;
        cells.reset(settings);
        for (int idx = 0; idx < 3; ++idx) {cells.add_node(anchor, true);}
        grid.tracking = &cells; grid.applyFilter(points, points.size(), labels);
        require(!grid.tracking, "一入力での集計設定の失効");
        uint32_t counted = 0; double sum = 0;
        for (const auto &value : cells.cells) {counted += value.num_points; sum += value.point_sum[0];}
        require(counted == 6 && std::abs(sum - (.1 + .7 + .7 + 2.7 + 2.7 - 3.1)) < 1e-6,
            "負座標・範囲外除外と元点による厳密な座標和");
        gng_sampling::frame_sampler sampler; sampler.rules = {cells.sampling_rule()};
        std::vector<Node> nodes;
        sampler.build(&grid, &points, nodes, {}, {}, 0);
        require(sampler.ratio == .25 && !sampler.stats.has_invalid_score, "共通抽選への配分");
        std::mt19937 random(5);
        for (int idx = 0; idx < 1000; ++idx) {
            const auto selected = sampler.sample(random, &grid);
            require(selected == 1 || selected == 2, "跨ぎセルの無関係な元点の除外");
        }
        sampler.finish_input();
        require(sampler.points_for(3, grid) == std::vector<uint32_t>({1,2}), "候補点群の厳密な元番号");
        if (!enable_voxels) {require(sampler.stats.num_point_evaluations == 0, "単一所属セルの追加点評価なし");}
    }
}

int main() {
    test_tracking_cells();
    {
        fuzzrobo::builtin_sampling::tracking_cells cells;
        gng_tracking_sampling_input config;
        config.mode = gng_tracking_sampling_mode::nearest_nonplane; config.ratio = .5;
        cells.reset(config); cells.nonplane_generations = {UINT32_MAX, 7};
        const auto rule = cells.sampling_rule();
        require(!rule.point_score && !rule.enable_cell_bounds && rule.enable_nearest, "軽量方式の追加空間集計なし");
        gng_sampling_cell cell; cell.num_points = 30; cell.node_id = 1; cell.node_frame = 7;
        require(rule.cell_score(cell, &cells).weight == 1, "非平面セルの元点均等重み");
        cell.node_frame = 8; require(rule.cell_score(cell, &cells).weight == 0, "軽量方式の世代不一致除外");
        cell.node_id = UINT32_MAX; require(rule.cell_score(cell, &cells).weight == 0, "未対応セルの除外");
        require(cells.cells.empty() && cells.fine_cells.empty(), "軽量方式の粗いセル未構築");
    }
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
