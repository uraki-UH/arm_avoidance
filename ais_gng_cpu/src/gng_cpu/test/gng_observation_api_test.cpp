#include <fuzzrobo/libgng/api.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

uint32_t supported_node_num(const std::vector<float> &expected) {
    const auto frame = gng_get_observation_frame();
    std::vector<gng_observation::ray_angles> rays;
    for (std::size_t idx = 0; idx < expected.size(); idx += 3) {
        gng_observation::ray_angles ray;
        if (gng_observation::can_quantize_ray(static_cast<double>(expected[idx]) - frame.origin.x,
            static_cast<double>(expected[idx + 1]) - frame.origin.y,
            static_cast<double>(expected[idx + 2]) - frame.origin.z, ray)) {rays.push_back(ray);}
    }
    const auto map = gng_getTopologicalMap();
    uint32_t num = 0;
    for (uint32_t idx = 0; idx < map.node_num; ++idx) {
        const auto support = gng_get_observation_angle_range(map.nodes[idx].id);
        if (!support.has_support) {continue;}
        require(frame.has_origin, "support without acquisition origin");
        require(std::any_of(rays.begin(), rays.end(), [&](const auto &ray) {return support.contains(ray);}),
            "range does not contain any measured ray");
        for (const auto endpoint : {support.min_pitch, support.max_pitch}) {
            require(std::any_of(rays.begin(), rays.end(), [&](const auto &ray) {return ray.pitch == endpoint;}),
                "pitch endpoint not derived from measured input");
        }
        if (support.has_yaw) {
            for (const auto endpoint : {support.min_yaw, support.max_yaw}) {
                require(std::any_of(rays.begin(), rays.end(), [&](const auto &ray) {return ray.has_yaw && ray.yaw == endpoint;}),
                    "yaw endpoint not derived from measured input");
            }
        }
        ++num;
    }
    return num;
}

// 入力構造体の組み立て補助。公開APIへの一括設定。
uint8_t set_input(Vec3 origin, const gng_observation::pixel_view *view = nullptr,
    const gng_observation::ray_angles *table = nullptr, uint32_t table_num = 0) {
    gng_observation_input input;
    input.origin = origin;
    input.has_origin = 1;
    if (view) {input.pixels = *view;}
    input.angle_table = table;
    input.table_num = table_num;
    return gng_set_observation_input(&input);
}

int main() {
    require(gng_setParameter("node.num_max", 0, 1024), "node configuration failed");
    require(gng_setParameter("input.point_cloud_num", 0, 100000), "input capacity configuration failed");
    require(gng_init() == SUCCESS, "initialization failed");
    require(gng_setParameter("node.enable_observation_support", 0, 1), "enable failed");
    require(!gng_setParameter("node.observation.max_point_num", 0, 4096), "legacy point cap accepted");
    require(!gng_setParameter("node.observation.max_cell_angle_deg", 0, 0.5f), "legacy grid accepted");
    require(!gng_setParameter("node.observation.max_block_num", 0, 256), "legacy blocks accepted");
    std::vector<float> points;
    for (int row = 0; row < 30; ++row) {
        for (int column = 0; column < 30; ++column) {
            points.insert(points.end(), {1.0f + 0.06f * column, -0.9f + 0.06f * row, 0.2f});
        }
    }
    LiDAR_Config config{};
    config.point_step = 3 * sizeof(float);
    const auto submit = [&] {
        gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size() / 3, &config);
    };
    submit();
    gng_exec();
    require(supported_node_num(points) == 0 && !gng_get_observation_frame().has_origin, "origin implicitly assumed");
    submit();
    set_input({0.434f, -0.693f, 0.279f});
    gng_exec();
    require(supported_node_num(points) > 0, "no measured points captured");
    const auto frame = gng_get_observation_frame();
    require(frame.has_origin && frame.origin.x == 0.434f && frame.origin.y == -0.693f &&
        frame.origin.z == 0.279f && frame.frame_number == gng_getTopologicalMap().frame_number,
        "output origin does not match acquisition");
    // 次入力用原点と直近出力フレームの分離。
    set_input({5, 6, 7});
    require(gng_get_observation_frame().origin.x == frame.origin.x, "output origin overwritten");
    submit();
    gng_exec();
    require(supported_node_num(points) == 0 && !gng_get_observation_frame().has_origin, "origin persisted across input replacement");
    submit();
    set_input({0, 0, 0});
    gng_exec();
    require(supported_node_num(points) > 0, "zero origin rejected");
    gng_exec();
    require(supported_node_num(points) == 0, "origin or points persisted across frames");
    submit();
    set_input({std::numeric_limits<float>::quiet_NaN(), 0, 0});
    gng_exec();
    require(supported_node_num(points) == 0, "invalid origin accepted");
    require(!gng_get_observation_angle_range(65535).has_support, "invalid node accepted");

    // 無効化・再有効化時の旧支持消去。
    require(gng_setParameter("node.enable_observation_support", 0, 0), "disable failed");
    require(supported_node_num(points) == 0 && !gng_get_observation_frame().has_origin, "disabled output retained");
    submit();
    set_input({0, 0, 0});
    gng_exec();
    require(supported_node_num(points) == 0 && !gng_get_observation_frame().has_origin, "disabled capture active");
    require(gng_setParameter("node.enable_observation_support", 0, 1), "enable failed");

    // 180度回転と並進後の点群。実測レイ由来の整数角度端点との照合。
    config.pos = {0.25f, 0.125f, 0.25f};
    config.quat = {0, 0, 1, 0};
    std::vector<float> expected;
    for (std::size_t idx = 0; idx < points.size(); idx += 3) {
        expected.insert(expected.end(), {-points[idx] + config.pos.x,
            -points[idx + 1] + config.pos.y, points[idx + 2] + config.pos.z});
    }
    submit();
    set_input(config.pos);
    gng_exec();
    require(supported_node_num(expected) > 0, "transformed measured points missing");
    require(gng_get_observation_frame().origin.x == config.pos.x, "transformed origin lost");
    gng_setPointCloud(nullptr, 0, &config);
    set_input(config.pos);
    gng_exec();
    require(supported_node_num(expected) == 0, "empty input retained points");
    // 画素表参照の直接検査。XYZからの方向とは異なる既知値による分岐の識別。
    std::vector<uint32_t> pixel_ids(points.size() / 3, 0);
    const gng_observation::ray_angles table[]{{1234, 2345, true}};
    gng_observation::pixel_view view;
    view.mode = gng_observation::pixel_view::format::pixel;
    view.data = reinterpret_cast<const uint8_t *>(pixel_ids.data());
    view.data_size = pixel_ids.size() * sizeof(uint32_t);
    view.width = pixel_ids.size(); view.height = 1; view.point_step = 4; view.row_step = view.data_size;
    view.point_num = pixel_ids.size(); view.image_width = 1; view.image_height = 1; view.first_size = 4;
    submit();
    set_input(config.pos);
    require(set_input(config.pos, &view, table, 1), "pixel setup failed");
    gng_exec();
    auto lookup = gng_get_observation_frame();
    require(lookup.pixel_hit_num > 0 && lookup.ray_num == 0, "pixel input used inverse ray calculation");
    const auto pixel_map = gng_getTopologicalMap();
    for (uint32_t idx = 0; idx < pixel_map.node_num; ++idx) {
        const auto range = gng_get_observation_angle_range(pixel_map.nodes[idx].id);
        if (!range.has_support) {continue;}
        require(range.min_yaw == 1234 && range.max_yaw == 1234 && range.min_pitch == 2345 && range.max_pitch == 2345,
            "pixel angle did not reach winner");
    }
    submit();
    set_input(config.pos);
    gng_exec();
    lookup = gng_get_observation_frame();
    require(lookup.pixel_hit_num == 0 && lookup.ray_num > 0, "pixel mapping retained across frames");
    require(supported_node_num(expected) > 0, "fallback ranges missing");
    submit();
    set_input(config.pos);
    --view.point_num;
    require(!set_input(config.pos, &view, table, 1), "short pixel input accepted");
    ++view.point_num;
    std::fill(pixel_ids.begin(), pixel_ids.end(), UINT32_MAX);
    require(set_input(config.pos, &view, table, 1), "fallback setup failed");
    gng_exec();
    lookup = gng_get_observation_frame();
    require(lookup.pixel_hit_num == 0 && lookup.ray_num > 0, "invalid pixels did not fall back");
    require(supported_node_num(expected) > 0, "invalid pixel fallback ranges missing");
    // 借用ビューの直接参照、次入力での失効、無効画素と短い配置の拒否。
    std::fill(pixel_ids.begin(), pixel_ids.end(), 0);
    submit(); set_input(config.pos);
    require(set_input(config.pos, &view, table, 1), "view setup failed");
    gng_exec();
    lookup = gng_get_observation_frame();
    require(lookup.pixel_hit_num > 0 && lookup.ray_num == 0, "view fallback");
    auto view_map = gng_getTopologicalMap();
    for (uint32_t idx = 0; idx < view_map.node_num; ++idx) {
        const auto range = gng_get_observation_angle_range(view_map.nodes[idx].id);
        if (range.has_support) {
            require(range.min_yaw == 1234 && range.max_yaw == 1234 && range.min_pitch == 2345 && range.max_pitch == 2345,
                "view angle mismatch");
        }
    }
    submit(); set_input(config.pos); gng_exec();
    require(gng_get_observation_frame().pixel_hit_num == 0, "view persisted");
    require(supported_node_num(expected) > 0, "view fallback support missing");
    submit(); set_input(config.pos);
    require(set_input(config.pos, &view, table, 1), "borrow setup failed");
    std::fill(pixel_ids.begin(), pixel_ids.end(), UINT32_MAX);
    gng_exec();
    lookup = gng_get_observation_frame();
    require(lookup.pixel_hit_num == 0 && lookup.ray_num > 0, "view copied instead of borrowed");
    submit(); set_input(config.pos);
    --view.data_size;
    require(!set_input(config.pos, &view, table, 1), "short view accepted");

    gng_exec();
    require(supported_node_num(expected) > 0, "invalid view fallback missing");
    // 直接配列と圧縮索引の境界、および同一GNG内でのフレーム間切替。
    const auto small_points = points;
    for (const uint32_t point_num : {65536U, 65537U, 900U, 65537U, 900U}) {
        points.resize(point_num * 3);
        for (uint32_t idx = 0; idx < point_num; ++idx) {
            std::copy_n(small_points.data() + (idx % (small_points.size()/3))*3, 3, points.data() + idx*3);
        }
        pixel_ids.assign(point_num, 0);
        view.data = reinterpret_cast<const uint8_t *>(pixel_ids.data());
        view.data_size = point_num * 4; view.width = point_num;
        view.row_step = point_num * 4; view.point_num = point_num;
        submit(); set_input(config.pos);
        require(set_input(config.pos, &view, table, 1), "switch view setup failed");
        gng_exec();
        lookup = gng_get_observation_frame();
        require(lookup.pixel_hit_num > 0 && lookup.ray_num == 0, "switch frame fallback");
        const auto switched = gng_getTopologicalMap();
        for (uint32_t idx = 0; idx < switched.node_num; ++idx) {
            const auto range = gng_get_observation_angle_range(switched.nodes[idx].id);
            if (range.has_support) {
                require(range.min_yaw == 1234 && range.max_yaw == 1234 && range.min_pitch == 2345 && range.max_pitch == 2345,
                    "switch frame stale range");
            }
        }
    }
    // 同一入力内の再設定と出力スナップショットの分離。
    submit();
    require(set_input(config.pos, &view, table, 1), "combined input rejected");
    gng_exec();
    const auto completed = gng_get_observation_frame();
    require(completed.pixel_hit_num > 0 && completed.ray_num == 0, "completed lookup missing");
    submit();
    require(set_input({5, 6, 7}, &view, table, 1), "next input rejected");
    const auto pending = gng_get_observation_frame();
    require(pending.origin.x == completed.origin.x && pending.frame_number == completed.frame_number &&
        pending.pixel_hit_num == completed.pixel_hit_num && pending.ray_num == completed.ray_num,
        "pending input overwrote completed frame");
    require(set_input(config.pos), "origin-only replacement rejected");
    gng_exec();
    require(gng_get_observation_frame().pixel_hit_num == 0 && gng_get_observation_frame().ray_num > 0,
        "origin-only replacement retained pixel view");
    require(supported_node_num(expected) > 0, "replacement origin mismatch");

    // 無効設定への置換後に旧ビューや旧原点が残存しないことの検査。
    for (const int mode : {0, 1, 2, 3, 4, 5}) {
        submit();
        require(set_input(config.pos, &view, table, 1), "valid replacement setup failed");
        gng_observation_input input;
        input.origin = config.pos;
        input.has_origin = 1;
        input.pixels = view;
        input.angle_table = table;
        input.table_num = 1;
        if (mode == 0) {input.has_origin = 0;}
        if (mode == 1) {input.origin.x = std::numeric_limits<float>::infinity();}
        if (mode == 2) {input.angle_table = nullptr;}
        if (mode == 3) {input.table_num = 0;}
        if (mode == 4) {--input.pixels.point_num;}
        require(!gng_set_observation_input(mode == 5 ? nullptr : &input), "invalid replacement accepted");
        gng_exec();
        const auto result = gng_get_observation_frame();
        require(result.pixel_hit_num == 0, "invalid replacement retained pixel mapping");
        if (mode == 0 || mode == 1 || mode == 5) {
            require(!result.has_origin && result.ray_num == 0 && supported_node_num(expected) == 0,
                "invalid origin retained observation");
        } else {
            require(result.has_origin && result.ray_num > 0 && supported_node_num(expected) > 0,
                "invalid lookup did not use current origin");
        }
    }
    std::cout << "gng_observation_api_test=passed\n";
}
