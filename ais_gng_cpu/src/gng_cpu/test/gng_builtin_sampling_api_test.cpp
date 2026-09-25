#include <fuzzrobo/libgng/api.h>
#include <algorithm>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
#include <vector>

void require(bool is_valid, const char *message) {if (!is_valid) {throw std::runtime_error(message);}}

int main() {
    // 自己申告フラグではなく、ロード済み共有ライブラリの公開入口の照合。
    for (const auto *name : {"gng_set_sampling_rules", "gng_set_priority_input", "gng_set_weighted_priority_input"}) {
        require((dlsym(RTLD_DEFAULT, name) != nullptr) == bool(allow_external_sampler_build), "外部APIの配布構成");
    }
    require(!gng_setParameter("allow_external_sampler", 0, 1), "実行時の権限変更拒否");
    gng_builtin_sampling_input settings;
    require(!gng_set_builtin_sampling(&settings), "初期化前の設定拒否");
    gng_setParameter("node.num_max", 0, 1024);
    gng_setParameter("input.point_cloud_num", 0, 5000);
    gng_setParameter("node.learning_num", 0, 1000);
    require(gng_init() == SUCCESS, "GNG初期化");
    std::vector<Vec3> points;
    for (int x = 0; x < 25; ++x) for (int y = 0; y < 25; ++y) {
        points.push_back({.5f + x * .03f, -.4f + y * .03f, .2f});
    }
    LiDAR_Config input; input.point_step = sizeof(Vec3);
    const auto submit = [&] {gng_setPointCloud(reinterpret_cast<const uint8_t *>(points.data()), points.size(), &input);};
    for (int frame = 0; frame < 5; ++frame) {submit(); gng_exec();}
    gng_sampling_box box{{.65, -.25, .19}, {.85, .1, .21}};
    Vec3 anchors[] = {{.5f, -.4f, .2f}, {1.1f, .15f, .2f}};
    settings.grasp_boxes = &box; settings.num_grasp_boxes = 1; settings.grasp_ratio = .3;
    settings.boundary_points = anchors; settings.num_boundary_points = 2;
    settings.boundary_radius = .09; settings.boundary_ratio = .2;
    std::vector<uint32_t> expected_grasp, expected_boundary;
    for (uint32_t idx = 0; idx < points.size(); ++idx) {
        const auto &p = points[idx];
        if (p.x >= box.min_pos[0] && p.x <= box.max_pos[0] && p.y >= box.min_pos[1] && p.y <= box.max_pos[1] &&
            p.z >= box.min_pos[2] && p.z <= box.max_pos[2]) {expected_grasp.push_back(idx);}
        for (const auto &anchor : anchors) {
            const double x = double(p.x) - anchor.x, y = double(p.y) - anchor.y, z = double(p.z) - anchor.z;
            if (x*x + y*y + z*z <= settings.boundary_radius * settings.boundary_radius) {
                expected_boundary.push_back(idx); break;
            }
        }
    }
    const auto check_candidates = [](uint32_t id, const std::vector<uint32_t> &expected) {
        uint32_t num = 0;
        const auto *ids = gng_get_sampling_points(id, &num);
        require(num == expected.size() && std::equal(expected.begin(), expected.end(), ids), "組込み候補と独立抽出の一致");
    };
    submit(); require(gng_set_builtin_sampling(&settings), "組込み条件の登録");
    const auto saved_box = box; const auto saved_anchor = anchors[0];
    box.max_pos[0] = -100; anchors[0].x = NAN;
    gng_exec();
    require(gng_get_sampling_stats().num_priority_samples == 500, "固定学習枠内の混合配分");
    require(!gng_get_sampling_stats().has_invalid_score && gng_getTopologicalMap().node_num > 0, "学習出力");
    check_candidates(1, expected_grasp); check_candidates(2, expected_boundary);
    box = saved_box; anchors[0] = saved_anchor;
    gng_exec(); require(gng_get_sampling_stats().num_priority_samples == 0, "再実行時の失効");
    submit(); require(gng_set_builtin_sampling(&settings), "再登録"); submit(); gng_exec();
    require(gng_get_sampling_stats().num_priority_samples == 0, "入力置換時の失効");
    submit(); require(gng_set_builtin_sampling(&settings), "再登録");
    require(gng_set_builtin_sampling(nullptr), "明示解除"); gng_exec();
    require(gng_get_sampling_stats().num_priority_samples == 0, "解除後の通常学習");
    const auto reject = [&](const gng_builtin_sampling_input &invalid) {
        submit(); require(gng_set_builtin_sampling(&settings), "正常条件の事前登録");
        require(!gng_set_builtin_sampling(&invalid), "不正条件の拒否");
        gng_exec(); require(gng_get_sampling_stats().num_priority_samples == 0, "不正条件での旧指定失効");
    };
    for (double ratio : {-1., 1., double(INFINITY), double(NAN)}) {
        auto invalid = settings; invalid.grasp_ratio = ratio; reject(invalid);
    }
    for (double radius : {0., -1., 1e-300, 1e300, double(NAN)}) {
        auto invalid = settings; invalid.boundary_radius = radius; reject(invalid);
    }
    auto invalid = settings; invalid.grasp_ratio = .8; reject(invalid);
    invalid = settings; invalid.grasp_boxes = nullptr; reject(invalid);
    invalid = settings; invalid.boundary_points = nullptr; reject(invalid);
    invalid = settings; invalid.num_grasp_boxes = 1025; reject(invalid);
    invalid = settings; invalid.num_boundary_points = 1025; reject(invalid);
    auto invalid_box = box; invalid_box.min_pos[0] = NAN;
    invalid = settings; invalid.grasp_boxes = &invalid_box; reject(invalid);
    invalid_box.min_pos[0] = 100; reject(invalid);
    Vec3 invalid_point{NAN, 0, 0}; invalid = settings;
    invalid.boundary_points = &invalid_point; invalid.num_boundary_points = 1; reject(invalid);
    std::cout << "gng_builtin_sampling_api_test=passed external=" << allow_external_sampler_build << '\n';
}
