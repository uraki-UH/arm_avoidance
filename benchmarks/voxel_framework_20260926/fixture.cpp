#include <ais_gng/grasp_attention.hpp>
#include <ais_gng/boundary_attention.hpp>
#include "cpu/cugng.hpp"
#include <array>
#include <cstddef>

// 変更前後で同一の把持・境界規則。ROS通信を介さない実GNGへの登録。
extern "C" bool verification_set_rules() {
    static const auto regions = [] {
        ais_gng_msgs::msg::TopologicalMap map;
        map.nodes.resize(2); map.nodes[0].id = 0; map.nodes[1].id = 1;
        map.clusters.resize(1); map.clusters[0].nodes = {0, 1};
        fuzzrobo::grasp_attention::regions regions;
        regions.assign(map, {{-20, -20, -2}, {20, 20, 4}}, 0);
        return regions;
    }();
#ifdef verification_builtin
    const auto &boxes = regions.boxes();
    static const Vec3 anchors[] = {{0, 0, 0}, {5, 5, 0}, {10, 0, 0}, {15, -5, 0}};
    gng_builtin_sampling_input input;
    input.grasp_boxes = boxes.data(); input.num_grasp_boxes = boxes.size(); input.grasp_ratio = .3;
    input.boundary_points = anchors; input.num_boundary_points = 4;
    input.boundary_radius = 3; input.boundary_ratio = .2;
    return gng_set_builtin_sampling(&input);
#else
    static const fuzzrobo::boundary_attention::sampling_data boundary(
        {{0, 0, 0}, {5, 5, 0}, {10, 0, 0}, {15, -5, 0}}, 3);
    const std::array rules{
        fuzzrobo::grasp_attention::sampling_rule(1, .3, regions),
        fuzzrobo::boundary_attention::sampling_rule(2, .2, boundary)};
    return gng_set_sampling_rules(rules.data(), rules.size());
#endif
}

extern "C" std::size_t verification_gng_size() {return sizeof(CUGNG);}
extern "C" std::size_t verification_sampler_size() {return sizeof(gng_sampling::frame_sampler);}

#ifdef verification_enable_framework
// 最適化後の機械語比較。無効経路の属性型・集計・更新・ファジィ評価の定義なし。
struct codegen_policy {
    double baseline(double input) const {return input * input + 2;}
    void collect(double) = delete;
    void fuzzy(double) = delete;
};
extern "C" __attribute__((noinline)) double verification_baseline(double value) {
    return codegen_policy{}.baseline(value);
}
extern "C" __attribute__((noinline)) double verification_disabled(double value) {
    codegen_policy rules;
    fuzzrobo::voxel_framework::pipeline<fuzzrobo::voxel_framework::features<false>, codegen_policy> pipeline;
    pipeline.begin_frame(1, 1, 100);
    const auto result = pipeline.evaluate(value, rules);
    pipeline.end_frame();
    return result;
}
#endif
