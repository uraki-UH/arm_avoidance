from pathlib import Path
import json
import shutil
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
shutil.copytree(source, target)
names = ["learn_search", "attention_search", "bsp_search", "node_add", "node_move",
         "node_delete", "tree_add", "tree_move", "tree_remove", "connect", "disconnect",
         "downsampling", "input_shuffle", "attention_sort", "voxel_sort",
         "edge_length_check", "node_age_check", "isolated_node_check", "edge_dist_update",
         "learn_sample"]
(target / "schema.json").write_text(json.dumps({
    "functions": names, "stages": ["voxel", "attention", "learn", "label", "maintenance", "cluster"]}, indent=2))
header = r"""#pragma once
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

// 一時コピー専用の計測。詳細タイマーの無効化による計測負荷の比較。
namespace bsp_profile {
using clock = std::chrono::steady_clock;
inline const bool enable_detail = [] {
    const char *value = std::getenv("GNG_PROFILE_DETAIL");
    return value != nullptr && value[0] == '1';
}();
inline const uint64_t sample_stride = [] {
    const char *value = std::getenv("GNG_PROFILE_SAMPLE_STRIDE");
    return value != nullptr ? std::max<uint64_t>(1, std::strtoull(value, nullptr, 10)) : 1;
}();
inline std::array<double, 20> time_ns{};
inline std::array<uint64_t, 20> num_calls{};
inline std::array<uint64_t, 20> num_samples{};
inline uint64_t frame_seq = 0;
inline uint64_t num_node_adds = 0;
inline uint64_t num_free_slot_checks = 0;
inline uint64_t num_add_reject_cap = 0;
inline void add(int idx, clock::time_point start) {
    time_ns[idx] += std::chrono::duration<double, std::nano>(clock::now() - start).count();
    ++num_samples[idx];
}
struct scope {
    int idx;
    clock::time_point start{};
    bool is_sampled = false;
    explicit scope(int value) : idx(value) {
        if (!enable_detail) {return;}
        ++num_calls[idx];
        // 高頻度の探索・接続・移動だけの間引き計時。GNGの入力点・処理回数への影響なし。
        const bool has_high_frequency = idx <= 2 || idx == 4 || idx == 7 || idx == 9 || idx == 10 || idx == 19;
        is_sampled = !has_high_frequency || (num_calls[idx] + frame_seq) % sample_stride == 0;
        if (is_sampled) {start = clock::now();}
    }
    ~scope() {if (is_sampled) {add(idx, start);}}
};
inline void reset() {
    time_ns.fill(0);
    num_calls.fill(0);
    num_samples.fill(0);
    ++frame_seq;
    num_node_adds = num_free_slot_checks = num_add_reject_cap = 0;
}
inline void print(const std::array<double, 6> &stages, int num_input, int num_voxel, int num_attention) {
    std::fprintf(stderr, "BSP_PROFILE {\"time_ms\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", (num_samples[idx] ? time_ns[idx] * num_calls[idx] / num_samples[idx] : 0) / 1e6);
    }
    std::fprintf(stderr, "],\"num_calls\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "", static_cast<unsigned long long>(num_calls[idx]));
    }
    std::fprintf(stderr, "],\"num_samples\":[");
    for (int idx = 0; idx < 20; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "", static_cast<unsigned long long>(num_samples[idx]));
    }
    std::fprintf(stderr, "],\"stage_ms\":[");
    for (int idx = 0; idx < 6; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", stages[idx]);
    }
    std::fprintf(stderr, "],\"num_input\":%d,\"num_voxel\":%d,\"num_attention\":%d,"
        "\"num_node_adds\":%llu,\"num_free_slot_checks\":%llu,\"num_add_reject_cap\":%llu}\n",
        num_input, num_voxel, num_attention,
        static_cast<unsigned long long>(num_node_adds),
        static_cast<unsigned long long>(num_free_slot_checks),
        static_cast<unsigned long long>(num_add_reject_cap));
}
}
"""
(target / "src/cpu/bsp_profile.hpp").write_text(header)
path = target / "src/cpu/cugng.cpp"
text = path.read_text().replace('#include "cugng.hpp"', '#include "cugng.hpp"\n#include "bsp_profile.hpp"', 1)
for method, idx in [
    ("getMinGrid", 0), ("getDownSamplingGrid", 1), ("add_node", 3), ("move_node", 4),
    ("delete_node", 5), ("connect", 9), ("disconnect", 10), ("getDownSampling", 11),
    ("check_edge_distance", 15), ("check_age", 16), ("check_delete_no_edge_and_decay_eta", 17),
    ("calc_edge_distanceXY", 18), ("learn_normal", 19)]:
    matches = [line for line in text.splitlines() if f" CUGNG::{method}(" in line and line.endswith("{")]
    assert len(matches) == 1, (method, matches)
    line = matches[0]
    text = text.replace(line, line + f"\n    bsp_profile::scope profile_scope{{{idx}}};", 1)
for statement, idx in [
    ("spatial_index->add(&entry);", 6),
    ("spatial_index->updatePosition(&spatial_entries[node.id],\n        {new_pos.p[0], new_pos.p[1], new_pos.p[2]});", 7),
    ("spatial_index->remove(&spatial_entries[idx]);", 8),
    ("std::shuffle(point_order.begin(), point_order.end(), random);", 12),
]:
    assert statement in text
    text = text.replace(statement, "{ bsp_profile::scope profile_scope{" + str(idx) + "};\n    " + statement + "\n    }", 1)
statement = """    const int num_nearest = spatial_index->findNBest(
        {point.p[0], point.p[1], point.p[2]}, 2, nearest);"""
assert statement in text
text = text.replace(statement, """    int num_nearest;
    {
        bsp_profile::scope profile_scope{2};
        num_nearest = spatial_index->findNBest({point.p[0], point.p[1], point.p[2]}, 2, nearest);
    }""", 1)
text = text.replace("    if(node_num == node_num_max){", "    if(node_num == node_num_max){\n        ++bsp_profile::num_add_reject_cap;", 1)
text = text.replace("        if (nodes[i].id == NODE_NOID) {", "        if (nodes[i].id == NODE_NOID) {\n            ++bsp_profile::num_node_adds;\n            bsp_profile::num_free_slot_checks += i + 1;", 1)
path.write_text(text)
path = target / "src/cpu/gng.cpp"
text = path.read_text().replace('#include "gng.hpp"', '#include "gng.hpp"\n#include "bsp_profile.hpp"', 1)
for idx in range(7):
    text = text.replace(f"auto t{idx} = std::chrono::system_clock::now();", f"auto t{idx} = bsp_profile::clock::now();", 1)
text = text.replace("    auto t0 = bsp_profile::clock::now();", "    bsp_profile::reset();\n    auto t0 = bsp_profile::clock::now();", 1)
key = "    auto t6 = bsp_profile::clock::now();"
assert key in text
text = text.replace(key, key + """
    bsp_profile::print({
        std::chrono::duration<double, std::milli>(t1-t0).count(),
        std::chrono::duration<double, std::milli>(t2-t1).count(),
        std::chrono::duration<double, std::milli>(t3-t2).count(),
        std::chrono::duration<double, std::milli>(t4-t3).count(),
        std::chrono::duration<double, std::milli>(t5-t4).count(),
        std::chrono::duration<double, std::milli>(t6-t5).count()}, input_pcl_num, vg.filtered_pcl_num, attention_pcl_num);
""", 1)
statement = """    boost::sort::spreadsort::integer_sort(voxel2node_ids.data(),
    voxel2node_ids.data() + voxel2node_ids_num,
        [](const Voxel &voxel, unsigned offset) { return voxel.voxel_index >> offset; });"""
assert statement in text
text = text.replace(statement, "{ bsp_profile::scope profile_scope{13};\n" + statement + "\n}", 1)
path.write_text(text)
path = target / "src/cpu/voxel_grid.cpp"
text = path.read_text().replace('#include "voxel_grid.hpp"', '#include "voxel_grid.hpp"\n#include "bsp_profile.hpp"', 1)
statement = """    boost::sort::spreadsort::integer_sort(voxel_index.data(), voxel_index.data() + voxel_index_num,
        [](const Voxel &voxel, unsigned offset) { return voxel.voxel_index >> offset; });"""
assert statement in text
text = text.replace(statement, "{ bsp_profile::scope profile_scope{14};\n" + statement + "\n}", 1)
path.write_text(text)
