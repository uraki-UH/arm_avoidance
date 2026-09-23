from pathlib import Path
import shutil
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
shutil.copytree(source, target)
header = r'''#pragma once
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>

// 一時コピー専用の処理別計測。各フレームで集計を初期化。
namespace spatial_profile {
using clock = std::chrono::steady_clock;
inline std::array<double, 9> time_ns{};
inline std::array<uint64_t, 9> num_calls{};
inline uint64_t num_candidates = 0;
inline void add(int idx, clock::time_point start) {
    time_ns[idx] += std::chrono::duration<double, std::nano>(clock::now() - start).count();
    ++num_calls[idx];
}
struct scope {
    int idx;
    clock::time_point start = clock::now();
    ~scope() {add(idx, start);}
};
inline void reset() {
    time_ns.fill(0);
    num_calls.fill(0);
    num_candidates = 0;
}
inline void print(const std::array<double, 6> &stages) {
    std::fprintf(stderr, "GNG_PROFILE {\"time_ms\":[");
    for (int idx = 0; idx < 9; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", time_ns[idx] / 1e6);
    }
    std::fprintf(stderr, "],\"num_calls\":[");
    for (int idx = 0; idx < 9; ++idx) {
        std::fprintf(stderr, "%s%llu", idx ? "," : "",
            static_cast<unsigned long long>(num_calls[idx]));
    }
    std::fprintf(stderr, "],\"num_candidates\":%llu,\"stage_ms\":[",
        static_cast<unsigned long long>(num_candidates));
    for (int idx = 0; idx < 6; ++idx) {
        std::fprintf(stderr, "%s%.6f", idx ? "," : "", stages[idx]);
    }
    std::fprintf(stderr, "]}\n");
}
}
'''
(target / "src/cpu/spatial_profile.hpp").write_text(header)
for name in ["cugng.cpp", "cugng_grid_reference.cpp"]:
    path = target / "src/cpu" / name
    text = path.read_text().replace('#include "cugng.hpp"', '#include "cugng.hpp"\n#include "spatial_profile.hpp"', 1)
    for signature, idx in [
        ("bool CUGNG::getMinGrid(Vec3f& p, Node_d& n){", 0),
        ("bool CUGNG::getDownSamplingGrid(Vec3f& p, uint8_t& label, Node_d &n){", 1),
    ]:
        assert signature in text
        text = text.replace(signature, signature + f"\n    spatial_profile::scope profile_scope{{{idx}}};", 1)
    if name == "cugng.cpp":
        start = text.index("bool CUGNG::query_spatial")
        body = text[start:]
        body = body.replace("{\n    const float origin", "{\n    const auto prepare_start = spatial_profile::clock::now();\n    const float origin", 1)
        body = body.replace("    spatial_candidates.clear();", "    spatial_profile::add(2, prepare_start);\n    const auto search_start = spatial_profile::clock::now();\n    spatial_candidates.clear();", 1)
        body = body.replace("    // 同距離候補", "    spatial_profile::add(3, search_start);\n    spatial_profile::num_candidates += spatial_candidates.size();\n    const auto sort_start = spatial_profile::clock::now();\n    // 同距離候補", 1)
        body = body.replace("    winners.id1 = winners.id2", "    spatial_profile::add(4, sort_start);\n    const auto evaluate_start = spatial_profile::clock::now();\n    winners.id1 = winners.id2", 1)
        body = body.replace("    return is_in_vigilance;", "    spatial_profile::add(5, evaluate_start);\n    return is_in_vigilance;", 1)
        text = text[:start] + body
        for statement, idx in [
            ("spatial_index->add(&entry);", 6),
            ("spatial_index->updatePosition(&spatial_entries[node.id],\n        {new_pos.p[0], new_pos.p[1], new_pos.p[2]});", 7),
            ("spatial_index->remove(&spatial_entries[idx]);", 8),
        ]:
            assert statement in text
            text = text.replace(statement, "{ spatial_profile::scope profile_scope{" + str(idx) + "};\n    " + statement + "\n    }", 1)
    path.write_text(text)
path = target / "src/cpu/gng.cpp"
text = path.read_text().replace('#include "gng.hpp"', '#include "gng.hpp"\n#include "spatial_profile.hpp"', 1)
text = text.replace("    auto t0 = std::chrono::system_clock::now();", "    spatial_profile::reset();\n    auto t0 = std::chrono::system_clock::now();", 1)
key = "    auto t6 = std::chrono::system_clock::now();"
text = text.replace(key, key + """
    spatial_profile::print({
        std::chrono::duration<double, std::milli>(t1-t0).count(),
        std::chrono::duration<double, std::milli>(t2-t1).count(),
        std::chrono::duration<double, std::milli>(t3-t2).count(),
        std::chrono::duration<double, std::milli>(t4-t3).count(),
        std::chrono::duration<double, std::milli>(t5-t4).count(),
        std::chrono::duration<double, std::milli>(t6-t5).count()});
""", 1)
path.write_text(text)
