#include "cpu/labelling.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <random>

int main() {
    CUGNG core;
    Labelling labelling;
    NodeConfig node_config{};
    LabelConfig label_config{1.f, 0.2f, 0.5f};
    std::mt19937 random(20260924);
    std::uniform_real_distribution<float> coordinate(-10.f, 10.f);
    for (const uint32_t num : {257U, 31U, 509U}) {
        core.nodes.clear();
        core.nodes.resize(num);
        for (uint32_t idx = 0; idx < num; ++idx) {
            auto point = Vec3f(coordinate(random), coordinate(random), coordinate(random));
            auto &node = core.nodes[idx];
            node.init(idx, 0.1f, 0.05f, point);
            node.edge_num = idx % (NODE_MAX_EDGE + 1);
            for (uint32_t edge_idx = 0; edge_idx < node.edge_num; ++edge_idx) {
                node.edges[edge_idx] = (idx + 1 + edge_idx) % num;
            }
        }
        labelling.init(&node_config, &label_config, &core);
        for (uint32_t iter = 0; iter < 6; ++iter) {
            if (iter == 1) {
                for (uint32_t idx = 0; idx < num; idx += 3) {
                    core.nodes[idx].pos[0] += 0.13f;
                    core.nodes[idx].pos[2] -= 0.27f;
                }
            }
            if (iter == 2) {
                for (auto &node : core.nodes) {
                    std::reverse(node.edges, node.edges + node.edge_num);
                }
            }
            if (iter == 3) {
                // 欠番と、全入辺の除去後の再集約。
                core.nodes[17].id = NODE_NOID;
                for (auto &node : core.nodes) {
                    const auto end = std::remove(node.edges, node.edges + node.edge_num, 17U);
                    node.edge_num = end - node.edges;
                }
            }
            if (iter == 4) {
                // 同じIDへの別位置・別隣接関係の再割当。
                auto point = Vec3f(0.37f, 0.91f, 1.73f);
                core.nodes[17].init(17, 0.1f, 0.05f, point);
                core.nodes[17].edge_num = 3;
                core.nodes[17].edges[0] = 18;
                core.nodes[17].edges[1] = 19;
                core.nodes[17].edges[2] = 20;
            }
            // 従来のノード直接参照を基準とする、全法線確定後の曲率照合。
            for (auto &node : core.nodes) {
                if (node.id != NODE_NOID) {core.normal_vector(node);}
            }
            for (auto &node : core.nodes) {
                if (node.id == NODE_NOID) {continue;}
                if (node.edge_num >= 2) {core.rho(node);}
                else {node.rho = 0;}
            }
            const auto expected = core.nodes;
            for (auto &node : core.nodes) {
                node.normal = Vec3f(2, 3, 4);
                node.rho = -1;
            }
            labelling.labelling_fuzzy();
            for (uint32_t idx = 0; idx < num; ++idx) {
                const auto &node = core.nodes[idx];
                if (node.id == NODE_NOID) {continue;}
                if (std::memcmp(node.normal.p, expected[idx].normal.p, sizeof(node.normal.p)) != 0 ||
                    std::memcmp(&node.rho, &expected[idx].rho, sizeof(node.rho)) != 0) {
                    std::cerr << "normal_cache_mismatch " << num << ' ' << iter << ' ' << idx << '\n';
                    return 1;
                }
            }
            if (iter == 0) {
                // 幾何形状が同じ場合にも継続するLPF更新。
                const auto previous = core.nodes;
                labelling.labelling_fuzzy();
                bool has_fuzzy_change = false;
                for (uint32_t idx = 0; idx < num; ++idx) {
                    has_fuzzy_change |= std::memcmp(core.nodes[idx].fuzzy_exp, previous[idx].fuzzy_exp,
                        sizeof(core.nodes[idx].fuzzy_exp)) != 0;
                }
                if (!has_fuzzy_change) {return 2;}
            }
        }
    }
    std::cout << "normal_cache_test=passed\n";
}
