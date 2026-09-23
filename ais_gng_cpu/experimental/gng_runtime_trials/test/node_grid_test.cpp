#include "../src/cpu/cugng.hpp"
#include <stdexcept>
#include <random>
void require(bool is_valid) {if (!is_valid) {throw std::runtime_error("グリッド更新・局所近傍の検証失敗");}}
int main() {
    Param params;
    params.node.num_max = 200;
    params.config.node_grid = 0.5f;
    params.config.x_min = params.config.y_min = params.config.z_min = -3;
    params.config.x_max = params.config.y_max = params.config.z_max = 3;
    CUGNG graph;
    require(graph.init(&params.node, &params.edge, &params.config));
    std::mt19937 random(31);
    std::uniform_real_distribution<float> select(-2.9f, 2.9f);
    for (int idx = 0; idx < 150; ++idx) {
        Vec3f pos(select(random), select(random), select(random)); graph.add_node(pos);
    }
    for (int iter = 0; iter < 400; ++iter) {
        const auto idx = random() % 150;
        Vec3f pos(select(random), select(random), select(random));
        if (iter % 3 == 0) {graph.delete_node(idx); graph.add_node(pos);}
        else {graph.move_node(graph.nodes[idx], pos);}
        Vec3f point(select(random), select(random), select(random));
        vector<float> expected;
        bool is_expected_vigilance = false;
        for (const auto &node : graph.nodes) {
            if (node.id == NODE_NOID) {continue;}
            bool is_local = true;
            for (int axis = 0; axis < 3; ++axis) {
                const int a = static_cast<int>((point.p[axis] + 3.5f) * 2);
                const int b = static_cast<int>((node.pos.p[axis] + 3.5f) * 2);
                is_local &= std::abs(a - b) <= 1;
            }
            if (!is_local) {continue;}
            const auto dist2 = point.squaredNorm(node.pos);
            expected.push_back(dist2);
            is_expected_vigilance |= dist2 < graph.gng_config.vigilance2[node.label];
        }
        std::sort(expected.begin(), expected.end());
        Node_d winners;
        require(graph.getMinGrid(point, winners) == is_expected_vigilance);
        require((winners.id1 != NODE_NOID) == !expected.empty());
        require((winners.id2 != NODE_NOID) == (expected.size() > 1));
        if (!expected.empty()) {require(std::abs(winners.id1_d2 - expected[0]) < 1e-5f);}
        if (expected.size() > 1) {require(std::abs(winners.id2_d2 - expected[1]) < 1e-5f);}
    }
    graph.clear(); require(graph.init(&params.node, &params.edge, &params.config));
    Vec3f near(0.1f,0.1f,0.1f), far(2,2,2);
    for (int idx = 0; idx < 10; ++idx) {require(graph.add_node(near) != NODE_NOID);}
    require(graph.add_node(near) == NODE_NOID);
    const auto far_idx = graph.add_node(far);
    graph.move_node(graph.nodes[far_idx], near);
    require(graph.nodes[far_idx].pos.p[0] == 2);
    graph.delete_node(3);
    graph.move_node(graph.nodes[far_idx], near);
    require(graph.nodes[far_idx].pos.p[0] == near.p[0]);
    Node_d winners; graph.getMinGrid(far, winners);
    require(winners.id1 == NODE_NOID);
}
