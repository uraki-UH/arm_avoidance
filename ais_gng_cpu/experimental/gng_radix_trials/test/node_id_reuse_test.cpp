#include "cpu/cugng.hpp"
#include <iostream>
#include <random>

int main() {
    NodeConfig config{};
    config.num_max = 64;
    EdgeConfig edge{};
    OtherConfig input{};
    input.node_grid = input.voxel_grid_unit = 1;
    input.x_min = input.y_min = input.z_min = -10;
    input.x_max = input.y_max = input.z_max = 10;
    CUGNG gng;
    if (!gng.init(&config, &edge, &input)) {return 1;}
    auto has_expected_added_id = [&](Vec3f point) {
        uint32_t expected = NODE_NOID;
        const uint32_t cell_idx = gng.grid_config.getIndex(point);
        if (cell_idx < gng.grid_config.maxXYZ &&
            gng.grid_node_num[cell_idx] < NODE_GRID_NODE_NUM_NAX) {
            for (uint32_t idx = 0; idx < gng.nodes.size(); ++idx) {
                if (gng.nodes[idx].id == NODE_NOID) {expected = idx; break;}
            }
        }
        return gng.add_node(point) == expected;
    };
    // セル上限・範囲外の追加失敗後の、未使用IDの保持。
    for (int idx = 0; idx < NODE_GRID_NODE_NUM_NAX + 2; ++idx) {
        if (!has_expected_added_id(Vec3f(0, 0, 0))) {return 2;}
    }
    if (!has_expected_added_id(Vec3f(100, 100, 100))) {return 3;}
    // ノード上限到達後の、削除順と異なる最小空きIDの再利用。
    for (int idx = 0; idx < 64; ++idx) {
        if (!has_expected_added_id(Vec3f(idx % 8 - 4, idx / 8 - 4, 2))) {return 4;}
    }
    gng.delete_node(49);
    gng.delete_node(3);
    gng.delete_node(30);
    for (int idx = 0; idx < 5; ++idx) {
        if (!has_expected_added_id(Vec3f(5, 5, 5))) {return 5;}
    }
    // 有効・無効削除と追加の混在、全走査による参照結果との照合。
    std::mt19937 random(20260924);
    for (int iter = 0; iter < 20000; ++iter) {
        if (random() % 2 == 0) {gng.delete_node(random() % 70);}
        else if (!has_expected_added_id(Vec3f(int(random() % 16) - 8,
                     int(random() % 16) - 8, int(random() % 16) - 8))) {return 6;}
    }
    for (int iter = 0; iter < 2; ++iter) {
        if (iter) {gng.clear();}
        if (!gng.init(&config, &edge, &input) || !has_expected_added_id(Vec3f(1, 1, 1))) {return 7;}
        if (gng.nodes[0].id != 0 || gng.node_num != 1) {return 8;}
    }
    std::cout << "node_id_reuse_test=passed\n";
}
