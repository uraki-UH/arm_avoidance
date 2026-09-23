#include "../src/cpu/cugng.hpp"
#include <iostream>
#include <stdexcept>

void require(bool is_valid, const char *message) {
    if (!is_valid) {throw std::runtime_error(message);}
}

int main() {
    try {
        for (const float cell_size : {0.001f, 0.1f, 1.0f, 10.0f}) {
            Param params;
            params.node.num_max = 128;
            params.config.node_grid = cell_size;
            params.config.voxel_grid_unit = 0.1f;
            params.config.x_min = params.config.y_min = -200;
            params.config.x_max = params.config.y_max = 200;
            params.config.z_min = -5;
            params.config.z_max = 10;
            CUGNG graph;
            require(graph.init(&params.node, &params.edge, &params.config), "広域・極小node.gridでの初期化");
            for (uint32_t idx = 0; idx < 64; ++idx) {
                Vec3f point(0.1f + idx * 0.0001f, 0.1f, 0.1f);
                require(graph.add_node(point) == idx, "同一領域への10個を超える追加");
            }
            for (uint32_t idx = 0; idx < 64; ++idx) {
                Vec3f point(1.1f + idx * 0.0001f, 1.1f, 1.1f);
                graph.move_node(graph.nodes[idx], point);
                require(graph.nodes[idx].pos.p[0] == point.p[0], "高密度領域への移動拒否なし");
            }
            Vec3f query(1.1032f, 1.1f, 1.1f);
            Node_d winners;
            graph.getMinGrid(query, winners);
            require(winners.id1 == 32 && winners.id2 != NODE_NOID, "移動後の最近傍一致");
            graph.delete_node(32);
            graph.getMinGrid(query, winners);
            require(winners.id1 != 32 && winners.id2 != 32, "索引からの削除反映");
            require(graph.add_node(query) == 32, "削除IDの再利用");
        }
        std::cout << "グリッド非依存・密集ノード・移動・削除の検証成功\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
