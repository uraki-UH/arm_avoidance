#include "cugng.hpp"
#ifdef GNG_USE_NODE_GRID
// 従来版と同じ27セル・1セル10ノード・遅延ページ確保。
std::array<uint32_t, CUGNG::max_nodes_per_cell> &CUGNG::get_cell(uint32_t idx) {
    auto &offset = page_offsets[idx / num_cells_per_page];
    if (offset == UINT32_MAX) {
        offset = cell_nodes.size();
        cell_nodes.resize(static_cast<size_t>(offset) + num_cells_per_page);
    }
    return cell_nodes[offset + idx % num_cells_per_page];
}
void CUGNG::remove_from_cell(Node &node) {
    auto &ids = get_cell(node.cell_idx);
    const auto last = --cell_counts[node.cell_idx];
    if (node.cell_slot != last) {
        nodes[ids[last]].cell_slot = node.cell_slot;
        ids[node.cell_slot] = ids[last];
    }
    ids[last] = NODE_NOID;
}
bool CUGNG::getMinGrid(Vec3f &point, Node_d &winners) {
    ++sampling_statistics.num_nearest_queries;
    winners = {NODE_NOID, FLT_MAX, NODE_NOID, FLT_MAX};
    int min_cell[3], max_cell[3];
    for (int axis = 0; axis < 3; ++axis) {
        const int center = static_cast<int>((point.p[axis] - node_cells.min_pos.p[axis]) * node_cells.inverse_unit);
        min_cell[axis] = std::max(0, center - 1);
        max_cell[axis] = std::min(static_cast<int>(node_cells.num_cells[axis]) - 1, center + 1);
    }
    bool is_in_vigilance = false;
    for (int x = min_cell[0]; x <= max_cell[0]; ++x)
        for (int y = min_cell[1]; y <= max_cell[1]; ++y)
            for (int z = min_cell[2]; z <= max_cell[2]; ++z) {
                const auto idx = x + y * node_cells.num_cells[0] + z * node_cells.num_xy;
                const auto num = cell_counts[idx];
                if (num == 0) {continue;}
                const auto &ids = cell_nodes[page_offsets[idx / num_cells_per_page] + idx % num_cells_per_page];
                for (uint32_t slot = 0; slot < num; ++slot) {
                    const auto id = ids[slot];
                    const auto &node = nodes[id];
                    const float dx = point.p[0] - node.pos.p[0];
                    const float dy = point.p[1] - node.pos.p[1];
                    const float dz = point.p[2] - node.pos.p[2];
                    const float dist2 = dx * dx + dy * dy + dz * dz;
                    if (dist2 < winners.id2_d2) {
                        if (dist2 < winners.id1_d2) {
                            winners.id2 = winners.id1; winners.id2_d2 = winners.id1_d2;
                            winners.id1 = id; winners.id1_d2 = dist2;
                        } else {winners.id2 = id; winners.id2_d2 = dist2;}
                    }
                    // 従来版と同じ、探索セル内の全候補に対する警戒領域判定。
                    is_in_vigilance |= dist2 < gng_config.vigilance2[node.label];
                }
            }
    return is_in_vigilance;
}
bool CUGNG::getDownSamplingGrid(Vec3f &point, uint8_t &label, Node_d &winners) {
    label = 0;
    return getMinGrid(point, winners);
}
void CUGNG::delete_node(uint32_t idx) {
    if (idx >= static_cast<uint32_t>(node_num_max) || node_num <= 2) {return;}
    auto &node = nodes[idx];
    if (node.id == NODE_NOID) {return;}
    recordNodeDelta(node, GNG_DELTA_REMOVE);
    remove_from_cell(node);
    --node_num; ++sampling_statistics.num_deleted_nodes;
    disconnect_all(idx);
    node.id = NODE_NOID;
    release_node_id(idx);
}
void CUGNG::move_node(Node &node, Vec3f &new_pos) {
    if (node.id == NODE_NOID || !is_input_in_range(new_pos)) {return;}
    const auto idx = node_cells.get_cell_idx(new_pos);
    if (idx == UINT32_MAX || (idx != node.cell_idx && cell_counts[idx] >= max_nodes_per_cell)) {return;}
    const bool is_position_changed = node.pos.p[0] != new_pos.p[0] ||
        node.pos.p[1] != new_pos.p[1] || node.pos.p[2] != new_pos.p[2];
    node.pos = new_pos;
    ++sampling_statistics.num_tree_moves;
    if (map_delta_capture_enabled && is_position_changed) {recordNodeDelta(node, GNG_DELTA_UPDATE);}
    if (idx == node.cell_idx) {return;}
    remove_from_cell(node);
    auto &ids = get_cell(idx);
    node.cell_idx = idx; node.cell_slot = cell_counts[idx]++;
    ids[node.cell_slot] = node.id;
}
uint32_t CUGNG::add_node(Vec3f &pos) {
    if (node_num == node_num_max || !is_input_in_range(pos)) {return NODE_NOID;}
    const auto cell_idx = node_cells.get_cell_idx(pos);
    if (cell_idx == UINT32_MAX || cell_counts[cell_idx] >= max_nodes_per_cell) {return NODE_NOID;}
    const auto idx = find_free_node();
    if (idx != NODE_NOID) {
        auto &node = nodes[idx];
        node.init(idx, gng_config.eta_s1, gng_config.eta_s2, pos); node.frame = frame_number;
        auto &ids = get_cell(cell_idx);
        node.cell_idx = cell_idx; node.cell_slot = cell_counts[cell_idx]++;
        ids[node.cell_slot] = idx;
        ++node_num; ++sampling_statistics.num_added_nodes;
        recordNodeDelta(node, GNG_DELTA_ADD);
        return idx;
    }
    return NODE_NOID;
}
#endif
