#include "cugng.hpp"
uint32_t CUGNG::find_free_node() {
#if GNG_FREE_NODE_MODE == 1
    // 削除時に下限を戻す、最小空き番号の探索開始位置。
    while (next_free_idx < static_cast<uint32_t>(node_num_max) &&
        nodes[next_free_idx].id != NODE_NOID) {++next_free_idx;}
    return next_free_idx < static_cast<uint32_t>(node_num_max) ? next_free_idx++ : NODE_NOID;
#elif GNG_FREE_NODE_MODE == 2
    if (free_node_ids.empty()) {return NODE_NOID;}
    const auto idx = free_node_ids.top();
    free_node_ids.pop();
    return idx;
#else
    for (uint32_t idx = 0; idx < static_cast<uint32_t>(node_num_max); ++idx) {
        if (nodes[idx].id == NODE_NOID) {return idx;}
    }
    return NODE_NOID;
#endif
}
void CUGNG::release_node_id(uint32_t idx) {
#if GNG_FREE_NODE_MODE == 1
    next_free_idx = std::min(next_free_idx, idx);
#elif GNG_FREE_NODE_MODE == 2
    free_node_ids.push(idx);
#endif
}
