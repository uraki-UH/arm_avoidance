#pragma once
#include <stdint.h>

// 独立bsp3d版の1フレーム統計。時間はGNG本体内の段階別実測。
typedef struct gng_sampling_statistics {
    double voxel_ms, attention_ms, learn_ms, label_ms, maintenance_ms, cluster_ms;
    uint64_t num_nearest_queries, num_tree_moves, num_probe_points;
    uint64_t num_observed_nodes, num_attention_candidates, num_attention_hits;
    uint64_t num_added_nodes, num_deleted_nodes;
} gng_sampling_statistics;

#ifdef __cplusplus
extern "C" {
#endif
const gng_sampling_statistics *gng_get_sampling_statistics(void);
#ifdef __cplusplus
}
#endif
