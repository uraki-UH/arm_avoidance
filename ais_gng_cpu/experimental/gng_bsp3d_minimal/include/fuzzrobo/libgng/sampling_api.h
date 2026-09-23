#pragma once
#include <stdint.h>

// 独立bsp3d版の1フレーム統計。時間はGNG本体内の段階別実測。
typedef struct gng_sampling_statistics {
    double voxel_ms, attention_ms, learn_ms, label_ms, maintenance_ms, cluster_ms;
    uint64_t num_nearest_queries, num_tree_moves, num_probe_points;
    uint64_t num_observed_nodes, num_attention_candidates, num_attention_hits;
    uint64_t num_added_nodes, num_deleted_nodes;
    uint64_t num_zero_samples;
    double input_prepare_ms; // YAML範囲内の元点番号の整理時間（ms）
    uint64_t num_input_points; // 学習候補となる範囲内元点数
} gng_sampling_statistics;

#ifdef __cplusplus
extern "C" {
#endif
const gng_sampling_statistics *gng_get_sampling_statistics(void);
const gng_sampling_statistics *gng_get_minimal_statistics(void);
#ifdef __cplusplus
}
#endif
