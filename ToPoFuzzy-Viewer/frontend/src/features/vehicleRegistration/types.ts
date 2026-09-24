import type { graph_selection, graph_snapshot } from '../../types';

export interface vehicle_candidate {
    model_id: string;
    label: string;
    model_note: string;
    dimensions_m: [number, number, number];
    yaw_deg: number;
    translation: [number, number, number];
    match_ratio: number;
    support_ratio: number;
    unmatched_ratio: number;
    inlier_rms_m: number | null;
    compatibility: number;
    rank_score: number;
    num_inliers: number;
    num_observed: number;
    matched_positions: number[];
    unmatched_positions: number[];
    outlier_positions: number[];
    min_position: [number, number, number];
    max_position: [number, number, number];
}

export interface vehicle_registration_result {
    source_id: string;
    selection: graph_selection;
    frame_id: string;
    timestamp: number;
    observation_kind: 'gng_nodes';
    dist_th: number;
    support_dist_th: number;
    state: 'class_candidate' | 'ambiguous' | 'insufficient';
    message: string;
    candidates: vehicle_candidate[];
    elapsed_ms: number;
    limitations: string;
}

export type register_vehicle = (snapshot: graph_snapshot, dist_th: number,
    support_dist_th: number) => Promise<vehicle_registration_result>;
