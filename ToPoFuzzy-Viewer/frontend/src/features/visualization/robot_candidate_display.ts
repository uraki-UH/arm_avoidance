// 単体表示の候補添字。nullは件数制限モード、候補減少時は現在の配信範囲へ補正
export function robot_candidate_idx(selected_idx: number | null | undefined, num_candidates: number): number | null {
    return typeof selected_idx === 'number' && Number.isFinite(selected_idx)
        ? Math.max(0, Math.min(Math.max(0, num_candidates - 1), Math.floor(selected_idx))) : null;
}
