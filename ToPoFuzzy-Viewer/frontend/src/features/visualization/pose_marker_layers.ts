import type { MarkerArrayData, MarkerMessage } from '../../types';

function is_same_pose(raw: MarkerMessage, evaluated: MarkerMessage): boolean {
    return raw.id === evaluated.id && raw.frameId === evaluated.frameId &&
        raw.header_stamp !== undefined && evaluated.header_stamp !== undefined &&
        raw.header_stamp.every((value, idx) => value === evaluated.header_stamp?.[idx]) &&
        raw.points.length === 2 && evaluated.points.length === 2 &&
        raw.points.every((point, idx) => point.every((value, axis) =>
            Math.abs(value - evaluated.points[idx][axis]) < 1e-8));
}

// 候補Poseを基準とした表示。対応する到達性評価は色だけを統合
export function pose_marker_layers(
    data: Record<string, MarkerArrayData>,
    settings: Record<string, { visible?: boolean }>,
    disabled: ReadonlySet<string>,
): Record<string, MarkerArrayData> {
    const result = { ...data };
    for (const [tag, raw] of Object.entries(data)) {
        if (raw.source_type !== 'pose_array' || settings[tag]?.visible === false || disabled.has(tag)) continue;
        const evaluation_tag = `${tag}/reachability_markers`;
        const evaluation = data[evaluation_tag];
        if (!evaluation || settings[evaluation_tag]?.visible === false || disabled.has(evaluation_tag)) continue;
        const evaluated = new Map(evaluation.markers
            .filter(marker => marker.action === 0 && marker.type === 'arrow')
            .map(marker => [marker.id, marker]));
        result[tag] = {
            ...raw,
            markers: raw.markers.map(marker => {
                const candidate = evaluated.get(marker.id);
                return candidate && is_same_pose(marker, candidate)
                    ? { ...marker, color: candidate.color } : marker;
            }),
        };
        // 古い評価に残る候補も含め、重複レイヤーの描画を抑制
        delete result[evaluation_tag];
    }
    return result;
}
