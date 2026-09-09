import type { GraphNode, LayerSettings } from '../../types';

export interface node_label_definition {
    id: string;
    name: string;
    color: string;
    enable_by_default: boolean;
    default_priority: number;
    parent_id?: string;
    is_match: (node: Partial<GraphNode>) => boolean;
}

/** 重複可能な可視化ラベルの定義。受信属性のみを参照、幾何判定なし。 */
export const node_label_definitions: readonly node_label_definition[] = [
    { id: 'boundary_fov', parent_id: 'boundary', name: '視野端', color: '#b388ff', enable_by_default: true, default_priority: -3,
        is_match: (node) => node.is_boundary_candidate === true && ((node.boundary_evidence ?? 0) & 4) !== 0 },
    { id: 'boundary_occlusion', parent_id: 'boundary', name: '遮蔽の証拠', color: '#ff5252', enable_by_default: true, default_priority: -2,
        is_match: (node) => node.is_boundary_candidate === true && ((node.boundary_evidence ?? 0) & 1) !== 0 },
    { id: 'boundary_free_space', parent_id: 'boundary', name: '自由空間の証拠', color: '#2196f3', enable_by_default: true, default_priority: -1,
        is_match: (node) => node.is_boundary_candidate === true && ((node.boundary_evidence ?? 0) & 2) !== 0 },
    { id: 'boundary_unknown', parent_id: 'boundary', name: '原因不明', color: '#ff8c00', enable_by_default: true, default_priority: 2,
        is_match: (node) => node.is_boundary_candidate === true && (node.boundary_evidence ?? 0) === 0 },
    { id: 'boundary', name: '境界候補', color: '#ff8c00', enable_by_default: true, default_priority: 0,
        is_match: (node) => node.is_boundary_candidate === true },
    { id: 'handle', name: 'HANDLE', color: '#00d1ff', enable_by_default: true, default_priority: 1,
        is_match: (node) => Number.isFinite(node.semanticLabel) && (node.semanticLabel ?? 0) > 0 },
];

export type node_label_options = Pick<LayerSettings,
    'node_label_visibility' | 'node_label_priority' | 'node_label_colors' | 'enable_boundary_highlight' |
    'overlap_label_priority' | 'visibleSemanticLabels'>;

/** 旧設定の引き継ぎと未知・重複IDの除外。追加定義は既定順で末尾へ補完。 */
export function normalize_node_label_settings(options: node_label_options = {},
    definitions: readonly node_label_definition[] = node_label_definitions) {
    const legacy_visibility: Record<string, boolean | undefined> = {
        boundary: options.enable_boundary_highlight,
        handle: options.visibleSemanticLabels?.handle,
    };
    const node_label_visibility: Record<string, boolean> = { ...options.node_label_visibility };
    const node_label_colors: Record<string, string> = {};
    for (const definition of definitions) {
        node_label_visibility[definition.id] = options.node_label_visibility?.[definition.id] ??
            legacy_visibility[definition.id] ?? definition.enable_by_default;
        const color = options.node_label_colors?.[definition.id];
        node_label_colors[definition.id] = color && /^#[\da-f]{6}$/i.test(color) ? color : definition.color;
    }
    const defaults = [...definitions].sort((a, b) => a.default_priority - b.default_priority).map((item) => item.id);
    const requested = options.node_label_priority ??
        (options.overlap_label_priority ? [options.overlap_label_priority] : []);
    const known_ids = new Set(defaults);
    const node_label_priority = [...new Set([...requested, ...defaults])].filter((id) => known_ids.has(id));
    return { node_label_visibility, node_label_priority, node_label_colors };
}

/** 一覧の親ラベル順。原因別ラベルの旧優先順位とは独立したグループ単位。 */
export function get_node_label_groups(priority: readonly string[],
    definitions: readonly node_label_definition[] = node_label_definitions) {
    const by_id = new Map(definitions.map((definition) => [definition.id, definition]));
    return priority.map((id) => by_id.get(id)).filter((item): item is node_label_definition =>
        item !== undefined && !item.parent_id);
}

/** グループ順と原因別の順による有効ラベル一覧。親OFF時は子も無効、全候補への迂回なし。 */
export function get_active_node_labels(options: node_label_options = {},
    definitions: readonly node_label_definition[] = node_label_definitions) {
    const settings = normalize_node_label_settings(options, definitions);
    const by_id = new Map(definitions.map((definition) => [definition.id, definition]));
    return get_node_label_groups(settings.node_label_priority, definitions).flatMap((group) => {
        if (!settings.node_label_visibility[group.id]) return [];
        const children = settings.node_label_priority.map((id) => by_id.get(id)!)
            .filter((item) => item.parent_id === group.id);
        return (children.length ? children : [group])
            .filter((item) => settings.node_label_visibility[item.id])
            .map((item) => ({ ...item, color: settings.node_label_colors[item.id] }));
    });
}

/** OR表示の根拠と最優先色の共通解決。該当なしは通常ラベルへ委譲。 */
export function resolve_node_label(node: Partial<GraphNode>, active_labels: readonly node_label_definition[]) {
    return active_labels.find((definition) => definition.is_match(node));
}

/** 優先順位の隣接交換。範囲外の移動は現状維持。 */
export function move_node_label(priority: readonly string[], id: string, offset: -1 | 1) {
    const next = [...priority];
    const idx = next.indexOf(id);
    const target_idx = idx + offset;
    if (idx < 0 || target_idx < 0 || target_idx >= next.length) return next;
    [next[idx], next[target_idx]] = [next[target_idx], next[idx]];
    return next;
}

/** 親ラベルだけの優先順位交換。原因別の既存相対順は保持。 */
export function move_node_label_group(priority: readonly string[], id: string, offset: -1 | 1) {
    const groups = get_node_label_groups(priority).map((item) => item.id);
    return [...move_node_label(groups, id, offset), ...priority.filter((item) => !groups.includes(item))];
}

/** 指定項目の直前への挿入。nullは末尾、無効IDは現状維持。 */
export function insert_node_label(priority: readonly string[], id: string, before_id: string | null) {
    if (!priority.includes(id) || id === before_id || (before_id !== null && !priority.includes(before_id))) return [...priority];
    const next = priority.filter((item) => item !== id);
    next.splice(before_id === null ? next.length : next.indexOf(before_id), 0, id);
    return next;
}

/** 対象グループ内のみの並べ替え。他グループの相対順は保持。 */
export function reorder_node_label_subset(priority: readonly string[], ids: readonly string[]) {
    return [...ids, ...priority.filter((id) => !ids.includes(id))];
}

/** 重なった項目への移動。元の並びが上なら対象の下、下なら対象の上への挿入。 */
export function get_node_label_overlap_target(priority: readonly string[], id: string, target_id: string) {
    const source_idx = priority.indexOf(id), target_idx = priority.indexOf(target_id);
    if (source_idx < 0 || target_idx < 0 || source_idx === target_idx) return id;
    return source_idx < target_idx ? priority[target_idx + 1] ?? null : target_id;
}
