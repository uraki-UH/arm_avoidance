/** 自己認識の補助ボクセルに対する初期非表示。手動選択は各レイヤー設定で保持。 */
export function is_auxiliary_voxel_layer(tag: string): boolean {
    return /(^|\/)(roi_voxels|self_voxel|self_filter_roi_voxels)$/.test(tag);
}

/** 同じ名前空間のL0受信時における標準グラフの初期非表示。 */
export function is_graph_visible_by_default(tag: string, available_tags: ReadonlySet<string>): boolean {
    return !(/(^|\/)Tmap_static$/.test(tag) && available_tags.has(`${tag}_L0`));
}
