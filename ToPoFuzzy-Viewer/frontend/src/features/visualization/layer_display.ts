/** 同じ名前空間のL0受信時における標準グラフの初期非表示。 */
export function is_graph_visible_by_default(tag: string, available_tags: ReadonlySet<string>): boolean {
    return !(/(^|\/)Tmap_static$/.test(tag) && available_tags.has(`${tag}_L0`));
}
