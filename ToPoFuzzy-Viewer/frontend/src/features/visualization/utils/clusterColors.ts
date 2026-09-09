import * as THREE from 'three';
import type { GraphData } from '../../../types';

/** クラスタ所属IDによる表示色。未所属は灰色、通常分類やsemantic属性は変更なし。 */
export function build_cluster_node_colors(graph: GraphData): Map<number, string> {
    const colors = new Map(graph.nodes.map((node, idx) => [node.id ?? idx, '#737373']));
    const clusters = [...graph.clusters].sort((a, b) => a.id - b.id);
    clusters.forEach((cluster, idx) => {
        const hue = ((idx + 1) * 0.618033988749895 % 1) * 6;
        const low = 0.18, high = 0.95;
        const up = low + (high - low) * (hue - Math.floor(hue));
        const down = high + low - up;
        const rgb = [
            [high, up, low], [down, high, low], [low, high, up],
            [low, down, high], [up, low, high], [high, low, down],
        ][Math.floor(hue)];
        const color = '#' + new THREE.Color(...rgb as [number, number, number]).getHexString();
        for (const id of cluster.nodeIds) {
            if (colors.has(id)) colors.set(id, color);
        }
    });
    return colors;
}
