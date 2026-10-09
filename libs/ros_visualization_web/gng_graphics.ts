import * as THREE from 'three';
import { GraphData, LAYER_COLORS, SEMANTIC_COLORS } from './graph_types';

const tempMatrix = new THREE.Matrix4();
const tempColor = new THREE.Color();
const up = new THREE.Vector3(0, 1, 0);
const direction = new THREE.Vector3();
const quaternion = new THREE.Quaternion();
const tempVec3 = new THREE.Vector3();
const startVec = new THREE.Vector3();
const endVec = new THREE.Vector3();
const midpointVec = new THREE.Vector3();

export function buildNodePalette(
    baseColor?: string,
    customPalette?: string[]
): string[] {
    if (customPalette && customPalette.length > 0) {
        return customPalette;
    }
    if (!baseColor) {
        return LAYER_COLORS;
    }

    return LAYER_COLORS.map((color, index) => (index === 1 ? baseColor : color));
}

export function resolveGraphNodeColor(
    node: GraphData['nodes'][number],
    palette: string[],
    fallbackIndex = 0,
    useSemanticColors = true
) {
    const semanticIndex = Number.isFinite(node.semanticLabel) ? Math.trunc(node.semanticLabel as number) : 0;
    if (useSemanticColors && semanticIndex > 0) {
        const safeSemanticIndex = ((semanticIndex % SEMANTIC_COLORS.length) + SEMANTIC_COLORS.length) % SEMANTIC_COLORS.length;
        return SEMANTIC_COLORS[safeSemanticIndex] ?? SEMANTIC_COLORS[0];
    }

    const state_color = resolve_state_ratio_color(node, palette);
    if (state_color) return state_color;

    const labelValue = Number.isFinite(node.label) ? Math.trunc(node.label as number) : fallbackIndex;
    const safeIndex = ((labelValue % palette.length) + palette.length) % palette.length;
    return palette[safeIndex] ?? palette[0];
}

/** 集約元の状態件数。未収録・不正値では割合表示なし。 */
export function get_node_state_counts(node: GraphData['nodes'][number]) {
    const counts = [node.num_safe_states, node.num_danger_states, node.num_collision_states];
    if (!counts.every(value => Number.isSafeInteger(value) && (value as number) >= 0)) return undefined;
    const [num_safe, num_danger, num_collision] = counts as number[];
    const num_total = num_safe + num_danger + num_collision;
    return num_total > 0 ? { num_safe, num_danger, num_collision, num_total } : undefined;
}

/** 安全・危険・衝突の構成割合による線形RGB混色。labelの変更なし。 */
export function resolve_state_ratio_color(node: GraphData['nodes'][number], palette = LAYER_COLORS) {
    const counts = get_node_state_counts(node);
    if (!counts) return undefined;
    const color = new THREE.Color(0, 0, 0);
    for (const [label, num] of [[1, counts.num_safe], [3, counts.num_danger], [2, counts.num_collision]]) {
        color.add(new THREE.Color(palette[label] ?? LAYER_COLORS[label]).multiplyScalar(num / counts.num_total));
    }
    return `#${color.getHexString()}`;
}

/** 追加メッシュなしのノード色分け用マテリアル設定。発光色にもインスタンス色を適用。 */
export function configure_node_material(material: THREE.MeshStandardMaterial) {
    material.color.set('#ffffff');
    material.emissive.set('#ffffff');
    material.onBeforeCompile = (shader) => {
        shader.fragmentShader = shader.fragmentShader.replace('#include <color_fragment>',
            '#include <color_fragment>\n#ifdef USE_COLOR\n totalEmissiveRadiance *= vColor.rgb;\n#endif');
    };
    return material;
}

type instance_cache = {
    positions: Float64Array;
    scale: number;
    matrix_attribute: THREE.InstancedBufferAttribute;
    matrix_version: number;
    colors: string[];
    color_attribute: THREE.InstancedBufferAttribute | null;
    color_version: number;
};
const node_instance_cache = new WeakMap<THREE.InstancedMesh, instance_cache>();
const edge_instance_cache = new WeakMap<THREE.InstancedMesh, instance_cache>();

/** 前回の表示入力。属性差替え・外部更新時は再生成。 */
function get_instance_cache(mesh: THREE.InstancedMesh, caches: WeakMap<THREE.InstancedMesh, instance_cache>, num_values: number) {
    let cache = caches.get(mesh);
    if (!cache || cache.positions.length < num_values || cache.matrix_attribute !== mesh.instanceMatrix ||
        cache.matrix_version !== mesh.instanceMatrix.version || cache.color_attribute !== mesh.instanceColor ||
        cache.color_version !== (mesh.instanceColor?.version ?? -1)) {
        cache = { positions: new Float64Array(num_values).fill(NaN), scale: NaN,
            matrix_attribute: mesh.instanceMatrix, matrix_version: mesh.instanceMatrix.version,
            colors: [], color_attribute: mesh.instanceColor, color_version: mesh.instanceColor?.version ?? -1 };
        caches.set(mesh, cache);
    }
    return cache;
}

/** 変更のある属性だけのGPU転送要求。 */
function finish_instance_update(mesh: THREE.InstancedMesh, cache: instance_cache, has_matrix_changes: boolean, has_color_changes: boolean) {
    if (has_matrix_changes) mesh.instanceMatrix.needsUpdate = true;
    if (has_color_changes && mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    cache.matrix_version = mesh.instanceMatrix.version;
    cache.color_attribute = mesh.instanceColor;
    cache.color_version = mesh.instanceColor?.version ?? -1;
}

/** GNGノードInstancedMeshの姿勢・色の更新。 */
export function updateNodeInstances(
    mesh: THREE.InstancedMesh,
    nodes: GraphData['nodes'],
    nodeScale: number,
    options?: {
        colorMode?: 'label' | 'uniform';
        uniformColor?: string;
        palette?: string[];
        baseColor?: string;
        useSemanticColors?: boolean;
        node_colors?: ReadonlyMap<GraphData['nodes'][number], string>;
    }
) {
    if (!mesh) return;
    const colorMode = options?.colorMode ?? 'label';
    const uniformColor = options?.uniformColor ?? LAYER_COLORS[1];
    const palette = buildNodePalette(options?.baseColor, options?.palette);
    const useSemanticColors = options?.useSemanticColors ?? true;
    mesh.count = nodes.length;
    const cache = get_instance_cache(mesh, node_instance_cache, nodes.length * 3);
    const has_scale_changes = cache.scale !== nodeScale;
    let has_matrix_changes = false, has_color_changes = false;

    nodes.forEach((node, i) => {
        const offset = i * 3, previous = cache.positions;
        if (has_scale_changes || previous[offset] !== node.x || previous[offset + 1] !== node.y || previous[offset + 2] !== node.z) {
            tempMatrix.makeTranslation(node.x, node.y, node.z);
            tempVec3.set(nodeScale, nodeScale, nodeScale);
            tempMatrix.scale(tempVec3);
            mesh.setMatrixAt(i, tempMatrix);
            previous[offset] = node.x; previous[offset + 1] = node.y; previous[offset + 2] = node.z;
            has_matrix_changes = true;
        }

        const colorHex = options?.node_colors?.get(node) ?? (colorMode === 'uniform'
            ? uniformColor
            : (() => {
                return resolveGraphNodeColor(node, palette, i, useSemanticColors);
            })());
        if (cache.colors[i] !== colorHex) {
            tempColor.set(colorHex);
            mesh.setColorAt(i, tempColor);
            cache.colors[i] = colorHex;
            has_color_changes = true;
        }
    });
    cache.scale = nodeScale;
    finish_instance_update(mesh, cache, has_matrix_changes, has_color_changes);
}

/**
 * GNG edgeの姿勢と任意のクラスタ所属色の更新。
 */
export function updateEdgeInstances(
    mesh: THREE.InstancedMesh,
    edges: number[],
    nodes: GraphData['nodes'],
    edgeWidth: number,
    node_colors?: ReadonlyMap<number, string>,
) {
    if (!mesh) return;
    const safeEdgeWidth = Number.isFinite(edgeWidth)
        ? Math.max(0.00003, Math.min(edgeWidth, 0.06))
        : 0.001;
    const edgePairCount = Math.floor(edges.length / 2);
    mesh.count = edgePairCount;
    const cache = get_instance_cache(mesh, edge_instance_cache, edgePairCount * 6);
    const has_scale_changes = cache.scale !== safeEdgeWidth;
    let has_matrix_changes = false, has_color_changes = false;

    for (let i = 0; i < edgePairCount; i++) {
        const srcIdx = edges[i * 2];
        const tgtIdx = edges[i * 2 + 1];

        if (srcIdx >= nodes.length || tgtIdx >= nodes.length) {
            tempMatrix.identity().scale(tempVec3.set(0, 0, 0));
            mesh.setMatrixAt(i, tempMatrix);
            cache.positions.fill(NaN, i * 6, i * 6 + 6);
            has_matrix_changes = true;
            continue;
        }

        const srcNode = nodes[srcIdx];
        const tgtNode = nodes[tgtIdx];
        if (node_colors || mesh.instanceColor) {
            const color = node_colors?.get(srcNode.id ?? srcIdx) ?? '#ffffff';
            if (cache.colors[i] !== color) {
                tempColor.set(color);
                mesh.setColorAt(i, tempColor);
                cache.colors[i] = color;
                has_color_changes = true;
            }
        }

        const offset = i * 6, previous = cache.positions;
        if (!has_scale_changes && previous[offset] === srcNode.x && previous[offset + 1] === srcNode.y &&
            previous[offset + 2] === srcNode.z && previous[offset + 3] === tgtNode.x &&
            previous[offset + 4] === tgtNode.y && previous[offset + 5] === tgtNode.z) continue;
        previous[offset] = srcNode.x; previous[offset + 1] = srcNode.y; previous[offset + 2] = srcNode.z;
        previous[offset + 3] = tgtNode.x; previous[offset + 4] = tgtNode.y; previous[offset + 5] = tgtNode.z;
        has_matrix_changes = true;
        startVec.set(srcNode.x, srcNode.y, srcNode.z);
        endVec.set(tgtNode.x, tgtNode.y, tgtNode.z);

        midpointVec.addVectors(startVec, endVec).multiplyScalar(0.5);
        const length = startVec.distanceTo(endVec);

        direction.subVectors(endVec, startVec).normalize();
        quaternion.setFromUnitVectors(up, direction);

        tempMatrix.makeRotationFromQuaternion(quaternion);
        tempVec3.set(safeEdgeWidth, length, safeEdgeWidth);
        tempMatrix.scale(tempVec3);
        tempMatrix.setPosition(midpointVec);

        mesh.setMatrixAt(i, tempMatrix);
    }

    cache.scale = safeEdgeWidth;
    finish_instance_update(mesh, cache, has_matrix_changes, has_color_changes);
}

/** 確定した人・車の所属ノードID別ラベル。元ノードの幾何ラベルの変更なし。 */
export function build_classified_node_labels(graph: GraphData): Map<number, number> {
    const labels = new Map<number, number>();
    for (const cluster of graph.clusters) {
        if (cluster.label !== 4 && cluster.label !== 5) continue;
        for (const id of cluster.nodeIds) labels.set(id, cluster.label);
    }
    return labels;
}

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
