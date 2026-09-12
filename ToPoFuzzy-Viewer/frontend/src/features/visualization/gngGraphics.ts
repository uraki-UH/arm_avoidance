import * as THREE from 'three';
import { GraphData, LAYER_COLORS, SEMANTIC_COLORS } from '../../types';

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

    const labelValue = Number.isFinite(node.label) ? Math.trunc(node.label as number) : fallbackIndex;
    const safeIndex = ((labelValue % palette.length) + palette.length) % palette.length;
    return palette[safeIndex] ?? palette[0];
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

    nodes.forEach((node, i) => {
        tempMatrix.makeTranslation(node.x, node.y, node.z);
        tempVec3.set(nodeScale, nodeScale, nodeScale);
        tempMatrix.scale(tempVec3);
        mesh.setMatrixAt(i, tempMatrix);

        const colorHex = options?.node_colors?.get(node) ?? (colorMode === 'uniform'
            ? uniformColor
            : (() => {
                return resolveGraphNodeColor(node, palette, i, useSemanticColors);
            })());
        tempColor.set(colorHex);
        mesh.setColorAt(i, tempColor);
    });

    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
        mesh.instanceColor.needsUpdate = true;
    }
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
        ? Math.max(0.0003, Math.min(edgeWidth, 0.06))
        : 0.001;
    const edgePairCount = Math.floor(edges.length / 2);
    mesh.count = edgePairCount;

    for (let i = 0; i < edgePairCount; i++) {
        const srcIdx = edges[i * 2];
        const tgtIdx = edges[i * 2 + 1];

        if (srcIdx >= nodes.length || tgtIdx >= nodes.length) {
            tempMatrix.identity().scale(tempVec3.set(0, 0, 0));
            mesh.setMatrixAt(i, tempMatrix);
            continue;
        }

        const srcNode = nodes[srcIdx];
        const tgtNode = nodes[tgtIdx];
        if (node_colors || mesh.instanceColor) {
            tempColor.set(node_colors?.get(srcNode.id ?? srcIdx) ?? '#ffffff');
            mesh.setColorAt(i, tempColor);
        }

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

    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
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
