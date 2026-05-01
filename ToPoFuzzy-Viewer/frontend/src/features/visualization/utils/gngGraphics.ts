import * as THREE from 'three';
import { GraphData, LAYER_COLORS } from '../../../types';

const tempMatrix = new THREE.Matrix4();
const tempColor = new THREE.Color();
const up = new THREE.Vector3(0, 1, 0);
const direction = new THREE.Vector3();
const quaternion = new THREE.Quaternion();
const tempVec3 = new THREE.Vector3();
const startVec = new THREE.Vector3();
const endVec = new THREE.Vector3();
const midpointVec = new THREE.Vector3();

const clamp01 = (value: number) => Math.min(1, Math.max(0, value));

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

    const color = new THREE.Color(baseColor);
    const hsl = { h: 0, s: 0, l: 0 };
    color.getHSL(hsl);

    const swatches = [
        { s: clamp01(hsl.s * 0.65), l: clamp01(hsl.l - 0.24) },
        { s: clamp01(hsl.s * 0.90), l: clamp01(hsl.l - 0.08) },
        { s: clamp01(hsl.s * 1.00), l: clamp01(hsl.l + 0.00) },
        { s: clamp01(hsl.s * 1.10), l: clamp01(hsl.l + 0.10) },
        { s: clamp01(hsl.s * 0.80), l: clamp01(hsl.l + 0.22) },
        { s: clamp01(hsl.s * 1.05), l: clamp01(hsl.l + 0.30) },
    ];

    return swatches.map(({ s, l }) => {
        const swatch = new THREE.Color();
        swatch.setHSL(hsl.h, s, l);
        return `#${swatch.getHexString()}`;
    });
}

/**
 * Updates the instance matrices and colors for a GNG node InstancedMesh.
 */
export function updateNodeInstances(
    mesh: THREE.InstancedMesh,
    nodes: GraphData['nodes'],
    nodeScale: number,
    options?: {
        colorMode?: 'label' | 'uniform';
        uniformColor?: string;
        palette?: string[];
        baseColor?: string;
    }
) {
    if (!mesh) return;
    const colorMode = options?.colorMode ?? 'label';
    const uniformColor = options?.uniformColor ?? LAYER_COLORS[1];
    const palette = buildNodePalette(options?.baseColor, options?.palette);
    mesh.count = nodes.length;

    nodes.forEach((node, i) => {
        tempMatrix.makeTranslation(node.x, node.y, node.z);
        tempVec3.set(nodeScale, nodeScale, nodeScale);
        tempMatrix.scale(tempVec3);
        mesh.setMatrixAt(i, tempMatrix);

        const colorHex = colorMode === 'uniform'
            ? uniformColor
            : (() => {
                const labelValue = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
                const safeIndex = ((labelValue % palette.length) + palette.length) % palette.length;
                return palette[safeIndex] ?? palette[0];
            })();
        tempColor.set(colorHex);
        mesh.setColorAt(i, tempColor);
    });

    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
        mesh.instanceColor.needsUpdate = true;
    }
}

/**
 * Updates the instance matrices for GNG edge InstancedMesh (using Cylinders).
 */
export function updateEdgeInstances(
    mesh: THREE.InstancedMesh,
    edges: number[],
    nodes: GraphData['nodes'],
    edgeWidth: number
) {
    if (!mesh) return;
    const safeEdgeWidth = Number.isFinite(edgeWidth)
        ? Math.max(0.0005, Math.min(edgeWidth, 0.02))
        : 0.007;
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
}
