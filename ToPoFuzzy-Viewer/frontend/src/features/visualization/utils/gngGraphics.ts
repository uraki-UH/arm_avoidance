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

/**
 * Updates the instance matrices and colors for a GNG node InstancedMesh.
 */
export function updateNodeInstances(
    mesh: THREE.InstancedMesh,
    nodes: GraphData['nodes'],
    nodeScale: number
) {
    if (!mesh) return;
    mesh.count = nodes.length;

    nodes.forEach((node, i) => {
        tempMatrix.makeTranslation(node.x, node.y, node.z);
        tempVec3.set(nodeScale, nodeScale, nodeScale);
        tempMatrix.scale(tempVec3);
        mesh.setMatrixAt(i, tempMatrix);

        const labelValue = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
        const safeIndex = ((labelValue % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
        const colorHex = LAYER_COLORS[safeIndex] ?? LAYER_COLORS[0];
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
        tempVec3.set(edgeWidth, length, edgeWidth);
        tempMatrix.scale(tempVec3);
        tempMatrix.setPosition(midpointVec);

        mesh.setMatrixAt(i, tempMatrix);
    }

    mesh.instanceMatrix.needsUpdate = true;
}
