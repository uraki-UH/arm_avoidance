import { useMemo, useRef, useEffect } from 'react';
import * as THREE from 'three';
import { GraphNode } from '../../types';

interface NormalVectorRendererProps {
    nodes: GraphNode[];
    nodeIds: number[];
    visible?: boolean;
    arrowLength?: number;
    arrowColor?: string;
}

/**
 * Renders normal vectors as arrows for selected cluster nodes.
 * Uses instanced cones and cylinders for efficient rendering.
 */
export function NormalVectorRenderer({
    nodes,
    nodeIds,
    visible = true,
    arrowLength = 0.15,
    arrowColor = '#00FFFF'
}: NormalVectorRendererProps) {
    const instancedCylinderRef = useRef<THREE.InstancedMesh>(null);
    const instancedConeRef = useRef<THREE.InstancedMesh>(null);

    // Get the nodes to render
    const selectedNodes = useMemo(() => {
        if (!nodeIds || nodeIds.length === 0) return [];
        return nodeIds
            .filter(id => id >= 0 && id < nodes.length)
            .map(id => nodes[id]);
    }, [nodes, nodeIds]);

    // Cylinder (shaft) geometry
    const cylinderGeometry = useMemo(() => {
        return new THREE.CylinderGeometry(0.01, 0.01, 1, 6);
    }, []);

    // Cone (head) geometry
    const coneGeometry = useMemo(() => {
        return new THREE.ConeGeometry(0.025, 0.05, 6);
    }, []);

    // Material
    const material = useMemo(() => {
        return new THREE.MeshBasicMaterial({ color: arrowColor });
    }, [arrowColor]);

    // Update instanced meshes
    useEffect(() => {
        if (!instancedCylinderRef.current || !instancedConeRef.current || !visible) return;
        if (selectedNodes.length === 0) return;

        const tempMatrix = new THREE.Matrix4();
        const up = new THREE.Vector3(0, 1, 0);
        const direction = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        const shaftLength = arrowLength * 0.75;
        const headLength = arrowLength * 0.25;

        selectedNodes.forEach((node, i) => {
            // Get normal direction
            direction.set(node.nx, node.ny, node.nz);
            if (direction.lengthSq() < 0.0001) {
                // Skip if normal is zero
                tempMatrix.identity().scale(new THREE.Vector3(0, 0, 0));
                instancedCylinderRef.current!.setMatrixAt(i, tempMatrix);
                instancedConeRef.current!.setMatrixAt(i, tempMatrix);
                return;
            }
            direction.normalize();

            // Calculate rotation from up vector to normal direction
            quaternion.setFromUnitVectors(up, direction);

            // Shaft position: start from node, center at midpoint of shaft
            const shaftCenter = new THREE.Vector3(
                node.x + direction.x * shaftLength / 2,
                node.y + direction.y * shaftLength / 2,
                node.z + direction.z * shaftLength / 2
            );

            tempMatrix.makeRotationFromQuaternion(quaternion);
            tempMatrix.scale(new THREE.Vector3(1, shaftLength, 1));
            tempMatrix.setPosition(shaftCenter);
            instancedCylinderRef.current!.setMatrixAt(i, tempMatrix);

            // Cone position: at the end of shaft
            const coneCenter = new THREE.Vector3(
                node.x + direction.x * (shaftLength + headLength / 2),
                node.y + direction.y * (shaftLength + headLength / 2),
                node.z + direction.z * (shaftLength + headLength / 2)
            );

            tempMatrix.makeRotationFromQuaternion(quaternion);
            tempMatrix.setPosition(coneCenter);
            instancedConeRef.current!.setMatrixAt(i, tempMatrix);
        });

        instancedCylinderRef.current.instanceMatrix.needsUpdate = true;
        instancedConeRef.current.instanceMatrix.needsUpdate = true;
    }, [selectedNodes, visible, arrowLength]);

    if (!visible || selectedNodes.length === 0) return null;

    return (
        <group>
            <instancedMesh
                ref={instancedCylinderRef}
                args={[cylinderGeometry, material, selectedNodes.length]}
                frustumCulled={false}
            />
            <instancedMesh
                ref={instancedConeRef}
                args={[coneGeometry, material, selectedNodes.length]}
                frustumCulled={false}
            />
        </group>
    );
}
