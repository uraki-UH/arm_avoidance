import { useMemo, useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { Billboard, Text } from '@react-three/drei';
import { useThree } from '@react-three/fiber';
import { ThreeEvent } from '@react-three/fiber';
import { GraphData, LAYER_COLORS, LAYER_LABELS } from '../../types';

const EMPTY_GRAPH: GraphData = {
    timestamp: 0,
    nodes: [],
    edges: [],
    clusters: [],
    clusterLabels: []
};

interface GraphRendererProps {
    data: GraphData | null;
    visible?: boolean;
    showNodes?: boolean;
    showEdges?: boolean;
    showClusters?: boolean;
    showClusterText?: boolean;
    nodeScale?: number;
    edgeWidth?: number;
    visibleLabels?: {
        0: boolean;
        1: boolean;
        2: boolean;
        3: boolean;
        4: boolean;
        5: boolean;
    };
    selectedClusterId?: number | null;
    onClusterSelect?: (clusterId: number | null) => void;
    enableClusterSelection?: boolean;
    opacity?: number;
    tf?: { pos: number[]; quat: number[] } | null;
}

export function GraphRenderer({
    data,
    visible = true,
    showNodes = true,
    showEdges = true,
    showClusters = true,
    showClusterText = true,
    nodeScale = 0.015,
    edgeWidth = 0.007,
    visibleLabels,
    selectedClusterId = null,
    onClusterSelect,
    enableClusterSelection = true,
    opacity = 1.0,
    tf = null,
}: GraphRendererProps) {
    const { invalidate } = useThree();
    const nodesRef = useRef<THREE.InstancedMesh>(null);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const groupRef = useRef<THREE.Group>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;
    // Handle cluster click with drag filtering
    const handleClusterClick = (clusterId: number, e: ThreeEvent<MouseEvent>) => {
        if (!selectionEnabled || !onClusterSelect) return;
        e.stopPropagation();

        // Check distance for drag detection (if available)
        // R3F events often bubble, so we check if this is a genuine click or a drag-release
        if (dragStartRef.current) {
            const dx = e.clientX - dragStartRef.current.x;
            const dy = e.clientY - dragStartRef.current.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 5) { // 5px threshold for "click" vs "drag"
                return;
            }
        }

        // Toggle: if already selected, deselect
        if (selectedClusterId === clusterId) {
            onClusterSelect(null);
        } else {
            onClusterSelect(clusterId);
        }
    };

    // --- Node sphere geometry (shared) ---
    const nodeSphereGeometry = useMemo(() => {
        return new THREE.SphereGeometry(1, 12, 8);
    }, []);

    // --- Node material ---
    const nodeMaterial = useMemo(() => {
        return new THREE.MeshBasicMaterial({
            color: '#c8ff4a',
            transparent: true,
            opacity: opacity,
            depthTest: false,
            depthWrite: false,
        });
    }, [opacity]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);

    // --- TF-based Positioning ---
    useEffect(() => {
        if (!groupRef.current) return;
        if (!tf) {
            groupRef.current.position.set(0, 0, 0);
            groupRef.current.rotation.set(0, 0, 0);
            return;
        }
        groupRef.current.position.set(tf.pos[0], tf.pos[1], tf.pos[2]);
        groupRef.current.quaternion.set(tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]);
        invalidate();
    }, [tf, invalidate]);

    useEffect(() => {
        if (graph.nodes.length > nodeCapacity) {
            setNodeCapacity(graph.nodes.length);
        }
    }, [graph.nodes.length, nodeCapacity]);

    // --- Update instanced nodes ---
    useEffect(() => {
        if (!nodesRef.current || !showNodes) return;
        if (graph.nodes.length > nodeCapacity) return;

        const tempMatrix = new THREE.Matrix4();
        const tempColor = new THREE.Color();

        nodesRef.current.count = graph.nodes.length;

        graph.nodes.forEach((node, i) => {
            tempMatrix.makeTranslation(node.x, node.y, node.z);
            tempMatrix.scale(new THREE.Vector3(nodeScale, nodeScale, nodeScale));
            nodesRef.current!.setMatrixAt(i, tempMatrix);

            const labelValue = Number.isFinite(node.label) ? Math.trunc(node.label) : 0;
            const safeIndex = ((labelValue % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            const colorHex = LAYER_COLORS[safeIndex] ?? LAYER_COLORS[0];
            tempColor.set(colorHex);
            nodesRef.current!.setColorAt(i, tempColor);
        });

        nodesRef.current.instanceMatrix.needsUpdate = true;
        if (nodesRef.current.instanceColor) {
            nodesRef.current.instanceColor.needsUpdate = true;
        }
        invalidate();
    }, [graph.nodes, showNodes, nodeScale, nodeCapacity, invalidate]);

    // --- Edge cylinder geometry (shared) ---
    const edgeCylinderGeometry = useMemo(() => {
        return new THREE.CylinderGeometry(1, 1, 1, 6);
    }, []);

    // --- Edge material ---
    const edgeMaterial = useMemo(() => {
        return new THREE.MeshBasicMaterial({
            color: '#b9ff3f',
            transparent: true,
            opacity: opacity,
            depthTest: false,
            depthWrite: false,
        });
    }, [opacity]);

    // --- Calculate edge count (pairs) ---
    const edgePairCount = useMemo(() => {
        return Math.floor(graph.edges.length / 2);
    }, [graph.edges]);

    const [edgeCapacity, setEdgeCapacity] = useState(edgePairCount);

    useEffect(() => {
        if (edgePairCount > edgeCapacity) {
            setEdgeCapacity(edgePairCount);
        }
    }, [edgePairCount, edgeCapacity]);

    // --- Update instanced edges ---
    useEffect(() => {
        if (!edgesRef.current || !showEdges || edgePairCount === 0) return;
        if (edgePairCount > edgeCapacity) return;

        const tempMatrix = new THREE.Matrix4();
        const up = new THREE.Vector3(0, 1, 0);
        const direction = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();

        edgesRef.current.count = edgePairCount;

        for (let i = 0; i < edgePairCount; i++) {
            const srcIdx = graph.edges[i * 2];
            const tgtIdx = graph.edges[i * 2 + 1];

            if (srcIdx >= graph.nodes.length || tgtIdx >= graph.nodes.length) {
                // Hide invalid edges by setting scale to 0
                tempMatrix.identity().scale(new THREE.Vector3(0, 0, 0));
                edgesRef.current!.setMatrixAt(i, tempMatrix);
                continue;
            }

            const srcNode = graph.nodes[srcIdx];
            const tgtNode = graph.nodes[tgtIdx];

            const start = new THREE.Vector3(srcNode.x, srcNode.y, srcNode.z);
            const end = new THREE.Vector3(tgtNode.x, tgtNode.y, tgtNode.z);

            const midpoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
            const length = start.distanceTo(end);

            direction.subVectors(end, start).normalize();
            quaternion.setFromUnitVectors(up, direction);

            tempMatrix.makeRotationFromQuaternion(quaternion);
            tempMatrix.scale(new THREE.Vector3(edgeWidth, length, edgeWidth));
            tempMatrix.setPosition(midpoint);

            edgesRef.current!.setMatrixAt(i, tempMatrix);
        }

        edgesRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [graph.edges, graph.nodes, showEdges, edgeWidth, edgePairCount, edgeCapacity, invalidate]);

    if (!data || !visible) return null;

    const canRenderNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canRenderEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;

    return (
        <group ref={groupRef}>
            {/* Nodes as instanced spheres */}
            {canRenderNodes && (
                <instancedMesh
                    key={`nodes-${nodeCapacity}`}
                    ref={nodesRef}
                    args={[nodeSphereGeometry, nodeMaterial, nodeCapacity]}
                    count={graph.nodes.length}
                    frustumCulled={false}
                    renderOrder={10}
                />
            )}

            {/* Edges as instanced cylinders */}
            {canRenderEdges && (
                <instancedMesh
                    key={`edges-${edgeCapacity}`}
                    ref={edgesRef}
                    args={[edgeCylinderGeometry, edgeMaterial, edgeCapacity]}
                    count={edgePairCount}
                    frustumCulled={false}
                    renderOrder={9}
                />
            )}

            {/* Clusters */}
            {showClusters && graph.clusters
                .filter(cluster => !visibleLabels || visibleLabels[cluster.label as 0 | 1 | 2 | 3 | 4 | 5])
                .map((cluster) => {
                    const isSelected = selectedClusterId === cluster.id;
                    const color = isSelected ? '#FFFFFF' : LAYER_COLORS[cluster.label % LAYER_COLORS.length];
                    const isHuman = cluster.label === 4; // HUMAN
                    const handlePointerDown = selectionEnabled
                        ? (e: ThreeEvent<PointerEvent>) => {
                            dragStartRef.current = { x: e.clientX, y: e.clientY };
                        }
                        : undefined;
                    const handleClick = selectionEnabled
                        ? (e: ThreeEvent<MouseEvent>) => handleClusterClick(cluster.id, e)
                        : undefined;

                    return (
                        <group key={cluster.id} position={cluster.pos} quaternion={new THREE.Quaternion(...cluster.quat)}>
                            <mesh
                                scale={isHuman
                                    ? [cluster.scale[0], cluster.scale[2], cluster.scale[1]]  // [X, Z, Y] for cylinder: radius_x, height, radius_y
                                    : cluster.scale
                                }
                                rotation={isHuman ? [Math.PI / 2, 0, 0] : [0, 0, 0]}
                                onPointerDown={handlePointerDown}
                                onClick={handleClick}
                            >
                                {isHuman ? (
                                    <cylinderGeometry args={[0.5, 0.5, 1, 16]} />
                                ) : (
                                    <boxGeometry args={[1, 1, 1]} />
                                )}
                                <meshStandardMaterial
                                    color={color}
                                    transparent
                                    opacity={isSelected ? 0.1 : 0.3 * opacity}
                                    emissive={isSelected ? '#FFFFFF' : '#000000'}
                                    emissiveIntensity={isSelected ? 0.2 : 0}
                                    depthWrite={false}
                                    side={THREE.DoubleSide}
                                />
                            </mesh>

                            {showClusterText && (
                                <Billboard
                                    follow
                                    lockX={false}
                                    lockY={false}
                                    lockZ={false}
                                    position={[0, 0, cluster.scale[2] / 2 + 0.2]}
                                >
                                    <Text
                                        fontSize={0.2}
                                        color="#FFFFFF"
                                        anchorX="center"
                                        anchorY="bottom"
                                    >
                                        {`${LAYER_LABELS[cluster.label] || 'obj'}\nR:${cluster.reliability.toFixed(2)}`}
                                    </Text>
                                </Billboard>
                            )}
                        </group>
                    );
                })}

        </group>
    );
}
