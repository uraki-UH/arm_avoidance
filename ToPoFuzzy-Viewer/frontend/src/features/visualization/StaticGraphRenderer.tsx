import { useMemo, useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { Billboard, Text } from '@react-three/drei';
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
}

export function StaticGraphRenderer({
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
    opacity = 1.0
}: GraphRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const nodesRef = useRef<THREE.InstancedMesh>(null);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    const { scene } = useThree();
    
    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;

    // --- Frame Anchoring (TF-like logic) ---
    useEffect(() => {
        if (!groupRef.current || !graph.frameId || graph.frameId === 'world') {
            // Re-attach to scene if no frame is specified or it's 'world'
            if (groupRef.current?.parent && groupRef.current.parent !== scene) {
                scene.add(groupRef.current);
                groupRef.current.position.set(0, 0, 0);
                groupRef.current.rotation.set(0, 0, 0);
            }
            return;
        }

        // Try to find the link in the scene (robot link name)
        const anchor = scene.getObjectByName(graph.frameId);
        if (anchor && groupRef.current.parent !== anchor) {
            console.log(`[StaticGraphRenderer] Anchoring to frame: ${graph.frameId}`);
            anchor.add(groupRef.current);
            
            // Set local transform to identity to align with the frame
            groupRef.current.position.set(0, 0, 0);
            groupRef.current.rotation.set(0, 0, 0);
        }
    }, [graph.frameId, scene]);

    // Handle cluster click with drag filtering
    const handleClusterClick = (clusterId: number, e: ThreeEvent<MouseEvent>) => {
        if (!selectionEnabled || !onClusterSelect) return;
        e.stopPropagation();

        if (dragStartRef.current) {
            const dx = e.clientX - dragStartRef.current.x;
            const dy = e.clientY - dragStartRef.current.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 5) return;
        }

        if (selectedClusterId === clusterId) {
            onClusterSelect(null);
        } else {
            onClusterSelect(clusterId);
        }
    };

    // --- Node sphere geometry (shared) ---
    const nodeSphereGeometry = useMemo(() => new THREE.SphereGeometry(1, 12, 8), []);

    // --- Node material ---
    const nodeMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: '#ffffff',
        transparent: true,
        opacity: opacity,
        depthTest: false,
        depthWrite: false,
    }), [opacity]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);

    useEffect(() => {
        if (graph.nodes.length > nodeCapacity) {
            setNodeCapacity(graph.nodes.length);
        }
    }, [graph.nodes.length, nodeCapacity]);

    // --- Update instanced nodes (Split into Matrix and Color for efficiency) ---
    const lastNodeCountRef = useRef<number>(0);

    // 1. Matrix Update (Positions) - Only when node count or data structure changes
    useEffect(() => {
        if (!nodesRef.current || !showNodes) return;
        if (graph.nodes.length === 0) return;
        if (graph.nodes.length > nodeCapacity) return;

        // In Static mode, we only need to set matrices once (or when nodes move in their local frame)
        // For urdf_trainer nodes, they NEVER move relative to their frame.
        const tempMatrix = new THREE.Matrix4();
        nodesRef.current.count = graph.nodes.length;

        graph.nodes.forEach((node, i) => {
            tempMatrix.makeTranslation(node.x, node.y, node.z);
            tempMatrix.scale(new THREE.Vector3(nodeScale, nodeScale, nodeScale));
            nodesRef.current!.setMatrixAt(i, tempMatrix);
        });

        nodesRef.current.instanceMatrix.needsUpdate = true;
        lastNodeCountRef.current = graph.nodes.length;
    }, [graph.nodes.length, showNodes, nodeScale, nodeCapacity]); // Note: only depend on length for matrices in static mode

    useEffect(() => {
        if (!nodesRef.current || showNodes) return;

        // When nodes are hidden, clear any previously uploaded instances so the layer
        // does not keep stale geometry around when the mesh is toggled back on.
        nodesRef.current.count = 0;
        nodesRef.current.instanceMatrix.needsUpdate = true;
        if (nodesRef.current.instanceColor) {
            nodesRef.current.instanceColor.needsUpdate = true;
        }
    }, [showNodes]);

    // 2. Color Update (Status) - When labels change (e.g. collision/danger)
    useEffect(() => {
        if (!nodesRef.current || !showNodes || graph.nodes.length === 0) return;
        
        const tempColor = new THREE.Color();
        const nodes = graph.nodes;

        for (let i = 0; i < nodes.length; i++) {
            const labelValue = Number.isFinite(nodes[i].label) ? Math.trunc(nodes[i].label) : 0;
            const safeIndex = ((labelValue % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            const colorHex = LAYER_COLORS[safeIndex] ?? LAYER_COLORS[0];
            tempColor.set(colorHex);
            nodesRef.current!.setColorAt(i, tempColor);
        }

        if (nodesRef.current.instanceColor) {
            nodesRef.current.instanceColor.needsUpdate = true;
        }
    }, [graph.nodes, showNodes]); // Depend on full nodes array for color/label updates

    // --- Edge cylinder geometry (shared) ---
    const edgeCylinderGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 6), []);

    // --- Edge material ---
    const edgeMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: '#08d408',
        transparent: true,
        opacity: opacity,
        depthTest: false,
        depthWrite: false,
    }), [opacity]);

    const edgePairCount = useMemo(() => Math.floor(graph.edges.length / 2), [graph.edges]);
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

        // Edges in static mode also don't change their matrices unless nodes move locally
        const tempMatrix = new THREE.Matrix4();
        const up = new THREE.Vector3(0, 1, 0);
        const direction = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();

        edgesRef.current.count = edgePairCount;

        for (let i = 0; i < edgePairCount; i++) {
            const srcIdx = graph.edges[i * 2];
            const tgtIdx = graph.edges[i * 2 + 1];

            if (srcIdx >= graph.nodes.length || tgtIdx >= graph.nodes.length) {
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
    }, [graph.nodes.length, edgePairCount, showEdges, edgeWidth, edgeCapacity]);

    useEffect(() => {
        if (!edgesRef.current || showEdges) return;

        // Clear stale edge instances when hidden.
        edgesRef.current.count = 0;
        edgesRef.current.instanceMatrix.needsUpdate = true;
    }, [showEdges]);

    if (!data || !visible) return null;

    const canRenderNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canRenderEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;

    return (
        <group ref={groupRef}>
            {canRenderNodes && (
                <instancedMesh
                    key={`static-nodes-${nodeCapacity}-${showNodes ? 'on' : 'off'}`}
                    ref={nodesRef}
                    args={[nodeSphereGeometry, nodeMaterial, nodeCapacity]}
                    count={graph.nodes.length}
                    frustumCulled={false}
                    renderOrder={10}
                />
            )}

            {canRenderEdges && (
                <instancedMesh
                    key={`static-edges-${edgeCapacity}-${showEdges ? 'on' : 'off'}`}
                    ref={edgesRef}
                    args={[edgeCylinderGeometry, edgeMaterial, edgeCapacity]}
                    count={edgePairCount}
                    frustumCulled={false}
                    renderOrder={9}
                />
            )}

            {showClusters && graph.clusters
                .filter(cluster => !visibleLabels || visibleLabels[cluster.label as 0 | 1 | 2 | 3 | 4 | 5])
                .map((cluster) => {
                    const isSelected = selectedClusterId === cluster.id;
                    const color = isSelected ? '#FFFFFF' : LAYER_COLORS[cluster.label % LAYER_COLORS.length];
                    const isHuman = cluster.label === 4; 
                    const handlePointerDown = selectionEnabled
                        ? (e: ThreeEvent<PointerEvent>) => { dragStartRef.current = { x: e.clientX, y: e.clientY }; }
                        : undefined;
                    const handleClick = selectionEnabled
                        ? (e: ThreeEvent<MouseEvent>) => handleClusterClick(cluster.id, e)
                        : undefined;

                    return (
                        <group key={cluster.id} position={cluster.pos} quaternion={new THREE.Quaternion(...cluster.quat)}>
                            <mesh
                                scale={isHuman ? [cluster.scale[0], cluster.scale[2], cluster.scale[1]] : cluster.scale}
                                rotation={isHuman ? [Math.PI / 2, 0, 0] : [0, 0, 0]}
                                onPointerDown={handlePointerDown}
                                onClick={handleClick}
                            >
                                {isHuman ? <cylinderGeometry args={[0.5, 0.5, 1, 16]} /> : <boxGeometry args={[1, 1, 1]} />}
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
                                <Billboard position={[0, 0, cluster.scale[2] / 2 + 0.2]}>
                                    <Text fontSize={0.2} color="#FFFFFF" anchorX="center" anchorY="bottom">
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
