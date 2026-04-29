import { useMemo, useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree, ThreeEvent } from '@react-three/fiber';
import { Billboard, Text } from '@react-three/drei';
import { GraphData, LAYER_COLORS, LAYER_LABELS } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { updateNodeInstances, updateEdgeInstances } from './utils/gngGraphics';

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
    opacity = 1.0,
    tf = null,
}: GraphRendererProps) {
    const { invalidate } = useThree();
    const groupRef = useRef<THREE.Group>(null);
    const nodesRef = useRef<THREE.InstancedMesh>(null);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    
    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;

    // Trigger re-render in demand mode for any visual changes
    useDemandUpdate([graph, visible, showNodes, showEdges, showClusters, nodeScale, edgeWidth, opacity, tf, selectedClusterId]);

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
    }, [tf]);

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

    // --- Geometries & Materials ---
    const nodeSphereGeometry = useMemo(() => new THREE.SphereGeometry(1, 12, 8), []);
    const nodeMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: '#c8ff4a',
        transparent: true,
        opacity: opacity,
        depthTest: false,
        depthWrite: false,
    }), [opacity]);

    const edgeCylinderGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 6), []);
    const edgeMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: '#b9ff3f',
        transparent: true,
        opacity: opacity,
        depthTest: false,
        depthWrite: false,
    }), [opacity]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);
    const edgePairCount = useMemo(() => Math.floor(graph.edges.length / 2), [graph.edges]);
    const [edgeCapacity, setEdgeCapacity] = useState(edgePairCount);

    useEffect(() => {
        if (graph.nodes.length > nodeCapacity) setNodeCapacity(graph.nodes.length);
    }, [graph.nodes.length, nodeCapacity]);

    useEffect(() => {
        if (edgePairCount > edgeCapacity) setEdgeCapacity(edgePairCount);
    }, [edgePairCount, edgeCapacity]);

    // --- Node Instances ---
    useEffect(() => {
        if (!nodesRef.current || !showNodes || graph.nodes.length === 0) return;
        if (graph.nodes.length > nodeCapacity) return;

        updateNodeInstances(nodesRef.current, graph.nodes, nodeScale);
        invalidate(); 
    }, [graph.nodes, showNodes, nodeScale, nodeCapacity, invalidate]);

    useEffect(() => {
        if (!nodesRef.current || showNodes) return;
        nodesRef.current.count = 0;
        nodesRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showNodes, invalidate]);

    // --- Edge Instances ---
    useEffect(() => {
        if (!edgesRef.current || !showEdges || edgePairCount === 0) return;
        if (edgePairCount > edgeCapacity) return;

        updateEdgeInstances(edgesRef.current, graph.edges, graph.nodes, edgeWidth);
        invalidate();
    }, [graph.edges, graph.nodes, showEdges, edgeWidth, edgeCapacity, edgePairCount, invalidate]);

    useEffect(() => {
        if (!edgesRef.current || showEdges) return;
        edgesRef.current.count = 0;
        edgesRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showEdges, invalidate]);

    if (!data || !visible) return null;

    const canRenderNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canRenderEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;

    return (
        <group ref={groupRef}>
            {canRenderNodes && (
                <instancedMesh
                    key={`static-nodes-${nodeCapacity}`}
                    ref={nodesRef}
                    args={[nodeSphereGeometry, nodeMaterial, nodeCapacity]}
                    count={graph.nodes.length}
                    frustumCulled={false}
                    renderOrder={10}
                />
            )}

            {canRenderEdges && (
                <instancedMesh
                    key={`static-edges-${edgeCapacity}`}
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
                    const handlePointerDown = (e: ThreeEvent<PointerEvent>) => {
                        if (selectionEnabled) dragStartRef.current = { x: e.clientX, y: e.clientY };
                    };

                    return (
                        <group key={cluster.id} position={cluster.pos} quaternion={new THREE.Quaternion(...cluster.quat)}>
                            <mesh
                                scale={isHuman ? [cluster.scale[0], cluster.scale[2], cluster.scale[1]] : cluster.scale}
                                rotation={isHuman ? [Math.PI / 2, 0, 0] : [0, 0, 0]}
                                onPointerDown={handlePointerDown}
                                onClick={(e) => handleClusterClick(cluster.id, e as any)}
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
