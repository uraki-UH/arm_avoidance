import { useMemo, useRef, useEffect, useLayoutEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree, ThreeEvent } from '@react-three/fiber';
import { Billboard, Text } from '@react-three/drei';
import { GraphData, Transform, LAYER_COLORS, LAYER_LABELS, DYNAMIC_GNG_DEFAULTS } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { buildNodePalette, updateNodeInstances, updateEdgeInstances } from './utils/gngGraphics';
import { DirectionalArrow } from './utils/DirectionalArrow';

const EMPTY_GRAPH: GraphData = {
    timestamp: 0,
    nodes: [],
    edges: [],
    clusters: [],
    clusterLabels: []
};

// removed


interface GraphRendererProps {
    tag: string;
    data: GraphData | null;
    visible?: boolean;
    showNodes?: boolean;
    showEdges?: boolean;
    showClusters?: boolean;
    showClusterText?: boolean;
    showNormals?: boolean;
    showVelocity?: boolean;
    nodeScale?: number;
    edgeWidth?: number;
    normalScale?: number;
    velocityScale?: number;
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
    nodeColor?: string;
    edgeColor?: string;
    normalColor?: string;
    velocityColor?: string;
    nodeEmissiveIntensity?: number;
    edgeEmissiveIntensity?: number;
    manualTransform?: Transform | null;
}

export function GraphRenderer({
    tag,
    data,
    visible = true,
    showNodes = true,
    showEdges = true,
    showClusters = true,
    showClusterText = true,
    showNormals = false,
    showVelocity = false,
    nodeScale = 0.015,
    edgeWidth = 0.007,
    normalScale = 0.075,
    velocityScale = 0.25,
    visibleLabels,
    selectedClusterId = null,
    onClusterSelect,
    enableClusterSelection = true,
    opacity = 1.0,
    tf = null,
    nodeColor = '#7c8c66',
    edgeColor = '#08d408',
    normalColor = '#00ffff',
    velocityColor = '#ffb347',
    nodeEmissiveIntensity = DYNAMIC_GNG_DEFAULTS.nodeEmissiveIntensity,
    edgeEmissiveIntensity = DYNAMIC_GNG_DEFAULTS.edgeEmissiveIntensity,
    manualTransform = null,
}: GraphRendererProps) {
    const { invalidate } = useThree();
    const groupRef = useRef<THREE.Group>(null);
    const nodeMeshRefs = useRef<(THREE.InstancedMesh | null)[]>([]);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);

    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;
    const transform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };
    const nodePalette = useMemo(() => buildNodePalette(nodeColor), [nodeColor]);
    const nodeBuckets = useMemo(() => {
        const buckets: GraphData['nodes'][] = Array.from(
            { length: LAYER_COLORS.length },
            () => []
        );
        for (const node of graph.nodes) {
            const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
            const labelIndex = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            buckets[labelIndex].push(node);
        }
        return buckets;
    }, [graph.nodes]);

    // Trigger re-render in demand mode for any visual changes
    useDemandUpdate([graph, visible, showNodes, showEdges, showClusters, showNormals, showVelocity, nodeScale, edgeWidth, normalScale, velocityScale, opacity, tf, selectedClusterId, nodeColor, edgeColor, normalColor, velocityColor, nodeEmissiveIntensity, edgeEmissiveIntensity, transform]);

    // --- TF-based Positioning ---
    useLayoutEffect(() => {
        if (!groupRef.current) return;
        if (!tf) {
            groupRef.current.position.set(0, 0, 0);
            groupRef.current.quaternion.set(0, 0, 0, 1);
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
    const nodeMaterials = useMemo(() => nodePalette.map((color) => new THREE.MeshStandardMaterial({
        color,
        emissive: new THREE.Color(color),
        emissiveIntensity: nodeEmissiveIntensity,
        vertexColors: true,
        transparent: opacity < 1,
        opacity,
        depthTest: true,
        depthWrite: false,
        toneMapped: false,
    })), [opacity, nodeEmissiveIntensity, nodePalette]);

    const edgeCylinderGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 6), []);
    const edgeMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: edgeColor,
        emissive: new THREE.Color(edgeColor),
        emissiveIntensity: edgeEmissiveIntensity,
        transparent: true,
        opacity: opacity,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
    }), [opacity, edgeColor, edgeEmissiveIntensity]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);
    const edgePairCount = useMemo(() => Math.floor(graph.edges.length / 2), [graph.edges]);
    const [edgeCapacity, setEdgeCapacity] = useState(edgePairCount);
    const [nodeReadySignature, setNodeReadySignature] = useState<string | null>(null);
    const [edgeReadySignature, setEdgeReadySignature] = useState<string | null>(null);

    const nodeRenderSignature = useMemo(() => {
        return [
            graph.nodes.length,
            showNodes ? 1 : 0,
            nodeScale,
            nodeCapacity,
            opacity,
            nodeColor,
            nodeEmissiveIntensity,
            graph.timestamp,
        ].join(':');
    }, [graph.nodes.length, showNodes, nodeScale, nodeCapacity, opacity, nodeColor, nodeEmissiveIntensity, graph.timestamp]);

    const edgeRenderSignature = useMemo(() => {
        return [
            edgePairCount,
            showEdges ? 1 : 0,
            edgeWidth,
            edgeCapacity,
            opacity,
            edgeColor,
            edgeEmissiveIntensity,
            graph.timestamp,
        ].join(':');
    }, [edgePairCount, showEdges, edgeWidth, edgeCapacity, opacity, edgeColor, edgeEmissiveIntensity, graph.timestamp]);
    const nodeRenderReady = nodeReadySignature === nodeRenderSignature;
    const edgeRenderReady = edgeReadySignature === edgeRenderSignature;

    useEffect(() => {
        if (graph.nodes.length > nodeCapacity) setNodeCapacity(graph.nodes.length);
    }, [graph.nodes.length, nodeCapacity]);

    useEffect(() => {
        if (edgePairCount > edgeCapacity) setEdgeCapacity(edgePairCount);
    }, [edgePairCount, edgeCapacity]);

    // --- Node Instances ---
    useLayoutEffect(() => {
        if (!showNodes || graph.nodes.length === 0) return;
        if (graph.nodes.length > nodeCapacity) return;

        nodeBuckets.forEach((bucket, labelIndex) => {
            const solidMesh = nodeMeshRefs.current[labelIndex];
            if (solidMesh) {
                updateNodeInstances(solidMesh, bucket, nodeScale, {
                    colorMode: 'uniform',
                    uniformColor: nodePalette[labelIndex] ?? LAYER_COLORS[labelIndex] ?? nodePalette[0],
                });
            }
        });
        setNodeReadySignature(nodeRenderSignature);
        invalidate();
    }, [graph.nodes, nodeBuckets, showNodes, nodeScale, nodeCapacity, nodeRenderSignature, invalidate]);

    useLayoutEffect(() => {
        if (showNodes) return;
        setNodeReadySignature(null);
        nodeMeshRefs.current.forEach((mesh) => {
            if (!mesh) return;
            mesh.count = 0;
            mesh.instanceMatrix.needsUpdate = true;
        });
        invalidate();
    }, [showNodes, invalidate]);

    // --- Edge Instances ---
    useLayoutEffect(() => {
        if (!edgesRef.current || !showEdges || edgePairCount === 0) return;
        if (edgePairCount > edgeCapacity) return;

        updateEdgeInstances(edgesRef.current, graph.edges, graph.nodes, edgeWidth);
        setEdgeReadySignature(edgeRenderSignature);
        invalidate();
    }, [graph.edges, graph.nodes, showEdges, edgeWidth, edgeCapacity, edgePairCount, edgeRenderSignature, invalidate]);

    useLayoutEffect(() => {
        if (showEdges) return;
        setEdgeReadySignature(null);
        if (!edgesRef.current) return;
        edgesRef.current.count = 0;
        edgesRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showEdges, invalidate]);

    if (!data || !visible) return null;

    const canMountNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canMountEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;
    const canMountNormals = showNormals && canMountNodes;
    const canMountVelocity = showVelocity && showClusters && graph.clusters.length > 0;

    const content = (
        <>
            {canMountNodes && LAYER_COLORS.map((_, labelIndex) => (
                <group key={`node-label-${labelIndex}`}>
                    <instancedMesh
                        key={`nodes-${labelIndex}-${nodeCapacity}`}
                        ref={(el) => { nodeMeshRefs.current[labelIndex] = el; }}
                        args={[nodeSphereGeometry, nodeMaterials[labelIndex], nodeCapacity]}
                        count={nodeRenderReady ? nodeBuckets[labelIndex].length : 0}
                        frustumCulled={false}
                        renderOrder={10}
                    />
                </group>
            ))}

            {canMountEdges && (
                <instancedMesh
                    key={`edges-${edgeCapacity}`}
                    ref={edgesRef}
                    args={[edgeCylinderGeometry, edgeMaterial, edgeCapacity]}
                    count={edgeRenderReady ? edgePairCount : 0}
                    frustumCulled={false}
                    renderOrder={9}
                />
            )}

            {canMountNormals && graph.nodes.map((node, index) => (
                <DirectionalArrow
                    key={`normal-${index}`}
                    origin={[node.x, node.y, node.z]}
                    direction={[node.nx, node.ny, node.nz]}
                    lengthScale={normalScale}
                    maxLength={0.35}
                    color={normalColor}
                    visible={showNormals}
                    headLengthRatio={0.24}
                    headWidthRatio={0.16}
                    shaftWidth={0.006}
                />
            ))}

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

                        {canMountVelocity && (
                            <DirectionalArrow
                                origin={cluster.pos}
                                direction={cluster.velocity}
                                lengthScale={velocityScale}
                                maxLength={0.5}
                                color={velocityColor}
                                visible={showVelocity}
                                headLengthRatio={0.28}
                                headWidthRatio={0.18}
                                shaftWidth={0.008}
                            />
                        )}
                    </group>
                );
            })}
        </>
    );

    return (
        <group ref={groupRef} name={tag}>
            <group
                position={transform.position}
                rotation={transform.rotation}
                scale={transform.scale}
            >
                {content}
            </group>
        </group>
    );
}
