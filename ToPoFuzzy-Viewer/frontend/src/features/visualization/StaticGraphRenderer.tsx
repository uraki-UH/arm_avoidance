import { useMemo, useRef, useEffect, useLayoutEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree, ThreeEvent } from '@react-three/fiber';
import { Billboard, Text } from '@react-three/drei';
import { GraphData, Transform, LAYER_COLORS, LAYER_LABELS, SEMANTIC_LABELS, SEMANTIC_COLORS, STATIC_GNG_DEFAULTS } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { buildNodePalette, updateNodeInstances, updateEdgeInstances } from './utils/gngGraphics';
import { DirectionalArrow } from './utils/DirectionalArrow';
import { updateEllipsoidInstances } from './utils/ellipsoid';

const EMPTY_GRAPH: GraphData = {
    timestamp: 0,
    nodes: [],
    edges: [],
    clusters: [],
    clusterLabels: []
};

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
    showCovarianceEllipsoids?: boolean;
    showManipulabilityEllipsoids?: boolean;
    manipEllipsoidMode?: 'all' | 'goal';
    manipEllipsoidType?: 'translational' | 'rotational' | 'both';
    nodeScale?: number;
    edgeWidth?: number;
    normalScale?: number;
    velocityScale?: number;
    covarianceEllipsoidScale?: number;
    visibleLabels?: {
        0: boolean;
        1: boolean;
        2: boolean;
        3: boolean;
        4: boolean;
        5: boolean;
    };
    visibleSemanticLabels?: {
        handle: boolean;
    };
    selectedClusterId?: number | null;
    onClusterSelect?: (clusterId: number | null) => void;
    onManipSelect?: (node: GraphData['nodes'][number]) => void;
    enableClusterSelection?: boolean;
    opacity?: number;
    tf?: { pos: number[]; quat: number[] } | null;
    nodeColor?: string;
    edgeColor?: string;
    normalColor?: string;
    velocityColor?: string;
    covarianceEllipsoidColor?: string;
    nodeEmissiveIntensity?: number;
    edgeEmissiveIntensity?: number;
    manualTransform?: Transform | null;
}

export function StaticGraphRenderer({
    tag,
    data,
    visible = true,
    showNodes = true,
    showEdges = true,
    showClusters = true,
    showClusterText = true,
    nodeScale = 0.005,
    edgeWidth = 0.003,
    visibleLabels,
    selectedClusterId = null,
    onClusterSelect,
    onManipSelect,
    enableClusterSelection = true,
    opacity = STATIC_GNG_DEFAULTS.opacity,
    tf = null,
    nodeColor = STATIC_GNG_DEFAULTS.nodeColor,
    edgeColor = STATIC_GNG_DEFAULTS.edgeColor,
    showNormals = false,
    showVelocity = false,
    showCovarianceEllipsoids = false,
    showManipulabilityEllipsoids = false,
    manipEllipsoidMode = 'all',
    manipEllipsoidType = 'translational',
    normalScale = 0.075,
    velocityScale = 0.25,
    covarianceEllipsoidScale = 2.0,
    normalColor = '#00ffff',
    velocityColor = '#ffb347',
    covarianceEllipsoidColor = '#aefeff',
    nodeEmissiveIntensity = STATIC_GNG_DEFAULTS.nodeEmissiveIntensity,
    edgeEmissiveIntensity = STATIC_GNG_DEFAULTS.edgeEmissiveIntensity,
    manualTransform = null,
    visibleSemanticLabels,
}: GraphRendererProps) {
    const manipDisplayScale = 0.25;
    const { invalidate } = useThree();
    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;
    const transform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };
    const nodePalette = useMemo(() => buildNodePalette(nodeColor), [nodeColor]);
    const semanticColorEnabled = visibleSemanticLabels?.handle ?? true;
    const semanticLabelText = (semanticLabel?: number) => {
        if (!Number.isFinite(semanticLabel) || (semanticLabel ?? 0) <= 0) return '';
        return SEMANTIC_LABELS[(Math.trunc(semanticLabel as number) - 1) % SEMANTIC_LABELS.length] || 'HANDLE';
    };
    const nodeSemanticLabels = useMemo(() => {
        const labels = new Map<number, number>();
        for (const cluster of graph.clusters) {
            const semanticLabel = Number.isFinite(cluster.semanticLabel) ? Math.trunc(cluster.semanticLabel as number) : 0;
            if (semanticLabel <= 0) continue;
            for (const nodeId of cluster.nodeIds || []) {
                if (!labels.has(nodeId)) {
                    labels.set(nodeId, semanticLabel);
                }
            }
        }
        return labels;
    }, [graph.clusters]);
    const ellipsoidRef = useRef<THREE.InstancedMesh>(null);
    const manipEllipsoidRef = useRef<THREE.InstancedMesh>(null);

    // Trigger re-render in demand mode for any visual changes
    useDemandUpdate([graph, visible, showNodes, showEdges, showClusters, showNormals, showVelocity, showCovarianceEllipsoids, showManipulabilityEllipsoids, manipEllipsoidMode, nodeScale, edgeWidth, normalScale, velocityScale, covarianceEllipsoidScale, opacity, tf, visibleLabels, selectedClusterId, nodeColor, edgeColor, normalColor, velocityColor, covarianceEllipsoidColor, nodeEmissiveIntensity, edgeEmissiveIntensity, transform]);

    const groupRef = useRef<THREE.Group>(null);
    const nodeMeshRefs = useRef<(THREE.InstancedMesh | null)[]>([]);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    const nodeBuckets = useMemo(() => {
        const buckets = Array.from({ length: LAYER_COLORS.length }, () => ({
            base: [] as GraphData['nodes'],
            semantic: [] as GraphData['nodes'],
        }));
        graph.nodes.forEach((node, nodeIndex) => {
            const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
            const labelIndex = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            const semanticLabel = Number.isFinite(node.semanticLabel)
                ? Math.trunc(node.semanticLabel as number)
                : (Number.isFinite(node.id) ? (nodeSemanticLabels.get(node.id as number) || 0) : (nodeSemanticLabels.get(nodeIndex) || 0));
            if (visibleLabels && !visibleLabels[labelIndex as 0 | 1 | 2 | 3 | 4 | 5]) {
                return;
            }
            const nextNode = {
                ...node,
                semanticLabel: semanticColorEnabled && semanticLabel > 0 ? semanticLabel : undefined,
            };
            if (semanticColorEnabled && semanticLabel > 0) {
                buckets[labelIndex].semantic.push(nextNode);
            } else {
                buckets[labelIndex].base.push(nextNode);
            }
        });
        return buckets;
    }, [graph.nodes, graph.clusters, nodeSemanticLabels, visibleLabels, semanticColorEnabled]);

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


// Removed imperative offset logic


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
        transparent: opacity < 1,
        opacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    })), [nodePalette, opacity, nodeEmissiveIntensity]);
    const semanticMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: SEMANTIC_COLORS[0] ?? '#00d1ff',
        emissive: new THREE.Color(SEMANTIC_COLORS[0] ?? '#00d1ff'),
        emissiveIntensity: nodeEmissiveIntensity,
        transparent: opacity < 1,
        opacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    }), [opacity, nodeEmissiveIntensity]);

    const edgeCylinderGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 6), []);
    const edgeMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: edgeColor,
        emissive: new THREE.Color(edgeColor),
        emissiveIntensity: edgeEmissiveIntensity,
        transparent: opacity < 1,
        opacity: opacity,
        depthTest: true,
        depthWrite: false,
        toneMapped: false,
    }), [opacity, edgeColor, edgeEmissiveIntensity]);
    const ellipsoidGeometry = useMemo(() => new THREE.SphereGeometry(1, 16, 12), []);
    const ellipsoidMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: covarianceEllipsoidColor,
        transparent: true,
        opacity: Math.max(0.18, Math.min(0.6, opacity * 0.35)),
        depthTest: true,
        depthWrite: false,
        emissive: new THREE.Color(covarianceEllipsoidColor),
        emissiveIntensity: 0.8,
        roughness: 0.35,
        metalness: 0.0,
        toneMapped: false,
    }), [covarianceEllipsoidColor, opacity]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);
    const edgePairCount = useMemo(() => Math.floor(graph.edges.length / 2), [graph.edges]);
    const [edgeCapacity, setEdgeCapacity] = useState(edgePairCount);
    const [ellipsoidCapacity, setEllipsoidCapacity] = useState(graph.nodes.length);
    const [manipEllipsoidCapacity, setManipEllipsoidCapacity] = useState(graph.nodes.length);
    const [nodeReadySignature, setNodeReadySignature] = useState<string | null>(null);
    const [edgeReadySignature, setEdgeReadySignature] = useState<string | null>(null);
    const [ellipsoidReadySignature, setEllipsoidReadySignature] = useState<string | null>(null);
    const [manipEllipsoidReadySignature, setManipEllipsoidReadySignature] = useState<string | null>(null);

    const covarianceEllipsoids = useMemo(() => {
        if (!showCovarianceEllipsoids) return [];
        return graph.nodes
            .filter((node) => (node.winnerPointCount ?? 0) > 0 && Array.isArray(node.winnerPointCovariance))
            .map((node) => {
                const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
                const labelIndex = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
                return {
                    center: [node.x, node.y, node.z] as [number, number, number],
                    covariance: node.winnerPointCovariance as [number, number, number, number, number, number, number, number, number],
                    color: covarianceEllipsoidColor || nodePalette[labelIndex] || '#7fd9ff',
                };
            });
    }, [graph.nodes, showCovarianceEllipsoids, covarianceEllipsoidColor, nodePalette]);

    const manipulabilityEllipsoids = useMemo(() => {
        if (!showManipulabilityEllipsoids) return [];
        const type = manipEllipsoidType ?? 'translational';
        const list: Array<{
            node: any;
            center: [number, number, number];
            scale: [number, number, number];
            quaternion: [number, number, number, number];
            color: string;
        }> = [];

        graph.nodes
            .filter((node) => manipEllipsoidMode === 'all' || !!node.isGoal)
            .forEach((node) => {
                const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
                const labelIndex = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
                const baseColor = covarianceEllipsoidColor || nodePalette[labelIndex] || '#7fd9ff';

                if ((type === 'translational' || type === 'both') && node.manipValid && node.manipScale && node.manipOrientation) {
                    list.push({
                        node,
                        center: [node.x, node.y, node.z] as [number, number, number],
                        scale: node.manipScale as [number, number, number],
                        quaternion: node.manipOrientation as [number, number, number, number],
                        color: baseColor,
                    });
                }

                if ((type === 'rotational' || type === 'both') && node.rotationalManipValid && node.rotationalManipScale && node.rotationalManipOrientation) {
                    list.push({
                        node,
                        center: [node.x, node.y, node.z] as [number, number, number],
                        scale: node.rotationalManipScale as [number, number, number],
                        quaternion: node.rotationalManipOrientation as [number, number, number, number],
                        color: '#ff7f50', // coral for rotational
                    });
                }
            });
        return list;
    }, [graph.nodes, showManipulabilityEllipsoids, manipEllipsoidMode, manipEllipsoidType, covarianceEllipsoidColor, nodePalette]);

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
    const ellipsoidRenderSignature = useMemo(() => {
        return [
            covarianceEllipsoids.length,
            showCovarianceEllipsoids ? 1 : 0,
            covarianceEllipsoidScale,
            ellipsoidCapacity,
            opacity,
            covarianceEllipsoidColor,
            graph.timestamp,
        ].join(':');
    }, [covarianceEllipsoids.length, showCovarianceEllipsoids, covarianceEllipsoidScale, ellipsoidCapacity, opacity, covarianceEllipsoidColor, graph.timestamp]);
    const ellipsoidRenderReady = ellipsoidReadySignature === ellipsoidRenderSignature;
    const manipEllipsoidRenderSignature = useMemo(() => {
        return [
            manipulabilityEllipsoids.length,
            showManipulabilityEllipsoids ? 1 : 0,
            manipEllipsoidMode,
            manipEllipsoidType,
            manipEllipsoidCapacity,
            opacity,
            covarianceEllipsoidColor,
            graph.timestamp,
        ].join(':');
    }, [manipulabilityEllipsoids.length, showManipulabilityEllipsoids, manipEllipsoidMode, manipEllipsoidType, manipEllipsoidCapacity, opacity, covarianceEllipsoidColor, graph.timestamp]);
    const manipEllipsoidRenderReady = manipEllipsoidReadySignature === manipEllipsoidRenderSignature;

    useEffect(() => {
        if (graph.nodes.length > nodeCapacity) setNodeCapacity(graph.nodes.length);
    }, [graph.nodes.length, nodeCapacity]);

    useEffect(() => {
        if (edgePairCount > edgeCapacity) setEdgeCapacity(edgePairCount);
    }, [edgePairCount, edgeCapacity]);

    useEffect(() => {
        if (covarianceEllipsoids.length > ellipsoidCapacity) setEllipsoidCapacity(covarianceEllipsoids.length);
    }, [covarianceEllipsoids.length, ellipsoidCapacity]);

    useEffect(() => {
        if (manipulabilityEllipsoids.length > manipEllipsoidCapacity) setManipEllipsoidCapacity(manipulabilityEllipsoids.length);
    }, [manipulabilityEllipsoids.length, manipEllipsoidCapacity]);

    // --- Node Instances ---
    useLayoutEffect(() => {
        if (!showNodes || graph.nodes.length === 0) return;
        if (graph.nodes.length > nodeCapacity) return;

        nodeBuckets.forEach((bucket, labelIndex) => {
            const baseMesh = nodeMeshRefs.current[labelIndex * 2];
            const semanticMesh = nodeMeshRefs.current[labelIndex * 2 + 1];
            if (baseMesh) {
                updateNodeInstances(baseMesh, bucket.base, nodeScale);
            }
            if (semanticMesh) {
                updateNodeInstances(semanticMesh, bucket.semantic, nodeScale);
            }
        });
        setNodeReadySignature(nodeRenderSignature);
        invalidate();
    }, [graph.nodes, nodeBuckets, showNodes, nodeScale, nodeCapacity, nodeRenderSignature, invalidate, semanticColorEnabled]);

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
        if (!edgesRef.current || showEdges) return;
        setEdgeReadySignature(null);
        edgesRef.current.count = 0;
        edgesRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showEdges, invalidate]);

    useLayoutEffect(() => {
        if (!ellipsoidRef.current || !showCovarianceEllipsoids || covarianceEllipsoids.length === 0) return;
        if (covarianceEllipsoids.length > ellipsoidCapacity) return;

        updateEllipsoidInstances(ellipsoidRef.current, covarianceEllipsoids, {
            defaultColor: covarianceEllipsoidColor,
            sigmaMultiplier: covarianceEllipsoidScale,
        });
        setEllipsoidReadySignature(ellipsoidRenderSignature);
        invalidate();
    }, [covarianceEllipsoids, showCovarianceEllipsoids, covarianceEllipsoidScale, ellipsoidCapacity, covarianceEllipsoidColor, ellipsoidRenderSignature, invalidate]);

    useLayoutEffect(() => {
        if (!manipEllipsoidRef.current || !showManipulabilityEllipsoids || manipulabilityEllipsoids.length === 0) return;
        if (manipulabilityEllipsoids.length > manipEllipsoidCapacity) return;

        updateEllipsoidInstances(manipEllipsoidRef.current, manipulabilityEllipsoids, {
            defaultColor: covarianceEllipsoidColor,
            sigmaMultiplier: manipDisplayScale,
        });
        setManipEllipsoidReadySignature(manipEllipsoidRenderSignature);
        invalidate();
    }, [manipulabilityEllipsoids, showManipulabilityEllipsoids, manipEllipsoidCapacity, covarianceEllipsoidColor, manipEllipsoidRenderSignature, invalidate]);

    useLayoutEffect(() => {
        if (showCovarianceEllipsoids) return;
        setEllipsoidReadySignature(null);
        if (!ellipsoidRef.current) return;
        ellipsoidRef.current.count = 0;
        ellipsoidRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showCovarianceEllipsoids, invalidate]);

    useLayoutEffect(() => {
        if (showManipulabilityEllipsoids) return;
        setManipEllipsoidReadySignature(null);
        if (!manipEllipsoidRef.current) return;
        manipEllipsoidRef.current.count = 0;
        manipEllipsoidRef.current.instanceMatrix.needsUpdate = true;
        invalidate();
    }, [showManipulabilityEllipsoids, invalidate]);

    if (!data || !visible) return null;

    const canMountNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canMountEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;
    const canMountNormals = showNormals && canMountNodes;
    const canMountCovarianceEllipsoids = showCovarianceEllipsoids && covarianceEllipsoids.length > 0 && ellipsoidCapacity >= covarianceEllipsoids.length;
    const canMountManipEllipsoids = showManipulabilityEllipsoids && manipulabilityEllipsoids.length > 0 && manipEllipsoidCapacity >= manipulabilityEllipsoids.length;
    const handleManipClick = (instanceId?: number) => {
        if (instanceId === undefined || instanceId === null) return;
        const picked = manipulabilityEllipsoids[instanceId];
        if (!picked?.node || !onManipSelect) return;
        onManipSelect(picked.node);
    };
    const canMountVelocity = showVelocity && showClusters && graph.clusters.length > 0;

    const content = (
        <>
            {canMountNodes && LAYER_COLORS.map((_, labelIndex) => (
                <group key={`static-node-label-${labelIndex}`}>
                    <instancedMesh
                        key={`static-nodes-base-${labelIndex}-${nodeCapacity}`}
                        ref={(el) => { nodeMeshRefs.current[labelIndex * 2] = el; }}
                        args={[nodeSphereGeometry, nodeMaterials[labelIndex], nodeCapacity]}
                        count={nodeRenderReady ? nodeBuckets[labelIndex].base.length : 0}
                        frustumCulled={false}
                        renderOrder={10}
                    />
                    {semanticColorEnabled && (
                        <instancedMesh
                            key={`static-nodes-semantic-${labelIndex}-${nodeCapacity}`}
                            ref={(el) => { nodeMeshRefs.current[labelIndex * 2 + 1] = el; }}
                            args={[nodeSphereGeometry, semanticMaterial, nodeCapacity]}
                            count={nodeRenderReady ? nodeBuckets[labelIndex].semantic.length : 0}
                            frustumCulled={false}
                            renderOrder={11}
                        />
                    )}
                </group>
            ))}

            {canMountEdges && (
                <instancedMesh
                    key={`static-edges-${edgeCapacity}`}
                    ref={edgesRef}
                    args={[edgeCylinderGeometry, edgeMaterial, edgeCapacity]}
                    count={edgeRenderReady ? edgePairCount : 0}
                    frustumCulled={false}
                    renderOrder={9}
                />
            )}

            {canMountCovarianceEllipsoids && (
                <instancedMesh
                    key={`static-cov-ellipsoids-${ellipsoidCapacity}`}
                    ref={ellipsoidRef}
                    args={[ellipsoidGeometry, ellipsoidMaterial, ellipsoidCapacity]}
                    count={ellipsoidRenderReady ? covarianceEllipsoids.length : 0}
                    frustumCulled={false}
                    renderOrder={8}
                />
            )}
            {canMountManipEllipsoids && (
                <instancedMesh
                    key={`static-manip-ellipsoids-${manipEllipsoidCapacity}`}
                    ref={manipEllipsoidRef}
                    args={[ellipsoidGeometry, ellipsoidMaterial, manipEllipsoidCapacity]}
                    count={manipEllipsoidRenderReady ? manipulabilityEllipsoids.length : 0}
                    frustumCulled={false}
                    renderOrder={7}
                    onClick={(e) => {
                        e.stopPropagation();
                        handleManipClick(e.instanceId);
                    }}
                />
            )}

            {canMountNormals && graph.nodes.map((node, index) => (
                <DirectionalArrow
                    key={`static-normal-${index}`}
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
                const semanticColor = semanticColorEnabled && Number.isFinite(cluster.semanticLabel) && (cluster.semanticLabel ?? 0) > 0
                    ? SEMANTIC_COLORS[(cluster.semanticLabel ?? 0) % SEMANTIC_COLORS.length] ?? SEMANTIC_COLORS[0]
                    : null;
                const color = isSelected ? '#FFFFFF' : (semanticColor || LAYER_COLORS[cluster.label % LAYER_COLORS.length]);
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
                            <meshBasicMaterial
                                color={color}
                                transparent
                                opacity={isSelected ? 0.1 : 0.3 * opacity}
                                depthWrite={false}
                                side={THREE.DoubleSide}
                            />
                        </mesh>

                        {showClusterText && (
                            <Billboard position={[0, 0, cluster.scale[2] / 2 + 0.2]}>
                                <Text fontSize={0.2} color="#FFFFFF" anchorX="center" anchorY="bottom">
                                {`${LAYER_LABELS[cluster.label] || 'obj'}${semanticLabelText(cluster.semanticLabel) ? ` / ${semanticLabelText(cluster.semanticLabel)}` : ''}\nR:${cluster.reliability.toFixed(2)}${Number.isFinite(cluster.semanticReliability) ? ` S:${cluster.semanticReliability!.toFixed(2)}` : ''}`}
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
