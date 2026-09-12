import { useMemo, useRef, useEffect, useLayoutEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree, ThreeEvent } from '@react-three/fiber';
import { Billboard, Text } from '@react-three/drei';
import { GraphData, GraphNode, Transform, LAYER_COLORS, LAYER_LABELS, SEMANTIC_LABELS, DYNAMIC_GNG_DEFAULTS, STATIC_GNG_DEFAULTS, isTrajectoryGraphTag } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { buildNodePalette, updateNodeInstances, updateEdgeInstances, configure_node_material } from './utils/gngGraphics';
import { ArrowBatch } from './arrows/ArrowBatch';
import { arrow_sample, normal_arrow_style, velocity_arrow_style } from './arrows/geometry';
import { build_cluster_node_colors } from './utils/clusterColors';
import { updateEllipsoidInstances } from './utils/ellipsoid';
import { get_active_node_labels, resolve_node_label } from './nodeLabelRegistry';
import type { node_label_options } from './nodeLabelRegistry';

const EMPTY_GRAPH: GraphData = {
    timestamp: 0,
    nodes: [],
    edges: [],
    clusters: [],
    clusterLabels: []
};

const CANDIDATE_GOAL_COLOR = '#a855f7';

// removed


interface GraphRendererProps {
    tag: string;
    data: GraphData | null;
    visible?: boolean;
    showNodes?: boolean;
    enable_cluster_colors?: boolean;
    label_settings?: node_label_options;
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
    covarianceEllipsoidScale?: number;
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
    onManipSelect?: (node: GraphNode) => void;
    enableClusterSelection?: boolean;
    nodeOpacity?: number;
    edgeOpacity?: number;
    tf?: { pos: number[]; quat: number[] } | null;
    nodeColor?: string;
    edgeColor?: string;
    covarianceEllipsoidColor?: string;
    nodeEmissiveIntensity?: number;
    edgeEmissiveIntensity?: number;
    manualTransform?: Transform | null;
}

interface GraphRendererCoreProps extends GraphRendererProps {
    variant: 'dynamic' | 'static';
}

function GraphRendererCore({
    variant,
    tag,
    data,
    visible = true,
    showNodes = true,
    enable_cluster_colors = /(^|\/)curved_surface_clusters$/.test(tag),
    label_settings,
    showEdges = true,
    showClusters = true,
    showClusterText = false,
    showNormals = false,
    showVelocity = false,
    showCovarianceEllipsoids = false,
    showManipulabilityEllipsoids = false,
    manipEllipsoidMode = 'all',
    manipEllipsoidType = 'translational',
    nodeScale = 0.005,
    edgeWidth = 0.003,
    covarianceEllipsoidScale = 2.0,
    visibleLabels,
    selectedClusterId = null,
    onClusterSelect,
    onManipSelect,
    enableClusterSelection = true,
    nodeOpacity = DYNAMIC_GNG_DEFAULTS.nodeOpacity,
    edgeOpacity = DYNAMIC_GNG_DEFAULTS.edgeOpacity,
    tf = null,
    nodeColor = '#81c720',
    edgeColor = '#08d408',
    covarianceEllipsoidColor = '#aefeff',
    nodeEmissiveIntensity = DYNAMIC_GNG_DEFAULTS.nodeEmissiveIntensity,
    edgeEmissiveIntensity = DYNAMIC_GNG_DEFAULTS.edgeEmissiveIntensity,
    manualTransform = null,
}: GraphRendererCoreProps) {
    const manipDisplayScale = 0.25;
    const { invalidate } = useThree();
    const groupRef = useRef<THREE.Group>(null);
    const nodeMeshRefs = useRef<(THREE.InstancedMesh | null)[]>([]);
    const goalNodeMeshRef = useRef<THREE.InstancedMesh>(null);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const ellipsoidRef = useRef<THREE.InstancedMesh>(null);
    const manipEllipsoidRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);

    const graph = data ?? EMPTY_GRAPH;
    const selectionEnabled = enableClusterSelection && !!onClusterSelect;
    const transform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };
    const nodePalette = useMemo(() => buildNodePalette(nodeColor), [nodeColor]);
    const cluster_node_colors = useMemo(
        () => enable_cluster_colors ? build_cluster_node_colors(graph) : undefined,
        [enable_cluster_colors, graph],
    );
    const active_labels = useMemo(() => get_active_node_labels(label_settings), [label_settings]);
    const label_signature = active_labels.map((item) => item.id).join('|');
    const isTrajectoryGraph = isTrajectoryGraphTag(tag);
    const highlightGoalNodes = variant === 'static' || isTrajectoryGraph;
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
    const { buckets: nodeBuckets, goal_nodes: goalNodes, node_colors } = useMemo(() => {
        const buckets = Array.from({ length: LAYER_COLORS.length }, () => [] as GraphData['nodes']);
        const goal_nodes: GraphData['nodes'] = [];
        const node_colors = new Map<GraphNode, string>();
        graph.nodes.forEach((node, nodeIndex) => {
            const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label as number) : 0;
            const labelIndex = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            const semanticLabel = Number.isFinite(node.semanticLabel)
                ? Math.trunc(node.semanticLabel as number)
                : (Number.isFinite(node.id) ? (nodeSemanticLabels.get(node.id as number) || 0) : (nodeSemanticLabels.get(nodeIndex) || 0));
            const nextNode = { ...node, semanticLabel };
            const selected_label = resolve_node_label(nextNode, active_labels);
            // 通常分類と有効な属性ラベルのOR判定。goalノードも共通処理。
            const is_label_visible = !visibleLabels || visibleLabels[labelIndex as 0 | 1 | 2 | 3 | 4 | 5];
            if (!is_label_visible && !selected_label) return;
            const cluster_color = cluster_node_colors?.get(node.id ?? nodeIndex);
            if (cluster_color) node_colors.set(nextNode, cluster_color);
            else if (selected_label) node_colors.set(nextNode, selected_label.color);
            if (highlightGoalNodes && node.isGoal) {
                goal_nodes.push(nextNode);
            } else {
                buckets[labelIndex].push(nextNode);
            }
        });
        return { buckets, goal_nodes, node_colors };
    }, [graph.nodes, nodeSemanticLabels, visibleLabels, highlightGoalNodes, active_labels, cluster_node_colors]);
    const goalNodeSignature = useMemo(
        () => goalNodes.map((node) => node.id ?? `${node.x},${node.y},${node.z}`).join('|'),
        [goalNodes]
    );

    // Trigger re-render in demand mode for any visual changes
    useDemandUpdate([
        graph,
        visible,
        showNodes,
        label_signature,
        showEdges,
        showClusters,
        showNormals,
        showVelocity,
        showCovarianceEllipsoids,
        showManipulabilityEllipsoids,
        manipEllipsoidMode,
        nodeScale,
        edgeWidth,
        covarianceEllipsoidScale,
        nodeOpacity,
        edgeOpacity,
        tf,
        visibleLabels,
        selectedClusterId,
        nodeColor,
        edgeColor,
        covarianceEllipsoidColor,
        nodeEmissiveIntensity,
        edgeEmissiveIntensity,
        transform,
        variant,
        goalNodeSignature,
    ]);

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
    const nodeMaterials = useMemo(() => nodePalette.map((color) => configure_node_material(new THREE.MeshStandardMaterial({
        color,
        emissive: new THREE.Color(color),
        emissiveIntensity: nodeEmissiveIntensity,
        transparent: nodeOpacity < 1,
        opacity: nodeOpacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    }))), [nodePalette, nodeOpacity, nodeEmissiveIntensity]);
    const goalNodeMaterial = useMemo(() => configure_node_material(new THREE.MeshStandardMaterial({
        color: CANDIDATE_GOAL_COLOR,
        emissive: new THREE.Color(CANDIDATE_GOAL_COLOR),
        emissiveIntensity: nodeEmissiveIntensity,
        transparent: nodeOpacity < 1,
        opacity: nodeOpacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    })), [nodeOpacity, nodeEmissiveIntensity]);

    const edgeCylinderGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 6), []);
    const ellipsoidGeometry = useMemo(() => new THREE.SphereGeometry(1, 16, 12), []);
    const ellipsoidMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: covarianceEllipsoidColor,
        transparent: true,
        opacity: Math.max(0.18, Math.min(0.6, nodeOpacity * 0.35)),
        depthTest: true,
        depthWrite: false,
        emissive: new THREE.Color(covarianceEllipsoidColor),
        emissiveIntensity: 0.8,
        roughness: 0.35,
        metalness: 0.0,
        toneMapped: false,
    }), [covarianceEllipsoidColor, nodeOpacity]);
    const edgeMaterial = useMemo(() => {
        const material = new THREE.MeshStandardMaterial({
            color: edgeColor,
            emissive: new THREE.Color(edgeColor),
            emissiveIntensity: edgeEmissiveIntensity,
            transparent: edgeOpacity < 1,
            opacity: edgeOpacity,
            depthTest: variant === 'static',
            depthWrite: false,
            toneMapped: false,
        });
        return enable_cluster_colors ? configure_node_material(material) : material;
    }, [edgeOpacity, edgeColor, edgeEmissiveIntensity, variant, enable_cluster_colors]);

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
            label_signature,
            nodeScale,
            nodeCapacity,
            nodeOpacity,
            nodeColor,
            nodeEmissiveIntensity,
            graph.timestamp,
            goalNodeSignature,
            variant,
        ].join(':');
    }, [graph.nodes, goalNodeSignature, showNodes, nodeScale, nodeCapacity, nodeOpacity, nodeColor, nodeEmissiveIntensity, graph.timestamp, variant, label_signature]);

    const edgeRenderSignature = useMemo(() => {
        return [
            edgePairCount,
            showEdges ? 1 : 0,
            edgeWidth,
            edgeCapacity,
            edgeOpacity,
            edgeColor,
            edgeEmissiveIntensity,
            graph.timestamp,
        ].join(':');
    }, [edgePairCount, showEdges, edgeWidth, edgeCapacity, edgeOpacity, edgeColor, edgeEmissiveIntensity, graph.timestamp]);
    const nodeRenderReady = nodeReadySignature === nodeRenderSignature;
    const edgeRenderReady = edgeReadySignature === edgeRenderSignature;
    const ellipsoidRenderSignature = useMemo(() => {
        return [
            covarianceEllipsoids.length,
            showCovarianceEllipsoids ? 1 : 0,
            covarianceEllipsoidScale,
            ellipsoidCapacity,
            nodeOpacity,
            covarianceEllipsoidColor,
            graph.timestamp,
        ].join(':');
    }, [covarianceEllipsoids.length, showCovarianceEllipsoids, covarianceEllipsoidScale, ellipsoidCapacity, nodeOpacity, covarianceEllipsoidColor, graph.timestamp]);
    const ellipsoidRenderReady = ellipsoidReadySignature === ellipsoidRenderSignature;
    const manipEllipsoidRenderSignature = useMemo(() => {
        return [
            manipulabilityEllipsoids.length,
            showManipulabilityEllipsoids ? 1 : 0,
            manipEllipsoidMode,
            manipEllipsoidType,
            manipEllipsoidCapacity,
            nodeOpacity,
            covarianceEllipsoidColor,
            graph.timestamp,
        ].join(':');
    }, [manipulabilityEllipsoids.length, showManipulabilityEllipsoids, manipEllipsoidMode, manipEllipsoidType, manipEllipsoidCapacity, nodeOpacity, covarianceEllipsoidColor, graph.timestamp]);
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
            const baseMesh = nodeMeshRefs.current[labelIndex];
            if (baseMesh) {
                updateNodeInstances(baseMesh, bucket, nodeScale, { node_colors, palette: nodePalette, useSemanticColors: false });
            }
        });
        if (goalNodeMeshRef.current) {
            updateNodeInstances(goalNodeMeshRef.current, goalNodes, nodeScale, {
                colorMode: 'uniform',
                uniformColor: CANDIDATE_GOAL_COLOR,
                node_colors,
            });
        }
        setNodeReadySignature(nodeRenderSignature);
        invalidate();
    }, [graph.nodes, nodeBuckets, goalNodes, showNodes, nodeScale, nodeCapacity, nodeRenderSignature, invalidate, nodePalette, node_colors]);

    useLayoutEffect(() => {
        if (showNodes) return;
        setNodeReadySignature(null);
        nodeMeshRefs.current.forEach((mesh) => {
            if (!mesh) return;
            mesh.count = 0;
            mesh.instanceMatrix.needsUpdate = true;
        });
        if (goalNodeMeshRef.current) {
            goalNodeMeshRef.current.count = 0;
            goalNodeMeshRef.current.instanceMatrix.needsUpdate = true;
        }
        invalidate();
    }, [showNodes, invalidate]);

    // --- Edge Instances ---
    useLayoutEffect(() => {
        if (!edgesRef.current || !showEdges || edgePairCount === 0) return;
        if (edgePairCount > edgeCapacity) return;

        updateEdgeInstances(edgesRef.current, graph.edges, graph.nodes, edgeWidth, cluster_node_colors);
        setEdgeReadySignature(edgeRenderSignature);
        invalidate();
    }, [graph.edges, graph.nodes, showEdges, edgeWidth, edgeCapacity, edgePairCount, edgeRenderSignature, invalidate, cluster_node_colors]);

    useLayoutEffect(() => {
        if (showEdges) return;
        setEdgeReadySignature(null);
        if (!edgesRef.current) return;
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

    const normal_samples = useMemo<arrow_sample[]>(() => graph.nodes.map(node => ({
        position: [node.x, node.y, node.z], direction: [node.nx, node.ny, node.nz],
        length: Math.min(0.175, Math.hypot(node.nx, node.ny, node.nz) * normal_arrow_style.length),
    })), [graph.nodes]);

    if (!data || !visible) return null;

    const canMountNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canMountEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;
    const canMountNormals = showNormals && graph.nodes.length > 0;
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
                <group key={`${variant}-node-label-${labelIndex}`}>
                    <instancedMesh
                        key={`${variant}-nodes-base-${labelIndex}-${nodeCapacity}`}
                        ref={(el) => { nodeMeshRefs.current[labelIndex] = el; }}
                        args={[nodeSphereGeometry, nodeMaterials[labelIndex], nodeCapacity]}
                        count={nodeRenderReady ? nodeBuckets[labelIndex].length : 0}
                        frustumCulled={false}
                        renderOrder={10}
                    />
                </group>
            ))}

            {canMountNodes && goalNodes.length > 0 && (
                <instancedMesh
                    key={`${variant}-candidate-goals-${nodeCapacity}`}
                    ref={goalNodeMeshRef}
                    args={[nodeSphereGeometry, goalNodeMaterial, nodeCapacity]}
                    count={nodeRenderReady ? goalNodes.length : 0}
                    frustumCulled={false}
                    renderOrder={12}
                />
            )}

            {canMountEdges && (
                <instancedMesh
                    key={`${variant}-edges-${edgeCapacity}`}
                    ref={edgesRef}
                    args={[edgeCylinderGeometry, edgeMaterial, edgeCapacity]}
                    count={edgeRenderReady ? edgePairCount : 0}
                    frustumCulled={false}
                    renderOrder={9}
                />
            )}

            {canMountCovarianceEllipsoids && (
                <instancedMesh
                    key={`${variant}-cov-ellipsoids-${ellipsoidCapacity}`}
                    ref={ellipsoidRef}
                    args={[ellipsoidGeometry, ellipsoidMaterial, ellipsoidCapacity]}
                    count={ellipsoidRenderReady ? covarianceEllipsoids.length : 0}
                    frustumCulled={false}
                    renderOrder={8}
                />
            )}
            {canMountManipEllipsoids && (
                <instancedMesh
                    key={`${variant}-manip-ellipsoids-${manipEllipsoidCapacity}`}
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

            {canMountNormals && <ArrowBatch samples={normal_samples} style={normal_arrow_style} />}

            {showClusters && graph.clusters
            .filter(cluster => !visibleLabels || visibleLabels[cluster.label as 0 | 1 | 2 | 3 | 4 | 5])
            .map((cluster) => {
                const isSelected = selectedClusterId === cluster.id;
                const semanticColor = resolve_node_label({ semanticLabel: cluster.semanticLabel }, active_labels)?.color;
                const color = isSelected ? '#FFFFFF' : (semanticColor || LAYER_COLORS[cluster.label % LAYER_COLORS.length]);
                const isHuman = cluster.label === 4;
                const handlePointerDown = (e: ThreeEvent<PointerEvent>) => {
                    if (selectionEnabled) dragStartRef.current = { x: e.clientX, y: e.clientY };
                };

                return (
                    <group key={cluster.id}>
                    <group position={cluster.pos} quaternion={new THREE.Quaternion(...cluster.quat)}>
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
                                opacity={isSelected ? 0.1 : 0.3 * nodeOpacity}
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

                    </group>
                        {canMountVelocity && (
                            <ArrowBatch
                                samples={[{ position: cluster.pos, direction: cluster.velocity }]}
                                style={velocity_arrow_style(Math.hypot(...cluster.velocity))}
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

export function GraphRenderer(props: GraphRendererProps) {
    return <GraphRendererCore {...props} variant="dynamic" />;
}

export function StaticGraphRenderer(props: GraphRendererProps) {
    return (
        <GraphRendererCore
            {...props}
            variant="static"
            nodeOpacity={props.nodeOpacity ?? STATIC_GNG_DEFAULTS.nodeOpacity}
            edgeOpacity={props.edgeOpacity ?? STATIC_GNG_DEFAULTS.edgeOpacity}
            nodeColor={props.nodeColor ?? STATIC_GNG_DEFAULTS.nodeColor}
            edgeColor={props.edgeColor ?? STATIC_GNG_DEFAULTS.edgeColor}
            nodeEmissiveIntensity={props.nodeEmissiveIntensity ?? STATIC_GNG_DEFAULTS.nodeEmissiveIntensity}
            edgeEmissiveIntensity={props.edgeEmissiveIntensity ?? STATIC_GNG_DEFAULTS.edgeEmissiveIntensity}
        />
    );
}
