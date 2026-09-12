import { ArrowBatch, EllipsoidBatch, DisplayFrame } from './SharedRenderers';
import { useMemo, useRef, useEffect, useLayoutEffect, useState } from 'react';
import * as THREE from 'three';
import { useThree, ThreeEvent } from '@react-three/fiber';
import { GraphData, GraphNode, LayerSettings, LAYER_COLORS, isTrajectoryGraphTag } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { buildNodePalette, updateNodeInstances, updateEdgeInstances, configure_node_material, build_cluster_node_colors } from './utils/gngGraphics';
import { arrow_sample, normal_arrow_style, velocity_arrow_style } from './arrows';
import { get_active_node_labels, resolve_node_label, resolve_graph_layer_settings } from './graphLayerSettings';

const CANDIDATE_GOAL_COLOR = '#a855f7';

interface GraphRendererProps {
    tag: string;
    data: GraphData;
    settings: LayerSettings;
    selectedClusterId?: number | null;
    onClusterSelect?: (clusterId: number | null) => void;
    onManipSelect?: (node: GraphNode) => void;
    enableClusterSelection?: boolean;
    tf?: { pos: number[]; quat: number[] } | null;
}

export function GraphRenderer({ tag, data: graph, settings, selectedClusterId = null,
    onClusterSelect, onManipSelect, enableClusterSelection = true, tf = null }: GraphRendererProps) {
    const variant = graph.mode === 'static' ? 'static' : 'dynamic';
    const enable_cluster_colors = /(^|\/)curved_surface_clusters$/.test(tag);
    const { visible, showNodes, showEdges, showClusters, showNormals, showVelocity,
        showCovarianceEllipsoids, showManipulabilityEllipsoids, manipEllipsoidMode, manipEllipsoidType,
        nodeScale, edgeWidth, covarianceEllipsoidScale, visibleLabels, nodeOpacity, edgeOpacity,
        nodeColor, edgeColor, covarianceEllipsoidColor, emissiveIntensity,
        graphTransform: transform } = resolve_graph_layer_settings(tag, graph, settings);
    const { invalidate } = useThree();
    const nodeMeshRefs = useRef<(THREE.InstancedMesh | null)[]>([]);
    const goalNodeMeshRef = useRef<THREE.InstancedMesh>(null);
    const edgesRef = useRef<THREE.InstancedMesh>(null);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);

    const selectionEnabled = enableClusterSelection && !!onClusterSelect;
    const nodePalette = useMemo(() => buildNodePalette(nodeColor), [nodeColor]);
    const cluster_node_colors = useMemo(
        () => enable_cluster_colors ? build_cluster_node_colors(graph) : undefined,
        [enable_cluster_colors, graph],
    );
    const active_labels = useMemo(() => get_active_node_labels(settings), [settings]);
    const label_signature = active_labels.map((item) => item.id).join('|');
    const isTrajectoryGraph = isTrajectoryGraphTag(tag);
    const highlightGoalNodes = variant === 'static' || isTrajectoryGraph;
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

    useDemandUpdate([graph, settings, tf, selectedClusterId, variant, goalNodeSignature]);

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
        emissiveIntensity,
        transparent: nodeOpacity < 1,
        opacity: nodeOpacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    }))), [nodePalette, nodeOpacity, emissiveIntensity]);
    const goalNodeMaterial = useMemo(() => configure_node_material(new THREE.MeshStandardMaterial({
        color: CANDIDATE_GOAL_COLOR,
        emissive: new THREE.Color(CANDIDATE_GOAL_COLOR),
        emissiveIntensity,
        transparent: nodeOpacity < 1,
        opacity: nodeOpacity,
        depthTest: false,
        depthWrite: false,
        roughness: 0.85,
        metalness: 0.0,
        toneMapped: false,
    })), [nodeOpacity, emissiveIntensity]);

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
    useEffect(() => () => ellipsoidGeometry.dispose(), [ellipsoidGeometry]);
    useEffect(() => () => ellipsoidMaterial.dispose(), [ellipsoidMaterial]);
    const edgeMaterial = useMemo(() => {
        const material = new THREE.MeshStandardMaterial({
            color: edgeColor,
            emissive: new THREE.Color(edgeColor),
            emissiveIntensity,
            transparent: edgeOpacity < 1,
            opacity: edgeOpacity,
            depthTest: variant === 'static',
            depthWrite: false,
            toneMapped: false,
        });
        return enable_cluster_colors ? configure_node_material(material) : material;
    }, [edgeOpacity, edgeColor, emissiveIntensity, variant, enable_cluster_colors]);

    const [nodeCapacity, setNodeCapacity] = useState(graph.nodes.length);
    const edgePairCount = useMemo(() => Math.floor(graph.edges.length / 2), [graph.edges]);
    const [edgeCapacity, setEdgeCapacity] = useState(edgePairCount);
    const [nodeReadySignature, setNodeReadySignature] = useState<string | null>(null);
    const [edgeReadySignature, setEdgeReadySignature] = useState<string | null>(null);
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
            node: GraphNode;
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
                        color: '#ff7f50', // 回転可操作性の配色
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
            emissiveIntensity,
            graph.timestamp,
            goalNodeSignature,
            variant,
        ].join(':');
    }, [graph.nodes, goalNodeSignature, showNodes, nodeScale, nodeCapacity, nodeOpacity, nodeColor, emissiveIntensity, graph.timestamp, variant, label_signature]);

    const edgeRenderSignature = useMemo(() => {
        return [
            edgePairCount,
            showEdges ? 1 : 0,
            edgeWidth,
            edgeCapacity,
            edgeOpacity,
            edgeColor,
            emissiveIntensity,
            graph.timestamp,
        ].join(':');
    }, [edgePairCount, showEdges, edgeWidth, edgeCapacity, edgeOpacity, edgeColor, emissiveIntensity, graph.timestamp]);
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

    const normal_samples = useMemo<arrow_sample[]>(() => graph.nodes.map(node => ({
        position: [node.x, node.y, node.z], direction: [node.nx, node.ny, node.nz],
        length: Math.min(0.175, Math.hypot(node.nx, node.ny, node.nz) * normal_arrow_style.length),
    })), [graph.nodes]);

    if (!visible) return null;

    const canMountNodes = showNodes && graph.nodes.length > 0 && nodeCapacity >= graph.nodes.length;
    const canMountEdges = showEdges && edgePairCount > 0 && edgeCapacity >= edgePairCount;
    const canMountNormals = showNormals && graph.nodes.length > 0;
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

            <EllipsoidBatch instances={covarianceEllipsoids} geometry={ellipsoidGeometry} material={ellipsoidMaterial}
                sigma_multiplier={covarianceEllipsoidScale} default_color={covarianceEllipsoidColor} render_order={8} />
            <EllipsoidBatch instances={manipulabilityEllipsoids} geometry={ellipsoidGeometry} material={ellipsoidMaterial}
                default_color={covarianceEllipsoidColor} render_order={7} on_pick={instance => onManipSelect?.(instance.node)} />

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

    return <DisplayFrame name={tag} tf={tf} manual_transform={transform}>{content}</DisplayFrame>;
}
