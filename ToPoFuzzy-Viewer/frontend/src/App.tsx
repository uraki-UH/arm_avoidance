import { useMemo, useEffect, useState } from 'react';
import * as THREE from 'three';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { SidebarContent } from './layout/SidebarContent';
import { PointCloudRenderer } from './features/visualization/PointCloudRenderer';
import {
    RobotSettings,
    Transform,
    VoxelSettings,
    EntityType,
    PointCloudData,
    HeatmapSettings,
    GraphNode,
    LayerSettings,
    ClippingPlane,
    ClippingAxis
} from './types';
import { GraphRenderer, StaticGraphRenderer } from './features/visualization/GraphRenderer';
import { RobotRenderer } from './features/visualization/RobotRenderer';
import { CollisionRenderer } from './features/visualization/CollisionRenderer';
import { MarkerArrayRenderer } from './features/visualization/MarkerArrayRenderer';
import { useWebSocket } from './hooks/useWebSocket';
import { useZoneMonitor } from './features/analysis/useZoneMonitor';
import { VoxelRenderer } from './features/visualization/VoxelRenderer';
import {
    nodeHasManipulabilityData,
    useGraphLayerSettings,
} from './features/visualization/graphLayerSettings';

type ClippingRange = Pick<ClippingPlane, 'min' | 'max'>;

function useClippingPlanes() {
    const [planes, setPlanes] = useState<ClippingPlane[]>([]);

    const addPlane = (axis: ClippingAxis, initialRange?: ClippingRange) => {
        const existingPlane = planes.find((plane) => plane.axis === axis);
        if (existingPlane) {
            setPlanes(prev => prev.map((plane) => (
                plane.axis === axis
                    ? { ...plane, enabled: true }
                    : plane
            )));
            return existingPlane.id;
        }

        const id = `plane-${Date.now()}`;
        const initialMin = initialRange?.min ?? -100;
        const initialMax = initialRange?.max ?? 100;
        const hasUsableRange =
            Number.isFinite(initialMin) &&
            Number.isFinite(initialMax) &&
            initialMin < initialMax;
        const newPlane: ClippingPlane = {
            id,
            axis,
            position: 0,
            min: hasUsableRange ? initialMin : -100,
            max: hasUsableRange ? initialMax : 100,
            inverted: false,
            enabled: true,
        };
        setPlanes(prev => [...prev, newPlane]);
        return id;
    };

    const updatePlane = (id: string, updates: Partial<ClippingPlane>) => {
        setPlanes(prev => prev.map(p => p.id === id ? { ...p, ...updates } : p));
    };

    const removePlane = (id: string) => {
        setPlanes(prev => prev.filter(p => p.id !== id));
    };

    const removeAll = () => setPlanes([]);

    const threePlanes = useMemo((): THREE.Plane[] => {
        return planes
            .filter(p => p.enabled && p.axis !== 'none')
            .flatMap((p) => {
                const normal = new THREE.Vector3(
                    p.axis === 'x' ? 1 : 0,
                    p.axis === 'y' ? 1 : 0,
                    p.axis === 'z' ? 1 : 0
                );
                const min = Math.min(
                    p.min ?? (p.position - 0.01),
                    p.max ?? (p.position + 0.01)
                );
                const max = Math.max(
                    p.min ?? (p.position - 0.01),
                    p.max ?? (p.position + 0.01)
                );

                const lower = new THREE.Plane(normal.clone(), -min);
                const upper = new THREE.Plane(normal.clone().multiplyScalar(-1), max);

                // Range clipping is expressed as the intersection of two planes.
                return [lower, upper];
            });
    }, [planes]);

    return { planes, addPlane, updatePlane, removePlane, removeAll, threePlanes };
}

function ClippingPlaneSync({ planes }: { planes: THREE.Plane[] }) {
    const { gl, invalidate } = useThree();

    useEffect(() => {
        gl.clippingPlanes = planes;
        invalidate();
    }, [gl, invalidate, planes]);

    return null;
}
import { ZoneVisualizer } from './features/analysis/ZoneVisualizer';
import { ClusterDetailPanel, ClusterSnapshot } from './features/visualization/ClusterDetailPanel';
import { GraphNodeDetailPanel, GraphNodeDetailSnapshot } from './features/visualization/GraphNodeDetailPanel';
import { GenericTransformModal } from './features/manipulation/GenericTransformModal';
import { RobotJointModal } from './features/manipulation/RobotJointModal';
import { EntityColorModal } from './features/manipulation/EntityColorModal';

import { Sidebar } from './layout/Sidebar';
import { MainLayout } from './layout/MainLayout';
import { calculateBounds } from './utils/bounds';
import { EditAabbTool } from './features/manipulation/EditAabbTool';
import { useEditSession } from './features/manipulation/editSession';
import { WebGLErrorBoundary } from './components/WebGLErrorBoundary';

type ColorContext = { type: 'robot' | 'voxel' | 'graph'; id: string; title: string };

function App() {
    const [pointClouds, setPointClouds] = useState<PointCloudData[]>([]);
    const [isSidebarOpen, setIsSidebarOpen] = useState(true);

    const [selectedLayerId, setSelectedLayerId] = useState<string | null>(null);
    const [transformMode, setTransformMode] = useState<'translate' | 'rotate' | 'scale'>('translate');
    const [heatmapSettings, setHeatmapSettings] = useState<HeatmapSettings>({
        mode: 'rgb',
        min: -2,
        max: 5,
        colorScheme: 'viridis',
        pointSize: 0.02,
        simpleColor: '#c8ff4a',
    });
    const [pointCloudOpacity, setPointCloudOpacity] = useState(1);
    const [robotSettings, setRobotSettings] = useState<Record<string, RobotSettings>>({});
    const [markerSettings, setMarkerSettings] = useState<Record<string, { visible: boolean, transform?: Transform }>>({});
    const [voxelSettings, setVoxelSettings] = useState<Record<string, VoxelSettings>>({});
    const [transformContext, setTransformContext] = useState<{ type: 'cloud' | 'layer' | 'robot' | 'marker' | 'voxel', id: string, title: string } | null>(null);
    const [robotJointContext, setRobotJointContext] = useState<{ id: string, title: string, selectedManipLink?: string } | null>(null);
    const [colorContext, setColorContext] = useState<ColorContext | null>(null);

    const viewerPort = import.meta.env.VITE_VIEWER_WS_PORT ?? '9001';
    const wsUrl = `ws://${window.location.hostname}:${viewerPort}`;
    const {
        pointClouds: wsPointClouds,
        markerData,
        graphData,
        robotData,
        voxelData,
        transforms,
        lastJobEvent,
        isConnected,
        error: wsError,
        connect,
        disconnect,
        deleteGraphLayer,
        sources,
        getSources,
        subscribeSource,
        unsubscribeSource,
        listRosbags,
        playRosbag,
        stopRosbag,
        getRosbagStatus,
        listPointCloudFiles,
        loadPointCloudFile,
        openEditSession,
        addEditRegion,
        removeEditRegion,
        clearEditRegions,
        commitEditSession,
        cancelEditSession,
        startGng,
        stopGng,
        getGngStatus,
        listGngConfigs,
        getParameters,
        setParameter,
        startContinuousPublish,
        stopContinuousPublish,
        getContinuousPublishStatus,
    } = useWebSocket(wsUrl);

    useEffect(() => {
        connect();
        return () => disconnect();
    }, [connect, disconnect]);

    const clipping = useClippingPlanes();
  const zoneMonitor = useZoneMonitor();
  const { getZoneCounts } = zoneMonitor;

    const threeClippingPlanes = clipping.threePlanes;

    // Stable gl config: clipping planes are synchronized inside the Canvas.
    const canvasGl = useMemo(() => ({
        localClippingEnabled: false,
        powerPreference: 'high-performance' as const,
        antialias: false,
    }), []);

    const {
        layerSettings,
        updateLayerSettings: handleUpdateLayerSettings,
        removeLayerSettings,
    } = useGraphLayerSettings(graphData);

    // --- Unified Entity Initialization ---
    useEffect(() => {
        const configs: Record<string, { data: any, set: any, defaults: any }> = {
            robot: {
                data: robotData,
                set: setRobotSettings,
                defaults: {
                    visible: true, color: 'skyblue', showVisual: true, showCollision: false, showManipulabilityEllipsoid: false, collisionColor: '#ff9f1c', opacity: 0.8, jointControlMode: 'live',
                    useUrdfColors: true,
                    manipLinkName: '',
                    manipEllipsoidType: 'translational',
                    transform: { position: [0, 0, 0] as [number, number, number], rotation: [0, 0, 0] as [number, number, number], scale: [1, 1, 1] as [number, number, number] }
                }
            },
            marker: {
                data: markerData,
                set: setMarkerSettings,
                defaults: {
                    visible: true,
                    transform: { position: [0, 0, 0] as [number, number, number], rotation: [0, 0, 0] as [number, number, number], scale: [1, 1, 1] as [number, number, number] }
                }
            },
            voxel: { data: voxelData, set: setVoxelSettings, defaults: { visible: true, color: '#00ff88', colorMode: 'uniform', wireframe: true, opacity: 0.5 } }
        };

        Object.entries(configs).forEach(([_, { data, set, defaults }]) => {
            set((prev: any) => {
                const next = { ...prev };
                let changed = false;
                Object.keys(data).forEach(tag => {
                    const entityData = data[tag];
                    const isLabeledVoxel = data === voxelData
                        && Array.isArray(entityData?.labels)
                        && entityData.labels.length > 0
                        && entityData.labels.length === entityData.data?.length;
                    if (!next[tag]) {
                        next[tag] = {
                            ...defaults,
                            ...(isLabeledVoxel ? { color: '#ffff00', colorMode: 'uniform' } : {}),
                            ...(data === robotData && tag.includes('candidate_goal_preview') ? { opacity: 0.18 } : {}),
                        };
                        changed = true;
                    } else if (isLabeledVoxel && next[tag].colorMode === undefined) {
                        next[tag] = {
                            ...next[tag],
                            color: next[tag].color === '#00ff88' ? '#ffff00' : next[tag].color,
                            colorMode: 'uniform',
                        };
                        changed = true;
                    }
                });
                return changed ? next : prev;
            });
        });
    }, [robotData, markerData, voxelData]);

    const updateEntitySettings = (type: EntityType, tag: string, updates: any) => {
        const updaters: Record<string, any> = { robot: setRobotSettings, marker: setMarkerSettings, voxel: setVoxelSettings };
        updaters[type]?.((prev: any) => ({ ...prev, [tag]: { ...prev[tag], ...updates } }));
    };

    const removeEntity = (type: EntityType, tag: string) => {
        const updaters: Record<string, any> = { robot: setRobotSettings, marker: setMarkerSettings, voxel: setVoxelSettings };
        updaters[type]?.((prev: any) => { const n = { ...prev }; delete n[tag]; return n; });

        // Also unsubscribe from the stream if it's a streamable entity
        if (type === 'marker' || type === 'voxel') {
            unsubscribeSource(tag, true);
        }
    };

    const removeLayer = (tag: string) => {
        deleteGraphLayer(tag);
        removeLayerSettings(tag);
    };

    const zoneCounts = useMemo(() => {
        // Aggregate zone counts across all visible GNG layers
        const aggregated = new Map<string, number>();
        Object.entries(graphData).forEach(([tag, data]) => {
            const settings = layerSettings[tag];
            if (settings?.visible) {
      const counts = getZoneCounts(data);
                counts.forEach((count, label) => {
                    aggregated.set(label, (aggregated.get(label) || 0) + count);
                });
            }
        });
        return aggregated;
  }, [getZoneCounts, graphData, layerSettings]);

    const [disabledSourceIds, setDisabledSourceIds] = useState<Set<string>>(new Set());

    const [selectedClusterSnapshot, setSelectedClusterSnapshot] = useState<ClusterSnapshot | null>(null);
    const [selectedManipSnapshot, setSelectedManipSnapshot] = useState<GraphNodeDetailSnapshot | null>(null);

    const {
        isEditMode,
        editLayerId,
        editRegions,
        draftRegion,
        regionGizmoMode,
        setRegionGizmoMode,
        editJobStatus,
        canStartEdit,
        startEditDisabledReason,
        startEdit: handleStartEdit,
        cancelEdit: handleCancelEdit,
        addRegion: handleAddRegion,
        removeRegion: handleRemoveRegion,
        clearRegions: handleClearRegions,
        publishEditedCloud: handlePublishEditedCloud,
        updateDraftRegion: handleDraftRegionChange,
    } = useEditSession({
        pointClouds,
        selectedLayerId,
        onSelectLayer: setSelectedLayerId,
        isConnected,
        lastJobEvent,
        api: {
            subscribeSource,
            openEditSession,
            addEditRegion,
            removeEditRegion,
            clearEditRegions,
            commitEditSession,
            cancelEditSession,
        },
    });

    useEffect(() => {
        const handleResize = () => {
            if (window.innerWidth < 768) {
                setIsSidebarOpen(false);
            } else {
                setIsSidebarOpen(true);
            }
        };

        handleResize();
        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    const toggleSidebar = () => {
        setIsSidebarOpen(!isSidebarOpen);
    };

    useEffect(() => {
        setPointClouds((prev) => {
            let next = [...prev];
            let changed = false;

            Object.values(wsPointClouds).forEach((cloud) => {
                if (disabledSourceIds.has(cloud.id)) return;
                if (isEditMode && editLayerId === cloud.id) return;

                const index = next.findIndex((pc) => pc.id === cloud.id);
                const newCloud = {
                    ...cloud,
                    visible: index >= 0 ? next[index].visible : true,
                    opacity: index >= 0 ? next[index].opacity : pointCloudOpacity,
                    position: index >= 0 ? next[index].position : (cloud.position || [0, 0, 0]),
                    rotation: index >= 0 ? next[index].rotation : (cloud.rotation || [0, 0, 0]),
                    scale: index >= 0 ? next[index].scale : (cloud.scale || [1, 1, 1]),
                };

                if (index === -1) {
                    next.push(newCloud);
                    changed = true;
                } else {
                    const existing = next[index];
                    // Compare content to avoid unnecessary updates
                    if (existing.points !== newCloud.points || existing.count !== newCloud.count) {
                        next[index] = newCloud;
                        changed = true;
                    }
                }
            });

            // Also remove pointclouds that are no longer in wsPointClouds
            const activeIds = new Set(Object.keys(wsPointClouds));
            const filtered = next.filter((pc) => activeIds.has(pc.id) || (isEditMode && editLayerId === pc.id));
            if (filtered.length !== next.length) {
                next = filtered;
                changed = true;
            }

            return changed ? next : prev;
        });
    }, [wsPointClouds, disabledSourceIds, pointCloudOpacity, isEditMode, editLayerId]);

    useEffect(() => {
        setPointClouds((prev) => prev.map((pc) => ({ ...pc, opacity: pointCloudOpacity })));
    }, [pointCloudOpacity]);

    const handleAddPointCloud = (data: PointCloudData) => {
        const cloud = {
            ...data,
            opacity: data.opacity ?? pointCloudOpacity,
        };
        setPointClouds((prev) => [...prev, cloud]);
        setSelectedLayerId(data.id);
    };

    const handleRemoveLayer = (id: string) => {
        if (isEditMode) return;
        setPointClouds((prev) => {
            const filtered = prev.filter((pc) => pc.id !== id);
            if (selectedLayerId === id) {
                const fallback = filtered.find((pc) => pc.visible !== false);
                setSelectedLayerId(fallback ? fallback.id : null);
            }
            return filtered;
        });
    };

    const handleToggleVisibility = (id: string) => {
        if (isEditMode) return;
        setPointClouds((prev) => prev.map((pc) => (
            pc.id === id ? { ...pc, visible: !pc.visible } : pc
        )));
    };

    const handleSourceToggled = (sourceId: string, active: boolean) => {
        if (isEditMode) return;
        if (active) {
            setDisabledSourceIds((prev) => {
                const next = new Set(prev);
                next.delete(sourceId);
                return next;
            });
            return;
        }

        setDisabledSourceIds((prev) => new Set(prev).add(sourceId));
    };

    const handleUpdateTransform = (id: string, updates: Partial<PointCloudData>) => {
        if (editJobStatus?.isRunning) return;
        if (isEditMode && id !== editLayerId) return;

        setPointClouds((prev) => prev.map((pc) => (
            pc.id === id ? { ...pc, ...updates } : pc
        )));
    };

    const handleTransformChange = (
        id: string,
        position: [number, number, number],
        rotation: [number, number, number],
        scale: [number, number, number]
    ) => {
        handleUpdateTransform(id, { position, rotation, scale });
    };

    const handleSelectLayer = (id: string | null) => {
        if (isEditMode && id !== editLayerId) return;
        setSelectedLayerId(id);
    };

    const totalPoints = pointClouds.reduce((sum, pc) => sum + pc.count, 0);
    const selectedCloud = pointClouds.find((pc) => pc.id === selectedLayerId);
    const renderClouds = isEditMode && editLayerId
        ? pointClouds.filter((pc) => pc.id === editLayerId)
        : pointClouds.filter((pc) => !disabledSourceIds.has(pc.id));

    const [boundsBuffer, setBoundsBuffer] = useState<ReturnType<typeof calculateBounds>[]>([]);
    const [smoothedBounds, setSmoothedBounds] = useState<ReturnType<typeof calculateBounds> | undefined>(undefined);

    useEffect(() => {
        if (pointClouds.length === 0) return;
        const currentBounds = calculateBounds(pointClouds);
        setBoundsBuffer((prev) => [...prev, currentBounds].slice(-30));
    }, [pointClouds]);

    useEffect(() => {
        if (boundsBuffer.length === 0) return;

        const stabilized = boundsBuffer.reduce((acc, curr) => ({
            minX: Math.min(acc.minX, curr.minX),
            maxX: Math.max(acc.maxX, curr.maxX),
            minY: Math.min(acc.minY, curr.minY),
            maxY: Math.max(acc.maxY, curr.maxY),
            minZ: Math.min(acc.minZ, curr.minZ),
            maxZ: Math.max(acc.maxZ, curr.maxZ),
            maxDist: Math.max(acc.maxDist, curr.maxDist),
        }), boundsBuffer[0]);

        setSmoothedBounds(stabilized);
    }, [boundsBuffer]);

    const bounds = smoothedBounds;

    const handleClusterSelect = (clusterId: number | null) => {
        if (clusterId === null) {
            setSelectedClusterSnapshot(null);
            return;
        }

        // Find which graph has this cluster (simplified: search all)
        let foundCluster = null;
        let foundGraph = null;
        for (const data of Object.values(graphData)) {
            const c = data.clusters.find((c) => c.id === clusterId);
            if (c) {
                foundCluster = c;
                foundGraph = data;
                break;
            }
        }

        if (!foundCluster || !foundGraph) return;

        const nodeIdsSet = new Set(foundCluster.nodeIds);
        const nodeById = new Map<number, GraphNode>();
        foundGraph.nodes.forEach((node, index) => {
            if (Number.isFinite(node.id)) {
                nodeById.set(node.id as number, node);
            }
            nodeById.set(index, node);
        });
        const nodes = foundCluster.nodeIds
            .map((nodeId) => nodeById.get(nodeId))
            .filter((n): n is GraphNode => n !== undefined);

        const edges: { source: GraphNode; target: GraphNode }[] = [];
        for (let i = 0; i < foundGraph.edges.length; i += 2) {
            const srcIdx = foundGraph.edges[i];
            const dstIdx = foundGraph.edges[i + 1];
            if (nodeIdsSet.has(srcIdx) && nodeIdsSet.has(dstIdx)) {
                const srcNode = nodeById.get(srcIdx);
                const dstNode = nodeById.get(dstIdx);
                if (srcNode && dstNode) {
                    edges.push({ source: srcNode, target: dstNode });
                }
            }
        }

        setSelectedClusterSnapshot({
            cluster: JSON.parse(JSON.stringify(foundCluster)),
            nodes: JSON.parse(JSON.stringify(nodes)),
            edges: JSON.parse(JSON.stringify(edges)),
        });
    };

    const handleManipSelect = (graphTag: string, node: GraphNode) => {
        const graph = graphData[graphTag];
        if (!graph || !nodeHasManipulabilityData(node)) return;
        setSelectedManipSnapshot({
            graphTag,
            graph: JSON.parse(JSON.stringify(graph)),
            node: JSON.parse(JSON.stringify(node)),
        });
    };

    return (
        <>
            <MainLayout
                isSidebarOpen={isSidebarOpen}
                sidebar={
                    <Sidebar isOpen={isSidebarOpen} onToggle={toggleSidebar}>
                        <SidebarContent
                            isConnected={isConnected}
                            connect={connect}
                            disconnect={disconnect}
                            wsError={wsError}
                            sources={sources}
                            getSources={getSources}
                            subscribeSource={subscribeSource}
                            unsubscribeSource={unsubscribeSource}
                            onSourceToggled={handleSourceToggled}
                            onLoadCloud={handleAddPointCloud}
                            listRosbags={listRosbags}
                            playRosbag={playRosbag}
                            stopRosbag={stopRosbag}
                            getRosbagStatus={getRosbagStatus}
                            listPointCloudFiles={listPointCloudFiles}
                            loadPointCloudFile={loadPointCloudFile}
                            totalPoints={totalPoints}
                            pointClouds={pointClouds}
                            selectedLayerId={selectedLayerId}
                            onSelectLayer={handleSelectLayer}
                            onToggleVisibility={handleToggleVisibility}
                            onRemoveLayer={handleRemoveLayer}
                            graphData={graphData}
                            layerSettings={layerSettings}
                            onUpdateLayerSettings={handleUpdateLayerSettings}
                            onRemoveGngLayer={removeLayer}
                            heatmapSettings={heatmapSettings}
                            setHeatmapSettings={setHeatmapSettings}
                            pointCloudOpacity={pointCloudOpacity}
                            setPointCloudOpacity={setPointCloudOpacity}
                            bounds={bounds}
                            selectedCloud={selectedCloud}
                            transformMode={transformMode}
                            setTransformMode={setTransformMode}
                            onUpdateTransform={handleUpdateTransform}
                            clipping={clipping}
                            onPublishEdited={handlePublishEditedCloud}
                            isEditMode={isEditMode}
                            onStartEdit={handleStartEdit}
                            onCancelEdit={handleCancelEdit}
                            canStartEdit={canStartEdit}
                            startEditDisabledReason={startEditDisabledReason}
                            editRegions={editRegions}
                            onAddRegion={handleAddRegion}
                            onRemoveRegion={handleRemoveRegion}
                            onClearRegions={handleClearRegions}
                            draftRegion={draftRegion}
                            regionGizmoMode={regionGizmoMode}
                            setRegionGizmoMode={setRegionGizmoMode}
                            editJobStatus={editJobStatus}
                            zoneMonitor={zoneMonitor}
                            zoneCounts={zoneCounts}
                            startGng={startGng}
                            stopGng={stopGng}
                            getGngStatus={getGngStatus}
                            listGngConfigs={listGngConfigs}
                            getParameters={getParameters}
                            setParameter={setParameter}
                            startContinuousPublish={startContinuousPublish}
                            stopContinuousPublish={stopContinuousPublish}
                            getContinuousPublishStatus={getContinuousPublishStatus}
                            robotData={robotData}
                            robotSettings={robotSettings}
                            markerData={markerData}
                            markerSettings={markerSettings}
                            onUpdateSettings={updateEntitySettings}
                            onRemoveEntity={removeEntity}
                            transforms={transforms}
                            voxelData={voxelData}
                            voxelSettings={voxelSettings}
                            onOpenTransform={(type, id, title) => {
                                console.log(`[App] onOpenTransform triggered:`, type, id, title);
                                setTransformContext(prev => (prev?.type === type && prev?.id === id) ? null : { type, id, title });
                            }}
                            onOpenRobotJoints={(id, title) => {
                                setRobotJointContext(prev => (prev?.id === id ? null : { id, title }));
                            }}
                            onOpenColorSettings={(type, id, title) => {
                                setColorContext(prev => (prev?.type === type && prev?.id === id ? null : { type, id, title }));
                            }}
                        />
                    </Sidebar>
                }
            >
                <div className="w-full h-full relative bg-gradient-to-br from-[var(--bg-primary)] to-black">
                    <WebGLErrorBoundary>
                    <Canvas
                        frameloop="demand"
                        camera={{ position: [5, 5, 5], up: [0, 0, 1], fov: 50 }}
                        gl={canvasGl}
                        onCreated={({ gl }) => {
                            if (import.meta.hot) {
                                import.meta.hot.dispose(() => gl.dispose());
                            }
                            // Diagnostics only — do NOT auto-retry here, see WebGLErrorBoundary.
                            gl.domElement.addEventListener('webglcontextlost', (e) => {
                                console.error('[WebGL] context lost', {
                                    time: new Date().toISOString(),
                                    statusMessage: (e as WebGLContextEvent).statusMessage,
                                });
                            });
                            gl.domElement.addEventListener('webglcontextrestored', () => {
                                console.warn('[WebGL] context restored event fired (not expected to recover automatically)', new Date().toISOString());
                            });
                        }}
                    >
                        <ClippingPlaneSync planes={threeClippingPlanes} />
                        <ambientLight intensity={0.3} />
                        <pointLight position={[10, 10, 10]} intensity={0.5} />
                        <pointLight position={[-10, -10, -10]} intensity={0.3} />

                        {renderClouds.map((pc) => (
                            <PointCloudRenderer
                                key={pc.id}
                                data={pc}
                                heatmapSettings={heatmapSettings}
                                selected={isEditMode && pc.id === editLayerId}
                                transformMode={transformMode}
                                onTransformChange={(pos, rot, scale) => handleTransformChange(pc.id, pos, rot, scale)}
                            />
                        ))}

                        <EditAabbTool
                            enabled={isEditMode && !editJobStatus?.isRunning}
                            center={draftRegion.center}
                            size={draftRegion.size}
                            mode={regionGizmoMode}
                            onChange={handleDraftRegionChange}
                        />

                        {/* Entities (Consolidated rendering logic) */}
                        {[
                                {
                                    data: robotData, settings: robotSettings, component: (tag: string, d: any, s: any, tf: any) => (
                                    <group key={tag}>
                                {s.showVisual && <RobotRenderer tag={tag} data={d} visible={true} color={s.color} useUrdfColors={s.useUrdfColors ?? true} emissiveIntensity={s.emissiveIntensity ?? 0.2} opacity={s.opacity ?? (tag.includes('candidate_goal_preview') ? 0.18 : 0.8)} jointValuesOverride={s.jointControlMode === 'manual' ? (s.jointValues || []) : []} tf={tf} manualTransform={s.transform} showManipulabilityEllipsoid={s.showManipulabilityEllipsoid ?? false} manipEllipsoidType={s.manipEllipsoidType || 'translational'} manipLinkName={s.manipLinkName || ''} onManipClick={(linkName) => setRobotJointContext({ id: tag, title: `Robot joints: ${tag}`, selectedManipLink: linkName })} />}
                                {s.showCollision && <CollisionRenderer tag={tag} data={d} visible={true} color={s.collisionColor} opacity={Math.min(s.opacity ?? 0.8, 0.28)} tf={tf} manualTransform={s.transform} />}
                            </group>
                                ), defaultSettings: { visible: true, color: 'skyblue', useUrdfColors: true, showVisual: true, showCollision: false, showManipulabilityEllipsoid: false, manipEllipsoidType: 'translational', manipLinkName: '', collisionColor: '#ff9f1c', emissiveIntensity: 0.2, transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } }
                            },
                            {
                                data: markerData, settings: markerSettings, component: (tag: string, d: any, s: any, tf: any) => (
                                    <MarkerArrayRenderer key={tag} tag={tag} data={d} visible={true} tf={tf} manualTransform={s.transform} />
                                ), defaultSettings: { visible: true, transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } }
                            },
                            {
                                data: voxelData, settings: voxelSettings, component: (tag: string, d: any, s: any, tf: any) => (
                                    <VoxelRenderer key={tag} message={{ type: 'stream.voxel', tag, data: d.data, labels: d.labels, layout: d.layout, frameId: d.frameId }} settings={s} tf={tf} manualTransform={s.transform} />
                                ), defaultSettings: { visible: true, color: '#00ff88', colorMode: 'uniform', wireframe: true, opacity: 0.5, emissiveIntensity: 0.2, transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } }
                            }
                        ].map(({ data, settings, component, defaultSettings }) =>
                            Object.entries(data).map(([tag, d]: [string, any]) => {
                                const s = (settings as any)[tag] || defaultSettings;
                                if (!s.visible || disabledSourceIds.has(tag)) return null;
                                const tf = d.frameId && d.frameId !== 'world' ? (transforms[d.frameId] ?? null) : null;
                                return component(tag, d, s, tf);
                            })
                        )}

                        {Object.entries(graphData).map(([tag, data]) => {
                            const settings = layerSettings[tag];
                            if (!settings || !settings.visible || disabledSourceIds.has(tag)) return null;
                            const tf = data.frameId && data.frameId !== 'world' ? (transforms[data.frameId] ?? null) : null;
                            const common = {
                                key: tag,
                                tag,
                                data,
                                visible: true,
                                nodeOpacity: settings.nodeOpacity,
                                edgeOpacity: settings.edgeOpacity,
                                tf,
                                manualTransform: settings.graphTransform,
                                nodeColor: settings.nodeColor,
                                edgeColor: settings.edgeColor,
                                nodeEmissiveIntensity: settings.emissiveIntensity,
                                edgeEmissiveIntensity: settings.emissiveIntensity,
                                nodeScale: settings.nodeScale ?? 0.01,
                                edgeWidth: settings.edgeWidth ?? 0.002,
                                showNormals: settings.showNormals ?? true,
                                showVelocity: settings.showVelocity ?? false,
                                showCovarianceEllipsoids: settings.showCovarianceEllipsoids ?? false,
                                showManipulabilityEllipsoids: settings.showManipulabilityEllipsoids ?? false,
                                manipEllipsoidMode: settings.manipEllipsoidMode ?? 'all',
                                manipEllipsoidType: settings.manipEllipsoidType ?? 'translational',
                                visibleSemanticLabels: settings.visibleSemanticLabels,
                                normalScale: settings.normalScale ?? 0.075,
                                velocityScale: settings.velocityScale ?? 0.25,
                                covarianceEllipsoidScale: settings.covarianceEllipsoidScale ?? 2.0,
                                normalColor: settings.normalColor ?? '#00FFFF',
                                velocityColor: settings.velocityColor ?? '#ffb347',
                                covarianceEllipsoidColor: settings.covarianceEllipsoidColor ?? '#7fd9ff',
                            };
                            return data.mode === 'static'
                                ? <StaticGraphRenderer {...common} showNodes={settings.showNodes} showEdges={settings.showEdges} selectedClusterId={selectedClusterSnapshot?.cluster.id ?? null} onClusterSelect={handleClusterSelect} onManipSelect={(node) => handleManipSelect(tag, node)} />
                                : <GraphRenderer {...common} showNodes={settings.showNodes} showEdges={settings.showEdges} showClusters={settings.showClusters} showClusterText visibleLabels={settings.visibleLabels} selectedClusterId={selectedClusterSnapshot?.cluster.id ?? null} onClusterSelect={handleClusterSelect} onManipSelect={(node) => handleManipSelect(tag, node)} enableClusterSelection={!zoneMonitor.isDrawing} />;
                        })}

                    <ZoneVisualizer points={zoneMonitor.points} isDrawing={zoneMonitor.isDrawing} zRange={zoneMonitor.zRange} isWarning={(zoneCounts.get('human') || 0) > 0} onAddPoint={zoneMonitor.addPoint} />
                    <gridHelper args={[20, 20, '#444444', '#222222']} rotation={[Math.PI / 2, 0, 0]} />
                    <OrbitControls makeDefault />
                </Canvas>
                    </WebGLErrorBoundary>
                {selectedClusterSnapshot && <ClusterDetailPanel snapshot={selectedClusterSnapshot} onClose={() => setSelectedClusterSnapshot(null)} />}
                {selectedManipSnapshot && <GraphNodeDetailPanel snapshot={selectedManipSnapshot} onClose={() => setSelectedManipSnapshot(null)} />}
            </div>
        </MainLayout>

            <GenericTransformModal
                open={!!transformContext}
                title={transformContext?.title || ''}
                transform={useMemo(() => {
                    if (!transformContext) return null;
                    const { type, id } = transformContext;
                    const map: any = { cloud: pointClouds.find(p => p.id === id), layer: layerSettings[id]?.graphTransform, robot: robotSettings[id]?.transform, marker: markerSettings[id]?.transform, voxel: voxelSettings[id]?.transform };
                    const res = map[type];
                    const defaultT: Transform = { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };
                    if (type === 'cloud') {
                        return res ? { position: res.position || [0, 0, 0], rotation: res.rotation || [0, 0, 0], scale: res.scale || [1, 1, 1] } : null;
                    }
                    return res || defaultT;
                }, [transformContext, pointClouds, layerSettings, robotSettings, markerSettings, voxelSettings])}
                onClose={() => setTransformContext(null)}
                onUpdate={(u) => {
                    if (!transformContext) return;
                    const { type, id } = transformContext;
                    const updaters: any = {
                        cloud: () => setPointClouds(prev => prev.map(p => p.id === id ? { ...p, ...u } : p)),
                        layer: () => handleUpdateLayerSettings(id, { graphTransform: { ...(layerSettings[id]?.graphTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] }), ...u } }),
                        robot: () => updateEntitySettings('robot', id, { transform: { ...(robotSettings[id]?.transform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] }), ...u } }),
                        marker: () => updateEntitySettings('marker', id, { transform: { ...(markerSettings[id]?.transform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] }), ...u } }),
                        voxel: () => updateEntitySettings('voxel', id, { transform: { ...(voxelSettings[id]?.transform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] }), ...u } })
                    };
                    updaters[type]?.();
                }}
                onReset={() => {
                    if (!transformContext) return;
                    const { type, id } = transformContext;
                    const iden = { position: [0, 0, 0] as [number, number, number], rotation: [0, 0, 0] as [number, number, number], scale: [1, 1, 1] as [number, number, number] };
                    const resets: any = {
                        cloud: () => setPointClouds(prev => prev.map(p => p.id === id ? { ...p, ...iden } : p)),
                        layer: () => handleUpdateLayerSettings(id, { graphTransform: iden }),
                        robot: () => updateEntitySettings('robot', id, { transform: iden }),
                        marker: () => updateEntitySettings('marker', id, { transform: iden }),
                        voxel: () => updateEntitySettings('voxel', id, { transform: iden })
                    };
                    resets[type]?.();
                }}
            />

            <RobotJointModal
                open={!!robotJointContext}
                title={robotJointContext?.title || ''}
                subtitle={robotJointContext ? `${robotData[robotJointContext.id]?.jointNames?.length || 0} joints` : undefined}
                robotData={robotJointContext ? (robotData[robotJointContext.id] || null) : null}
                controlMode={robotJointContext ? (robotSettings[robotJointContext.id]?.jointControlMode || 'live') : 'live'}
                jointValues={robotJointContext ? (robotSettings[robotJointContext.id]?.jointValues || []) : []}
                selectedManipLink={robotJointContext ? (robotJointContext.selectedManipLink || robotSettings[robotJointContext.id]?.manipLinkName || '') : ''}
                onClose={() => setRobotJointContext(null)}
                onUpdate={(updates) => {
                    if (!robotJointContext) return;
                    updateEntitySettings('robot', robotJointContext.id, { ...updates, jointControlMode: 'manual' });
                }}
                onModeChange={(mode) => {
                    if (!robotJointContext) return;
                    updateEntitySettings('robot', robotJointContext.id, { jointControlMode: mode });
                }}
                onManipLinkChange={(linkName) => {
                    if (!robotJointContext) return;
                    const robotId = robotJointContext.id;
                    updateEntitySettings('robot', robotId, { manipLinkName: linkName });
                    setRobotJointContext((current) => (
                        current?.id === robotId
                            ? { ...current, selectedManipLink: linkName }
                            : current
                    ));
                }}
                onReset={() => {
                    if (!robotJointContext) return;
                    const robotId = robotJointContext.id;
                    updateEntitySettings('robot', robotId, { jointValues: [], jointControlMode: 'live', manipLinkName: '' });
                    setRobotJointContext((current) => (
                        current?.id === robotId
                            ? { ...current, selectedManipLink: '' }
                            : current
                    ));
                }}
            />

            <EntityColorModal
                open={!!colorContext}
                title={colorContext?.title || ''}
                subtitle={colorContext ? (colorContext.type === 'graph'
                    ? `${graphData[colorContext.id]?.nodes.length || 0} nodes`
                    : colorContext.type === 'robot'
                        ? `${robotData[colorContext.id]?.jointNames?.length || 0} joints`
                        : `${voxelData[colorContext.id]?.data?.length || 0} voxels`) : undefined}
                entityType={colorContext?.type || 'robot'}
                settings={colorContext
                    ? (colorContext.type === 'robot'
                        ? robotSettings[colorContext.id] || null
                        : colorContext.type === 'voxel'
                            ? voxelSettings[colorContext.id] || null
                            : layerSettings[colorContext.id] || null)
                    : null}
                graphData={colorContext?.type === 'graph' ? (graphData[colorContext.id] || null) : null}
                onClose={() => setColorContext(null)}
                onUpdate={(updates) => {
                    if (!colorContext) return;
                    if (colorContext.type === 'robot') {
                        updateEntitySettings('robot', colorContext.id, updates);
                    } else if (colorContext.type === 'voxel') {
                        updateEntitySettings('voxel', colorContext.id, updates);
                    } else {
                        handleUpdateLayerSettings(colorContext.id, updates as Partial<LayerSettings>);
                    }
                }}
            />
        </>
    );
}

export default App;
