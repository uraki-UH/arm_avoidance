import { useMemo, useEffect, useState, useRef, useCallback } from 'react';
import * as THREE from 'three';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { SidebarContent } from './layout/SidebarContent';
import { useLocalMeshes } from './features/meshes/use_local_meshes';
import { LocalMeshRenderer } from './features/meshes/LocalMeshRenderer';
import { PointCloudRenderer } from './features/visualization/PointCloudRenderer';
import {
    RobotSettings,
    Transform,
    VoxelSettings,
    EntityType,
    PointCloudData,
    HeatmapSettings,
    point_cloud_display_settings,
    GraphNode,
    graph_selection,
    LayerSettings,
    ClippingPlane,
    ClippingAxis
} from './types';
import { GraphRenderer } from './features/visualization/GraphRenderer';
import { RobotRenderer } from './features/visualization/RobotRenderer';
import { robot_candidate_idx } from './features/visualization/robot_candidate_display';
import { CollisionRenderer } from './features/visualization/CollisionRenderer';
import { MarkerArrayRenderer } from './features/visualization/MarkerArrayRenderer';
import { CandidateHoverFrame } from './features/visualization/CandidateHoverFrame';
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
import { GenericTransformModal } from './features/manipulation/GenericTransformPanel';
import { RobotJointModal } from './features/manipulation/RobotJointModal';
import { EntityColorModal } from './features/manipulation/EntityColorModal';

import { Sidebar, MainLayout } from './layout/MainLayout';
import { calculateBounds } from './utils/bounds';
import { EditAabbTool } from './features/manipulation/EditAabbTool';
import { useEditSession } from './features/manipulation/editSession';
import { WebGLErrorBoundary } from './components/WebGLErrorBoundary';

type ColorContext = { type: 'robot' | 'voxel' | 'graph'; id: string; title: string };
type point_cloud_view_settings = Pick<PointCloudData,
    'visible' | 'opacity' | 'position' | 'rotation' | 'scale' | 'matrix'>;

function App() {
    const local_meshes = useLocalMeshes();
    const [pointClouds, setPointClouds] = useState<PointCloudData[]>([]);
    // 配信元の停止を跨ぐ表示設定のみの保持。点群バッファの保持なし。
    const point_cloud_view_settings_ref = useRef(new Map<string, point_cloud_view_settings>());
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
    const [point_cloud_display, set_point_cloud_display] = useState<Record<string, point_cloud_display_settings>>({});
    const [robotSettings, setRobotSettings] = useState<Record<string, RobotSettings>>({});
    const [markerSettings, setMarkerSettings] = useState<Record<string, { visible: boolean, transform?: Transform, max_visible_candidates?: number }>>({});
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
        getTemplateMatchConfig,
        applyTemplateMatchConfig,
        register_vehicle,
        inspect_graph,
        inspect_graph_bounds,
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
                    visible: true, color: 'skyblue', showVisual: true, showCollision: false, showManipulabilityEllipsoid: false, collisionColor: '#ff9f1c', opacity: 1, jointControlMode: 'live',
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
                    const isIsolatedVoxel = data === voxelData
                        && /(^|\/)isolated$/.test(tag);
                    const isLabeledVoxel = data === voxelData
                        && Array.isArray(entityData?.labels)
                        && entityData.labels.length > 0
                        && entityData.labels.length === entityData.data?.length;
                    if (!next[tag]) {
                        next[tag] = {
                            ...defaults,
                            ...(isLabeledVoxel ? { color: '#ffff00', colorMode: 'uniform' } : {}),
                            ...(isIsolatedVoxel ? { color: '#ff3131', colorMode: 'uniform' } : {}),
                            ...(data === robotData && tag.includes('candidate_goal_preview') ? { opacity: 0.18 } : {}),
                        };
                        changed = true;
                    } else if (isLabeledVoxel && next[tag].colorMode === undefined) {
                        next[tag] = {
                            ...next[tag],
                            color: isIsolatedVoxel
                                ? '#ff3131'
                                : next[tag].color === '#00ff88' ? '#ffff00' : next[tag].color,
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
    const [is_inspecting, set_is_inspecting] = useState(false);
    const [inspection_error, set_inspection_error] = useState<string | null>(null);
    const inspection_request = useRef(0);
    const inspection_sources = useRef({ graphData, markerData, layerSettings });
    inspection_sources.current = { graphData, markerData, layerSettings };
    const inspection_snapshot = useRef(selectedClusterSnapshot);
    inspection_snapshot.current = selectedClusterSnapshot;
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
            for (const cloud of prev) {
                const { visible, opacity, position, rotation, scale, matrix } = cloud;
                point_cloud_view_settings_ref.current.set(cloud.id,
                    { visible, opacity, position, rotation, scale, matrix });
            }
            let next = [...prev];
            let changed = false;

            Object.values(wsPointClouds).forEach((cloud) => {
                if (disabledSourceIds.has(cloud.id)) return;
                if (isEditMode && editLayerId === cloud.id) return;

                const index = next.findIndex((pc) => pc.id === cloud.id);
                const saved_settings = next[index] ?? point_cloud_view_settings_ref.current.get(cloud.id);
                const newCloud = {
                    ...cloud,
                    visible: saved_settings?.visible ?? true,
                    opacity: saved_settings?.opacity ?? pointCloudOpacity,
                    position: saved_settings?.position ?? cloud.position ?? [0, 0, 0],
                    rotation: saved_settings?.rotation ?? cloud.rotation ?? [0, 0, 0],
                    scale: saved_settings?.scale ?? cloud.scale ?? [1, 1, 1],
                    matrix: saved_settings?.matrix ?? cloud.matrix,
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
        for (const [id, settings] of point_cloud_view_settings_ref.current) {
            point_cloud_view_settings_ref.current.set(id, { ...settings, opacity: pointCloudOpacity });
        }
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

    const update_point_cloud_display = (id: string, settings: point_cloud_display_settings | null) => {
        set_point_cloud_display(prev => {
            const next = { ...prev };
            if (settings) next[id] = settings;
            else delete next[id];
            return next;
        });
    };

    const handleRemoveLayer = (id: string) => {
        if (isEditMode) return;
        point_cloud_view_settings_ref.current.delete(id);
        update_point_cloud_display(id, null);
        setDisabledSourceIds((prev) => new Set(prev).add(id));
        setPointClouds((prev) => {
            const filtered = prev.filter((pc) => pc.id !== id);
            if (selectedLayerId === id) {
                const fallback = filtered.find((pc) => pc.visible !== false);
                setSelectedLayerId(fallback ? fallback.id : null);
            }
            return filtered;
        });
        void unsubscribeSource(id, true).catch((unsubscribeError) => {
            console.warn('点群レイヤーの購読解除に失敗しました:', unsubscribeError);
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

    const close_inspection = useCallback(() => {
        ++inspection_request.current;
        setSelectedClusterSnapshot(null);
        set_is_inspecting(false);
        set_inspection_error(null);
    }, []);

    // 選択時の全受信フレームを送信し、切り出しはバックエンドへ委譲。
    const handle_inspect = useCallback(async (source_id: string, selection: graph_selection) => {
        if (source_id === '/topological_map' || inspection_sources.current.layerSettings[source_id]?.enable_bounding_box !== true) return;
        const request_id = ++inspection_request.current;
        const source = inspection_sources.current;
        set_is_inspecting(true);
        set_inspection_error(null);
        try {
            const graph = source.graphData[source_id];
            const marker_array = source.markerData[source_id];
            if (selection.kind === 'marker' ? !marker_array : !graph) throw new Error('選択元のデータがありません');
            const snapshot = await inspect_graph(source_id, selection,
                selection.kind === 'marker' ? undefined : graph, selection.kind === 'marker' ? marker_array : undefined);
            if (request_id !== inspection_request.current ||
                inspection_sources.current.layerSettings[source_id]?.enable_bounding_box !== true) return;
            setSelectedClusterSnapshot({ ...snapshot, settings: source.layerSettings[source_id] });
        } catch (error) {
            if (request_id === inspection_request.current &&
                inspection_sources.current.layerSettings[source_id]?.enable_bounding_box === true) set_inspection_error(
                error instanceof Error ? error.message : String(error));
        } finally {
            if (request_id === inspection_request.current) set_is_inspecting(false);
        }
    }, [inspect_graph]);

    const refresh_inspection = useCallback(() => {
        const snapshot = inspection_snapshot.current;
        if (snapshot) void handle_inspect(snapshot.source_id, snapshot.selection);
    }, [handle_inspect]);

    const get_hover_bounds = useCallback(async (source_id: string) => {
        if (source_id === '/topological_map') return [];
        const source = inspection_sources.current;
        return inspect_graph_bounds(source_id, source.graphData[source_id]);
    }, [inspect_graph_bounds]);

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
                            local_meshes={local_meshes}
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
                            point_cloud_display={point_cloud_display}
                            on_update_point_cloud_display={update_point_cloud_display}
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
                            getTemplateMatchConfig={getTemplateMatchConfig}
                            applyTemplateMatchConfig={applyTemplateMatchConfig}
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
                                if (/(^|[/_-])candidate(?:[/_-]|$)/i.test(id)) return;
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
                        dpr={1}
                        camera={{ position: [5, 5, 5], up: [0, 0, 1], fov: 50 }}
                        gl={canvasGl}
                        onCreated={({ gl, invalidate }) => {
                            gl.domElement.addEventListener('webglcontextlost', (e) => {
                                e.preventDefault();
                                console.warn('[WebGL] context lost; waiting for browser restoration', {
                                    time: new Date().toISOString(),
                                    statusMessage: (e as WebGLContextEvent).statusMessage,
                                });
                            });
                            gl.domElement.addEventListener('webglcontextrestored', () => {
                                console.info('[WebGL] context restored', new Date().toISOString());
                                invalidate();
                            });
                        }}
                    >
                        <ClippingPlaneSync planes={threeClippingPlanes} />
                        <ambientLight intensity={0.3} />
                        {local_meshes.items.map(item => <LocalMeshRenderer key={item.id} item={item}
                            focus_req={local_meshes.focus?.id === item.id ? local_meshes.focus.req : undefined} />)}
                        <pointLight position={[10, 10, 10]} intensity={0.5} />
                        <pointLight position={[-10, -10, -10]} intensity={0.3} />

                        {renderClouds.map((pc) => {
                            const tf = pc.frameId && pc.frameId !== 'world' ? (transforms[pc.frameId] ?? null) : null;
                            return (
                                <PointCloudRenderer
                                    key={pc.id}
                                    data={pc}
                                    tf={tf}
                                    heatmapSettings={point_cloud_display[pc.id] ?? heatmapSettings}
                                    opacity={point_cloud_display[pc.id]?.opacity}
                                    selected={isEditMode && pc.id === editLayerId}
                                    transformMode={transformMode}
                                    onTransformChange={(pos, rot, scale) => handleTransformChange(pc.id, pos, rot, scale)}
                                />
                            );
                        })}

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
                                    data: robotData, settings: robotSettings, component: (tag: string, d: any, s: any, tf: any) => {
                                    const selected_idx = robot_candidate_idx(s.selected_candidate_idx, d.instances?.length ?? 0);
                                    return (
                                    <group key={tag}>
                                {s.showVisual && <RobotRenderer tag={tag} data={d} visible={true} color={s.color} useUrdfColors={s.useUrdfColors ?? true} link_colors={s.link_colors} link_appearance={s.link_appearance} emissiveIntensity={s.emissiveIntensity ?? 0.2} opacity={s.opacity ?? (tag.includes('candidate_goal_preview') ? 0.18 : 1)} jointValuesOverride={s.jointControlMode === 'manual' ? (s.jointValues || []) : []} tf={tf} manualTransform={s.transform} showManipulabilityEllipsoid={s.showManipulabilityEllipsoid ?? false} manipEllipsoidType={s.manipEllipsoidType || 'translational'} manipLinkName={s.manipLinkName || ''} max_visible_candidates={s.max_visible_candidates} selected_candidate_idx={s.selected_candidate_idx} onManipClick={(linkName) => setRobotJointContext({ id: tag, title: `Robot joints: ${tag}`, selectedManipLink: linkName })} />}
                                {s.showCollision && (!d.instances || d.instances.length > 0) && <CollisionRenderer tag={tag} data={selected_idx === null ? d : { ...d, ...d.instances?.[selected_idx] }} visible={true} color={s.collisionColor} opacity={Math.min(s.opacity ?? 1, 0.28)} tf={tf} manualTransform={s.transform} />}
                            </group>
                                ); }, defaultSettings: { visible: true, color: 'skyblue', useUrdfColors: true, showVisual: true, showCollision: false, showManipulabilityEllipsoid: false, manipEllipsoidType: 'translational', manipLinkName: '', collisionColor: '#ff9f1c', emissiveIntensity: 0.2, transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } }
                            },
                            {
                                data: markerData, settings: markerSettings, component: (tag: string, d: any, s: any) => (
                                    <MarkerArrayRenderer key={tag} tag={tag} data={d} visible={true} transforms={transforms} manualTransform={s.transform}
                                        max_visible_candidates={s.max_visible_candidates}
                                        on_inspect={tag !== '/topological_map' && !isEditMode && !zoneMonitor.isDrawing && layerSettings[tag]?.enable_bounding_box === true ? marker => void handle_inspect(tag,
                                            { kind: 'marker', id: marker.id, ns: marker.ns }) : undefined} />
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
                            return <GraphRenderer key={tag} tag={tag} data={data} settings={settings} tf={tf}
                                selectedClusterId={selectedClusterSnapshot?.source_id === tag && selectedClusterSnapshot.selection.kind === 'cluster'
                                    ? selectedClusterSnapshot.selection.id : null}
                                onClusterSelect={id => id === null ? close_inspection() : void handle_inspect(tag, { kind: 'cluster', id })}
                                on_node_select={node => void handle_inspect(tag, { kind: 'node', id: node.id! })}
                                onManipSelect={(node) => handleManipSelect(tag, node)}
                                enableClusterSelection={tag !== '/topological_map' && !isEditMode && !zoneMonitor.isDrawing && settings.enable_bounding_box === true} />;
                        })}

                    <ZoneVisualizer points={zoneMonitor.points} isDrawing={zoneMonitor.isDrawing} zRange={zoneMonitor.zRange} isWarning={(zoneCounts.get('human') || 0) > 0} onAddPoint={zoneMonitor.addPoint} />
                    <CandidateHoverFrame is_enabled={!isEditMode && !zoneMonitor.isDrawing} get_bounds={get_hover_bounds} on_inspect={handle_inspect}
                        transforms={transforms} layer_settings={layerSettings} />
                    <gridHelper args={[20, 20, '#444444', '#222222']} rotation={[Math.PI / 2, 0, 0]} />
                    <OrbitControls makeDefault />
                </Canvas>
                    </WebGLErrorBoundary>
                {selectedClusterSnapshot && <ClusterDetailPanel snapshot={selectedClusterSnapshot} onClose={close_inspection}
                    on_refresh={refresh_inspection} is_loading={is_inspecting} error={inspection_error} register_vehicle={register_vehicle} />}
                {!selectedClusterSnapshot && (is_inspecting || inspection_error) &&
                    <div role="status" className="surface-panel absolute right-4 top-4 z-50 flex max-w-md items-center gap-3 p-3 text-sm">
                        <span>{is_inspecting ? '候補の形状を取得中...' : inspection_error}</span>
                        <button className="btn-secondary px-2" onClick={close_inspection}>閉じる</button>
                    </div>}
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
                robot_data={colorContext?.type === 'robot' ? (robotData[colorContext.id] || null) : null}
                is_candidate_robot={colorContext?.type === 'robot' && /(^|[/_-])candidate(?:[/_-]|$)/i.test(colorContext.id)}
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
