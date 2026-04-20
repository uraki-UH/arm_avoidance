import { useMemo, useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import { SidebarContent } from './layout/SidebarContent';
import { PointCloudRenderer } from './features/visualization/PointCloudRenderer';
import { PointCloudData, HeatmapSettings, GraphNode, EditRegion } from './types';
import { GraphRenderer } from './features/visualization/GraphRenderer';
import { useWebSocket } from './hooks/useWebSocket';
import { useClippingPlanes } from './hooks/useClippingPlanes';
import { useZoneMonitor } from './features/analysis/useZoneMonitor';
import { ZoneVisualizer } from './features/analysis/ZoneVisualizer';
import { ClusterDetailPanel, ClusterSnapshot } from './features/visualization/ClusterDetailPanel';

import { Sidebar } from './layout/Sidebar';
import { MainLayout } from './layout/MainLayout';
import { calculateBounds } from './utils/bounds';
import { EditAabbTool } from './features/manipulation/EditAabbTool';

interface DraftRegion {
    center: [number, number, number];
    size: [number, number, number];
}

interface EditJobStatus {
    isRunning: boolean;
    jobId: string;
    progress: number;
    stage: string;
    error?: string;
}

const cloneVec3 = (
    value: [number, number, number] | undefined,
    fallback: [number, number, number]
): [number, number, number] => {
    if (!value) return fallback;
    return [value[0], value[1], value[2]];
};

const defaultDraftRegion: DraftRegion = {
    center: [0, 0, 0],
    size: [1, 1, 1],
};

function createDraftRegionFromCloud(cloud: PointCloudData): DraftRegion {
    if (!cloud.points || cloud.count === 0) {
        return defaultDraftRegion;
    }

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;

    for (let i = 0; i < cloud.count; i++) {
        const x = cloud.points[i * 3];
        const y = cloud.points[i * 3 + 1];
        const z = cloud.points[i * 3 + 2];
        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        minZ = Math.min(minZ, z);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
        maxZ = Math.max(maxZ, z);
    }

    const sizeX = Math.max(0.5, (maxX - minX) * 0.2);
    const sizeY = Math.max(0.5, (maxY - minY) * 0.2);
    const sizeZ = Math.max(0.5, (maxZ - minZ) * 0.2);

    return {
        center: [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2],
        size: [sizeX, sizeY, sizeZ],
    };
}

function draftToMinMax(draft: DraftRegion): {
    min: [number, number, number];
    max: [number, number, number];
} {
    const halfX = draft.size[0] / 2;
    const halfY = draft.size[1] / 2;
    const halfZ = draft.size[2] / 2;

    return {
        min: [draft.center[0] - halfX, draft.center[1] - halfY, draft.center[2] - halfZ],
        max: [draft.center[0] + halfX, draft.center[1] + halfY, draft.center[2] + halfZ],
    };
}

function App() {
    const [pointClouds, setPointClouds] = useState<PointCloudData[]>([]);
    const [selectedLayerId, setSelectedLayerId] = useState<string | null>(null);
    const [transformMode, setTransformMode] = useState<'translate' | 'rotate' | 'scale'>('translate');
    const [heatmapSettings, setHeatmapSettings] = useState<HeatmapSettings>({
        mode: 'rgb',
        min: -2,
        max: 5,
        colorScheme: 'viridis',
        pointSize: 0.02,
        simpleColor: '#ffffff',
    });
    const [pointCloudOpacity, setPointCloudOpacity] = useState(1);
    const [isSidebarOpen, setIsSidebarOpen] = useState(true);

    const wsUrl = `ws://${window.location.hostname}:9001`;
    const {
        pointCloud: wsPointCloud,
        graphData,
        lastJobEvent,
        isConnected,
        error: wsError,
        connect,
        disconnect,
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

    const clipping = useClippingPlanes();
    const zoneMonitor = useZoneMonitor();

    const zoneCounts = useMemo(() => zoneMonitor.getZoneCounts(graphData), [graphData, zoneMonitor.points]);

    const [disabledSourceIds, setDisabledSourceIds] = useState<Set<string>>(new Set());

    const [gngLayer, setGngLayer] = useState({
        visible: true,
        removed: false,
        showGraph: true,
        showClusters: true,
        showClusterText: true,
        showNormals: true,
        normalArrowLength: 0.15,
        normalArrowColor: '#00FFFF',
        nodeScale: 0.015,
        edgeWidth: 0.007,
        visibleLabels: {
            0: true,
            1: true,
            2: true,
            3: true,
            4: true,
            5: true,
        },
    });

    const [selectedClusterSnapshot, setSelectedClusterSnapshot] = useState<ClusterSnapshot | null>(null);

    const [isEditMode, setIsEditMode] = useState(false);
    const [editLayerId, setEditLayerId] = useState<string | null>(null);
    const [editSessionId, setEditSessionId] = useState<string | null>(null);
    const [editRegions, setEditRegions] = useState<EditRegion[]>([]);
    const [draftRegion, setDraftRegion] = useState<DraftRegion>(defaultDraftRegion);
    const [regionGizmoMode, setRegionGizmoMode] = useState<'translate' | 'scale'>('translate');
    const [editJobStatus, setEditJobStatus] = useState<EditJobStatus | null>(null);

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
        if (wsPointCloud && !disabledSourceIds.has(wsPointCloud.id)) {
            if (isEditMode && editLayerId === wsPointCloud.id) {
                return;
            }

            setPointClouds((prev) => {
                const existingCloud = prev.find((pc) => pc.id === wsPointCloud.id);
                const filtered = prev.filter((pc) => pc.id !== wsPointCloud.id);

                const newCloud = {
                    ...wsPointCloud,
                    visible: existingCloud ? existingCloud.visible : true,
                    opacity: existingCloud?.opacity ?? pointCloudOpacity,
                    position: existingCloud?.position || wsPointCloud.position || [0, 0, 0],
                    rotation: existingCloud?.rotation || wsPointCloud.rotation || [0, 0, 0],
                    scale: existingCloud?.scale || wsPointCloud.scale || [1, 1, 1],
                };

                return [...filtered, newCloud];
            });
        }
    }, [wsPointCloud, disabledSourceIds, pointCloudOpacity, isEditMode, editLayerId]);

    useEffect(() => {
        setPointClouds((prev) => prev.map((pc) => ({ ...pc, opacity: pointCloudOpacity })));
    }, [pointCloudOpacity]);

    useEffect(() => {
        if (graphData) {
            setGngLayer((prev) => prev.removed ? { ...prev, removed: false, visible: true } : prev);
        }
    }, [graphData]);

    useEffect(() => {
        if (!lastJobEvent || !editJobStatus) {
            return;
        }

        if (lastJobEvent.jobId !== editJobStatus.jobId) {
            return;
        }

        if (lastJobEvent.type === 'job.progress') {
            setEditJobStatus((prev) => prev ? {
                ...prev,
                progress: lastJobEvent.progress,
                stage: lastJobEvent.stage,
            } : prev);
            return;
        }

        if (lastJobEvent.type === 'job.failed') {
            setEditJobStatus((prev) => prev ? {
                ...prev,
                isRunning: false,
                stage: 'failed',
                error: `${lastJobEvent.error.code}: ${lastJobEvent.error.message}`,
            } : prev);
            return;
        }

        if (lastJobEvent.type === 'job.completed') {
            (async () => {
                try {
                    await subscribeSource(lastJobEvent.publishedTopic);
                } catch (subscribeError) {
                    console.warn('Failed to subscribe edited topic:', subscribeError);
                }

                setSelectedLayerId(lastJobEvent.publishedTopic);
                setIsEditMode(false);
                setEditLayerId(null);
                setEditSessionId(null);
                setEditRegions([]);
                setEditJobStatus((prev) => prev ? { ...prev, isRunning: false, progress: 100, stage: 'completed' } : prev);
            })();
        }
    }, [editJobStatus, lastJobEvent, subscribeSource]);

    const handleAddPointCloud = (data: PointCloudData) => {
        const cloud = {
            ...data,
            opacity: data.opacity ?? pointCloudOpacity,
        };
        setPointClouds((prev) => [...prev, cloud]);
        setSelectedLayerId(data.id);
    };

    const handleStartEdit = async () => {
        if (!selectedLayerId || isEditMode) return;
        const cloud = pointClouds.find((pc) => pc.id === selectedLayerId);
        if (!cloud || cloud.id.endsWith('/edited')) return;
        if (!cloud.id.startsWith('/')) {
            alert('Only ROS topic layers can be edited in backend session mode.');
            return;
        }

        const maxRetries = 3;
        for (let attempt = 0; attempt <= maxRetries; attempt++) {
            try {
                const session = await openEditSession(cloud.id, 'map');
                setEditSessionId(session.sessionId);
                setEditLayerId(cloud.id);
                setIsEditMode(true);
                setSelectedLayerId(cloud.id);
                setEditRegions(session.regions || []);
                setDraftRegion(createDraftRegionFromCloud(cloud));
                setRegionGizmoMode('translate');
                setEditJobStatus(null);
                return;
            } catch (startError) {
                const msg = startError instanceof Error ? startError.message : '';
                const isSnapshotNotReady = msg.includes('snapshot') || msg.includes('Retry');
                if (isSnapshotNotReady && attempt < maxRetries) {
                    console.log(`Edit session snapshot not ready, retrying (${attempt + 1}/${maxRetries})...`);
                    await new Promise((r) => setTimeout(r, 1000));
                    continue;
                }
                console.error('Failed to start edit session:', startError);
                alert(`Failed to start edit session: ${msg || 'Unknown error'}`);
            }
        }
    };

    const handleCancelEdit = async () => {
        const currentSessionId = editSessionId;
        if (currentSessionId) {
            try {
                await cancelEditSession(currentSessionId);
            } catch (cancelError) {
                console.warn('Failed to cancel edit session on backend:', cancelError);
            }
        }

        setIsEditMode(false);
        setEditLayerId(null);
        setEditSessionId(null);
        setEditRegions([]);
        setEditJobStatus(null);
    };

    const handleAddRegion = async () => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;
        const { min, max } = draftToMinMax(draftRegion);

        try {
            const result = await addEditRegion(editSessionId, min, max, 'map');
            setEditRegions((prev) => [...prev, result.region]);
        } catch (addError) {
            console.error('Failed to add region:', addError);
            alert(`Failed to add region: ${addError instanceof Error ? addError.message : 'Unknown error'}`);
        }
    };

    const handleRemoveRegion = async (regionId: string) => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;

        try {
            await removeEditRegion(editSessionId, regionId);
            setEditRegions((prev) => prev.filter((region) => region.regionId !== regionId));
        } catch (removeError) {
            console.error('Failed to remove region:', removeError);
            alert(`Failed to remove region: ${removeError instanceof Error ? removeError.message : 'Unknown error'}`);
        }
    };

    const handleClearRegions = async () => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;

        try {
            await clearEditRegions(editSessionId);
            setEditRegions([]);
        } catch (clearError) {
            console.error('Failed to clear regions:', clearError);
            alert(`Failed to clear regions: ${clearError instanceof Error ? clearError.message : 'Unknown error'}`);
        }
    };

    const handlePublishEditedCloud = async () => {
        if (!isEditMode || !editSessionId || !isConnected) return;

        const cloud = pointClouds.find((pc) => pc.id === editLayerId);
        if (!cloud) return;

        try {
            const position = cloneVec3(cloud.position, [0, 0, 0]);
            const rotation = cloneVec3(cloud.rotation, [0, 0, 0]);
            const scale = cloneVec3(cloud.scale, [1, 1, 1]);

            const result = await commitEditSession(editSessionId, {
                position,
                rotation,
                scale,
            });

            setEditJobStatus({
                isRunning: true,
                jobId: result.jobId,
                progress: 0,
                stage: 'queued',
            });
        } catch (commitError) {
            console.error('Failed to commit edit session:', commitError);
            alert(`Failed to commit edit session: ${commitError instanceof Error ? commitError.message : 'Unknown error'}`);
        }
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
            // Show existing cloud immediately (for latched/one-shot topics like /edited)
            setPointClouds((prev) => prev.map((pc) =>
                pc.id === sourceId ? { ...pc, visible: true } : pc
            ));
            return;
        }

        setDisabledSourceIds((prev) => new Set(prev).add(sourceId));
        // Hide the cloud instead of removing it, so it can be shown again on re-enable
        setPointClouds((prev) => {
            const exists = prev.some((pc) => pc.id === sourceId);
            if (exists) {
                return prev.map((pc) =>
                    pc.id === sourceId ? { ...pc, visible: false } : pc
                );
            }
            return prev;
        });
        if (selectedLayerId === sourceId) {
            const fallback = pointClouds.find((pc) => pc.id !== sourceId && pc.visible !== false);
            setSelectedLayerId(fallback ? fallback.id : null);
        }
    };

    const handleUpdateTransform = (id: string, updates: Partial<PointCloudData>) => {
        if (!isEditMode || id !== editLayerId) return;
        if (editJobStatus?.isRunning) return;

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
    const selectedIsEditedLayer = selectedCloud ? selectedCloud.id.endsWith('/edited') : false;
    const selectedIsTopicLayer = selectedCloud ? selectedCloud.id.startsWith('/') : false;
    const canStartEdit = Boolean(selectedCloud) && selectedIsTopicLayer && !selectedIsEditedLayer && !isEditMode;
    const startEditDisabledReason = !selectedCloud
        ? 'Select a layer to edit.'
        : !selectedIsTopicLayer
            ? 'Only ROS topic layers can be edited.'
            : selectedIsEditedLayer
                ? 'Select the original topic layer (not /edited).'
                : undefined;

    const renderClouds = isEditMode && editLayerId
        ? pointClouds.filter((pc) => pc.id === editLayerId)
        : pointClouds;

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

        if (!graphData) return;

        const cluster = graphData.clusters.find((c) => c.id === clusterId);
        if (!cluster) return;

        const nodeIdsSet = new Set(cluster.nodeIds);
        const nodes = cluster.nodeIds.map((idx) => graphData.nodes[idx]).filter((n) => n !== undefined);

        const edges: { source: GraphNode; target: GraphNode }[] = [];
        for (let i = 0; i < graphData.edges.length; i += 2) {
            const srcIdx = graphData.edges[i];
            const dstIdx = graphData.edges[i + 1];
            if (nodeIdsSet.has(srcIdx) && nodeIdsSet.has(dstIdx)) {
                const srcNode = graphData.nodes[srcIdx];
                const dstNode = graphData.nodes[dstIdx];
                if (srcNode && dstNode) {
                    edges.push({ source: srcNode, target: dstNode });
                }
            }
        }

        setSelectedClusterSnapshot({
            cluster: JSON.parse(JSON.stringify(cluster)),
            nodes: JSON.parse(JSON.stringify(nodes)),
            edges: JSON.parse(JSON.stringify(edges)),
        });
    };

    const handleDraftRegionChange = (center: [number, number, number], size: [number, number, number]) => {
        setDraftRegion({ center, size });
    };

    return (
        <MainLayout
            isSidebarOpen={isSidebarOpen}
            sidebar={
                <Sidebar isOpen={isSidebarOpen} onToggle={toggleSidebar}>
                    <SidebarContent
                        isConnected={isConnected}
                        connect={connect}
                        disconnect={disconnect}
                        wsError={wsError}
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
                        gngLayer={gngLayer}
                        setGngLayer={setGngLayer}
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
                    />
                </Sidebar>
            }
        >
            <div className="w-full h-full relative bg-gradient-to-br from-[var(--bg-primary)] to-black">
                <Canvas
                    camera={{ position: [5, 5, 5], up: [0, 0, 1], fov: 50 }}
                    gl={{
                        localClippingEnabled: false,
                        clippingPlanes: clipping.getThreePlanes(),
                    }}
                >
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

                    {graphData && gngLayer.visible && !gngLayer.removed && (
                        <GraphRenderer
                            data={graphData}
                            showNodes={gngLayer.showGraph}
                            showEdges={gngLayer.showGraph}
                            showClusters={gngLayer.showClusters}
                            showClusterText={gngLayer.showClusterText}
                            visibleLabels={gngLayer.visibleLabels}
                            nodeScale={gngLayer.nodeScale}
                            edgeWidth={gngLayer.edgeWidth}
                            selectedClusterId={selectedClusterSnapshot?.cluster.id ?? null}
                            onClusterSelect={handleClusterSelect}
                            enableClusterSelection={!zoneMonitor.isDrawing}
                        />
                    )}

                    <ZoneVisualizer
                        points={zoneMonitor.points}
                        isDrawing={zoneMonitor.isDrawing}
                        zRange={zoneMonitor.zRange}
                        isWarning={(zoneCounts.get('human') || 0) > 0}
                        onAddPoint={zoneMonitor.addPoint}
                    />

                    <gridHelper args={[20, 20, '#444444', '#222222']} rotation={[Math.PI / 2, 0, 0]} />
                    <OrbitControls makeDefault />
                    <Stats />
                </Canvas>

                {selectedClusterSnapshot && (
                    <ClusterDetailPanel
                        snapshot={selectedClusterSnapshot}
                        onClose={() => setSelectedClusterSnapshot(null)}
                    />
                )}
            </div>
        </MainLayout>
    );
}

export default App;
