import React, { useState } from 'react';
import {
    Activity,
    Box,
    Database,
    Eye,
    EyeOff,
    Gauge,
    Layers,
    Move,
    PlayCircle,
    Scissors,
    Server,
    Trash2,
    UploadCloud,
    Wifi,
    WifiOff,
} from 'lucide-react';
import { Tabs } from '../components/ui/Tabs';
import { CollapsibleSection } from '../components/ui/CollapsibleSection';
import { ServerFileBrowser } from '../features/io/ServerFileBrowser';
import { SourceSelector } from '../features/io/SourceSelector';
import { HeatmapControls } from '../features/visualization/HeatmapControls';
import { ExportPanel } from '../features/io/ExportPanel';
import { GenericTransformPanel } from '../features/manipulation/GenericTransformPanel';
// removed
import { ClippingControls } from '../features/manipulation/ClippingControls';
import { GngLayerControls, type GngLayerState } from '../features/visualization/GngLayerControls';
import { GngLabelModal } from '../features/visualization/GngLabelModal';
import { ZoneMonitorPanel } from '../features/analysis/ZoneMonitorPanel';
import { GngDownsamplingPanel } from '../features/analysis/GngDownsamplingPanel';
import { RosbagPlayer } from '../features/io/RosbagPlayer';
import { LayerItem, ControlSlider } from '../components/ui/SharedControls';
import { TfCalibrationPanel } from '../features/manipulation/TfCalibrationPanel';

import {
    PointCloudData,
    HeatmapSettings,
    GraphData,
    EditRegion,
    DataSource,
    RosbagInfo,
    PlaybackStatus,
    PointCloudFileInfo,
    GngStatus,
    GngParams,
    ContinuousPublishStatus,
    NodeParameters,
    SetParameterResult,
    LayerSettings,
    RobotData,
    RobotSettings,
    MarkerArrayData,
    Transform,
    TransformData,
    VoxelData,
    VoxelSettings,
    EntityType,
} from '../types';

interface SidebarContentProps {
    isConnected: boolean;
    connect: () => void;
    disconnect: () => void;
    wsError: string | null;

    getSources: () => Promise<DataSource[]>;
    subscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string }>;
    unsubscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string }>;
    onSourceToggled: (sourceId: string, active: boolean) => void;
    onLoadCloud: (data: PointCloudData) => void;

    listRosbags: () => Promise<RosbagInfo[]>;
    playRosbag: (path: string, remaps: string[], loop: boolean) => Promise<{ success: boolean }>;
    stopRosbag: () => Promise<{ success: boolean }>;
    getRosbagStatus: () => Promise<PlaybackStatus>;

    listPointCloudFiles: () => Promise<PointCloudFileInfo[]>;
    loadPointCloudFile: (path: string) => Promise<{ success: boolean; pointCount?: number }>;

    startGng: (params: GngParams) => Promise<{ success: boolean; pid?: number; inputTopic?: string }>;
    stopGng: () => Promise<{ success: boolean }>;
    getGngStatus: () => Promise<GngStatus>;
    listGngConfigs: () => Promise<{ name: string; path: string }[]>;
    getParameters: () => Promise<NodeParameters>;
    setParameter: (paramName: string, value: number | string | boolean) => Promise<SetParameterResult>;

    startContinuousPublish: (topic: string, rateHz: number) => Promise<{ success: boolean; topic?: string; rateHz?: number }>;
    stopContinuousPublish: () => Promise<{ success: boolean }>;
    getContinuousPublishStatus: () => Promise<ContinuousPublishStatus>;

    totalPoints: number;
    pointClouds: PointCloudData[];
    selectedLayerId: string | null;
    onSelectLayer: (id: string | null) => void;
    onToggleVisibility: (id: string) => void;
    onRemoveLayer: (id: string) => void;

    graphData: Record<string, GraphData>;
    layerSettings: Record<string, LayerSettings>;
    onUpdateLayerSettings: (tag: string, updates: Partial<LayerSettings>) => void;
    onRemoveGngLayer: (tag: string) => void;
    gngLayer: GngLayerState;
    setGngLayer: React.Dispatch<React.SetStateAction<GngLayerState>>;

    heatmapSettings: HeatmapSettings;
    setHeatmapSettings: (settings: HeatmapSettings) => void;
    pointCloudOpacity: number;
    setPointCloudOpacity: (opacity: number) => void;
    bounds: any;

    selectedCloud: PointCloudData | undefined;
    transformMode: 'translate' | 'rotate' | 'scale';
    setTransformMode: (mode: 'translate' | 'rotate' | 'scale') => void;
    onUpdateTransform: (id: string, updates: Partial<PointCloudData>) => void;

    clipping: any;

    onPublishEdited: () => void;
    isEditMode: boolean;
    onStartEdit: () => void;
    onCancelEdit: () => void;
    canStartEdit: boolean;
    startEditDisabledReason?: string;
    editRegions: EditRegion[];
    onAddRegion: () => void;
    onRemoveRegion: (regionId: string) => void;
    onClearRegions: () => void;
    draftRegion: {
        center: [number, number, number];
        size: [number, number, number];
    };
    regionGizmoMode: 'translate' | 'scale';
    setRegionGizmoMode: (mode: 'translate' | 'scale') => void;
    editJobStatus: {
        isRunning: boolean;
        jobId: string;
        progress: number;
        stage: string;
        error?: string;
    } | null;

    zoneMonitor: any;
    zoneCounts: Map<string, number>;

    robotData: Record<string, RobotData>;
    robotSettings: Record<string, RobotSettings>;
    markerData: Record<string, MarkerArrayData>;
    markerSettings: Record<string, { visible: boolean, transform?: Transform }>;
    voxelData: Record<string, VoxelData>;
    voxelSettings: Record<string, VoxelSettings>;

    onUpdateSettings: (type: EntityType, tag: string, updates: any) => void;
    onRemoveEntity: (type: EntityType, tag: string) => void;
    transforms: Record<string, TransformData>;
    onOpenTransform: (type: 'cloud' | 'layer' | 'robot' | 'marker' | 'voxel', id: string, title: string) => void;
    onOpenRobotJoints: (id: string, title: string) => void;
    onOpenColorSettings: (type: 'robot' | 'voxel' | 'graph', id: string, title: string) => void;
}

const ColorActionButton: React.FC<{
    title: string;
    swatches: { color?: string; title: string }[];
    onClick: () => void;
}> = ({ title, swatches, onClick }) => {
    const previewSwatches = swatches.length > 0 ? swatches : [{ color: '#7c8c66', title }];
    return (
        <button
            onClick={onClick}
            title={title}
            className="entity-btn w-full justify-between gap-2 px-2 py-1.5 text-[11px]"
        >
            <span className="flex items-center gap-2">
                {previewSwatches.map((swatch) => (
                    <span
                        key={swatch.title}
                        className="h-3.5 w-3.5 rounded border border-white/20 shadow-[0_0_0_1px_rgba(0,0,0,0.25)_inset]"
                        style={{ backgroundColor: swatch.color || '#7c8c66' }}
                        aria-hidden="true"
                        title={swatch.title}
                    />
                ))}
            </span>
        </button>
    );
};

export const SidebarContent: React.FC<SidebarContentProps> = (props) => {
    const hasGngLayer = Boolean(props.graphData && !props.gngLayer.removed);
    const isLayerActionDisabled = props.isEditMode;
    const [labelContext, setLabelContext] = useState<{ tag: string; title: string } | null>(null);

    const layersTab = (
        <div className="space-y-3">
            <div className="surface-soft grid grid-cols-2 gap-2 p-3">
                <div className="surface-muted p-2">
                    <p className="panel-title mb-1">Connection</p>
                    <span className={`status-pill ${props.isConnected ? 'is-connected' : 'is-disconnected'}`}>
                        {props.isConnected ? 'Online' : 'Offline'}
                    </span>
                </div>
                <div className="surface-muted p-2">
                    <p className="panel-title mb-1">Workspace</p>
                    <p className="text-sm font-semibold text-[var(--text-primary)]">{props.pointClouds.length} Layers</p>
                    <p className="text-[11px] text-[var(--text-secondary)]">{props.totalPoints.toLocaleString()} points</p>
                </div>
            </div>

            <CollapsibleSection title="Connection & Streams" icon={<Database size={16} />} defaultOpen={true}>
                <div className="surface-muted space-y-3 p-3">
                    <button
                        onClick={props.isConnected ? props.disconnect : props.connect}
                        className={`w-full px-4 py-2 text-sm font-semibold ${props.isConnected ? 'btn-danger text-white' : 'btn-primary'}`}
                    >
                        <span className="inline-flex items-center gap-2">
                            {props.isConnected ? <WifiOff size={16} /> : <Wifi size={16} />}
                            {props.isConnected ? 'Disconnect WebSocket' : 'Connect WebSocket'}
                        </span>
                    </button>
                    {props.wsError && (
                        <div className="rounded-md border border-[var(--danger)]/40 bg-[var(--danger)]/10 px-3 py-2 text-xs text-red-200">
                            {props.wsError}
                        </div>
                    )}
                    <SourceSelector
                        isConnected={props.isConnected}
                        getSources={props.getSources}
                        subscribeSource={props.subscribeSource}
                        unsubscribeSource={props.unsubscribeSource}
                        startGng={props.startGng}
                        stopGng={props.stopGng}
                        getGngStatus={props.getGngStatus}
                        listGngConfigs={props.listGngConfigs}
                        onSourceToggled={props.onSourceToggled}
                    />
                </div>
            </CollapsibleSection>

            <CollapsibleSection title="Scene Layers" icon={<Layers size={16} />} defaultOpen={true}>
                <div className="surface-muted space-y-2 px-3 pt-2.5 pb-5">
                    <div className="max-h-[28rem] space-y-2 overflow-y-auto pr-1 scrollbar-thin">
                        {props.isEditMode && (
                            <div className="rounded-md border border-amber-400/40 bg-amber-500/10 px-2 py-1.5 text-[11px] text-amber-200">
                                Edited Mode active: layer switching and layer actions are locked.
                            </div>
                        )}
                        {props.pointClouds.map((pc) => (
                            <LayerItem
                                key={pc.id}
                                id={pc.id}
                                type="cloud"
                                visible={pc.visible !== false}
                                isSelected={props.selectedLayerId === pc.id}
                                isActionDisabled={isLayerActionDisabled}
                                onSelect={() => !isLayerActionDisabled && props.onSelectLayer(pc.id)}
                                onToggleVisibility={() => props.onToggleVisibility(pc.id)}
                                onRemove={() => props.onRemoveLayer(pc.id)}
                                onOpenTransform={() => props.onOpenTransform('cloud', pc.id, pc.name)}
                            >
                                <p className="text-[11px] text-[var(--text-secondary)]">{pc.count.toLocaleString()} pts</p>
                            </LayerItem>
                        ))}

                        {Object.entries(props.graphData).map(([tag, data]) => (
                            <GngLayerControls
                                key={tag}
                                tag={tag}
                                graphData={data}
                                settings={props.layerSettings[tag] || {
                                    visible: true,
                                    showNodes: true,
                                    showEdges: data.mode !== 'static',
                                    showClusters: false,
                                    visibleLabels: {
                                        0: true,
                                        1: true,
                                        2: true,
                                        3: true,
                                        4: true,
                                        5: true,
                                    },
                                    showNormals: false,
                                    showVelocity: false,
                                    showCovarianceEllipsoids: false,
                                    showManipulabilityEllipsoids: false,
                                    manipEllipsoidMode: 'all',
                                    opacity: 1.0,
                                    normalColor: '#00ffff',
                                    velocityColor: '#ffb347',
                                    covarianceEllipsoidColor: '#aefeff',
                                    normalScale: 0.075,
                                    velocityScale: 0.25,
                                    covarianceEllipsoidScale: 2.0,
                                    emissiveIntensity: data.mode === 'static' ? 0.10 : 0.14,
                                }}
                                onUpdate={(updates) => props.onUpdateLayerSettings(tag, updates)}
                            onRemove={() => props.onRemoveGngLayer(tag)}
                            hasTf={!!(data.frameId && data.frameId !== 'world' && props.transforms[data.frameId])}
                            onOpenTransform={() => props.onOpenTransform('layer', tag, `Graph: ${tag}`)}
                            onOpenLabelSettings={() => setLabelContext({ tag, title: `Graph labels: ${tag}` })}
                            onOpenColorSettings={() => props.onOpenColorSettings('graph', tag, `Graph colors: ${tag}`)}
                            nodeColorPreview={props.layerSettings[tag]?.nodeColor || (data.mode === 'static' ? '#1f8f3a' : '#7c8c66')}
                            edgeColorPreview={props.layerSettings[tag]?.edgeColor || '#08d408'}
                        />
                        ))}

                        {/* Unified Entity Layers */}
                        {[
                            { type: 'robot', data: props.robotData, settings: props.robotSettings, label: 'Source ID', hasTf: true, 
                              extra: (tag: string, s: any) => {
                                return (
                                    <div className="mt-2 space-y-2">
                                        <div className="grid grid-cols-[1.2fr_1.2fr_1.4fr_1.0fr] gap-1">
                                            <button onClick={() => props.onUpdateSettings('robot', tag, { showVisual: !s.showVisual })} className={`entity-btn px-3 py-1 text-[10px] ${s.showVisual ? 'active-indigo' : ''}`}>
                                                {s.showVisual ? <Eye size={12} /> : <EyeOff size={12} />} Visual
                                            </button>
                                            <button onClick={() => props.onUpdateSettings('robot', tag, { showCollision: !s.showCollision })} className={`entity-btn px-3 py-1 text-[10px] ${s.showCollision ? 'active-orange' : ''}`}>
                                                <Box size={12} /> Collision
                                            </button>
                                            <button onClick={() => props.onUpdateSettings('robot', tag, { showManipulabilityEllipsoid: !s.showManipulabilityEllipsoid })} className={`entity-btn px-3 py-1 text-[10px] ${s.showManipulabilityEllipsoid ? 'active-indigo' : ''}`}>
                                                <Box size={12} /> Manip
                                            </button>
                                            <ColorActionButton
                                                title="Robot color"
                                                swatches={[
                                                    { color: s.color, title: 'Visual color' },
                                                    { color: s.collisionColor, title: 'Collision color' },
                                                ]}
                                                onClick={() => props.onOpenColorSettings('robot', tag, `Robot colors: ${tag}`)}
                                            />
                                            <button
                                                onClick={() => props.onOpenRobotJoints(tag, `Robot joints: ${tag}`)}
                                                className="entity-btn w-full justify-center px-3 py-1 text-[10px]"
                                            >
                                                Joint
                                            </button>
                                        </div>
                                    </div>
                                );
                              }},
                            { type: 'marker', data: props.markerData, settings: props.markerSettings, label: 'Source ID', hasTf: true },
                            { type: 'voxel', data: props.voxelData, settings: props.voxelSettings, label: 'Voxel ID', hasTf: true,
                              extra: (tag: string, _s: any, d: any) => (
                                <div className="relative mt-0.5">
                                        <button
                                            onClick={() => props.onOpenColorSettings('voxel', tag, `Voxel colors: ${tag}`)}
                                            title="Voxel color"
                                            className="entity-btn absolute -right-[2.1rem] -top-[2.05em] inline-flex h-7 w-14 flex-none items-center justify-center px-2 py-1"
                                        >
                                        <span
                                            className="h-3.5 w-3.5 rounded border border-white/20 shadow-[0_0_0_1px_rgba(0,0,0,0.25)_inset]"
                                            style={{ backgroundColor: _s.color || '#00ff88' }}
                                            aria-hidden="true"
                                        />
                                    </button>
                                    <div className="text-[10px] leading-none text-[var(--text-secondary)]">
                                        Resolution: <span className="text-[var(--text-primary)]">{Math.round(d.layout?.voxelSize * 1000) / 1000}m</span>
                                        <span className="mx-1 opacity-50">|</span>
                                        Voxels: <span className="text-[var(--text-primary)]">{d.data?.length ?? 0}</span>
                                    </div>
                                </div>
                              )}
                        ].map(({ type, data, settings, label, hasTf, extra }) => 
                            Object.entries(data).map(([tag, d]: [string, any]) => {
                                const s = settings[tag] || {};
                                return (
                                    <LayerItem key={`${type}-${tag}`} id={tag} displayName={d.displayName} type={type as any} visible={s.visible !== false}
                                        onToggleVisibility={() => {
                                            const isVisible = s.visible !== false;
                                            props.onUpdateSettings(type as EntityType, tag, { visible: !isVisible });
                                        }}
                                        onRemove={() => props.onRemoveEntity(type as EntityType, tag)}
                                        onOpenTransform={() => {
                                            console.log(`[Sidebar] Opening transform for ${type}: ${tag}`);
                                            props.onOpenTransform(type as any, tag, `${type}: ${tag}`);
                                        }}
                                    >
                                        <div className="mt-[1px] flex items-center gap-1 text-[10px] leading-none text-[var(--text-secondary)]">
                                            <span className="opacity-70">{label}:</span>
                                            <span className="font-mono opacity-70">{tag}</span>
                                        </div>
                                        {hasTf && (
                                            <div className="mt-[1px] flex items-center gap-1 text-[10px] leading-none text-[var(--text-secondary)]">
                                                <span>Frame:</span>
                                                <span className="flex items-center gap-1">
                                                    <span className={`tf-dot ${d.frameId && d.frameId !== 'world' && props.transforms[d.frameId] ? 'active' : 'inactive'}`} />
                                                    <span className="font-mono opacity-70">{d.frameId || 'world'}</span>
                                                </span>
                                            </div>
                                        )}
                                {extra?.(tag, s, d)}
                            </LayerItem>
                        );
                    })
                        )}
                    </div>

                    {props.pointClouds.length === 0 && !hasGngLayer && (
                        <p className="py-4 text-center text-xs italic text-[var(--text-secondary)]">No layers available.</p>
                    )}
                </div>
            </CollapsibleSection>

            <CollapsibleSection title="Server Files" icon={<Server size={16} />} defaultOpen={false}>
                <ServerFileBrowser
                    isConnected={props.isConnected}
                    listPointCloudFiles={props.listPointCloudFiles}
                    loadPointCloudFile={props.loadPointCloudFile}
                    startContinuousPublish={props.startContinuousPublish}
                    stopContinuousPublish={props.stopContinuousPublish}
                    getContinuousPublishStatus={props.getContinuousPublishStatus}
                />
            </CollapsibleSection>

            <CollapsibleSection title="Rosbag Playback" icon={<PlayCircle size={16} />} defaultOpen={false}>
                <RosbagPlayer
                    isConnected={props.isConnected}
                    listRosbags={props.listRosbags}
                    playRosbag={props.playRosbag}
                    stopRosbag={props.stopRosbag}
                    getRosbagStatus={props.getRosbagStatus}
                />
            </CollapsibleSection>

            <CollapsibleSection title="Export" icon={<UploadCloud size={16} />} defaultOpen={false}>
                <ExportPanel pointClouds={props.pointClouds} selectedLayerId={props.selectedLayerId} />
            </CollapsibleSection>


        </div>
    );

    const displayTab = (
        <div className="space-y-3">
            <CollapsibleSection title="Rendering" icon={<Gauge size={16} />} defaultOpen={true}>
                <div className="surface-muted space-y-4 p-3">
                    <ControlSlider
                        label="Point Size"
                        value={props.heatmapSettings.pointSize}
                        min={0.01}
                        max={0.2}
                        step={0.01}
                        onChange={(val) => props.setHeatmapSettings({ ...props.heatmapSettings, pointSize: val })}
                    />
                    <ControlSlider
                        label="Opacity"
                        value={props.pointCloudOpacity}
                        min={0}
                        max={1}
                        step={0.05}
                        onChange={props.setPointCloudOpacity}
                        formatValue={(v) => `${Math.round(v * 100)}%`}
                    />
                </div>
            </CollapsibleSection>

            <CollapsibleSection title="Heatmap" icon={<Eye size={16} />} defaultOpen={false}>
                <HeatmapControls
                    settings={props.heatmapSettings}
                    onSettingsChange={props.setHeatmapSettings}
                    bounds={props.bounds}
                />
            </CollapsibleSection>
        </div>
    );

    const editTab = (
        <div className="space-y-3">
            {props.selectedCloud ? (
                <div className="surface-soft p-3">
                    <p className="panel-title mb-1">Target Layer</p>
                    <p className="truncate text-sm font-semibold text-[var(--text-primary)]">{props.selectedCloud.name}</p>
                    <div className="mt-3 grid grid-cols-2 gap-2">
                        <button
                            onClick={props.onStartEdit}
                            disabled={!props.canStartEdit}
                            className="btn-primary px-3 py-2 text-sm disabled:cursor-not-allowed disabled:opacity-45"
                            title={props.startEditDisabledReason}
                        >
                            Start Edit
                        </button>
                        <button
                            onClick={props.onCancelEdit}
                            disabled={!props.isEditMode}
                            className="btn-secondary px-3 py-2 text-sm disabled:cursor-not-allowed disabled:opacity-45"
                        >
                            Cancel Edit
                        </button>
                    </div>
                    {!props.canStartEdit && !props.isEditMode && props.startEditDisabledReason && (
                        <p className="mt-2 text-xs text-[var(--text-secondary)]">{props.startEditDisabledReason}</p>
                    )}
                </div>
            ) : null}

            {props.selectedCloud && props.isEditMode && (
                <CollapsibleSection title="Transform" icon={<Move size={16} />} defaultOpen={true}>
                <GenericTransformPanel
                    title="Active Transform"
                    transform={props.selectedCloud ? {
                        position: props.selectedCloud.position || [0, 0, 0],
                        rotation: props.selectedCloud.rotation || [0, 0, 0],
                        scale: props.selectedCloud.scale || [1, 1, 1]
                    } : { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] }}
                    onUpdate={(updates) => {
                        if (!props.selectedCloud) return;
                        props.onUpdateTransform(props.selectedCloud.id, updates);
                    }}
                    onReset={() => {
                        if (!props.selectedCloud) return;
                        props.onUpdateTransform(props.selectedCloud.id, {
                            position: [0, 0, 0],
                            rotation: [0, 0, 0],
                            scale: [1, 1, 1]
                        });
                    }}
                />
                </CollapsibleSection>
            )}

            <CollapsibleSection title="Point Editing" icon={<Trash2 size={16} />} defaultOpen={false}>
                <div className="surface-muted space-y-4 p-3">
                    {props.isEditMode && (
                        <div className="space-y-3">
                            <div className="grid grid-cols-2 gap-2">
                                <button
                                    onClick={() => props.setRegionGizmoMode('translate')}
                                    className={`rounded-md px-3 py-2 text-xs font-semibold ${props.regionGizmoMode === 'translate'
                                        ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)] ring-1 ring-[var(--accent-color)]/40'
                                        : 'btn-secondary'
                                        }`}
                                >
                                    Region Move
                                </button>
                                <button
                                    onClick={() => props.setRegionGizmoMode('scale')}
                                    className={`rounded-md px-3 py-2 text-xs font-semibold ${props.regionGizmoMode === 'scale'
                                        ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)] ring-1 ring-[var(--accent-color)]/40'
                                        : 'btn-secondary'
                                        }`}
                                >
                                    Region Scale
                                </button>
                            </div>

                            <div className="rounded-md border border-white/10 bg-white/5 px-3 py-2 text-xs text-[var(--text-secondary)]">
                                <div className="mb-1 font-semibold text-[var(--text-primary)]">Draft Region (map)</div>
                                <div>
                                    center: [{props.draftRegion.center.map((v) => v.toFixed(2)).join(', ')}]
                                </div>
                                <div>
                                    size: [{props.draftRegion.size.map((v) => v.toFixed(2)).join(', ')}]
                                </div>
                            </div>

                            <CollapsibleSection title="TF Calibration (Real-time)" icon={<Move size={16} />} defaultOpen={true}>
                                <TfCalibrationPanel
                                    isConnected={props.isConnected}
                                    getParameters={props.getParameters}
                                    setParameter={props.setParameter}
                                />
                            </CollapsibleSection>

                            <button
                                onClick={props.onAddRegion}
                                disabled={props.editJobStatus?.isRunning}
                                className="btn-secondary w-full px-4 py-2 text-sm font-semibold disabled:cursor-not-allowed disabled:opacity-45"
                            >
                                Add AABB Region
                            </button>

                            <div className="space-y-2">
                                <div className="text-xs text-[var(--text-secondary)]">
                                    Regions: <span className="font-semibold text-[var(--text-primary)]">{props.editRegions.length}</span>
                                </div>
                                <div className="max-h-40 space-y-1 overflow-y-auto pr-1 scrollbar-thin">
                                    {props.editRegions.map((region) => (
                                        <div key={region.regionId} className="rounded border border-white/10 bg-black/25 p-2 text-[11px]">
                                            <div className="mb-1 font-mono text-[var(--text-primary)]">{region.regionId}</div>
                                            <div className="text-[var(--text-secondary)]">
                                                min [{region.min.map((v) => v.toFixed(2)).join(', ')}]
                                            </div>
                                            <div className="text-[var(--text-secondary)]">
                                                max [{region.max.map((v) => v.toFixed(2)).join(', ')}]
                                            </div>
                                            <button
                                                onClick={() => props.onRemoveRegion(region.regionId)}
                                                disabled={props.editJobStatus?.isRunning}
                                                className="mt-2 btn-danger px-2 py-1 text-[10px] disabled:opacity-45"
                                            >
                                                Remove
                                            </button>
                                        </div>
                                    ))}
                                    {props.editRegions.length === 0 && (
                                        <div className="text-xs italic text-[var(--text-secondary)]">No regions yet.</div>
                                    )}
                                </div>
                                <button
                                    onClick={props.onClearRegions}
                                    disabled={props.editRegions.length === 0 || props.editJobStatus?.isRunning}
                                    className="btn-secondary w-full px-3 py-2 text-xs disabled:cursor-not-allowed disabled:opacity-45"
                                >
                                    Clear Regions
                                </button>
                            </div>

                            {props.editJobStatus && (
                                <div className={`rounded-md border px-3 py-2 text-xs ${props.editJobStatus.error
                                    ? 'border-red-500/40 bg-red-500/10 text-red-200'
                                    : props.editJobStatus.isRunning
                                        ? 'border-sky-400/40 bg-sky-500/10 text-sky-200'
                                        : 'border-green-500/40 bg-green-500/10 text-green-200'
                                    }`}>
                                    <div className="font-semibold">
                                        Job {props.editJobStatus.jobId}
                                    </div>
                                    <div>Stage: {props.editJobStatus.stage}</div>
                                    <div>Progress: {props.editJobStatus.progress}%</div>
                                    {props.editJobStatus.error && (
                                        <div className="mt-1">{props.editJobStatus.error}</div>
                                    )}
                                </div>
                            )}
                        </div>
                    )}
                    <button
                        onClick={props.onPublishEdited}
                        disabled={!props.selectedCloud || !props.isEditMode || Boolean(props.editJobStatus?.isRunning)}
                        className="btn-primary w-full px-4 py-2 text-sm disabled:cursor-not-allowed disabled:opacity-45"
                        title="Commit edit regions and publish edited point cloud to {source}/edited"
                    >
                        <span className="inline-flex items-center gap-2">
                            <UploadCloud size={15} />
                            Commit & Publish Edited Cloud
                        </span>
                    </button>
                </div>
            </CollapsibleSection>

            <CollapsibleSection title="Clipping Planes" icon={<Scissors size={16} />} defaultOpen={true} headerClassName="py-4">
                <ClippingControls
                    planes={props.clipping.planes}
                    onAddPlane={props.clipping.addPlane}
                    onUpdatePlane={props.clipping.updatePlane}
                    onRemovePlane={props.clipping.removePlane}
                    onRemoveAll={props.clipping.removeAll}
                    bounds={props.bounds ? {
                        minX: props.bounds.minX,
                        maxX: props.bounds.maxX,
                        minY: props.bounds.minY,
                        maxY: props.bounds.maxY,
                        minZ: props.bounds.minZ,
                        maxZ: props.bounds.maxZ,
                    } : undefined}
                />
            </CollapsibleSection>
        </div>
    );

    const analysisTab = (
        <div className="space-y-3">
            <div className="surface-soft p-3 text-xs text-[var(--text-secondary)]">
                <div className="mb-1 flex items-center gap-2 text-[var(--text-primary)]">
                    <Activity size={14} />
                    Zone Monitor
                </div>
                Monitor cluster labels inside a custom polygon and height band.
            </div>
            <CollapsibleSection title="Zone Monitoring" icon={<Activity size={16} />} defaultOpen={true}>
                <ZoneMonitorPanel
                    isDrawing={props.zoneMonitor.isDrawing}
                    hasPoints={props.zoneMonitor.points.length > 0}
                    counts={props.zoneCounts}
                    onStartDrawing={props.zoneMonitor.startDrawing}
                    onFinishDrawing={props.zoneMonitor.finishDrawing}
                    onClearZone={props.zoneMonitor.clearZone}
                    zRange={props.zoneMonitor.zRange}
                    onZRangeChange={props.zoneMonitor.setZRange}
                />
            </CollapsibleSection>
            <CollapsibleSection title="Downsampling" icon={<Database size={16} />} defaultOpen={false}>
                <GngDownsamplingPanel
                    isConnected={props.isConnected}
                    getGngStatus={props.getGngStatus}
                    getParameters={props.getParameters}
                    setParameter={props.setParameter}
                />
            </CollapsibleSection>
        </div>
    );

    return (
        <>
            <Tabs
                tabs={[
                    { id: 'layers', label: 'Data', icon: <Layers size={14} />, content: layersTab },
                    { id: 'display', label: 'View', icon: <Eye size={14} />, content: displayTab },
                    { id: 'edit', label: 'Edit', icon: <Move size={14} />, content: editTab },
                    { id: 'analysis', label: 'Analyze', icon: <Activity size={14} />, content: analysisTab },
                ]}
            />

            {labelContext && props.graphData[labelContext.tag] && props.layerSettings[labelContext.tag] && (
                <GngLabelModal
                    open={true}
                    title={labelContext.title}
                    visibleSemanticLabels={props.layerSettings[labelContext.tag].visibleSemanticLabels || {
                        handle: true,
                    }}
                    visibleLabels={props.layerSettings[labelContext.tag].visibleLabels || {
                        0: true,
                        1: true,
                        2: true,
                        3: true,
                        4: true,
                        5: true,
                    }}
                    onClose={() => setLabelContext(null)}
                    onUpdate={(updates) => props.onUpdateLayerSettings(labelContext.tag, updates)}
                />
            )}
        </>
    );
};
