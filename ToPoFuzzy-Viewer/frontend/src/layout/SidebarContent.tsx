import React from 'react';
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
import { TransformPanel } from '../features/manipulation/TransformPanel';
import { PointCloudTransformPanel } from '../features/manipulation/PointCloudTransformPanel';
import { ClippingControls } from '../features/manipulation/ClippingControls';
import { GngLayerControls, type GngLayerState } from '../features/visualization/GngLayerControls';
import { ZoneMonitorPanel } from '../features/analysis/ZoneMonitorPanel';
import { GngDownsamplingPanel } from '../features/analysis/GngDownsamplingPanel';
import { RosbagPlayer } from '../features/io/RosbagPlayer';

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
    onUpdateRobotSettings: (tag: string, updates: Partial<RobotSettings>) => void;
    onRemoveRobot: (tag: string) => void;
    transforms: Record<string, any>;
}

export const SidebarContent: React.FC<SidebarContentProps> = (props) => {
    const selectedLayer = props.pointClouds.find((pc) => pc.id === props.selectedLayerId);
    const visibleLayerCount = props.pointClouds.filter((pc) => pc.visible !== false).length;
    const hasGngLayer = Boolean(props.graphData && !props.gngLayer.removed);
    const isLayerActionDisabled = props.isEditMode;

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
                <div className="surface-muted space-y-2 p-3">
                    <div className="grid grid-cols-2 gap-2 text-xs text-[var(--text-secondary)]">
                        <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                            Visible: <span className="font-semibold text-[var(--text-primary)]">{visibleLayerCount}</span>
                        </div>
                        <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                            Selected: <span className="font-semibold text-[var(--text-primary)]">{selectedLayer ? '1' : '0'}</span>
                        </div>
                    </div>

                    <div className="max-h-64 space-y-2 overflow-y-auto pr-1 scrollbar-thin">
                        {props.isEditMode && (
                            <div className="rounded-md border border-amber-400/40 bg-amber-500/10 px-2 py-1.5 text-[11px] text-amber-200">
                                Edited Mode active: layer switching and layer actions are locked.
                            </div>
                        )}
                        {props.pointClouds.map((pc) => (
                            <div
                                key={pc.id}
                                onClick={() => {
                                    if (!isLayerActionDisabled) {
                                        props.onSelectLayer(pc.id);
                                    }
                                }}
                                className={`rounded-lg border p-2 transition-colors cursor-pointer ${props.selectedLayerId === pc.id
                                    ? 'border-[var(--accent-color)]/70 bg-[var(--accent-soft)]'
                                    : 'border-white/10 bg-white/5 hover:bg-white/10'
                                    } ${isLayerActionDisabled ? 'opacity-70 cursor-not-allowed' : ''}`}
                            >
                                <div className="flex items-start justify-between gap-2">
                                    <div className="min-w-0 flex-1">
                                        <div className="mb-1 flex items-center gap-2">
                                            <button
                                                onClick={(e) => {
                                                    e.stopPropagation();
                                                    if (isLayerActionDisabled) return;
                                                    props.onToggleVisibility(pc.id);
                                                }}
                                                disabled={isLayerActionDisabled}
                                                className="inline-flex h-6 w-6 items-center justify-center rounded-md border border-white/10 bg-black/20 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                                                title={pc.visible !== false ? 'Hide layer' : 'Show layer'}
                                            >
                                                {pc.visible !== false ? <Eye size={14} /> : <EyeOff size={14} />}
                                            </button>
                                            <p className="truncate text-sm font-semibold text-[var(--text-primary)]">{pc.name}</p>
                                        </div>
                                        <p className="text-[11px] text-[var(--text-secondary)]">{pc.count.toLocaleString()} pts</p>
                                    </div>
                                    <button
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            if (isLayerActionDisabled) return;
                                            props.onRemoveLayer(pc.id);
                                        }}
                                        disabled={isLayerActionDisabled}
                                        className="btn-icon btn-icon-danger disabled:cursor-not-allowed disabled:opacity-50"
                                        title="Remove layer"
                                    >
                                        <Trash2 size={13} />
                                    </button>
                                </div>
                            </div>
                        ))}

                        {Object.entries(props.graphData).map(([tag, data]) => (
                            <GngLayerControls
                                key={tag}
                                tag={tag}
                                graphData={data}
                                settings={props.layerSettings[tag] || {
                                    visible: true,
                                    showNodes: true,
                                    showEdges: true,
                                    showClusters: false,
                                    opacity: 1.0
                                }}
                                onUpdate={(updates) => props.onUpdateLayerSettings(tag, updates)}
                                onRemove={() => props.onRemoveGngLayer(tag)}
                                showOpacity={false}
                                hasTf={!!(data.frameId && data.frameId !== 'world' && props.transforms[data.frameId])}
                            />
                        ))}

                        {Object.entries(props.robotData).map(([tag, data]) => (
                            <div
                                key={`robot-${tag}`}
                                className={`rounded-lg border p-2 transition-colors border-white/10 bg-white/5 hover:bg-white/10`}
                            >
                                <div className="flex items-start justify-between gap-2">
                                    <div className="min-w-0 flex-1">
                                        <div className="mb-1 flex items-center gap-2">
                                            <button
                                                onClick={() => props.onUpdateRobotSettings(tag, { visible: !props.robotSettings[tag]?.visible })}
                                                className="inline-flex h-6 w-6 items-center justify-center rounded-md border border-white/10 bg-black/20 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                                            >
                                                {props.robotSettings[tag]?.visible !== false ? <Eye size={14} /> : <EyeOff size={14} />}
                                            </button>
                                            <p className="truncate text-sm font-semibold text-[var(--text-primary)]">Robot: {tag}</p>
                                        </div>
                                        <div className="mt-0.5 flex items-center gap-2 text-[10px] text-[var(--text-secondary)]">
                                            <span>Frame:</span>
                                            <span className="flex items-center gap-1">
                                                <span
                                                    className={`inline-block h-1.5 w-1.5 rounded-full ${data.frameId && data.frameId !== 'world' && props.transforms[data.frameId] ? 'bg-green-400 shadow-[0_0_4px_#4ade80]' : 'bg-yellow-400'}`}
                                                    title={props.transforms[data.frameId] ? 'TF active' : 'TF not yet received'}
                                                />
                                                <span className="font-mono opacity-70">{data.frameId || 'world'}</span>
                                            </span>
                                        </div>
                                        <div className="mt-2 grid grid-cols-2 gap-2">
                                            <button
                                                onClick={() => props.onUpdateRobotSettings(tag, { showVisual: !props.robotSettings[tag]?.showVisual })}
                                                className={`inline-flex h-7 items-center gap-2 rounded-md border px-2 text-[11px] font-medium transition-all ${
                                                    props.robotSettings[tag]?.showVisual
                                                        ? 'border-[var(--accent-color)]/40 bg-[var(--accent-soft)]/60 text-[var(--text-primary)]'
                                                        : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'
                                                }`}
                                                title={props.robotSettings[tag]?.showVisual ? 'Hide visual model' : 'Show visual model'}
                                            >
                                                {props.robotSettings[tag]?.showVisual ? <Eye size={12} /> : <EyeOff size={12} />}
                                                Visual
                                            </button>
                                            <button
                                                onClick={() => props.onUpdateRobotSettings(tag, { showCollision: !props.robotSettings[tag]?.showCollision })}
                                                className={`inline-flex h-7 items-center gap-2 rounded-md border px-2 text-[11px] font-medium transition-all ${
                                                    props.robotSettings[tag]?.showCollision
                                                        ? 'border-[var(--accent-color)]/40 bg-[var(--accent-soft)]/60 text-[var(--text-primary)]'
                                                        : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'
                                                }`}
                                                title={props.robotSettings[tag]?.showCollision ? 'Hide collision model' : 'Show collision model'}
                                            >
                                                <Box size={12} />
                                                Collision
                                            </button>
                                        </div>
                                    </div>
                                    <button
                                        onClick={() => props.onRemoveRobot(tag)}
                                        className="btn-icon btn-icon-danger"
                                    >
                                        <Trash2 size={13} />
                                    </button>
                                </div>
                            </div>
                        ))}
                    </div>

                    {props.pointClouds.length === 0 && !hasGngLayer && (
                        <p className="py-4 text-center text-xs italic text-[var(--text-secondary)]">No layers available.</p>
                    )}
                </div>
            </CollapsibleSection>

            {selectedLayer && !props.isEditMode && (
                <CollapsibleSection title="Selected Cloud Transform" icon={<Move size={16} />} defaultOpen={true}>
                    <div className="surface-muted p-3">
                        <PointCloudTransformPanel
                            cloudData={selectedLayer}
                            onUpdate={(updates) => props.onUpdateTransform(selectedLayer.id, updates)}
                        />
                    </div>
                </CollapsibleSection>
            )}

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
                    <div>
                        <label className="control-label mb-1 block">Point Size: {props.heatmapSettings.pointSize.toFixed(3)}</label>
                        <input
                            type="range"
                            min="0.01"
                            max="0.2"
                            step="0.01"
                            value={props.heatmapSettings.pointSize}
                            onChange={(e) => props.setHeatmapSettings({ ...props.heatmapSettings, pointSize: parseFloat(e.target.value) })}
                            className="w-full"
                        />
                    </div>
                    <div>
                        <label className="control-label mb-1 block">Opacity: {props.pointCloudOpacity.toFixed(2)}</label>
                        <input
                            type="range"
                            min="0"
                            max="1"
                            step="0.05"
                            value={props.pointCloudOpacity}
                            onChange={(e) => props.setPointCloudOpacity(parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                </div>
            </CollapsibleSection>

            {hasGngLayer && (
                <CollapsibleSection title="Topology Display" icon={<Box size={16} />} defaultOpen={false}>
                    <div className="surface-muted space-y-4 p-3">
                        <div>
                            <label className="control-label mb-1 block">Node Size: {props.gngLayer.nodeScale.toFixed(3)}</label>
                            <input
                                type="range"
                                min="0.005"
                                max="0.1"
                                step="0.001"
                                value={props.gngLayer.nodeScale}
                                onChange={(e) => props.setGngLayer((prev: GngLayerState) => ({ ...prev, nodeScale: parseFloat(e.target.value) }))}
                                className="w-full"
                            />
                        </div>
                        <div>
                            <label className="control-label mb-1 block">Edge Width: {props.gngLayer.edgeWidth.toFixed(3)}</label>
                            <input
                                type="range"
                                min="0.001"
                                max="0.05"
                                step="0.001"
                                value={props.gngLayer.edgeWidth}
                                onChange={(e) => props.setGngLayer((prev: GngLayerState) => ({ ...prev, edgeWidth: parseFloat(e.target.value) }))}
                                className="w-full"
                            />
                        </div>
                    </div>
                </CollapsibleSection>
            )}

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
            ) : (
                <div className="surface-soft p-3 text-xs italic text-[var(--text-secondary)]">
                    Select a layer to edit transforms and point selections.
                </div>
            )}

            {props.selectedCloud && props.isEditMode && (
                <CollapsibleSection title="Transform" icon={<Move size={16} />} defaultOpen={true}>
                    <TransformPanel
                        cloudData={props.selectedCloud}
                        onUpdate={(updates) => props.onUpdateTransform(props.selectedCloud!.id, updates)}
                        transformMode={props.transformMode}
                        onModeChange={props.setTransformMode}
                    />
                </CollapsibleSection>
            )}

            <CollapsibleSection title="Point Editing" icon={<Trash2 size={16} />} defaultOpen={false}>
                <div className="surface-muted space-y-4 p-3">
                    {!props.isEditMode && (
                        <div className="rounded-md border border-white/10 bg-white/5 px-3 py-2 text-xs text-[var(--text-secondary)]">
                            Start Edit to enable backend AABB region editing.
                        </div>
                    )}
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

            <CollapsibleSection title="Clipping Planes" icon={<Scissors size={16} />} defaultOpen={false}>
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
        <Tabs
            tabs={[
                { id: 'layers', label: 'Data', icon: <Layers size={14} />, content: layersTab },
                { id: 'display', label: 'View', icon: <Eye size={14} />, content: displayTab },
                { id: 'edit', label: 'Edit', icon: <Move size={14} />, content: editTab },
                { id: 'analysis', label: 'Analyze', icon: <Activity size={14} />, content: analysisTab },
            ]}
        />
    );
};
