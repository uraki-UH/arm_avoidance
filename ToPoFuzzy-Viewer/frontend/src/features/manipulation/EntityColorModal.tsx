import { Palette, X } from 'lucide-react';
import { ControlSlider } from '../../components/ui/SharedControls';
import {
    DYNAMIC_GNG_DEFAULTS,
    GraphData,
    LayerSettings,
    RobotSettings,
    STATIC_GNG_DEFAULTS,
    VoxelSettings,
} from '../../types';

type ColorEntityType = 'robot' | 'voxel' | 'graph';

interface EntityColorModalProps {
    title: string;
    subtitle?: string;
    open: boolean;
    entityType: ColorEntityType;
    settings: RobotSettings | VoxelSettings | LayerSettings | null;
    graphData?: GraphData | null;
    graphSizeDefaults?: {
        nodeScale: number;
        edgeWidth: number;
    };
    onClose: () => void;
    onUpdate: (updates: Record<string, unknown>) => void;
}

export function EntityColorModal({
    title,
    subtitle,
    open,
    entityType,
    settings,
    graphData,
    graphSizeDefaults,
    onClose,
    onUpdate,
}: EntityColorModalProps) {
    if (!open || !settings) return null;

    const isStaticGraph = entityType === 'graph' && graphData?.mode === 'static';
    const graphDefaultColor = isStaticGraph ? STATIC_GNG_DEFAULTS.nodeColor : DYNAMIC_GNG_DEFAULTS.nodeColor;
    const graphDefaultEdgeColor = isStaticGraph ? STATIC_GNG_DEFAULTS.edgeColor : DYNAMIC_GNG_DEFAULTS.edgeColor;
    const graphDefaultOpacity = isStaticGraph ? STATIC_GNG_DEFAULTS.opacity : 1.0;
    const graphDefaultEmissive = isStaticGraph ? STATIC_GNG_DEFAULTS.nodeEmissiveIntensity : DYNAMIC_GNG_DEFAULTS.nodeEmissiveIntensity;

    const robotSettings = settings as RobotSettings;
    const voxelSettings = settings as VoxelSettings;
    const layerSettings = settings as LayerSettings;
    const graphNodeScaleDefault = graphSizeDefaults?.nodeScale ?? 0.01;
    const graphEdgeWidthDefault = graphSizeDefaults?.edgeWidth ?? 0.002;

    return (
        <div className="fixed left-4 top-20 z-[9999] w-[560px] animate-in fade-in slide-in-from-left-2 duration-300">
            <div className="surface-panel flex max-h-[80vh] flex-col overflow-hidden shadow-2xl ring-1 ring-white/10">
                <div className="flex items-center justify-between border-b border-white/5 bg-black/40 px-4 py-3">
                    <div className="min-w-0">
                        <div className="flex items-center gap-2">
                            <Palette size={15} className="text-[var(--accent-strong)]" />
                            <h2 className="truncate text-sm font-bold text-white leading-tight">{title}</h2>
                        </div>
                        {subtitle && (
                            <p className="mt-0.5 truncate text-[10px] font-mono text-gray-400 opacity-70">{subtitle}</p>
                        )}
                    </div>
                    <button
                        onClick={onClose}
                        className="flex h-7 w-7 items-center justify-center rounded-md text-gray-400 hover:bg-white/10 hover:text-white transition-all"
                    >
                        <X size={16} />
                    </button>
                </div>

                <div className="flex-1 overflow-y-auto bg-[#0c141d]/50 p-4">
                    {entityType === 'robot' && (
                        <div className="space-y-3">
                            <div className="grid grid-cols-2 gap-2 rounded-md border border-white/5 bg-black/15 p-2">
                                <div className="flex flex-col gap-1.5">
                                    <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                        Visual Color
                                    </label>
                                    <input
                                        type="color"
                                        value={robotSettings.color || '#87ceeb'}
                                        onChange={(e) => onUpdate({ color: e.target.value })}
                                        onInput={(e) => onUpdate({ color: (e.target as HTMLInputElement).value })}
                                        className="h-8 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                    />
                                </div>
                                <div className="flex flex-col gap-1.5">
                                    <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                        Collision Color
                                    </label>
                                    <input
                                        type="color"
                                        value={robotSettings.collisionColor || '#ff9f1c'}
                                        onChange={(e) => onUpdate({ collisionColor: e.target.value })}
                                        onInput={(e) => onUpdate({ collisionColor: (e.target as HTMLInputElement).value })}
                                        className="h-8 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                    />
                                </div>
                            </div>
                            <ControlSlider
                                label="Opacity"
                                value={robotSettings.opacity ?? 0.8}
                                min={0}
                                max={1}
                                step={0.01}
                                onChange={(v) => onUpdate({ opacity: v })}
                                formatValue={(v) => `${Math.round(v * 100)}%`}
                            />
                            <ControlSlider
                                label="Emissive"
                                value={robotSettings.emissiveIntensity ?? 0.2}
                                min={0}
                                max={1.5}
                                step={0.01}
                                onChange={(v) => onUpdate({ emissiveIntensity: v })}
                                formatValue={(v) => `${v.toFixed(2)}x`}
                            />
                        </div>
                    )}

                    {entityType === 'voxel' && (
                        <div className="space-y-3">
                            <div className="grid grid-cols-2 gap-2 rounded-md border border-white/5 bg-black/15 p-2">
                                <div className="flex flex-col gap-1.5">
                                    <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                        Color
                                    </label>
                                    <input
                                        type="color"
                                        value={voxelSettings.color || '#00ff88'}
                                        onChange={(e) => onUpdate({ color: e.target.value })}
                                        onInput={(e) => onUpdate({ color: (e.target as HTMLInputElement).value })}
                                        className="h-8 w-full cursor-pointer rounded border border-white/ bg-transparent p-0"
                                    />
                                </div>
                            </div>
                            <ControlSlider
                                label="Opacity"
                                value={voxelSettings.opacity ?? 0.5}
                                min={0}
                                max={1}
                                step={0.01}
                                onChange={(v) => onUpdate({ opacity: v })}
                                formatValue={(v) => `${Math.round(v * 100)}%`}
                            />
                            <ControlSlider
                                label="Emissive"
                                value={voxelSettings.emissiveIntensity ?? 0.2}
                                min={0}
                                max={1.5}
                                step={0.01}
                                onChange={(v) => onUpdate({ emissiveIntensity: v })}
                                formatValue={(v) => `${v.toFixed(2)}x`}
                            />
                        </div>
                    )}

                    {entityType === 'graph' && (
                        <div className="space-y-3">
                            <div className="grid grid-cols-2 gap-2 text-[10px] text-[var(--text-secondary)]">
                                <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                                    Mode: <span className="font-semibold text-[var(--text-primary)]">{graphData?.mode === 'static' ? 'Static' : 'Dynamic'}</span>
                                </div>
                                <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                                    Frame: <span className="font-mono font-semibold text-[var(--text-primary)]">{graphData?.frameId || 'world'}</span>
                                </div>
                                <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                                    Nodes: <span className="font-semibold text-[var(--text-primary)]">{graphData?.nodes.length ?? 0}</span>
                                </div>
                                <div className="rounded-md border border-white/10 bg-white/5 px-2 py-1.5">
                                    Edges: <span className="font-semibold text-[var(--text-primary)]">{Math.floor((graphData?.edges.length ?? 0) / 2)}</span>
                                </div>
                            </div>

                            <div className="grid grid-cols-2 gap-2 rounded-md border border-white/5 bg-black/15 p-2">
                                <div className="flex flex-col gap-1.5">
                                    <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                        Node Color
                                    </label>
                                    <input
                                        type="color"
                                        value={layerSettings.nodeColor || graphDefaultColor}
                                        onChange={(e) => onUpdate({ nodeColor: e.target.value })}
                                        onInput={(e) => onUpdate({ nodeColor: (e.target as HTMLInputElement).value })}
                                        className="h-8 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                    />
                                </div>
                                <div className="flex flex-col gap-1.5">
                                    <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                        Edge Color
                                    </label>
                                    <input
                                        type="color"
                                        value={layerSettings.edgeColor || graphDefaultEdgeColor}
                                        onChange={(e) => onUpdate({ edgeColor: e.target.value })}
                                        onInput={(e) => onUpdate({ edgeColor: (e.target as HTMLInputElement).value })}
                                        className="h-8 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                    />
                                </div>
                            </div>
                            <ControlSlider
                                label="Opacity"
                                value={layerSettings.opacity ?? graphDefaultOpacity}
                                min={0}
                                max={1}
                                step={0.01}
                                onChange={(v) => onUpdate({ opacity: v })}
                                formatValue={(v) => `${Math.round(v * 100)}%`}
                            />
                            <ControlSlider
                                label="Emissive"
                                value={layerSettings.emissiveIntensity ?? graphDefaultEmissive}
                                min={0}
                                max={1.5}
                                step={0.01}
                                onChange={(v) => onUpdate({ emissiveIntensity: v })}
                                formatValue={(v) => `${v.toFixed(2)}x`}
                            />
                            <ControlSlider
                                label="Node Size"
                                value={layerSettings.nodeScale ?? graphNodeScaleDefault}
                                min={0.001}
                                max={0.1}
                                step={0.001}
                                onChange={(v) => onUpdate({ nodeScale: v })}
                                formatValue={(v) => `${v.toFixed(3)}m`}
                            />
                            <ControlSlider
                                label="Edge Width"
                                value={layerSettings.edgeWidth ?? graphEdgeWidthDefault}
                                min={0.0005}
                                max={0.05}
                                step={0.0005}
                                onChange={(v) => onUpdate({ edgeWidth: v })}
                                formatValue={(v) => `${v.toFixed(3)}m`}
                            />
                        </div>
                    )}
                </div>
            </div>
        </div>
    );
}
