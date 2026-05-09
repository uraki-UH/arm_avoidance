import { useState, useEffect } from 'react';
import { Share2, Square } from 'lucide-react';
import { GraphData, LayerSettings, STATIC_GNG_DEFAULTS, DYNAMIC_GNG_DEFAULTS } from '../../types';

export interface GngLayerState {
    visible: boolean;
    removed: boolean;
    showGraph: boolean;
    showEdges: boolean;
    showClusterText: boolean;
    showNormals: boolean;
    normalArrowLength: number;
    normalArrowColor: string;
    nodeScale: number;
    edgeWidth: number;
    visibleLabels: {
        0: boolean;
        1: boolean;
        2: boolean;
        3: boolean;
        4: boolean;
        5: boolean;
    };
}

interface GngLayerControlsProps {
    tag: string;
    graphData: GraphData;
    settings: LayerSettings;
    onUpdate: (updates: Partial<LayerSettings>) => void;
    onRemove: () => void;
    onOpenTransform?: () => void;
    showOpacity?: boolean;
    hasTf?: boolean;
}

import { getStatusLabel, LayerItem, CompactToggle, ControlSlider } from '../../components/ui/SharedControls';

export function GngLayerControls({
    tag,
    graphData,
    settings,
    onUpdate,
    onRemove,
    onOpenTransform,
    showOpacity = false,
    hasTf = false,
}: GngLayerControlsProps) {
    const isStatic = graphData.mode === 'static';

    const [localOpacity, setLocalOpacity] = useState<number>(settings.opacity ?? STATIC_GNG_DEFAULTS.opacity);
    const [localNodeColor, setLocalNodeColor] = useState<string>(settings.nodeColor || STATIC_GNG_DEFAULTS.nodeColor);
    const [localEdgeColor, setLocalEdgeColor] = useState<string>(settings.edgeColor || STATIC_GNG_DEFAULTS.edgeColor);

    useEffect(() => {
        setLocalOpacity(settings.opacity ?? STATIC_GNG_DEFAULTS.opacity);
    }, [settings.opacity]);

    useEffect(() => {
        setLocalNodeColor(settings.nodeColor || (isStatic ? STATIC_GNG_DEFAULTS.nodeColor : DYNAMIC_GNG_DEFAULTS.nodeColor));
    }, [settings.nodeColor, isStatic]);

    useEffect(() => {
        setLocalEdgeColor(settings.edgeColor || (isStatic ? STATIC_GNG_DEFAULTS.edgeColor : DYNAMIC_GNG_DEFAULTS.edgeColor));
    }, [settings.edgeColor, isStatic]);

    return (
        <div className="surface-muted border-l-2 border-[var(--accent-color)]/30 p-3 transition-colors mb-2">
            <LayerItem
                id={tag}
                type="graph"
                headerOnly
                visible={settings.visible}
                statusLabel={getStatusLabel('graph', graphData.mode)}
                onToggleVisibility={() => onUpdate({ visible: !settings.visible })}
                onRemove={onRemove}
                onOpenTransform={onOpenTransform}
            >
                <div className="mt-0.5 flex items-center gap-2 text-[10px] text-[var(--text-secondary)]">
                    <span>{graphData.nodes.length} nodes • {Math.floor(graphData.edges.length / 2)} edges</span>
                    {graphData.frameId && (() => {
                        const isWorld = graphData.frameId === 'world';
                        const dotClass = isWorld ? 'bg-white/30' : hasTf ? 'bg-green-400 shadow-[0_0_4px_#4ade80]' : 'bg-yellow-400';
                        const dotTitle = isWorld ? 'Fixed world frame' : hasTf ? 'TF active' : 'TF not yet received';
                        return (
                            <span className="flex items-center gap-1">
                                <span className={`inline-block h-1.5 w-1.5 rounded-full ${dotClass}`} title={dotTitle} />
                                <span className="font-mono opacity-70">{graphData.frameId}</span>
                            </span>
                        );
                    })()}
                </div>
            </LayerItem>

            {settings.visible && (
                <div className="mt-3 space-y-3">
                    <div className="grid grid-cols-2 gap-2">
                        <CompactToggle
                            icon={<Square size={12} />}
                            label="Nodes"
                            isOn={settings.showNodes}
                            onToggle={() => onUpdate({ showNodes: !settings.showNodes })}
                        />
                        <CompactToggle
                            icon={<Share2 size={12} />}
                            label="Edges"
                            isOn={settings.showEdges}
                            onToggle={() => onUpdate({ showEdges: !settings.showEdges })}
                        />
                    </div>

                    <div className="grid grid-cols-2 gap-2 rounded-md border border-white/5 bg-black/15 p-2">
                        <div className="flex flex-col gap-1.5">
                            <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                Node Color
                            </label>
                            <input
                                type="color"
                                value={localNodeColor}
                                onChange={(e) => setLocalNodeColor(e.target.value)}
                                onBlur={() => onUpdate({ nodeColor: localNodeColor })}
                                onPointerUp={() => onUpdate({ nodeColor: localNodeColor })}
                                className="h-7 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                title={isStatic ? 'Static node color' : 'Dynamic node palette base color'}
                            />
                        </div>
                        <div className="flex flex-col gap-1.5">
                            <label className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-70">
                                Edge Color
                            </label>
                            <input
                                type="color"
                                value={localEdgeColor}
                                onChange={(e) => setLocalEdgeColor(e.target.value)}
                                onBlur={() => onUpdate({ edgeColor: localEdgeColor })}
                                onPointerUp={() => onUpdate({ edgeColor: localEdgeColor })}
                                className="h-7 w-full cursor-pointer rounded border border-white/10 bg-transparent p-0"
                                title={isStatic ? 'Static edge color' : 'Dynamic edge color'}
                            />
                        </div>
                    </div>

                    {showOpacity && (
                        <ControlSlider
                            label="Opacity"
                            value={localOpacity}
                            min={0}
                            max={1}
                            step={0.01}
                            onChange={setLocalOpacity}
                            onPointerUp={() => onUpdate({ opacity: localOpacity })}
                            formatValue={(v) => `${Math.round(v * 100)}%`}
                        />
                    )}
                </div>
            )}
        </div>
    );
}
