import React, { useState, useEffect } from 'react';
import { Eye, EyeOff, Trash2, Share2, Square } from 'lucide-react';
import { GraphData, LayerSettings } from '../../types';

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
    showOpacity?: boolean;
    hasTf?: boolean;
}

export function GngLayerControls({
    tag,
    graphData,
    settings,
    onUpdate,
    onRemove,
    showOpacity = false,
    hasTf = false,
}: GngLayerControlsProps) {
    const isStatic = graphData.mode === 'static';
    const displayTag = graphData.tag || tag;

    // Local buffered opacity to avoid firing frequent onUpdate during drag.
    const [localOpacity, setLocalOpacity] = useState<number>(settings.opacity ?? 1);

    useEffect(() => {
        setLocalOpacity(settings.opacity ?? 1);
    }, [settings.opacity]);

    return (
        <div className="surface-muted border-l-2 border-[var(--accent-color)]/30 p-3 transition-colors mb-2">
            <div className="mb-2 flex items-start justify-between gap-2">
                <div className="flex min-w-0 flex-1 items-center gap-2">
                    <button
                        onClick={(e) => {
                            e.stopPropagation();
                            onUpdate({ visible: !settings.visible });
                        }}
                        className={`inline-flex h-7 w-7 items-center justify-center rounded-md border transition-all ${
                            settings.visible 
                            ? 'border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]' 
                            : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'
                        }`}
                        title={settings.visible ? 'Hide layer' : 'Show layer'}
                    >
                        {settings.visible ? <Eye size={14} /> : <EyeOff size={14} />}
                    </button>
                    <div className="min-w-0 flex-1">
                        <div className="flex items-center gap-1.5">
                            <span className="text-[10px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-50">
                                {isStatic ? 'Static' : 'Dynamic'}
                            </span>
                            <div className="truncate text-sm font-semibold text-[var(--text-primary)]">
                                {displayTag === 'default' ? 'GNG Topology' : displayTag}
                            </div>
                        </div>
                        <div className="mt-0.5 flex items-center gap-2 text-[10px] text-[var(--text-secondary)]">
                            <span>{graphData.nodes.length} nodes • {Math.floor(graphData.edges.length / 2)} edges</span>
                            {graphData.frameId && (() => {
                                const isWorld = graphData.frameId === 'world';
                                const dotClass = isWorld
                                    ? 'bg-white/30'
                                    : hasTf
                                        ? 'bg-green-400 shadow-[0_0_4px_#4ade80]'
                                        : 'bg-yellow-400';
                                const dotTitle = isWorld ? 'Fixed world frame' : hasTf ? 'TF active' : 'TF not yet received';
                                return (
                                    <span className="flex items-center gap-1">
                                        <span className={`inline-block h-1.5 w-1.5 rounded-full ${dotClass}`} title={dotTitle} />
                                        <span className="font-mono opacity-70">{graphData.frameId}</span>
                                    </span>
                                );
                            })()}
                        </div>
                    </div>
                </div>
                <button
                    onClick={(e) => {
                        e.stopPropagation();
                        onRemove();
                    }}
                    className="btn-icon btn-icon-danger"
                    title="Remove layer"
                >
                    <Trash2 size={13} />
                </button>
            </div>

            {settings.visible && (
                <div className="mt-3 space-y-3">
                    {/* Main Toggles */}
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

                    {/* Opacity Slider (optional) */}
                    {showOpacity && (
                        <div className="space-y-1">
                            <div className="flex items-center justify-between">
                                <label className="text-[10px] font-semibold uppercase tracking-wider text-[var(--text-secondary)]">
                                    Opacity
                                </label>
                                <span className="text-[10px] font-mono text-[var(--accent-strong)]">
                                    {Math.round(localOpacity * 100)}%
                                </span>
                            </div>
                            <input
                                type="range"
                                min="0"
                                max="1"
                                step="0.01"
                                value={localOpacity}
                                onChange={(e) => setLocalOpacity(parseFloat(e.target.value))}
                                onMouseUp={() => onUpdate({ opacity: localOpacity })}
                                onPointerUp={() => onUpdate({ opacity: localOpacity })}
                                onTouchEnd={() => onUpdate({ opacity: localOpacity })}
                                className="w-full accent-[var(--accent-color)]"
                            />
                        </div>
                    )}

                </div>
            )}
        </div>
    );
}

interface CompactToggleProps {
    icon: React.ReactNode;
    label: string;
    isOn: boolean;
    onToggle: () => void;
}

function CompactToggle({ icon, label, isOn, onToggle }: CompactToggleProps) {
    return (
        <button
            onClick={(e) => {
                e.stopPropagation();
                onToggle();
            }}
            className={`flex items-center justify-between gap-2 rounded-md border px-2 py-1.5 transition-all w-full ${
                isOn 
                ? 'border-[var(--accent-color)]/30 bg-[var(--accent-soft)]/50 text-[var(--text-primary)]' 
                : 'border-white/5 bg-black/20 text-[var(--text-secondary)] opacity-60'
            }`}
        >
            <div className="flex items-center gap-1.5">
                <span className={isOn ? 'text-[var(--accent-strong)]' : ''}>{icon}</span>
                <span className="text-[11px] font-medium">{label}</span>
            </div>
            <div className={`h-1.5 w-1.5 rounded-full ${isOn ? 'bg-[var(--accent-color)] shadow-[0_0_5px_var(--accent-color)]' : 'bg-white/20'}`} />
        </button>
    );
}
