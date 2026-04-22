import React from 'react';
import { Eye, EyeOff, Trash2 } from 'lucide-react';
import { GraphData } from '../../types';

export interface GngLayerState {
    visible: boolean;
    removed: boolean;
    showGraph: boolean;
    showEdges: boolean;
    showClusters: boolean;
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
    graphData: GraphData;
    gngLayer: GngLayerState;
    setGngLayer: React.Dispatch<React.SetStateAction<GngLayerState>>;
}

const LABEL_CONFIG = [
    { label: 1, name: 'Safe', color: '#00FF00' },
    { label: 2, name: 'Collision', color: '#FF0000' },
    { label: 3, name: 'Danger', color: '#FFFF00' },
    { label: 4, name: 'Unused', color: '#808080' },
    { label: 5, name: 'Other', color: '#808080' },
] as const;

export function GngLayerControls({
    graphData,
    gngLayer,
    setGngLayer
}: GngLayerControlsProps) {
    return (
        <div className="surface-muted p-3 transition-colors">
            <div className="mb-2 flex items-start justify-between gap-2">
                <div className="flex min-w-0 flex-1 items-center gap-2">
                    <button
                        onClick={(e) => {
                            e.stopPropagation();
                            setGngLayer((prev) => ({ ...prev, visible: !prev.visible }));
                        }}
                        className="btn-secondary inline-flex h-7 w-7 items-center justify-center p-0"
                        title={gngLayer.visible ? 'Hide layer' : 'Show layer'}
                    >
                        {gngLayer.visible ? <Eye size={14} /> : <EyeOff size={14} />}
                    </button>
                    <div className="min-w-0 flex-1">
                        <div className="truncate text-sm font-semibold text-[var(--accent-strong)]">GNG Topology</div>
                        <div className="text-xs text-[var(--text-secondary)]">{graphData.nodes.length} nodes</div>
                    </div>
                </div>
                <button
                    onClick={(e) => {
                        e.stopPropagation();
                        setGngLayer((prev) => ({ ...prev, removed: true }));
                    }}
                    className="btn-icon btn-icon-danger"
                    title="Remove layer"
                >
                    <Trash2 size={13} />
                </button>
            </div>

            {gngLayer.visible && (
                <div className="space-y-2">
                    <ToggleRow
                        label="Nodes"
                        isOn={gngLayer.showGraph}
                        onToggle={() => setGngLayer((prev) => ({ ...prev, showGraph: !prev.showGraph }))}
                    />

                    <ToggleRow
                        label="Edges"
                        isOn={gngLayer.showEdges}
                        onToggle={() => setGngLayer((prev) => ({ ...prev, showEdges: !prev.showEdges }))}
                    />

                    <ToggleRow
                        label="Clusters"
                        isOn={gngLayer.showClusters}
                        onToggle={() => setGngLayer((prev) => ({ ...prev, showClusters: !prev.showClusters }))}
                    />

                    {gngLayer.showClusters && (
                        <div className="space-y-2 rounded-md border border-white/10 bg-black/20 p-2">
                            <ToggleRow
                                label="Cluster Text"
                                isOn={gngLayer.showClusterText}
                                onToggle={() => setGngLayer((prev) => ({ ...prev, showClusterText: !prev.showClusterText }))}
                            />
                            <div className="panel-title mb-1">Filter Labels</div>
                            {LABEL_CONFIG.map(({ label, name, color }) => {
                                const typedLabel = label as 0 | 1 | 2 | 3 | 4 | 5;
                                return (
                                    <button
                                        key={label}
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            setGngLayer((prev) => ({
                                                ...prev,
                                                visibleLabels: {
                                                    ...prev.visibleLabels,
                                                    [label]: !prev.visibleLabels[typedLabel]
                                                }
                                            }));
                                        }}
                                        className={`flex w-full items-center justify-between rounded px-2 py-1 text-xs transition-colors ${!gngLayer.visibleLabels[typedLabel] ? 'opacity-50' : 'hover:bg-white/10'}`}
                                    >
                                        <span className="flex items-center gap-2 text-[var(--text-secondary)]">
                                            <span className="h-2 w-2 rounded-full" style={{ backgroundColor: color, boxShadow: `0 0 6px ${color}` }} />
                                            {name}
                                        </span>
                                        <span className="text-[var(--text-secondary)]">{gngLayer.visibleLabels[typedLabel] ? 'On' : 'Off'}</span>
                                    </button>
                                );
                            })}
                        </div>
                    )}
                </div>
            )}
        </div>
    );
}

interface ToggleRowProps {
    label: string;
    isOn: boolean;
    onToggle: () => void;
}

function ToggleRow({ label, isOn, onToggle }: ToggleRowProps) {
    return (
        <div className="flex items-center justify-between rounded-md border border-white/10 bg-black/20 px-2 py-1.5">
            <span className="text-xs text-[var(--text-secondary)]">{label}</span>
            <button
                onClick={(e) => {
                    e.stopPropagation();
                    onToggle();
                }}
                className={`rounded px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.08em] ${isOn ? 'bg-green-500/20 text-green-300' : 'bg-white/10 text-[var(--text-secondary)]'}`}
            >
                {isOn ? 'On' : 'Off'}
            </button>
        </div>
    );
}
