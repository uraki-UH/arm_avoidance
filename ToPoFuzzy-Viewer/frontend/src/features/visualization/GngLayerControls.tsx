import React from 'react';
import { Eye, EyeOff, Trash2, Box, Share2, Square } from 'lucide-react';
import { GraphData, LayerSettings, LAYER_COLORS, LAYER_LABELS } from '../../types';

interface GngLayerControlsProps {
    tag: string;
    graphData: GraphData;
    settings: LayerSettings;
    onUpdate: (updates: Partial<LayerSettings>) => void;
    onRemove: () => void;
}

const LABEL_CONFIG = [
    { label: 1, name: 'Safe', color: '#00FF00' },
    { label: 2, name: 'Collision', color: '#FF0000' },
    { label: 3, name: 'Danger', color: '#FFFF00' },
    { label: 4, name: 'Unused', color: '#808080' },
    { label: 5, name: 'Other', color: '#808080' },
] as const;

export function GngLayerControls({
    tag,
    graphData,
    settings,
    onUpdate,
    onRemove
}: GngLayerControlsProps) {
    const isStatic = tag.toLowerCase().includes('static') || tag.toLowerCase().includes('map');

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
                                {isStatic ? 'Static' : 'Stream'}
                            </span>
                            <div className="truncate text-sm font-semibold text-[var(--text-primary)]">
                                {tag === 'default' ? 'GNG Topology' : tag}
                            </div>
                        </div>
                        <div className="text-[10px] text-[var(--text-secondary)]">
                            {graphData.nodes.length} nodes • {Math.floor(graphData.edges.length / 2)} edges
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

                    {/* Opacity Slider */}
                    <div className="space-y-1">
                        <div className="flex items-center justify-between">
                            <label className="text-[10px] font-semibold uppercase tracking-wider text-[var(--text-secondary)]">
                                Opacity
                            </label>
                            <span className="text-[10px] font-mono text-[var(--accent-strong)]">
                                {Math.round(settings.opacity * 100)}%
                            </span>
                        </div>
                        <input
                            type="range"
                            min="0"
                            max="1"
                            step="0.05"
                            value={settings.opacity}
                            onChange={(e) => onUpdate({ opacity: parseFloat(e.target.value) })}
                            className="w-full accent-[var(--accent-color)]"
                        />
                    </div>

                    {/* Clusters Toggle (Collapsible-like) */}
                    <div className="space-y-2 rounded-md border border-white/10 bg-black/20 p-2">
                        <CompactToggle
                            icon={<Box size={12} />}
                            label="Clusters"
                            isOn={settings.showClusters}
                            onToggle={() => onUpdate({ showClusters: !settings.showClusters })}
                        />
                        
                        {settings.showClusters && (
                            <div className="mt-2 grid grid-cols-5 gap-1 pt-1 border-t border-white/5">
                                {LABEL_CONFIG.map(({ label, name, color }) => {
                                    return (
                                        <div 
                                            key={label}
                                            className="h-1.5 rounded-full"
                                            style={{ 
                                                backgroundColor: color, 
                                                boxShadow: `0 0 4px ${color}44`,
                                                opacity: 0.8
                                            }}
                                            title={name}
                                        />
                                    );
                                })}
                            </div>
                        )}
                    </div>
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
