import { Share2, Square } from 'lucide-react';
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
    onOpenTransform?: () => void;
    onOpenColorSettings?: () => void;
    hasTf?: boolean;
}

import { getStatusLabel, LayerItem, CompactToggle } from '../../components/ui/SharedControls';

export function GngLayerControls({
    tag,
    graphData,
    settings,
    onUpdate,
    onRemove,
    onOpenTransform,
    onOpenColorSettings,
    hasTf = false,
}: GngLayerControlsProps) {
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
                    {onOpenColorSettings && (
                        <button
                            onClick={onOpenColorSettings}
                            className="entity-btn w-full justify-center px-2 py-1.5 text-[11px]"
                        >
                            Color
                        </button>
                    )}
                </div>
            )}
        </div>
    );
}
