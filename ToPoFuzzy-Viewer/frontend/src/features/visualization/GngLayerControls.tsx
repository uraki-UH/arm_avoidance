import { Share2, Square } from 'lucide-react';
import { GraphData, LayerSettings } from '../../types';
import { LayerItem, CompactToggle } from '../../components/ui/SharedControls';
import { graphHasManipulabilityData } from './graphLayerSettings';

const getStatusLabel = (mode?: string) => mode === 'static' ? 'Static Graph' : 'Dynamic Graph';

interface GngLayerControlsProps {
    tag: string;
    graphData: GraphData;
    settings: LayerSettings;
    onUpdate: (updates: Partial<LayerSettings>) => void;
    onRemove: () => void;
    onOpenTransform?: () => void;
    onOpenColorSettings?: () => void;
    onOpenLabelSettings?: () => void;
    nodeColorPreview?: string;
    edgeColorPreview?: string;
    hasTf?: boolean;
}

export function GngLayerControls({
    tag,
    graphData,
    settings,
    onUpdate,
    onRemove,
    onOpenTransform,
    onOpenColorSettings,
    onOpenLabelSettings,
    nodeColorPreview,
    edgeColorPreview,
    hasTf = false,
}: GngLayerControlsProps) {
    const visibleLabelCount = settings.visibleLabels
        ? Object.values(settings.visibleLabels).filter(Boolean).length
        : 6;
    const hasManipulabilityData = graphHasManipulabilityData(graphData);

    return (
        <div className="surface-muted border-l-2 border-[var(--accent-color)]/30 p-3 transition-colors mb-2">
            <LayerItem
                id={tag}
                type="graph"
                headerOnly
                visible={settings.visible}
                statusLabel={getStatusLabel(graphData.mode)}
                onToggleVisibility={() => onUpdate({ visible: !settings.visible })}
                onRemove={onRemove}
                onOpenTransform={onOpenTransform}
            >
                <div className="mt-0 flex flex-col gap-0 pl-2 text-[10px] leading-none text-[var(--text-secondary)]">
                    <div className="flex items-center gap-2 whitespace-nowrap font-mono tabular-nums">
                        <span>{graphData.nodes.length} nodes</span>
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
                    <span className="whitespace-nowrap font-mono tabular-nums leading-none">{Math.floor(graphData.edges.length / 2)} edges</span>
                </div>
            </LayerItem>

            {settings.visible && (
                <div className="mt-1">
                    <div className="-ml-1 grid grid-cols-8 gap-1">
                        <CompactToggle
                            icon={<Square size={12} />}
                            label="Nodes"
                            isOn={settings.showNodes}
                            onToggle={() => onUpdate({ showNodes: !settings.showNodes })}
                            className="col-span-3 w-[90%] justify-self-start"
                        />
                        <CompactToggle
                            icon={<Share2 size={12} />}
                            label="Edges"
                            isOn={settings.showEdges}
                            onToggle={() => onUpdate({ showEdges: !settings.showEdges })}
                            className="col-span-3 w-[90%] justify-self-start"
                        />
                        <CompactToggle
                            icon={<Square size={12} />}
                            label="Normals"
                            isOn={settings.showNormals ?? false}
                            onToggle={() => onUpdate({ showNormals: !(settings.showNormals ?? false) })}
                            className="col-span-3 w-[90%] justify-self-start"
                        />
                        <CompactToggle
                            icon={<Square size={12} />}
                            label="Ellipses"
                            isOn={settings.showCovarianceEllipsoids ?? false}
                            onToggle={() => onUpdate({ showCovarianceEllipsoids: !(settings.showCovarianceEllipsoids ?? false) })}
                            className="col-span-3 w-[90%] justify-self-start"
                        />
                        {hasManipulabilityData && (
                            <CompactToggle
                                icon={<Square size={12} />}
                                label="Manip"
                                isOn={settings.showManipulabilityEllipsoids ?? false}
                                onToggle={() => onUpdate({ showManipulabilityEllipsoids: !(settings.showManipulabilityEllipsoids ?? false) })}
                                className="col-span-3 w-[90%] justify-self-start"
                            />
                        )}
                        <CompactToggle
                            icon={<Share2 size={12} />}
                            label="Velocity"
                            isOn={settings.showVelocity ?? false}
                            onToggle={() => onUpdate({ showVelocity: !(settings.showVelocity ?? false) })}
                            className="col-span-3 w-[90%] justify-self-start"
                        />

                        {hasManipulabilityData && settings.showManipulabilityEllipsoids && (
                            <div className="col-span-8 mt-1 flex flex-col gap-1.5 rounded-md border border-white/5 bg-black/20 px-2 py-1.5 text-[10px] text-[var(--text-secondary)]">
                                <div className="flex items-center gap-2">
                                    <span className="font-semibold uppercase tracking-wider min-w-[32px]">Mode</span>
                                    {(['all', 'goal'] as const).map((mode) => (
                                        <button
                                            key={mode}
                                            onClick={() => onUpdate({ manipEllipsoidMode: mode })}
                                            className={`rounded px-2 py-0.5 font-mono ${((settings.manipEllipsoidMode ?? 'all') === mode) ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'bg-white/5 text-[var(--text-secondary)]'}`}
                                        >
                                            {mode}
                                        </button>
                                    ))}
                                </div>
                                <div className="flex items-center gap-2">
                                    <span className="font-semibold uppercase tracking-wider min-w-[32px]">Type</span>
                                    {(['translational', 'rotational', 'both'] as const).map((type) => (
                                        <button
                                            key={type}
                                            onClick={() => onUpdate({ manipEllipsoidType: type })}
                                            className={`rounded px-2 py-0.5 font-mono ${((settings.manipEllipsoidType ?? 'translational') === type) ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'bg-white/5 text-[var(--text-secondary)]'}`}
                                        >
                                            {type}
                                        </button>
                                    ))}
                                </div>
                            </div>
                        )}

                        {onOpenLabelSettings && (
                            <button
                                onClick={onOpenLabelSettings}
                                title="Graph labels"
                                className="entity-btn col-span-4 min-w-0 flex items-center justify-between gap-2 px-2 py-1 text-[10px]"
                            >
                                <span className="truncate">Labels</span>
                                <span className="rounded border border-white/10 bg-black/20 px-1.5 py-0 font-mono text-[10px] text-[var(--accent-strong)]">
                                    {visibleLabelCount}/6
                                </span>
                            </button>
                        )}

                        {onOpenColorSettings && (
                            <button
                                onClick={onOpenColorSettings}
                                title="Graph colors"
                                className="entity-btn col-span-4 min-w-0 flex-col items-stretch justify-center gap-0.5 overflow-hidden px-1 py-0.5 text-[10px] leading-none"
                            >
                                <span className="flex items-center justify-between gap-1 rounded border border-white/10 bg-black/20 px-1.5 py-0">
                                    <span className="text-[9px] font-medium leading-none">Node</span>
                                    <span
                                        className="h-3 w-3 rounded border border-white/20 shadow-[0_0_0_1px_rgba(0,0,0,0.25)_inset]"
                                        style={{ backgroundColor: nodeColorPreview || settings.nodeColor || '#7c8c66' }}
                                        aria-hidden="true"
                                        title="Node color"
                                    />
                                </span>
                                <span className="flex items-center justify-between gap-1 rounded border border-white/10 bg-black/20 px-1.5 py-0">
                                    <span className="text-[9px] font-medium leading-none">Edge</span>
                                    <span
                                        className="h-3 w-3 rounded border border-white/20 shadow-[0_0_0_1px_rgba(0,0,0,0.25)_inset]"
                                        style={{ backgroundColor: edgeColorPreview || settings.edgeColor || '#08d408' }}
                                        aria-hidden="true"
                                        title="Edge color"
                                    />
                                </span>
                            </button>
                        )}
                    </div>
                </div>
            )}
        </div>
    );
}
