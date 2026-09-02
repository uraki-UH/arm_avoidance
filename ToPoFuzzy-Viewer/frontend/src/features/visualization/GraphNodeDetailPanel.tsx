import { memo, useMemo, useState, useEffect, useRef } from 'react';
import { X } from 'lucide-react';
import { GraphData, GraphNode, LAYER_LABELS, SEMANTIC_LABELS } from '../../types';

export interface GraphNodeDetailSnapshot {
    graphTag: string;
    graph: GraphData;
    node: GraphNode;
}

interface GraphNodeDetailPanelProps {
    snapshot: GraphNodeDetailSnapshot;
    onClose: () => void;
}

function GraphNodeDetailPanelInner({ snapshot, onClose }: GraphNodeDetailPanelProps) {
    const { graphTag, graph, node } = snapshot;
    const [position, setPosition] = useState({ x: window.innerWidth - 520, y: 40 });
    const [isDragging, setIsDragging] = useState(false);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    const initialPosRef = useRef<{ x: number, y: number } | null>(null);

    useEffect(() => {
        const handleMouseMove = (e: MouseEvent) => {
            if (isDragging && dragStartRef.current && initialPosRef.current) {
                const dx = e.clientX - dragStartRef.current.x;
                const dy = e.clientY - dragStartRef.current.y;
                setPosition({
                    x: initialPosRef.current.x + dx,
                    y: initialPosRef.current.y + dy,
                });
            }
        };

        const handleMouseUp = () => setIsDragging(false);

        if (isDragging) {
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
        }

        return () => {
            window.removeEventListener('mousemove', handleMouseMove);
            window.removeEventListener('mouseup', handleMouseUp);
        };
    }, [isDragging]);

    const handleMouseDown = (e: React.MouseEvent) => {
        setIsDragging(true);
        dragStartRef.current = { x: e.clientX, y: e.clientY };
        initialPosRef.current = { x: position.x, y: position.y };
    };

    const layerLabel = useMemo(() => {
        const rawLabel = Number.isFinite(node.label) ? Math.trunc(node.label) : 0;
        return LAYER_LABELS[rawLabel] || `label ${rawLabel}`;
    }, [node.label]);

    const semanticLabel = useMemo(() => {
        if (!Number.isFinite(node.semanticLabel) || (node.semanticLabel ?? 0) <= 0) return '';
        return SEMANTIC_LABELS[(Math.trunc(node.semanticLabel as number) - 1) % SEMANTIC_LABELS.length] || 'HANDLE';
    }, [node.semanticLabel]);

    const nonplaneComponentLabel = useMemo(() => {
        if (node.nonplaneComponentId === undefined || node.nonplaneComponentId === 4294967295) {
            return 'none';
        }
        return String(node.nonplaneComponentId);
    }, [node.nonplaneComponentId]);

    const hasManipulabilityData = useMemo(() => (
        node.manipValid !== undefined ||
        node.manipValue !== undefined ||
        node.manipConditionNumber !== undefined ||
        node.manipScale !== undefined ||
        node.manipOrientation !== undefined
    ), [node.manipConditionNumber, node.manipOrientation, node.manipScale, node.manipValid, node.manipValue]);

    return (
        <div
            className="surface-panel absolute z-50 flex w-[420px] flex-col overflow-hidden"
            style={{
                left: position.x,
                top: position.y,
                cursor: isDragging ? 'grabbing' : 'auto',
                minWidth: 320,
                maxWidth: '90vw',
                maxHeight: '90vh',
            }}
        >
            <div
                className="flex shrink-0 cursor-grab items-center justify-between border-b border-white/10 bg-black/25 p-2 active:cursor-grabbing"
                onMouseDown={handleMouseDown}
            >
                <div className="min-w-0">
                    <h3 className="truncate text-sm font-bold text-[var(--text-primary)]">
                        Manip node details
                    </h3>
                    <p className="truncate text-[10px] font-mono text-[var(--text-secondary)]">
                        {graphTag} / node {node.id ?? 'n/a'}
                    </p>
                </div>
                <button
                    onClick={onClose}
                    className="btn-secondary inline-flex h-7 w-7 items-center justify-center p-0 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                >
                    <X size={16} />
                </button>
            </div>

            <div className="space-y-2 overflow-y-auto p-3 text-xs text-[var(--text-primary)]">
                <div className="grid grid-cols-2 gap-2">
                    <div className="rounded-md border border-white/10 bg-black/20 p-2">
                        <div className="text-[10px] uppercase tracking-wider text-[var(--text-secondary)]">Layer</div>
                        <div className="mt-1 font-semibold">{layerLabel}{semanticLabel ? ` / ${semanticLabel}` : ''}</div>
                    </div>
                    <div className="rounded-md border border-white/10 bg-black/20 p-2">
                        <div className="text-[10px] uppercase tracking-wider text-[var(--text-secondary)]">Goal</div>
                        <div className="mt-1 font-semibold">{node.isGoal ? 'yes' : 'no'}</div>
                    </div>
                    <div className="rounded-md border border-white/10 bg-black/20 p-2">
                        <div className="text-[10px] uppercase tracking-wider text-[var(--text-secondary)]">Non-plane component</div>
                        <div className="mt-1 font-mono">{nonplaneComponentLabel}</div>
                    </div>
                    {hasManipulabilityData && (
                        <>
                            <div className="rounded-md border border-white/10 bg-black/20 p-2">
                                <div className="text-[10px] uppercase tracking-wider text-[var(--text-secondary)]">Manip</div>
                                <div className="mt-1 font-mono">{node.manipValue?.toFixed(4) ?? 'n/a'}</div>
                            </div>
                            <div className="rounded-md border border-white/10 bg-black/20 p-2">
                                <div className="text-[10px] uppercase tracking-wider text-[var(--text-secondary)]">Cond</div>
                                <div className="mt-1 font-mono">{node.manipConditionNumber?.toFixed(4) ?? 'n/a'}</div>
                            </div>
                        </>
                    )}
                </div>

                <div className="rounded-md border border-white/10 bg-black/20 p-2 font-mono text-[10px] leading-5 text-[var(--text-secondary)]">
                    <div>pos: [{node.x.toFixed(4)}, {node.y.toFixed(4)}, {node.z.toFixed(4)}]</div>
                    <div>normal: [{node.nx.toFixed(4)}, {node.ny.toFixed(4)}, {node.nz.toFixed(4)}]</div>
                    <div>age: {node.age}</div>
                    <div>timestamp: {graph.timestamp}</div>
                </div>

                {hasManipulabilityData && (
                    <div className="rounded-md border border-white/10 bg-black/20 p-2 font-mono text-[10px] leading-5 text-[var(--text-secondary)]">
                        <div>manip scale: [{node.manipScale?.map((v) => v.toFixed(4)).join(', ') ?? 'n/a'}]</div>
                        {node.rotationalManipValid && (
                            <div>rotational scale: [{node.rotationalManipScale?.map((v) => v.toFixed(4)).join(', ') ?? 'n/a'}]</div>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
}

export const GraphNodeDetailPanel = memo(GraphNodeDetailPanelInner);
export default GraphNodeDetailPanel;
