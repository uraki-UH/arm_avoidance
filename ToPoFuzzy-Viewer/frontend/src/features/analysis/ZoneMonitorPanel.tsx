import { Play, Square, Trash2, AlertCircle } from 'lucide-react';
import { DualRangeSlider } from '../../components/ui/DualRangeSlider';
import { LAYER_LABELS, LAYER_COLORS } from '../../types';

interface ZoneMonitorPanelProps {
    isDrawing: boolean;
    hasPoints: boolean;
    counts: Map<string, number>;
    onStartDrawing: () => void;
    onFinishDrawing: () => void;
    onClearZone: () => void;
    zRange: [number, number];
    onZRangeChange: (range: [number, number]) => void;
}

export function ZoneMonitorPanel({
    isDrawing,
    hasPoints,
    counts,
    onStartDrawing,
    onFinishDrawing,
    onClearZone,
    zRange,
    onZRangeChange,
}: ZoneMonitorPanelProps) {
    return (
        <div className="space-y-4">
            <div className="flex gap-2">
                {!isDrawing ? (
                    <button
                        onClick={onStartDrawing}
                        className="btn-primary flex flex-1 items-center justify-center gap-2 px-3 py-2 text-xs"
                    >
                        <Play size={12} />
                        {hasPoints ? 'Redraw Zone' : 'Draw Zone'}
                    </button>
                ) : (
                    <button
                        onClick={onFinishDrawing}
                        className="btn-secondary flex flex-1 items-center justify-center gap-2 border-green-400/40 bg-green-500/20 px-3 py-2 text-xs font-semibold text-green-100"
                    >
                        <Square size={12} fill="currentColor" />
                        Finish Drawing
                    </button>
                )}

                <button
                    onClick={onClearZone}
                    disabled={!hasPoints && !isDrawing}
                    className="btn-danger px-3 py-2 disabled:opacity-35"
                    title="Clear Zone"
                >
                    <Trash2 size={14} />
                </button>
            </div>

            {isDrawing && (
                <div className="rounded-md border border-amber-400/30 bg-amber-400/10 p-2 text-[11px] text-amber-200">
                    <div className="flex items-start gap-2">
                        <AlertCircle size={12} className="mt-0.5 shrink-0" />
                        <span>Click on the ground to place polygon points, then press Finish Drawing.</span>
                    </div>
                </div>
            )}

            <div className="surface-muted space-y-2 p-3">
                <h4 className="panel-title">Height Filter (Z)</h4>
                <div className="px-1">
                    <DualRangeSlider
                        min={-5}
                        max={10}
                        step={0.5}
                        value={zRange}
                        onChange={onZRangeChange}
                    />
                </div>
                <div className="flex justify-between font-mono text-[10px] text-[var(--text-secondary)]">
                    <span>{zRange[0].toFixed(1)}m</span>
                    <span>{zRange[1].toFixed(1)}m</span>
                </div>
            </div>

            <div className="surface-muted p-3">
                <h4 className="panel-title mb-2">Detected Objects</h4>

                <div className="space-y-1">
                    {LAYER_LABELS.map((label, index) => {
                        const count = counts.get(label) || 0;
                        const color = LAYER_COLORS[index % LAYER_COLORS.length];

                        return (
                            <div key={label} className="flex items-center justify-between rounded px-2 py-1 hover:bg-white/10">
                                <div className="flex items-center gap-2">
                                    <div
                                        className="h-2 w-2 rounded-full"
                                        style={{ backgroundColor: color, boxShadow: `0 0 6px ${color}` }}
                                    />
                                    <span className="text-xs capitalize text-[var(--text-primary)]">{label}</span>
                                </div>
                                <span className={`font-mono text-xs font-bold ${count > 0 ? 'text-white' : 'text-[var(--text-secondary)]'}`}>
                                    {count}
                                </span>
                            </div>
                        );
                    })}
                </div>

                <div className="mt-2 flex justify-between border-t border-white/10 pt-2">
                    <span className="text-xs font-semibold text-[var(--text-secondary)]">Total</span>
                    <span className="font-mono text-xs font-bold text-white">
                        {Array.from(counts.values()).reduce((a, b) => a + b, 0)}
                    </span>
                </div>
            </div>
        </div>
    );
}
