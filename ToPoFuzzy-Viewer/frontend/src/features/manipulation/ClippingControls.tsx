import { Eye, EyeOff, Trash2 } from 'lucide-react';
import { DualRangeSlider } from '../../components/ui/DualRangeSlider';
import { ClippingPlane, ClippingAxis } from '../../types';

interface ClippingBounds {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
}

interface ClippingControlsProps {
    planes: ClippingPlane[];
    onAddPlane: (axis: ClippingAxis, initialRange?: { min: number; max: number }) => void;
    onUpdatePlane: (id: string, updates: Partial<ClippingPlane>) => void;
    onRemovePlane: (id: string) => void;
    onRemoveAll: () => void;
    bounds?: ClippingBounds;
}

export function ClippingControls({
    planes,
    onAddPlane,
    onUpdatePlane,
    onRemovePlane,
    onRemoveAll,
    bounds,
}: ClippingControlsProps) {
    const getFallbackRange = (axis: ClippingAxis): { min: number; max: number } => {
        let range: { min: number; max: number };
        if (!bounds) return { min: -100, max: 100 };
        switch (axis) {
            case 'x':
                range = { min: bounds.minX, max: bounds.maxX };
                break;
            case 'y':
                range = { min: bounds.minY, max: bounds.maxY };
                break;
            case 'z':
                range = { min: bounds.minZ, max: bounds.maxZ };
                break;
            default:
                return { min: -100, max: 100 };
        }

        if (!Number.isFinite(range.min) || !Number.isFinite(range.max)) {
            return { min: -100, max: 100 };
        }
        if (range.min === range.max) {
            return { min: range.min - 0.01, max: range.max + 0.01 };
        }
        return {
            min: Math.min(range.min, range.max),
            max: Math.max(range.min, range.max),
        };
    };

    return (
        <div className="space-y-2">
            <div className="surface-muted space-y-2 px-5 py-2.5">
                <label className="control-label mb-0.5 block">Add Plane</label>
                <div className="grid grid-cols-4 gap-4">
                    {(['x', 'y', 'z'] as ClippingAxis[]).map((axis) => (
                        <button
                            key={axis}
                            onClick={() => onAddPlane(axis, getFallbackRange(axis))}
                            className="btn-secondary px-3 py-3 text-xs font-semibold uppercase tracking-[0.08em]"
                        >
                            {axis}
                        </button>
                    ))}
                    {planes.length > 0 && (
                        <button
                            onClick={onRemoveAll}
                            className="btn-danger inline-flex items-center justify-center px-3 py-3 text-[11px] font-semibold leading-none text-white"
                        >
                            Clear All
                        </button>
                    )}
                </div>
            </div>

            {planes.length > 0 ? (
                <div className="max-h-[36rem] space-y-3 overflow-y-auto pr-1 pb-1 scrollbar-thin">
                    {planes.map((plane) => {
                        const fallbackRange = getFallbackRange(plane.axis);
                        const min = plane.min ?? fallbackRange.min;
                        const max = plane.max ?? fallbackRange.max;
                        const clamp = (value: number) => Math.min(
                            fallbackRange.max,
                            Math.max(fallbackRange.min, value)
                        );
                        let safeMin = clamp(Math.min(min, max));
                        let safeMax = clamp(Math.max(min, max));
                        if (safeMin >= safeMax) {
                            safeMin = fallbackRange.min;
                            safeMax = fallbackRange.max;
                        }
                        const safeStep = Math.max(
                            (fallbackRange.max - fallbackRange.min) / 200,
                            0.001
                        );
                        return (
                            <div key={plane.id} className="surface-muted space-y-2 px-3 py-3">
                                <div className="flex items-center justify-between gap-2">
                                    <div className="flex items-center gap-2">
                                        <span className="rounded bg-white/10 px-1.5 py-0.5 text-[11px] font-semibold uppercase tracking-[0.06em] text-[var(--accent-strong)]">
                                            {plane.axis}-axis
                                        </span>
                                        <button
                                            onClick={() => onUpdatePlane(plane.id, { enabled: !plane.enabled })}
                                            className="btn-secondary inline-flex h-7 w-7 items-center justify-center p-0"
                                            title={plane.enabled ? 'Disable plane' : 'Enable plane'}
                                        >
                                            {plane.enabled ? <Eye size={14} /> : <EyeOff size={14} />}
                                        </button>
                                    </div>
                                    <button
                                        onClick={() => onRemovePlane(plane.id)}
                                        className="btn-icon btn-icon-danger"
                                        title="Remove plane"
                                    >
                                        <Trash2 size={13} />
                                    </button>
                                </div>

                                <div className="space-y-1.5">
                                    <label className="control-label mb-0.5 block">
                                        Range: {safeMin.toFixed(2)} to {safeMax.toFixed(2)}
                                    </label>
                                    <DualRangeSlider
                                        min={fallbackRange.min}
                                        max={fallbackRange.max}
                                        step={safeStep}
                                        value={[safeMin, safeMax]}
                                        onChange={([nextMin, nextMax]) => onUpdatePlane(plane.id, {
                                            min: nextMin,
                                            max: nextMax,
                                        })}
                                        className={!plane.enabled ? 'opacity-50 pointer-events-none' : ''}
                                    />
                                </div>
                            </div>
                        );
                    })}
                </div>
            ) : (
                <div className="surface-muted p-4 text-center text-xs italic text-[var(--text-secondary)]">
                    No clipping planes. Add one to begin slicing.
                </div>
            )}
        </div>
    );
}
