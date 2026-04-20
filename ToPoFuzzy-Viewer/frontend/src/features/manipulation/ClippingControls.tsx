import { Eye, EyeOff, Trash2 } from 'lucide-react';
import { ClippingPlane, ClippingAxis } from '../../hooks/useClippingPlanes';

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
    onAddPlane: (axis: ClippingAxis) => void;
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
    const getRange = (axis: ClippingAxis): { min: number; max: number } => {
        if (!bounds) {
            return { min: -100, max: 100 };
        }
        switch (axis) {
            case 'x':
                return { min: bounds.minX, max: bounds.maxX };
            case 'y':
                return { min: bounds.minY, max: bounds.maxY };
            case 'z':
                return { min: bounds.minZ, max: bounds.maxZ };
            default:
                return { min: -100, max: 100 };
        }
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center justify-between">
                <h3 className="panel-title">Clipping Planes</h3>
                {planes.length > 0 && (
                    <button onClick={onRemoveAll} className="btn-danger px-2.5 py-1 text-[11px] font-semibold text-white">
                        Clear All
                    </button>
                )}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <label className="control-label">Add Plane</label>
                <div className="grid grid-cols-3 gap-2">
                    {(['x', 'y', 'z'] as ClippingAxis[]).map((axis) => (
                        <button
                            key={axis}
                            onClick={() => onAddPlane(axis)}
                            className="btn-secondary px-3 py-2 text-xs font-semibold uppercase tracking-[0.08em]"
                        >
                            {axis}
                        </button>
                    ))}
                </div>
            </div>

            {planes.length > 0 ? (
                <div className="max-h-80 space-y-3 overflow-y-auto pr-1 scrollbar-thin">
                    {planes.map((plane) => {
                        const range = getRange(plane.axis);
                        return (
                            <div key={plane.id} className="surface-muted space-y-2 p-3">
                                <div className="flex items-center justify-between gap-2">
                                    <div className="flex items-center gap-2">
                                        <span className="rounded bg-white/10 px-2 py-0.5 text-xs font-semibold uppercase tracking-[0.08em] text-[var(--accent-strong)]">
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

                                <div>
                                    <label className="control-label mb-1 block">
                                        Position: {plane.position.toFixed(2)}
                                    </label>
                                    <p className="mb-1 text-[10px] text-[var(--text-secondary)]">
                                        Range {range.min.toFixed(1)} to {range.max.toFixed(1)}
                                    </p>
                                    <input
                                        type="range"
                                        min={range.min}
                                        max={range.max}
                                        step={(range.max - range.min) / 200}
                                        value={plane.position}
                                        onChange={(e) => onUpdatePlane(plane.id, { position: parseFloat(e.target.value) })}
                                        className="w-full"
                                        disabled={!plane.enabled}
                                    />
                                </div>

                                <label className="inline-flex items-center gap-2 text-xs text-[var(--text-secondary)]">
                                    <input
                                        type="checkbox"
                                        id={`invert-${plane.id}`}
                                        checked={plane.inverted}
                                        onChange={(e) => onUpdatePlane(plane.id, { inverted: e.target.checked })}
                                        disabled={!plane.enabled}
                                        className="h-4 w-4 rounded border-white/30 bg-white/10 text-[var(--accent-color)]"
                                    />
                                    Invert normal direction
                                </label>
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
