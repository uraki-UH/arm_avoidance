import { useState, useEffect } from 'react';
import { PointCloudData } from '../../types';

interface TransformPanelProps {
    cloudData: PointCloudData;
    onUpdate: (updates: Partial<PointCloudData>) => void;
    transformMode: 'translate' | 'rotate' | 'scale';
    onModeChange: (mode: 'translate' | 'rotate' | 'scale') => void;
}

export function TransformPanel({ cloudData, onUpdate, transformMode, onModeChange }: TransformPanelProps) {
    const [position, setPosition] = useState<[number, number, number]>(cloudData.position || [0, 0, 0]);
    const [rotation, setRotation] = useState<[number, number, number]>(cloudData.rotation || [0, 0, 0]);

    useEffect(() => {
        if (cloudData.position) setPosition(cloudData.position);
    }, [cloudData.position]);

    useEffect(() => {
        if (cloudData.rotation) setRotation(cloudData.rotation);
    }, [cloudData.rotation]);

    const handlePositionChange = (axis: number, value: number) => {
        const next: [number, number, number] = [position[0], position[1], position[2]];
        next[axis] = value;
        setPosition(next);
        onUpdate({ position: next });
    };

    const handleRotationChange = (axis: number, value: number) => {
        const next: [number, number, number] = [rotation[0], rotation[1], rotation[2]];
        next[axis] = (value * Math.PI) / 180;
        setRotation(next);
        onUpdate({ rotation: next });
    };

    const handleReset = () => {
        const resetPos: [number, number, number] = [0, 0, 0];
        const resetRot: [number, number, number] = [0, 0, 0];
        const resetScale: [number, number, number] = [1, 1, 1];
        setPosition(resetPos);
        setRotation(resetRot);
        onUpdate({ position: resetPos, rotation: resetRot, scale: resetScale });
    };

    return (
        <div className="space-y-4">
            <div>
                <p className="panel-title">Transform Target</p>
                <p className="mt-1 truncate text-sm font-semibold text-[var(--text-primary)]">{cloudData.name}</p>
            </div>

            <div className="surface-muted p-2">
                <label className="control-label mb-2 block">Transform Mode</label>
                <div className="grid grid-cols-3 gap-1">
                    {(['translate', 'rotate', 'scale'] as const).map((mode) => (
                        <button
                            key={mode}
                            onClick={() => onModeChange(mode)}
                            className={`rounded-md px-2 py-1.5 text-xs font-semibold uppercase tracking-[0.06em] transition-colors ${transformMode === mode
                                ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)] ring-1 ring-[var(--accent-color)]/40'
                                : 'text-[var(--text-secondary)] hover:bg-white/10 hover:text-[var(--text-primary)]'
                                }`}
                        >
                            {mode}
                        </button>
                    ))}
                </div>
            </div>

            <div className="surface-muted space-y-2 p-3">
                <p className="panel-title">Position</p>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="flex items-center gap-2">
                        <label className="w-4 text-xs font-mono text-[var(--text-secondary)]">{axis}</label>
                        <input
                            type="number"
                            value={position[i].toFixed(2)}
                            onChange={(e) => handlePositionChange(i, parseFloat(e.target.value) || 0)}
                            step="0.1"
                            className="input-field p-2 text-xs"
                        />
                    </div>
                ))}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <p className="panel-title">Rotation (deg)</p>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="flex items-center gap-2">
                        <label className="w-4 text-xs font-mono text-[var(--text-secondary)]">{axis}</label>
                        <input
                            type="number"
                            value={((rotation[i] * 180) / Math.PI).toFixed(1)}
                            onChange={(e) => handleRotationChange(i, parseFloat(e.target.value) || 0)}
                            step="5"
                            className="input-field p-2 text-xs"
                        />
                    </div>
                ))}
            </div>

            <button onClick={handleReset} className="btn-secondary w-full px-3 py-2 text-sm font-semibold">
                Reset Transform
            </button>
        </div>
    );
}
