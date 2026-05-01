import { useEffect, useMemo, useState } from 'react';
import { RotateCcw } from 'lucide-react';
import { PointCloudData } from '../../types';

interface PointCloudTransformPanelProps {
    cloudData: PointCloudData;
    onUpdate: (updates: Partial<PointCloudData>) => void;
}

const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export function PointCloudTransformPanel({ cloudData, onUpdate }: PointCloudTransformPanelProps) {
    const [position, setPosition] = useState<PointCloudData['position']>(cloudData.position || [0, 0, 0]);
    const [rotation, setRotation] = useState<PointCloudData['rotation']>(cloudData.rotation || [0, 0, 0]);
    const [scale, setScale] = useState<PointCloudData['scale']>(cloudData.scale || [1, 1, 1]);
    const [copyState, setCopyState] = useState<'idle' | 'copied'>('idle');

    useEffect(() => setPosition(cloudData.position || [0, 0, 0]), [cloudData.position]);
    useEffect(() => setRotation(cloudData.rotation || [0, 0, 0]), [cloudData.rotation]);
    useEffect(() => setScale(cloudData.scale || [1, 1, 1]), [cloudData.scale]);

    const summaryText = useMemo(() => {
        const pos = (position || [0, 0, 0]).map((v) => v.toFixed(4)).join(' ');
        const rotDeg = (rotation || [0, 0, 0]).map((v) => ((v * 180) / Math.PI).toFixed(2)).join(' ');
        const scl = (scale || [1, 1, 1]).map((v) => v.toFixed(4)).join(' ');
        return [
            `position: ${pos}`,
            `rotation_deg: ${rotDeg}`,
            `scale: ${scl}`,
        ].join('\n');
    }, [position, rotation, scale]);

    const copyTransform = async () => {
        try {
            await navigator.clipboard.writeText(summaryText);
            setCopyState('copied');
            window.setTimeout(() => setCopyState('idle'), 1200);
        } catch {
            setCopyState('idle');
        }
    };

    const updatePosition = (axis: number, value: number) => {
        const next: PointCloudData['position'] = [...(position || [0, 0, 0])] as PointCloudData['position'];
        next![axis] = clamp(value, -20, 20);
        setPosition(next);
        onUpdate({ position: next });
    };

    const updateRotationDeg = (axis: number, value: number) => {
        const next: PointCloudData['rotation'] = [...(rotation || [0, 0, 0])] as PointCloudData['rotation'];
        next![axis] = (clamp(value, -180, 180) * Math.PI) / 180;
        setRotation(next);
        onUpdate({ rotation: next });
    };

    const updateScale = (axis: number, value: number) => {
        const next: PointCloudData['scale'] = [...(scale || [1, 1, 1])] as PointCloudData['scale'];
        next![axis] = clamp(value, 0.1, 4);
        setScale(next);
        onUpdate({ scale: next });
    };

    const reset = () => {
        const next = {
            position: [0, 0, 0] as [number, number, number],
            rotation: [0, 0, 0] as [number, number, number],
            scale: [1, 1, 1] as [number, number, number],
        };
        setPosition(next.position);
        setRotation(next.rotation);
        setScale(next.scale);
        onUpdate(next);
    };

    return (
        <div className="space-y-3">
            <div className="grid grid-cols-2 gap-2">
                <p className="panel-title">Point Cloud Transform</p>
                <div className="flex items-center justify-end gap-1">
                    <button
                        onClick={copyTransform}
                        className="btn-secondary inline-flex items-center justify-center gap-1 px-2 py-1 text-xs"
                        title="Copy current transform"
                    >
                        {copyState === 'copied' ? 'Copied' : 'Copy'}
                    </button>
                    <button
                        onClick={reset}
                        className="btn-secondary inline-flex items-center justify-center gap-1 px-2 py-1 text-xs"
                        title="Reset transform"
                    >
                        <RotateCcw size={12} />
                        Reset
                    </button>
                </div>
            </div>

            <div className="surface-muted px-3 py-2">
                <p className="control-label mb-1 block">Current Offset</p>
                <p className="mb-1 text-[10px] text-[var(--text-secondary)]">
                    Position and rotation can be carried into TF. Scale is viewer-only.
                </p>
                <pre className="max-h-28 overflow-auto whitespace-pre-wrap font-mono text-[10px] leading-4 text-[var(--text-secondary)]">
                    {summaryText}
                </pre>
            </div>

            <div className="surface-muted space-y-2 p-3">
                <p className="control-label mb-1 block">Position</p>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <span className="font-mono">{(position?.[i] ?? 0).toFixed(2)}</span>
                        </div>
                        <input
                            type="range"
                            min="-20"
                            max="20"
                            step="0.01"
                            value={position?.[i] ?? 0}
                            onChange={(e) => updatePosition(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <p className="control-label mb-1 block">Rotation (deg)</p>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <span className="font-mono">{(((rotation?.[i] ?? 0) * 180) / Math.PI).toFixed(1)}</span>
                        </div>
                        <input
                            type="range"
                            min="-180"
                            max="180"
                            step="1"
                            value={((rotation?.[i] ?? 0) * 180) / Math.PI}
                            onChange={(e) => updateRotationDeg(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <p className="control-label mb-1 block">Scale (viewer only)</p>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <span className="font-mono">{(scale?.[i] ?? 1).toFixed(2)}</span>
                        </div>
                        <input
                            type="range"
                            min="0.1"
                            max="4"
                            step="0.01"
                            value={scale?.[i] ?? 1}
                            onChange={(e) => updateScale(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>
        </div>
    );
}
