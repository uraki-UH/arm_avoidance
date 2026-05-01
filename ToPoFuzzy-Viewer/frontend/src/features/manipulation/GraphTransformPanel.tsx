import { useEffect, useMemo, useRef, useState } from 'react';
import { RotateCcw } from 'lucide-react';
import { GraphTransform } from '../../types';

interface GraphTransformPanelProps {
    transform: GraphTransform;
    onUpdate: (updates: Partial<GraphTransform>) => void;
    onReset?: () => void;
}

const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export function GraphTransformPanel({ transform, onUpdate, onReset }: GraphTransformPanelProps) {
    const [position, setPosition] = useState<GraphTransform['position']>(transform.position);
    const [rotation, setRotation] = useState<GraphTransform['rotation']>(transform.rotation);
    const [scale, setScale] = useState<GraphTransform['scale']>(transform.scale);
    const [copyState, setCopyState] = useState<'idle' | 'copied'>('idle');
    const [positionStep, setPositionStep] = useState(0.01);
    const [rotationStepDeg, setRotationStepDeg] = useState(1);
    const [scaleStep, setScaleStep] = useState(0.01);
    const repeatTimerRef = useRef<number | null>(null);

    useEffect(() => setPosition(transform.position), [transform.position]);
    useEffect(() => setRotation(transform.rotation), [transform.rotation]);
    useEffect(() => setScale(transform.scale), [transform.scale]);
    useEffect(() => {
        return () => {
            if (repeatTimerRef.current !== null) {
                window.clearInterval(repeatTimerRef.current);
                repeatTimerRef.current = null;
            }
        };
    }, []);

    const summaryText = useMemo(() => {
        const pos = position.map((v) => v.toFixed(4)).join(' ');
        const rotDeg = rotation.map((v) => ((v * 180) / Math.PI).toFixed(2)).join(' ');
        const scl = scale.map((v) => v.toFixed(4)).join(' ');
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

    const stopRepeat = () => {
        if (repeatTimerRef.current !== null) {
            window.clearInterval(repeatTimerRef.current);
            repeatTimerRef.current = null;
        }
    };

    const startRepeat = (fn: () => void) => {
        stopRepeat();
        fn();
        repeatTimerRef.current = window.setInterval(fn, 80);
    };

    const updatePosition = (axis: number, value: number) => {
        const next: GraphTransform['position'] = [...position] as GraphTransform['position'];
        next[axis] = clamp(value, -10, 10);
        setPosition(next);
        onUpdate({ position: next });
    };

    const nudgePosition = (axis: number, direction: number) => {
        updatePosition(axis, (position[axis] || 0) + (positionStep * direction));
    };

    const updateRotationDeg = (axis: number, value: number) => {
        const next: GraphTransform['rotation'] = [...rotation] as GraphTransform['rotation'];
        next[axis] = (clamp(value, -180, 180) * Math.PI) / 180;
        setRotation(next);
        onUpdate({ rotation: next });
    };

    const nudgeRotationDeg = (axis: number, direction: number) => {
        updateRotationDeg(axis, ((rotation[axis] || 0) * 180 / Math.PI) + (rotationStepDeg * direction));
    };

    const updateScale = (axis: number, value: number) => {
        const next: GraphTransform['scale'] = [...scale] as GraphTransform['scale'];
        next[axis] = clamp(value, 0.1, 4);
        setScale(next);
        onUpdate({ scale: next });
    };

    const nudgeScale = (axis: number, direction: number) => {
        updateScale(axis, (scale[axis] || 1) + (scaleStep * direction));
    };

    const reset = () => {
        const next: GraphTransform = {
            position: [0, 0, 0],
            rotation: [0, 0, 0],
            scale: [1, 1, 1],
        };
        setPosition(next.position);
        setRotation(next.rotation);
        setScale(next.scale);
        if (onReset) {
            onReset();
            return;
        }
        onUpdate(next);
    };

    return (
        <div className="space-y-3">
            <div className="grid grid-cols-2 gap-2">
                <p className="panel-title">Graph Transform</p>
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
                <div className="flex items-center justify-between gap-2">
                    <p className="control-label mb-1 block">Position</p>
                    <div className="flex items-center gap-1 text-[10px] text-[var(--text-secondary)]">
                        <span>Step</span>
                        {[0.001, 0.01, 0.1].map((step) => (
                            <button
                                key={step}
                                onClick={() => setPositionStep(step)}
                                className={`rounded border px-1.5 py-0.5 font-mono ${positionStep === step ? 'border-[var(--accent-color)]/40 bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'border-white/10 bg-black/20'}`}
                            >
                                {step.toFixed(3)}
                            </button>
                        ))}
                    </div>
                </div>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between gap-2 text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <div className="flex items-center gap-1">
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgePosition(i, -1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >-</button>
                                <span className="font-mono">{position[i].toFixed(2)}</span>
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgePosition(i, 1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >+</button>
                            </div>
                        </div>
                        <input
                            type="range"
                            min="-10"
                            max="10"
                            step="0.01"
                            value={position[i]}
                            onChange={(e) => updatePosition(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <div className="flex items-center justify-between gap-2">
                    <p className="control-label mb-1 block">Rotation (deg)</p>
                    <div className="flex items-center gap-1 text-[10px] text-[var(--text-secondary)]">
                        <span>Step</span>
                        {[0.1, 1, 5].map((step) => (
                            <button
                                key={step}
                                onClick={() => setRotationStepDeg(step)}
                                className={`rounded border px-1.5 py-0.5 font-mono ${rotationStepDeg === step ? 'border-[var(--accent-color)]/40 bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'border-white/10 bg-black/20'}`}
                            >
                                {step.toFixed(1)}
                            </button>
                        ))}
                    </div>
                </div>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between gap-2 text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <div className="flex items-center gap-1">
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgeRotationDeg(i, -1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >-</button>
                                <span className="font-mono">{((rotation[i] * 180) / Math.PI).toFixed(1)}</span>
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgeRotationDeg(i, 1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >+</button>
                            </div>
                        </div>
                        <input
                            type="range"
                            min="-180"
                            max="180"
                            step="1"
                            value={(rotation[i] * 180) / Math.PI}
                            onChange={(e) => updateRotationDeg(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>

            <div className="surface-muted space-y-2 p-3">
                <div className="flex items-center justify-between gap-2">
                    <p className="control-label mb-1 block">Scale (viewer only)</p>
                    <div className="flex items-center gap-1 text-[10px] text-[var(--text-secondary)]">
                        <span>Step</span>
                        {[0.001, 0.01, 0.1].map((step) => (
                            <button
                                key={step}
                                onClick={() => setScaleStep(step)}
                                className={`rounded border px-1.5 py-0.5 font-mono ${scaleStep === step ? 'border-[var(--accent-color)]/40 bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'border-white/10 bg-black/20'}`}
                            >
                                {step.toFixed(3)}
                            </button>
                        ))}
                    </div>
                </div>
                {(['X', 'Y', 'Z'] as const).map((axis, i) => (
                    <div key={axis} className="space-y-1">
                        <div className="flex items-center justify-between gap-2 text-[10px] text-[var(--text-secondary)]">
                            <span>{axis}</span>
                            <div className="flex items-center gap-1">
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgeScale(i, -1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >-</button>
                                <span className="font-mono">{scale[i].toFixed(2)}</span>
                                <button
                                    type="button"
                                    className="rounded border border-white/10 bg-black/20 px-1.5 py-0.5 font-mono"
                                    onPointerDown={() => startRepeat(() => nudgeScale(i, 1))}
                                    onPointerUp={stopRepeat}
                                    onPointerLeave={stopRepeat}
                                    onContextMenu={(e) => e.preventDefault()}
                                >+</button>
                            </div>
                        </div>
                        <input
                            type="range"
                            min="0.1"
                            max="4"
                            step="0.01"
                            value={scale[i]}
                            onChange={(e) => updateScale(i, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>
        </div>
    );
}
