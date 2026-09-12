import React, { useEffect, useRef, useState } from 'react';
import { Eye, EyeOff, Move, Trash2, Minus, Plus } from 'lucide-react';

const fmt = (id: string, type: string) => {
    if (!id || id === 'default') return type === 'graph' ? 'GNG Topology' : type === 'robot' ? 'Robot Model' : type === 'marker' ? 'Markers' : 'Layer';
    return id.split('/').pop() || id;
};

interface ControlSliderProps { label: string; value: number; min: number; max: number; step: number; onChange: (v: number) => void; onPointerUp?: () => void; formatValue?: (v: number) => string; disabled?: boolean }

export const LayerItem: React.FC<any> = ({ id, displayName, type, visible, onToggleVisibility, onRemove, onOpenTransform, statusLabel, isSelected, onSelect, isActionDisabled, children, headerOnly }) => {
    const c = (
        <div className="flex items-start justify-between gap-1.5">
            <div className="min-w-0 flex-1">
                <div className="mb-0.5 flex items-start gap-2">
                    <button onClick={e => { e.stopPropagation(); if (!isActionDisabled) onToggleVisibility(); }} className={`h-6 w-6 inline-flex items-center justify-center rounded-md border ${visible ? 'border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'}`}>
                        {visible ? <Eye size={14} /> : <EyeOff size={14} />}
                    </button>
                    {onOpenTransform && <button onClick={e => { e.stopPropagation(); onOpenTransform(); }} className="h-6 w-6 inline-flex items-center justify-center rounded-md border border-white/10 bg-black/20 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"><Move size={12} /></button>}
                    <div className="min-w-0 flex-1">
                        <span className="block text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-50">
                            {statusLabel || (type === 'graph' ? 'GNG' : type.toUpperCase())}
                        </span>
                        <p className="truncate text-sm font-semibold text-[var(--text-primary)]" title={displayName || id}>
                            {displayName || fmt(id, type)}
                        </p>
                    </div>
                </div>
                {children}
            </div>
            <button onClick={e => { e.stopPropagation(); if (!isActionDisabled) onRemove(); }} className="btn-icon btn-icon-danger self-start mt-0.5"><Trash2 size={13} /></button>
        </div>
    );
    return headerOnly ? c : <div onClick={onSelect} className={`rounded-lg border px-2 py-1 mb-1 ${isSelected ? 'border-[var(--accent-color)]/70 bg-[var(--accent-soft)]' : 'border-white/10 bg-white/5 hover:bg-white/10'} ${isActionDisabled ? 'opacity-70 cursor-not-allowed' : ''}`}>{c}</div>;
};

export const CompactToggle: React.FC<any> = ({ icon, label, isOn, onToggle, className = '' }) => (
    <button onClick={e => { e.stopPropagation(); onToggle(); }} className={`flex items-center justify-between gap-1.5 rounded-md border px-2.5 py-[0.375rem] w-full ${className} ${isOn ? 'border-[var(--accent-color)]/30 bg-[var(--accent-soft)]/50 text-[var(--text-primary)]' : 'border-white/5 bg-black/20 text-[var(--text-secondary)] opacity-60'}`}>
        <div className="flex items-center gap-1.5"><span className={isOn ? 'text-[var(--accent-strong)]' : ''}>{icon}</span><span className="text-[10px] font-medium leading-none">{label}</span></div>
        <div className={`h-1.5 w-1.5 rounded-full ${isOn ? 'bg-[var(--accent-color)] shadow-[0_0_5px_var(--accent-color)]' : 'bg-white/20'}`} />
    </button>
);

export const ControlSlider: React.FC<ControlSliderProps> = ({ label, value, min, max, step, onChange, onPointerUp, formatValue, disabled = false }) => {
    const t = useRef<any>();
    const sv = (d: number) => onChange(Math.min(max, Math.max(min, value + d)));
    const st = (d: number, ms = 400) => { sv(d); t.current = setTimeout(() => st(d, Math.max(30, ms * 0.8)), ms); };
    const sp = () => { clearTimeout(t.current); onPointerUp?.(); };
    useEffect(() => () => clearTimeout(t.current), []);
    return (
        <div className="space-y-1">
            <div className="flex items-center justify-between"><label className="text-[10px] font-semibold uppercase tracking-wider text-[var(--text-secondary)]">{label}</label><span className="text-[10px] font-mono text-[var(--accent-strong)]">{formatValue ? formatValue(value) : value.toFixed(step < 0.1 ? 3 : 2)}</span></div>
            <div className="flex items-center gap-2">
                <button disabled={disabled} onPointerDown={() => !disabled && st(-step)} onPointerUp={sp} onPointerLeave={sp} className={`h-6 w-6 flex items-center justify-center rounded border border-white/10 bg-white/5 text-[var(--text-secondary)] hover:bg-white/10 ${disabled ? 'opacity-40 cursor-not-allowed hover:bg-white/5' : ''}`}><Minus size={12} /></button>
                <input disabled={disabled} type="range" min={min} max={max} step={step} value={value} onChange={e => !disabled && onChange(parseFloat(e.target.value))} onPointerUp={onPointerUp} className="flex-1 accent-[var(--accent-color)] h-1.5 bg-white/10 rounded-lg appearance-none cursor-pointer disabled:opacity-50" />
                <button disabled={disabled} onPointerDown={() => !disabled && st(step)} onPointerUp={sp} onPointerLeave={sp} className={`h-6 w-6 flex items-center justify-center rounded border border-white/10 bg-white/5 text-[var(--text-secondary)] hover:bg-white/10 ${disabled ? 'opacity-40 cursor-not-allowed hover:bg-white/5' : ''}`}><Plus size={12} /></button>
            </div>
        </div>
    );
};

interface DualRangeSliderProps {
    min: number;
    max: number;
    step?: number;
    value: [number, number];
    onChange: (value: [number, number]) => void;
    className?: string;
}

export const DualRangeSlider: React.FC<DualRangeSliderProps> = ({
    min,
    max,
    step = 1,
    value,
    onChange,
    className = ''
}) => {
    const [localValue, setLocalValue] = useState(value);
    const containerRef = useRef<HTMLDivElement>(null);
    const isDraggingRef = useRef<'min' | 'max' | null>(null);

    useEffect(() => {
        setLocalValue(value);
    }, [value]);

    const getPercentage = (val: number) => ((val - min) / (max - min)) * 100;

    const handlePointerDown = (thumb: 'min' | 'max') => (e: React.PointerEvent) => {
        e.preventDefault();
        isDraggingRef.current = thumb;
        document.addEventListener('pointermove', handlePointerMove);
        document.addEventListener('pointerup', handlePointerUp);
        // 要素外へ移動した場合のイベント受信継続
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
    };

    const handlePointerMove = (e: PointerEvent) => {
        if (!isDraggingRef.current || !containerRef.current) return;

        const rect = containerRef.current.getBoundingClientRect();
        const percentage = Math.min(Math.max((e.clientX - rect.left) / rect.width, 0), 1);
        let newValue = min + percentage * (max - min);

        // 操作刻みへの丸め
        newValue = Math.round(newValue / step) * step;

        setLocalValue(prev => {
            const next = [...prev] as [number, number];
            if (isDraggingRef.current === 'min') {
                next[0] = Math.min(newValue, prev[1] - step);
            } else {
                next[1] = Math.max(newValue, prev[0] + step);
            }
            onChange(next);
            return next;
        });
    };

    const handlePointerUp = () => {
        isDraggingRef.current = null;
        document.removeEventListener('pointermove', handlePointerMove);
        document.removeEventListener('pointerup', handlePointerUp);
    };

    const minPos = getPercentage(localValue[0]);
    const maxPos = getPercentage(localValue[1]);

    return (
        <div className={`relative w-full h-6 flex items-center select-none ${className}`} ref={containerRef}>
            {/* スライダーの背景 */}
            <div className="absolute w-full h-1 bg-white/20 rounded-full overflow-hidden">
                {/* 選択範囲 */}
                <div
                    className="absolute h-full bg-[var(--accent-color)]"
                    style={{ left: `${minPos}%`, width: `${maxPos - minPos}%` }}
                />
            </div>

            {/* 下限ハンドル */}
            <div
                className="absolute w-4 h-4 bg-white rounded-full shadow-md cursor-grab active:cursor-grabbing hover:scale-110 transition-transform"
                style={{ left: `${minPos}%`, transform: 'translateX(-50%)' }}
                onPointerDown={handlePointerDown('min')}
            />

            {/* 上限ハンドル */}
            <div
                className="absolute w-4 h-4 bg-white rounded-full shadow-md cursor-grab active:cursor-grabbing hover:scale-110 transition-transform"
                style={{ left: `${maxPos}%`, transform: 'translateX(-50%)' }}
                onPointerDown={handlePointerDown('max')}
            />
        </div>
    );
};
