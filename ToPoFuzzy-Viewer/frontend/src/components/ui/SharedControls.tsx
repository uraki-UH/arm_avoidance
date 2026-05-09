import React, { useEffect, useRef } from 'react';
import { Eye, EyeOff, Move, Trash2, Minus, Plus } from 'lucide-react';
export const getStatusLabel = (type: string, mode?: string) => type === 'graph' ? (mode === 'static' ? 'Static' : 'Dynamic') : type.toUpperCase();

const fmt = (id: string, type: string) => {
    if (!id || id === 'default') return type === 'graph' ? 'GNG Topology' : type === 'robot' ? 'Robot Model' : type === 'marker' ? 'Markers' : 'Layer';
    return id.split('/').pop() || id;
};

interface ControlSliderProps { label: string; value: number; min: number; max: number; step: number; onChange: (v: number) => void; onPointerUp?: () => void; formatValue?: (v: number) => string }

export const LayerItem: React.FC<any> = ({ id, type, visible, onToggleVisibility, onRemove, onOpenTransform, statusLabel, isSelected, onSelect, isActionDisabled, children, headerOnly }) => {
    const c = (
        <div className="flex items-start justify-between gap-2">
            <div className="min-w-0 flex-1">
                <div className="mb-1 flex items-center gap-2">
                    <button onClick={e => { e.stopPropagation(); !isActionDisabled && onToggleVisibility(); }} className={`h-6 w-6 inline-flex items-center justify-center rounded-md border ${visible ? 'border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'}`}>
                        {visible ? <Eye size={14} /> : <EyeOff size={14} />}
                    </button>
                    {onOpenTransform && <button onClick={e => { e.stopPropagation(); onOpenTransform(); }} className="h-6 w-6 inline-flex items-center justify-center rounded-md border border-white/10 bg-black/20 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"><Move size={12} /></button>}
                    <span className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-50">{statusLabel || (type === 'graph' ? 'GNG' : type.toUpperCase())}</span>
                    <p className="truncate text-sm font-semibold text-[var(--text-primary)]" title={id}>{fmt(id, type)}</p>
                </div>
                {children}
            </div>
            <button onClick={e => { e.stopPropagation(); !isActionDisabled && onRemove(); }} className="btn-icon btn-icon-danger"><Trash2 size={13} /></button>
        </div>
    );
    return headerOnly ? c : <div onClick={onSelect} className={`rounded-lg border p-2 mb-2 ${isSelected ? 'border-[var(--accent-color)]/70 bg-[var(--accent-soft)]' : 'border-white/10 bg-white/5 hover:bg-white/10'} ${isActionDisabled ? 'opacity-70 cursor-not-allowed' : ''}`}>{c}</div>;
};

export const CompactToggle: React.FC<any> = ({ icon, label, isOn, onToggle }) => (
    <button onClick={e => { e.stopPropagation(); onToggle(); }} className={`flex items-center justify-between gap-2 rounded-md border px-2 py-1.5 w-full ${isOn ? 'border-[var(--accent-color)]/30 bg-[var(--accent-soft)]/50 text-[var(--text-primary)]' : 'border-white/5 bg-black/20 text-[var(--text-secondary)] opacity-60'}`}>
        <div className="flex items-center gap-1.5"><span className={isOn ? 'text-[var(--accent-strong)]' : ''}>{icon}</span><span className="text-[11px] font-medium">{label}</span></div>
        <div className={`h-1.5 w-1.5 rounded-full ${isOn ? 'bg-[var(--accent-color)] shadow-[0_0_5px_var(--accent-color)]' : 'bg-white/20'}`} />
    </button>
);

export const ControlSlider: React.FC<ControlSliderProps> = ({ label, value, min, max, step, onChange, onPointerUp, formatValue }) => {
    const t = useRef<any>();
    const sv = (d: number) => onChange(Math.min(max, Math.max(min, value + d)));
    const st = (d: number, ms = 400) => { sv(d); t.current = setTimeout(() => st(d, Math.max(30, ms * 0.8)), ms); };
    const sp = () => { clearTimeout(t.current); onPointerUp?.(); };
    useEffect(() => () => clearTimeout(t.current), []);
    return (
        <div className="space-y-1">
            <div className="flex items-center justify-between"><label className="text-[10px] font-semibold uppercase tracking-wider text-[var(--text-secondary)]">{label}</label><span className="text-[10px] font-mono text-[var(--accent-strong)]">{formatValue ? formatValue(value) : value.toFixed(step < 0.1 ? 3 : 2)}</span></div>
            <div className="flex items-center gap-2">
                <button onPointerDown={() => st(-step)} onPointerUp={sp} onPointerLeave={sp} className="h-6 w-6 flex items-center justify-center rounded border border-white/10 bg-white/5 text-[var(--text-secondary)] hover:bg-white/10"><Minus size={12} /></button>
                <input type="range" min={min} max={max} step={step} value={value} onChange={e => onChange(parseFloat(e.target.value))} onPointerUp={onPointerUp} className="flex-1 accent-[var(--accent-color)] h-1.5 bg-white/10 rounded-lg appearance-none cursor-pointer" />
                <button onPointerDown={() => st(step)} onPointerUp={sp} onPointerLeave={sp} className="h-6 w-6 flex items-center justify-center rounded border border-white/10 bg-white/5 text-[var(--text-secondary)] hover:bg-white/10"><Plus size={12} /></button>
            </div>
        </div>
    );
};
