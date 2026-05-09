import { useState, useEffect, useRef } from 'react';
import { RotateCcw, Copy, Check } from 'lucide-react';
import { Transform } from '../../types';

function useLongPress(callback: () => void, ms = 80) {
    const timerRef = useRef<number | null>(null);
    const callbackRef = useRef(callback);

    useEffect(() => {
        callbackRef.current = callback;
    }, [callback]);

    const stop = () => {
        if (timerRef.current !== null) {
            window.clearInterval(timerRef.current);
            timerRef.current = null;
        }
    };

    const start = () => {
        stop();
        callbackRef.current();
        timerRef.current = window.setInterval(() => callbackRef.current(), ms);
    };

    useEffect(() => stop, []);

    return {
        onPointerDown: start,
        onPointerUp: stop,
        onPointerLeave: stop,
    };
}

interface GenericTransformPanelProps {
    title: string;
    transform: Transform;
    onUpdate: (updates: Partial<Transform>) => void;
    onReset?: () => void;
    description?: string;
}

const CLAMP_VAL = (v: number, min: number, max: number) => Math.min(max, Math.max(min, v));
const BTN_CLS = "flex h-5 w-5 items-center justify-center rounded border border-white/5 bg-black/20 text-gray-400 hover:bg-black/40 hover:text-white transition-colors";

export function GenericTransformPanel({ title, transform, onUpdate, onReset, description }: GenericTransformPanelProps) {
    const [copyState, setCopyState] = useState(false);
    const [steps, setSteps] = useState({ pos: 0.01, rot: 1, scl: 0.01 });

    const handleCopy = async () => {
        const text = `pos: ${transform.position.join(' ')}\nrot_deg: ${transform.rotation.map(r => (r * 180 / Math.PI).toFixed(2)).join(' ')}\nscl: ${transform.scale.join(' ')}`;
        await navigator.clipboard.writeText(text);
        setCopyState(true);
        setTimeout(() => setCopyState(false), 1200);
    };

    const updateValue = (type: keyof Transform, axis: number, val: number) => {
        const next = [...transform[type]] as [number, number, number];
        next[axis] = CLAMP_VAL(val, type === 'scale' ? 0.01 : -50, 50);
        onUpdate({ [type]: next });
    };

    return (
        <div className="space-y-3">
            <div className="flex items-center justify-between pb-1">
                <div>
                    <h3 className="text-xs font-bold text-white/90 tracking-tight">{title}</h3>
                    {description && <p className="text-[9px] text-gray-500 leading-none mt-0.5">{description}</p>}
                </div>
                <div className="flex gap-1.5">
                    <button onClick={handleCopy} className="text-[10px] text-gray-400 hover:text-white transition-colors flex items-center gap-1 bg-white/5 px-2 py-0.5 rounded border border-white/5">
                        {copyState ? <Check size={10} className="text-green-400" /> : <Copy size={10} />} {copyState ? 'Copied' : 'Copy'}
                    </button>
                    <button onClick={onReset} className="text-[10px] text-gray-400 hover:text-red-400 transition-colors flex items-center gap-1 bg-white/5 px-2 py-0.5 rounded border border-white/5">
                        <RotateCcw size={10} /> Reset
                    </button>
                </div>
            </div>

            <ControlSection label="Position (m)" type="position" values={transform.position} step={steps.pos} onStepChange={(v: number) => setSteps({...steps, pos: v})} 
                onUpdate={(a: number, v: number) => updateValue('position', a, v)} format={(v: number) => v.toFixed(3)} steps={[0.001, 0.01, 0.1]} />
            
            <ControlSection label="Rotation (deg)" type="rotation" values={transform.rotation.map(r => r * 180 / Math.PI)} step={steps.rot} onStepChange={(v: number) => setSteps({...steps, rot: v})}
                onUpdate={(a: number, v: number) => updateValue('rotation', a, v * Math.PI / 180)} format={(v: number) => v.toFixed(1) + '°'} steps={[0.1, 1, 5]} min={-180} max={180} />

            <ControlSection label="Scale" type="scale" values={transform.scale} step={steps.scl} onStepChange={(v: number) => setSteps({...steps, scl: v})}
                onUpdate={(a: number, v: number) => updateValue('scale', a, v)} format={(v: number) => v.toFixed(3)} steps={[0.001, 0.01, 0.1]} min={0.01} max={5} />
        </div>
    );
}

function ControlSection({ label, values, step, onStepChange, onUpdate, format, steps, min = -20, max = 20 }: any) {
    return (
        <div className="rounded-lg bg-white/5 border border-white/5 p-3 space-y-3">
            <div className="flex items-center justify-between">
                <span className="text-[11px] font-bold uppercase tracking-wider text-blue-400/80">{label}</span>
                <div className="flex gap-1.5">
                    {steps.map((s: number) => (
                        <button key={s} onClick={() => onStepChange(s)} className={`rounded min-w-[32px] px-3 py-0.5 font-mono text-[9px] border transition-all ${step === s ? 'bg-blue-500/30 text-blue-300 border-blue-500/50 shadow-[0_0_8px_rgba(59,130,246,0.3)]' : 'bg-black/30 text-gray-500 border-white/5 hover:border-white/20 hover:text-gray-300'}`}>
                            {s}
                        </button>
                    ))}
                </div>
            </div>
            <div className="grid grid-cols-3 gap-3">
                {['X', 'Y', 'Z'].map((axis, i) => (
                    <AxisControl key={axis} label={axis} value={values[i]} step={step} onUpdate={(v: number) => onUpdate(i, v)} format={format} min={min} max={max} />
                ))}
            </div>
        </div>
    );
}

function AxisControl({ label, value, step, onUpdate, format, min, max }: any) {
    const down = useLongPress(() => onUpdate(value - step));
    const up = useLongPress(() => onUpdate(value + step));

    return (
        <div className="space-y-1.5">
            <div className="flex items-center justify-between px-1">
                <span className="font-mono text-[10px] font-bold text-gray-500">{label}</span>
                <span className="font-mono text-[10px] text-white tabular-nums">{format(value)}</span>
            </div>
            <div className="flex items-center gap-1">
                <button {...down} className={`${BTN_CLS} flex-1`}>-</button>
                <button {...up} className={`${BTN_CLS} flex-1`}>+</button>
            </div>
            <input 
                type="range" min={min} max={max} step={0.001} value={value} 
                onChange={e => onUpdate(parseFloat(e.target.value))} 
                className="h-1 w-full accent-blue-500 bg-white/5 rounded-full appearance-none cursor-pointer" 
            />
        </div>
    );
}
