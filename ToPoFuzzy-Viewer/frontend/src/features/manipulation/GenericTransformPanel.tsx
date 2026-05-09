import { useState } from 'react';
import { RotateCcw, Copy, Check } from 'lucide-react';
import { Transform } from '../../types';
import { useLongPress } from '../../hooks/useLongPress';

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
        <div className="space-y-4">
            <div className="flex items-center justify-between">
                <div>
                    <h3 className="text-sm font-medium text-white">{title}</h3>
                    {description && <p className="text-[10px] text-gray-400">{description}</p>}
                </div>
                <div className="flex gap-1">
                    <button onClick={handleCopy} className="btn-secondary px-2 py-1 text-[10px] flex items-center gap-1">
                        {copyState ? <Check size={10} className="text-green-400" /> : <Copy size={10} />} {copyState ? 'Copied' : 'Copy'}
                    </button>
                    <button onClick={onReset} className="btn-secondary px-2 py-1 text-[10px] flex items-center gap-1 hover:text-red-400">
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
                <span className="text-[11px] font-medium text-gray-300">{label}</span>
                <div className="flex gap-1">
                    {steps.map((s: number) => (
                        <button key={s} onClick={() => onStepChange(s)} className={`rounded px-1.5 py-0.5 font-mono text-[9px] border ${step === s ? 'bg-blue-500/20 text-blue-400 border-blue-500/30' : 'bg-black/20 text-gray-500 border-white/5'}`}>
                            {s < 1 ? s.toString().replace(/^0/, '') : s}
                        </button>
                    ))}
                </div>
            </div>
            {['X', 'Y', 'Z'].map((axis, i) => (
                <AxisControl key={axis} label={axis} value={values[i]} step={step} onUpdate={(v: number) => onUpdate(i, v)} format={format} min={min} max={max} />
            ))}
        </div>
    );
}

function AxisControl({ label, value, step, onUpdate, format, min, max }: any) {
    const down = useLongPress(() => onUpdate(value - step));
    const up = useLongPress(() => onUpdate(value + step));

    return (
        <div className="space-y-1">
            <div className="flex items-center justify-between">
                <span className="font-mono text-[10px] text-gray-500">{label}</span>
                <div className="flex items-center gap-2">
                    <button {...down} className={BTN_CLS}>-</button>
                    <span className="min-w-[4rem] text-center font-mono text-[11px] text-white">{format(value)}</span>
                    <button {...up} className={BTN_CLS}>+</button>
                </div>
            </div>
            <input type="range" min={min} max={max} step={0.001} value={value} onChange={e => onUpdate(parseFloat(e.target.value))} className="h-1 w-full accent-blue-500" />
        </div>
    );
}
