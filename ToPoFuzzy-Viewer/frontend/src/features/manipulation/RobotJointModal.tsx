import { X, RotateCcw } from 'lucide-react';
import { RobotData } from '../../types';
import { ControlSlider } from '../../components/ui/SharedControls';

interface RobotJointModalProps {
    title: string;
    subtitle?: string;
    open: boolean;
    robotData: RobotData | null;
    controlMode: 'live' | 'manual';
    jointValues: number[];
    onClose: () => void;
    onUpdate: (updates: { jointValues: number[] }) => void;
    onModeChange: (mode: 'live' | 'manual') => void;
    selectedManipLink?: string;
    onManipLinkChange?: (linkName: string) => void;
    onReset?: () => void;
}

const clamp = (v: number, min: number, max: number) => Math.min(max, Math.max(min, v));

export function RobotJointModal({
    title,
    subtitle,
    open,
    robotData,
    controlMode,
    jointValues,
    onClose,
    onUpdate,
    onModeChange,
    selectedManipLink = '',
    onManipLinkChange,
    onReset,
}: RobotJointModalProps) {
    if (!open || !robotData) return null;

    const manualValues = robotData.jointValues.length === jointValues.length && jointValues.length > 0
        ? jointValues
        : robotData.jointValues;
    const effectiveValues = controlMode === 'manual' ? manualValues : robotData.jointValues;

    return (
        <div className="fixed inset-0 z-[9999]" onClick={onClose}>
            <div
                className="fixed left-4 top-20 w-[560px] animate-in fade-in slide-in-from-left-2 duration-300"
                onClick={(e) => e.stopPropagation()}
            >
            <div className="surface-panel flex max-h-[80vh] flex-col overflow-hidden shadow-2xl ring-1 ring-white/10">
                <div className="flex items-center justify-between border-b border-white/5 bg-black/40 px-4 py-3">
                    <div className="min-w-0">
                        <h2 className="truncate text-sm font-bold text-white leading-tight">{title}</h2>
                        {subtitle && <p className="mt-0.5 truncate text-[10px] font-mono text-gray-400 opacity-70">{subtitle}</p>}
                    </div>
                    <div className="flex items-center gap-2">
                        {onReset && (
                            <button
                                onClick={onReset}
                                className="flex h-7 items-center gap-1 rounded-md border border-white/10 px-2 text-[10px] text-gray-300 hover:bg-white/10 hover:text-white transition-all"
                            >
                                <RotateCcw size={14} />
                                Reset
                            </button>
                        )}
                        <button
                            onClick={onClose}
                            className="flex h-7 w-7 items-center justify-center rounded-md text-gray-400 hover:bg-white/10 hover:text-white transition-all"
                        >
                            <X size={16} />
                        </button>
                    </div>
                </div>

                <div className="flex-1 overflow-y-auto bg-[#0c141d]/50 p-4">
                    <div className="mb-3 rounded-md border border-white/5 bg-black/15 px-3 py-2 text-[10px] text-gray-400">
                        <div className="mb-2 flex items-center gap-2">
                            <button
                                onClick={() => {
                                    if (controlMode === 'live') return;
                                    onModeChange('live');
                                    onUpdate({ jointValues: [] });
                                }}
                                className={`rounded-md px-2 py-1 font-semibold transition-all ${controlMode === 'live' ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'bg-white/10 text-gray-600 hover:bg-white/10 hover:text-white'}`}
                            >
                                Follow joint_states
                            </button>
                            <button
                                onClick={() => {
                                    if (controlMode === 'manual') return;
                                    onUpdate({ jointValues: [...robotData.jointValues] });
                                    onModeChange('manual');
                                }}
                                className={`rounded-md px-2 py-1 font-semibold transition-all ${controlMode === 'manual' ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)]' : 'bg-white/5 text-gray-300 hover:bg-white/10 hover:text-white'}`}
                            >
                                Manual
                            </button>
                        </div>
                        {robotData.linkNames && robotData.linkNames.length > 0 && onManipLinkChange && (
                            <div className="mt-2 flex items-center gap-2">
                                <span className="min-w-16 text-[10px] uppercase tracking-wider text-gray-500">Link</span>
                                <select
                                    value={selectedManipLink || robotData.linkNames[robotData.linkNames.length - 1] || ''}
                                    onChange={(e) => onManipLinkChange(e.target.value)}
                                    className="min-w-0 flex-1 rounded-md border border-white/10 bg-black/30 px-2 py-1 font-mono text-[10px] text-gray-200 outline-none"
                                >
                                    {robotData.linkNames.map((linkName) => (
                                        <option key={linkName} value={linkName}>{linkName}</option>
                                    ))}
                                </select>
                            </div>
                        )}
                        {robotData.linkManipulabilities && robotData.linkManipulabilities.length > 0 && (
                            <div className="mt-2 rounded-md border border-white/5 bg-black/20 px-2 py-1 text-[10px] text-gray-400">
                                <div className="flex items-center justify-between gap-2">
                                    <span className="font-mono text-gray-500">Selected Manip</span>
                                    <span className="font-mono text-gray-200">
                                        {(() => {
                                            const linkName = selectedManipLink || robotData.linkNames?.[robotData.linkNames.length - 1] || '';
                                            const info = robotData.linkManipulabilities?.find((x) => x.linkName === linkName);
                                            return info?.manipValue?.toFixed(3) ?? 'n/a';
                                        })()}
                                    </span>
                                </div>
                            </div>
                        )}
                    </div>

                    <div className="space-y-3">
                        {robotData.jointNames.map((jointName, index) => {
                            const value = effectiveValues[index] ?? robotData.jointValues[index] ?? 0;
                            return (
                                <ControlSlider
                                    key={`${robotData.tag || robotData.frameId}-${jointName}-${index}`}
                                    label={jointName}
                                    value={value}
                                    min={-Math.PI}
                                    max={Math.PI}
                                    step={0.01}
                                    disabled={controlMode === 'live'}
                                    onChange={(next) => {
                                        if (controlMode !== 'manual') return;
                                        const nextJointValues = [...effectiveValues];
                                        nextJointValues[index] = clamp(next, -Math.PI, Math.PI);
                                        onUpdate({ jointValues: nextJointValues });
                                    }}
                                    formatValue={(v) => `${v.toFixed(2)} rad`}
                                />
                            );
                        })}
                    </div>
                </div>
            </div>
        </div>
        </div>
    );
}
