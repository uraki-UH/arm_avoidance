import { createPortal } from 'react-dom';
import { Layers, X } from 'lucide-react';
import { LAYER_COLORS, LAYER_LABELS } from '../../types';

interface GngLabelModalProps {
    open: boolean;
    title?: string;
    subtitle?: string;
    visibleLabels: {
        0: boolean;
        1: boolean;
        2: boolean;
        3: boolean;
        4: boolean;
        5: boolean;
    };
    onClose: () => void;
    onUpdate: (updates: {
        visibleLabels: {
            0: boolean;
            1: boolean;
            2: boolean;
            3: boolean;
            4: boolean;
            5: boolean;
        };
    }) => void;
}

const ALL_VISIBLE_LABELS = {
    0: true,
    1: true,
    2: true,
    3: true,
    4: true,
    5: true,
};

const NONE_VISIBLE_LABELS = {
    0: false,
    1: false,
    2: false,
    3: false,
    4: false,
    5: false,
};

export function GngLabelModal({
    open,
    title = 'Visible Labels',
    subtitle = '',
    visibleLabels,
    onClose,
    onUpdate,
}: GngLabelModalProps) {
    if (!open) return null;

    const setAll = (value: boolean) => {
        onUpdate({
            visibleLabels: value ? ALL_VISIBLE_LABELS : NONE_VISIBLE_LABELS,
        });
    };

    const toggleLabel = (labelIndex: 0 | 1 | 2 | 3 | 4 | 5) => {
        onUpdate({
            visibleLabels: {
                ...visibleLabels,
                [labelIndex]: !visibleLabels[labelIndex],
            },
        });
    };

    return createPortal(
        <div
            className="fixed inset-0 z-[9999] bg-black/25 backdrop-blur-[1px]"
            onPointerDown={onClose}
            role="presentation"
        >
            <div
                className="fixed left-4 top-20 w-[360px] max-w-[calc(100vw-2rem)] animate-in fade-in slide-in-from-left-2 duration-300"
                onPointerDown={(e) => e.stopPropagation()}
                role="dialog"
                aria-modal="true"
                aria-label={title}
            >
                <div className="surface-panel flex max-h-[80vh] flex-col overflow-hidden shadow-2xl ring-1 ring-white/10">
                    <div className="flex items-center justify-between border-b border-white/5 bg-black/40 px-4 py-3">
                        <div className="min-w-0">
                            <div className="flex items-center gap-2">
                                <Layers size={15} className="text-[var(--accent-strong)]" />
                                <h2 className="truncate text-sm font-bold text-white leading-tight">{title}</h2>
                            </div>
                            <p className="mt-0.5 truncate text-[10px] font-mono text-gray-400 opacity-70">
                                {subtitle}
                            </p>
                        </div>
                        <button
                            onClick={onClose}
                            className="flex h-7 w-7 items-center justify-center rounded-md text-gray-400 hover:bg-white/10 hover:text-white transition-all"
                        >
                            <X size={16} />
                        </button>
                    </div>

                    <div className="flex-1 overflow-y-auto bg-[#0c141d]/50 p-4">
                        <div className="mb-3 grid grid-cols-2 gap-2">
                            <button
                                onClick={() => setAll(true)}
                                className="rounded-md border border-[var(--accent-color)]/30 bg-[var(--accent-soft)] px-3 py-2 text-xs font-semibold text-[var(--text-primary)] transition-colors hover:bg-[var(--accent-soft)]/80"
                            >
                                All
                            </button>
                            <button
                                onClick={() => setAll(false)}
                                className="rounded-md border border-white/10 bg-black/20 px-3 py-2 text-xs font-semibold text-[var(--text-secondary)] transition-colors hover:bg-white/10 hover:text-[var(--text-primary)]"
                            >
                                None
                            </button>
                        </div>

                        <div className="space-y-2">
                            {LAYER_LABELS.map((label, index) => {
                                const labelIndex = index as 0 | 1 | 2 | 3 | 4 | 5;
                                const isOn = visibleLabels[labelIndex];
                                return (
                                    <button
                                        key={label}
                                        onClick={() => toggleLabel(labelIndex)}
                                        className={`flex w-full items-center justify-between gap-3 rounded-md border px-3 py-2 text-left transition-colors ${
                                            isOn
                                                ? 'border-[var(--accent-color)]/30 bg-[var(--accent-soft)]/60 text-[var(--text-primary)]'
                                                : 'border-white/10 bg-black/20 text-[var(--text-secondary)] opacity-75 hover:bg-white/10'
                                        }`}
                                    >
                                        <span className="flex min-w-0 items-center gap-2">
                                            <span
                                                className="h-3.5 w-3.5 rounded border border-white/20 shadow-[0_0_0_1px_rgba(0,0,0,0.25)_inset]"
                                                style={{ backgroundColor: LAYER_COLORS[labelIndex] }}
                                                aria-hidden="true"
                                            />
                                            <span className="truncate text-xs font-semibold capitalize">
                                                {label}
                                            </span>
                                        </span>
                                        <span className={`text-[10px] font-mono ${isOn ? 'text-[var(--accent-strong)]' : 'text-[var(--text-secondary)]'}`}>
                                            {isOn ? 'ON' : 'OFF'}
                                        </span>
                                    </button>
                                );
                            })}
                        </div>
                    </div>
                </div>
            </div>
        </div>,
        document.body
    );
}
