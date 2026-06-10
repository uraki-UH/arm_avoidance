import { X } from 'lucide-react';
import { LayerSettings } from '../../types';

interface GngLabelModalProps {
    open: boolean;
    title: string;
    visibleLabels: NonNullable<LayerSettings['visibleLabels']>;
    onClose: () => void;
    onUpdate: (updates: Partial<LayerSettings>) => void;
}

const LABELS: Array<keyof NonNullable<LayerSettings['visibleLabels']>> = [0, 1, 2, 3, 4, 5];

export function GngLabelModal({ open, title, visibleLabels, onClose, onUpdate }: GngLabelModalProps) {
    if (!open) return null;

    const activeCount = LABELS.filter((label) => visibleLabels[label]).length;

    const setAll = (value: boolean) => {
        onUpdate({
            visibleLabels: {
                0: value,
                1: value,
                2: value,
                3: value,
                4: value,
                5: value,
            },
        });
    };

    return (
        <div className="fixed inset-0 z-[9999]" onClick={onClose}>
            <div
                className="fixed left-4 top-20 w-[360px] animate-in fade-in slide-in-from-left-2 duration-300"
                onClick={(e) => e.stopPropagation()}
            >
                <div className="surface-panel flex max-h-[80vh] flex-col overflow-hidden shadow-2xl ring-1 ring-white/10">
                    <div className="flex items-center justify-between border-b border-white/5 bg-black/40 px-4 py-3">
                        <div className="min-w-0">
                            <h2 className="truncate text-sm font-bold text-white leading-tight">{title}</h2>
                            <p className="mt-0.5 text-[10px] font-mono text-gray-400 opacity-70">
                                {activeCount}/6 labels visible
                            </p>
                        </div>
                        <button
                            onClick={onClose}
                            className="flex h-7 w-7 items-center justify-center rounded-md text-gray-400 transition-all hover:bg-white/10 hover:text-white"
                        >
                            <X size={16} />
                        </button>
                    </div>

                    <div className="flex-1 overflow-y-auto bg-[#0c141d]/50 p-4">
                        <div className="grid grid-cols-2 gap-2">
                            <button
                                type="button"
                                onClick={() => setAll(true)}
                                className="entity-btn justify-center px-3 py-2 text-[10px]"
                            >
                                SHOW ALL
                            </button>
                            <button
                                type="button"
                                onClick={() => setAll(false)}
                                className="entity-btn justify-center px-3 py-2 text-[10px]"
                            >
                                HIDE ALL
                            </button>
                        </div>

                        <div className="mt-4 space-y-2">
                            {LABELS.map((label) => {
                                const checked = visibleLabels[label];
                                return (
                                    <label
                                        key={label}
                                        className="flex cursor-pointer items-center justify-between rounded-md border border-white/5 bg-black/15 px-3 py-2 text-sm text-white transition-colors hover:bg-white/5"
                                    >
                                        <span className="font-mono text-xs uppercase tracking-wider text-[var(--text-secondary)]">
                                            Label {label}
                                        </span>
                                        <input
                                            type="checkbox"
                                            checked={checked}
                                            onChange={() =>
                                                onUpdate({
                                                    visibleLabels: {
                                                        ...visibleLabels,
                                                        [label]: !checked,
                                                    },
                                                })
                                            }
                                            className="h-4 w-4 accent-[var(--accent-strong)]"
                                        />
                                    </label>
                                );
                            })}
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
}
