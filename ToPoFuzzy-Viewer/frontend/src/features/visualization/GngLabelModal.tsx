import { createPortal } from 'react-dom';
import { ChevronDown, Layers, X } from 'lucide-react';
import { LAYER_COLORS, LAYER_LABELS } from '../../types';
import { get_node_label_groups, reorder_node_label_subset, node_label_definitions, normalize_node_label_settings } from './graphLayerSettings';
import type { node_label_options } from './graphLayerSettings';
import { LabelPriorityList } from './LabelPriorityList';

interface GngLabelModalProps {
    open: boolean;
    title?: string;
    subtitle?: string;
    label_settings?: node_label_options;
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
        node_label_visibility?: Record<string, boolean>;
        node_label_priority?: string[];
        node_label_colors?: Record<string, string>;
        visibleLabels?: {
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
    label_settings,
    visibleLabels,
    onClose,
    onUpdate,
}: GngLabelModalProps) {
    if (!open) return null;
    const settings = normalize_node_label_settings(label_settings);
    const ordered_labels = get_node_label_groups(settings.node_label_priority);

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
                            <div role="group" aria-label="重複ラベル"
                                className="mb-2 flex flex-col gap-2 rounded-md border border-white/10 bg-black/20 px-3 py-2">
                                <div className="flex flex-wrap items-baseline gap-x-2 gap-y-1">
                                    <span className="shrink-0 text-xs font-semibold text-[var(--text-primary)]">重複ラベル</span>
                                    <span className="text-[10px] text-[var(--text-secondary)]">複数選択可・上ほど色を優先</span>
                                </div>
                                <LabelPriorityList ids={ordered_labels.map((item) => item.id)} names={ordered_labels.map((item) => item.name)}
                                    title="重複ラベルの優先順位"
                                    on_reorder={(ids) => onUpdate({ node_label_priority: reorder_node_label_subset(settings.node_label_priority, ids) })}>
                                    {ordered_labels.map((item) => {
                                        const is_enabled = settings.node_label_visibility[item.id];
                                        const children = settings.node_label_priority
                                            .map((id) => node_label_definitions.find((definition) => definition.id === id)!)
                                            .filter((definition) => definition.parent_id === item.id);
                                        return (
                                            <div key={item.id}>
                                                <div className="flex min-h-[32px] items-center gap-1 pl-7 pr-12">
                                                    <button
                                                        aria-label={item.name + 'の色分け'}
                                                        aria-pressed={is_enabled}
                                                        onClick={() => onUpdate({ node_label_visibility: {
                                                            ...settings.node_label_visibility, [item.id]: !is_enabled,
                                                        } })}
                                                        className={`flex flex-1 items-center gap-2 rounded-md border px-3 py-1 text-[10px] font-semibold transition-colors ${is_enabled
                                                            ? 'border-[var(--accent-color)]/30 bg-[var(--accent-soft)] text-[var(--text-primary)]'
                                                            : 'border-white/10 bg-black/20 text-[var(--text-secondary)] opacity-75 hover:bg-white/10'}`}
                                                    >
                                                        <span className="h-2.5 w-2.5 rounded" style={{ backgroundColor: item.color }} aria-hidden="true" />
                                                        <span className="flex-1 text-left">{item.name}</span>
                                                        {is_enabled ? 'ON' : 'OFF'}
                                                    </button>
                                                </div>
                                                {children.length > 0 && (
                                                    <details className="group mt-2 rounded-md border border-white/10 bg-black/20 text-[10px]">
                                                        <summary className="flex min-h-[44px] w-full cursor-pointer list-none items-center justify-between gap-2 rounded-md px-3 py-3 text-xs font-semibold text-[var(--text-primary)] transition-colors hover:bg-white/10 focus-visible:outline focus-visible:outline-2 focus-visible:outline-[var(--accent-color)] [&::-webkit-details-marker]:hidden">
                                                            <span>原因別の表示・色</span>
                                                            <ChevronDown size={16} className="shrink-0 transition-transform group-open:rotate-180" aria-hidden="true" />
                                                        </summary>
                                                        <fieldset disabled={!is_enabled} className="space-y-2 px-2 pb-2 disabled:opacity-40">
                                                            <legend className="sr-only">{item.name}の原因別設定</legend>
                                                            <p className="text-[var(--text-secondary)]">複数選択可。証拠が重なる場合は上の色を優先。</p>
                                                            <LabelPriorityList ids={children.map((child) => child.id)} names={children.map((child) => child.name)}
                                                                title={item.name + 'の原因別優先順位'}
                                                                on_reorder={(ids) => onUpdate({ node_label_priority: reorder_node_label_subset(settings.node_label_priority, ids) })}>
                                                                {children.map((child) => (
                                                                    <div key={child.id} className="flex min-h-[32px] items-center gap-2 pl-7 pr-12">
                                                                        <label className="flex flex-1 cursor-pointer items-center gap-2 text-[var(--text-primary)]">
                                                                            <input type="checkbox" checked={settings.node_label_visibility[child.id]}
                                                                                onChange={(event) => onUpdate({ node_label_visibility: {
                                                                                    ...settings.node_label_visibility, [child.id]: event.target.checked,
                                                                                } })} />
                                                                            {child.name}
                                                                        </label>
                                                                        <input type="color" aria-label={child.name + 'の色'}
                                                                            value={settings.node_label_colors[child.id]}
                                                                            onChange={(event) => onUpdate({ node_label_colors: {
                                                                                ...settings.node_label_colors, [child.id]: event.target.value,
                                                                            } })}
                                                                            className="h-6 w-8 cursor-pointer rounded border border-white/10 bg-transparent p-0" />
                                                                    </div>
                                                                ))}
                                                            </LabelPriorityList>
                                                        </fieldset>
                                                    </details>
                                                )}
                                            </div>
                                        );
                                    })}
                                </LabelPriorityList>
                            </div>
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
