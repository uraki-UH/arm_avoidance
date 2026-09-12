import { Children, useCallback, useEffect, useRef, useState } from 'react';
import type { PointerEvent, ReactNode } from 'react';
import { HandGrab } from 'lucide-react';
import { get_node_label_overlap_target, insert_node_label } from './graphLayerSettings';

interface label_priority_list_props {
    ids: string[];
    names: string[];
    title: string;
    children: ReactNode;
    on_reorder: (ids: string[]) => void;
}

interface label_drag {
    id: string;
    pointer_id: number;
    element: Element;
    start_x: number;
    start_y: number;
    timer: ReturnType<typeof setTimeout>;
    is_active: boolean;
}

/** 項目枠全体の長押し・押下移動による挿入。短いクリックは既存操作へ委譲。 */
export function LabelPriorityList({ ids, names, title, children, on_reorder }: label_priority_list_props) {
    const list = useRef<HTMLDivElement>(null);
    const pending = useRef<label_drag | null>(null);
    const suppress_click = useRef(false);
    const [pressed_id, set_pressed_id] = useState<string | null>(null);
    const [drag, set_drag] = useState<{ id: string; before_id: string | null; x: number; y: number } | null>(null);

    const release = useCallback(() => {
        const current = pending.current;
        pending.current = null;
        if (current) {
            clearTimeout(current.timer);
            if (current.element.hasPointerCapture(current.pointer_id)) current.element.releasePointerCapture(current.pointer_id);
        }
    }, []);
    const cancel = useCallback(() => { release(); set_drag(null); set_pressed_id(null); }, [release]);
    const order_key = ids.join('\0');
    useEffect(() => { cancel(); }, [order_key, cancel]);
    useEffect(() => {
        const on_key = (event: KeyboardEvent) => { if (event.key === 'Escape') cancel(); };
        window.addEventListener('blur', cancel);
        window.addEventListener('keydown', on_key);
        return () => {
            window.removeEventListener('blur', cancel);
            window.removeEventListener('keydown', on_key);
            release();
        };
    }, [cancel, release]);

    const destination = (id: string, x: number, y: number) => {
        const element = list.current;
        if (!element) return undefined;
        const bounds = element.getBoundingClientRect();
        if (x < bounds.left || x > bounds.right || y < bounds.top - 8 || y > bounds.bottom + 8) return undefined;
        for (const row of Array.from(element.children)) {
            const row_id = (row as HTMLElement).dataset.priorityId;
            if (!row_id || row_id === id) continue;
            const rect = row.getBoundingClientRect();
            if (y < rect.top) return row_id;
            if (y <= rect.bottom) return get_node_label_overlap_target(ids, id, row_id);
        }
        return null;
    };
    const start = (event: PointerEvent<HTMLDivElement>, id: string) => {
        if (event.button !== 0 || !event.isPrimary || pending.current) return;
        suppress_click.current = false;
        const element = event.target as Element;
        if (element.closest('[data-priority-id]') !== event.currentTarget ||
            element.closest('input, select, textarea, [data-priority-action], fieldset:disabled')) return;
        // 短いクリックの宛先を維持するため、押下元要素での捕捉。
        element.setPointerCapture(event.pointerId);
        set_pressed_id(id);
        const x = event.clientX, y = event.clientY;
        const timer = setTimeout(() => {
            const current = pending.current;
            if (!current) return;
            current.is_active = true;
            suppress_click.current = true;
            set_drag({ id, before_id: destination(id, x, y) ?? null, x, y });
        }, 350);
        pending.current = { id, pointer_id: event.pointerId, element, start_x: x, start_y: y, timer, is_active: false };
    };
    const move = (event: PointerEvent<HTMLDivElement>) => {
        const current = pending.current;
        if (!current || event.pointerId !== current.pointer_id) return;
        if (!current.is_active) {
            if (Math.hypot(event.clientX - current.start_x, event.clientY - current.start_y) <= 8) return;
            // 長押し成立前の移動もドラッグ開始。自然な押下移動の取りこぼしを回避。
            clearTimeout(current.timer);
            current.is_active = true;
            suppress_click.current = true;
        }
        event.preventDefault();
        const before_id = destination(current.id, event.clientX, event.clientY);
        set_drag(before_id === undefined ? null : { id: current.id, before_id, x: event.clientX, y: event.clientY });
    };
    const finish = (event: PointerEvent<HTMLDivElement>) => {
        const current = pending.current;
        if (!current || event.pointerId !== current.pointer_id) return;
        const before_id = destination(current.id, event.clientX, event.clientY);
        cancel();
        if (current.is_active && before_id !== undefined) on_reorder(insert_node_label(ids, current.id, before_id));
    };
    const items = Children.toArray(children);
    return (
        <div ref={list} role="group" aria-label={title} className="relative space-y-2"
            onClickCapture={(event) => {
                if (suppress_click.current && event.detail > 0) {
                    event.preventDefault(); event.stopPropagation(); suppress_click.current = false;
                }
            }}
            onPointerMove={move} onPointerUp={finish}
            onPointerCancel={cancel} onLostPointerCapture={() => { if (pending.current) cancel(); }}>
            {ids.map((id, idx) => (
                <div key={id} data-priority-id={id} role="group" aria-label={names[idx] + 'を長押しして移動'}
                    title="枠内を押したままドラッグ、または長押しで移動" onPointerDown={(event) => start(event, id)}
                    onContextMenu={(event) => event.preventDefault()}
                    className={`relative touch-none select-none rounded-md border cursor-grab active:cursor-grabbing transition-colors ${pressed_id === id
                        ? 'border-sky-400 bg-sky-950/50 ring-1 ring-sky-400'
                        : 'border-slate-600 bg-slate-800/40 hover:border-slate-400 hover:bg-slate-700/40'}`}>
                    {drag?.before_id === id && <div data-insertion-marker className="pointer-events-none absolute -top-1 left-0 right-0 h-0.5 bg-[var(--accent-color)]" />}
                    <div className={drag?.id === id ? 'opacity-40' : ''}>
                        <span data-drag-indicator aria-hidden="true" className="pointer-events-none absolute left-1 top-2 text-[var(--text-secondary)]">
                            <HandGrab size={16} />
                        </span>
                        <div className="min-w-0">{items[idx]}</div>
                        <button type="button" data-priority-action aria-label={names[idx] + 'を先頭へ移動'} title="先頭へ移動" disabled={idx === 0}
                            onClick={() => on_reorder(insert_node_label(ids, id, ids[0]))}
                            className="absolute right-0 top-0 min-h-[32px] w-11 rounded border border-slate-500 bg-slate-700 px-1.5 text-[10px] text-slate-100 transition-colors enabled:hover:border-sky-400 enabled:hover:bg-slate-600 focus-visible:outline focus-visible:outline-2 focus-visible:outline-sky-400 disabled:cursor-default disabled:border-slate-600 disabled:bg-slate-800 disabled:text-slate-400">先頭へ</button>
                    </div>
                </div>
            ))}
            {drag && drag.before_id === null && <div data-insertion-marker className="pointer-events-none absolute -bottom-1 left-0 right-0 h-0.5 bg-[var(--accent-color)]" />}
            {drag && <div role="status" className="pointer-events-none fixed z-[10000] rounded border border-[var(--accent-color)] bg-[#0c141d] px-3 py-2 text-xs text-white shadow-xl"
                style={{ left: drag.x + 12, top: drag.y + 12 }}>{names[ids.indexOf(drag.id)]}</div>}
        </div>
    );
}
