import { useEffect, useRef, useState } from 'react';
import { X } from 'lucide-react';
import { GraphTransform } from '../../types';
import { GraphTransformPanel } from './GraphTransformPanel';

interface GraphTransformModalProps {
    title: string;
    open: boolean;
    transform: GraphTransform | null;
    onClose: () => void;
    onUpdate: (updates: Partial<GraphTransform>) => void;
}

export function GraphTransformModal({
    title,
    open,
    transform,
    onClose,
    onUpdate,
}: GraphTransformModalProps) {
    const dialogRef = useRef<HTMLDivElement>(null);
    const dragStateRef = useRef<{
        startX: number;
        startY: number;
        originX: number;
        originY: number;
    } | null>(null);
    const [offset, setOffset] = useState({ x: 0, y: 0 });
    const [dragging, setDragging] = useState(false);

    useEffect(() => {
        if (!open) {
            setOffset({ x: 0, y: 0 });
            setDragging(false);
            dragStateRef.current = null;
        }
    }, [open, title]);

    useEffect(() => {
        const onMove = (event: PointerEvent) => {
            const drag = dragStateRef.current;
            if (!drag) return;
            setOffset({
                x: drag.originX + (event.clientX - drag.startX),
                y: drag.originY + (event.clientY - drag.startY),
            });
        };

        const onUp = () => {
            dragStateRef.current = null;
            setDragging(false);
        };

        window.addEventListener('pointermove', onMove);
        window.addEventListener('pointerup', onUp);
        return () => {
            window.removeEventListener('pointermove', onMove);
            window.removeEventListener('pointerup', onUp);
        };
    }, []);

    if (!open || !transform) return null;

    return (
        <div className="fixed inset-0 z-[80] flex items-center justify-center bg-black/60 px-4 py-6 backdrop-blur-sm">
            <button
                type="button"
                aria-label="Close graph transform dialog"
                className="absolute inset-0 cursor-default"
                onClick={onClose}
            />

            <div
                ref={dialogRef}
                role="dialog"
                aria-modal="true"
                className={`relative z-[81] w-full max-w-3xl overflow-hidden rounded-xl border border-white/10 bg-[rgba(8,19,29,0.96)] shadow-2xl ${dragging ? 'cursor-grabbing' : 'cursor-default'}`}
                style={{
                    position: 'fixed',
                    left: '50%',
                    top: '50%',
                    transform: `translate(-50%, -50%) translate(${offset.x}px, ${offset.y}px)`,
                }}
                onClick={(e) => e.stopPropagation()}
            >
                <div
                    className="flex cursor-grab items-start justify-between gap-3 border-b border-white/10 px-4 py-3 active:cursor-grabbing"
                    onPointerDown={(e) => {
                        if (e.button !== 0) return;
                        const current = offset;
                        dragStateRef.current = {
                            startX: e.clientX,
                            startY: e.clientY,
                            originX: current.x,
                            originY: current.y,
                        };
                        setDragging(true);
                        (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId);
                    }}
                >
                    <div className="min-w-0">
                        <p className="panel-title">Graph Transform</p>
                        <p className="truncate text-sm text-[var(--text-secondary)]">{title}</p>
                        <p className="text-[10px] text-[var(--text-secondary)]">Drag this header to move the window.</p>
                    </div>
                    <button
                        type="button"
                        onClick={onClose}
                        className="btn-icon btn-icon-danger"
                        title="Close"
                    >
                        <X size={15} />
                    </button>
                </div>

                <div className="max-h-[78vh] overflow-y-auto p-4">
                    <GraphTransformPanel transform={transform} onUpdate={onUpdate} />
                </div>
            </div>
        </div>
    );
}
