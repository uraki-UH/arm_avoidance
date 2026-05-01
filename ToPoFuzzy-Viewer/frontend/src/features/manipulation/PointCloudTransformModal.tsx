import { useEffect, useRef, useState } from 'react';
import { X } from 'lucide-react';
import { PointCloudData } from '../../types';
import { PointCloudTransformPanel } from './PointCloudTransformPanel';

interface PointCloudTransformModalProps {
    cloudData: PointCloudData | null;
    open: boolean;
    onClose: () => void;
    onUpdate: (updates: Partial<PointCloudData>) => void;
    onReset: () => void;
}

export function PointCloudTransformModal({
    cloudData,
    open,
    onClose,
    onUpdate,
    onReset,
}: PointCloudTransformModalProps) {
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
    }, [open, cloudData?.id]);

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

    if (!open || !cloudData) return null;

    return (
        <div className="fixed inset-0 z-[80] flex items-center justify-center bg-black/60 px-4 py-6 backdrop-blur-sm">
            <button
                type="button"
                aria-label="Close point cloud transform dialog"
                className="absolute inset-0 cursor-default"
                onClick={onClose}
            />

            <div
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
                        <p className="panel-title">Point Cloud Transform</p>
                        <p className="truncate text-sm text-[var(--text-secondary)]">{cloudData.name}</p>
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
                    <PointCloudTransformPanel cloudData={cloudData} onUpdate={onUpdate} onReset={onReset} />
                </div>
            </div>
        </div>
    );
}
