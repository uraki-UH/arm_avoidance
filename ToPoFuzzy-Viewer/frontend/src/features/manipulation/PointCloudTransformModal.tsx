import { X } from 'lucide-react';
import { PointCloudData } from '../../types';
import { PointCloudTransformPanel } from './PointCloudTransformPanel';

interface PointCloudTransformModalProps {
    cloudData: PointCloudData | null;
    open: boolean;
    onClose: () => void;
    onUpdate: (updates: Partial<PointCloudData>) => void;
}

export function PointCloudTransformModal({
    cloudData,
    open,
    onClose,
    onUpdate,
}: PointCloudTransformModalProps) {
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
                className="relative z-[81] w-full max-w-3xl overflow-hidden rounded-xl border border-white/10 bg-[rgba(8,19,29,0.96)] shadow-2xl"
                onClick={(e) => e.stopPropagation()}
            >
                <div className="flex items-start justify-between gap-3 border-b border-white/10 px-4 py-3">
                    <div className="min-w-0">
                        <p className="panel-title">Point Cloud Transform</p>
                        <p className="truncate text-sm text-[var(--text-secondary)]">{cloudData.name}</p>
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
                    <PointCloudTransformPanel cloudData={cloudData} onUpdate={onUpdate} />
                </div>
            </div>
        </div>
    );
}
