import { X } from 'lucide-react';
import { Transform } from '../../types';
import { GenericTransformPanel } from './GenericTransformPanel';

interface GenericTransformModalProps {
    title: string;
    subtitle?: string;
    open: boolean;
    transform: Transform | null;
    onClose: () => void;
    onUpdate: (updates: Partial<Transform>) => void;
    onReset?: () => void;
}

export function GenericTransformModal({
    title,
    subtitle,
    open,
    transform,
    onClose,
    onUpdate,
    onReset,
}: GenericTransformModalProps) {
    if (!open || !transform) return null;

    return (
        <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/70 px-4 py-6 backdrop-blur-sm transition-all animate-in fade-in">
            <div className="absolute inset-0 cursor-default" onClick={onClose} />

            <div
                role="dialog"
                className="relative z-[101] w-full max-w-lg overflow-hidden rounded-2xl border border-white/10 bg-[#0c141d] shadow-2xl flex flex-col max-h-[90vh] animate-in zoom-in-95 duration-200"
                onClick={(e) => e.stopPropagation()}
            >
                <div className="flex items-center justify-between border-b border-white/5 bg-white/5 px-5 py-4">
                    <div className="min-w-0">
                        <h2 className="text-base font-semibold text-white leading-tight">{title}</h2>
                        {subtitle && <p className="mt-1 truncate text-xs text-gray-400">{subtitle}</p>}
                    </div>
                    <button
                        onClick={onClose}
                        className="flex h-8 w-8 items-center justify-center rounded-full text-gray-400 hover:bg-white/10 hover:text-white transition-all"
                    >
                        <X size={18} />
                    </button>
                </div>

                <div className="flex-1 overflow-y-auto p-5 custom-scrollbar">
                    <GenericTransformPanel 
                        title="Transformation Settings"
                        transform={transform} 
                        onUpdate={onUpdate} 
                        onReset={onReset}
                        description="Adjust the spatial orientation for visualization."
                    />
                </div>

                <div className="border-t border-white/5 bg-black/20 px-5 py-3">
                    <p className="text-[10px] text-gray-500 italic">
                        * Changes are applied instantly to the visualization.
                    </p>
                </div>
            </div>
        </div>
    );
}
