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
        <div className="fixed top-20 left-4 z-[9999] w-72 animate-in fade-in slide-in-from-left-2 duration-300">
            <div className="surface-panel flex flex-col overflow-hidden shadow-2xl ring-1 ring-white/10">
                <div className="flex items-center justify-between border-b border-white/5 bg-black/40 px-4 py-3">
                    <div className="min-w-0">
                        <h2 className="text-sm font-bold text-white leading-tight">{title}</h2>
                        {subtitle && <p className="mt-0.5 truncate text-[10px] text-gray-400 font-mono opacity-70">{subtitle}</p>}
                    </div>
                    <button
                        onClick={onClose}
                        className="flex h-7 w-7 items-center justify-center rounded-md text-gray-400 hover:bg-white/10 hover:text-white transition-all"
                    >
                        <X size={16} />
                    </button>
                </div>

                <div className="max-h-[70vh] overflow-y-auto p-4 custom-scrollbar bg-[#0c141d]/50">
                    <GenericTransformPanel 
                        title="Manual Transform"
                        transform={transform} 
                        onUpdate={onUpdate} 
                        onReset={onReset}
                    />
                </div>

                <div className="border-t border-white/5 bg-black/30 px-4 py-2">
                    <p className="text-[9px] text-gray-500 italic leading-tight">
                        * Real-time spatial transformation for current layer.
                    </p>
                </div>
            </div>
        </div>
    );
}
