import { useEffect } from 'react';
import { createPortal } from 'react-dom';
import { ScanSearch, X } from 'lucide-react';
import { ObjectTemplateMatchTuner } from './ObjectTemplateMatchTuner';
import {
    TemplateMatchConfig,
    TemplateMatchConfigResult,
    TemplateMatchTargets,
} from './types';

interface ObjectTemplateMatchDialogProps {
    open: boolean;
    onClose: () => void;
    isConnected: boolean;
    getConfig: (targets: TemplateMatchTargets) => Promise<TemplateMatchConfigResult>;
    applyConfig: (config: TemplateMatchConfig) => Promise<TemplateMatchConfigResult>;
}

export function ObjectTemplateMatchDialog({ open, onClose, ...tunerProps }: ObjectTemplateMatchDialogProps) {
    useEffect(() => {
        if (!open) return;
        const handleKeyDown = (event: KeyboardEvent) => {
            if (event.key === 'Escape') onClose();
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [onClose, open]);

    if (!open) return null;

    return createPortal(
        <div
            className="fixed inset-0 z-[10000] bg-black/70 p-2 backdrop-blur-sm md:p-4"
            onPointerDown={onClose}
            role="presentation"
        >
            <section
                className="surface-panel flex h-full min-h-0 w-full flex-col overflow-hidden shadow-2xl ring-1 ring-white/10"
                onPointerDown={(event) => event.stopPropagation()}
                role="dialog"
                aria-modal="true"
                aria-labelledby="object-template-match-dialog-title"
            >
                <header className="flex h-20 shrink-0 items-center justify-between border-b border-white/10 bg-black/30 px-4 md:px-5">
                    <div className="flex min-w-0 items-center gap-2">
                        <ScanSearch size={28} className="shrink-0 text-[var(--accent-strong)]" />
                        <h2 id="object-template-match-dialog-title" className="truncate text-[28px] font-bold text-[var(--text-primary)]">
                            Object Template Matching
                        </h2>
                    </div>
                    <button
                        className="flex h-12 w-12 items-center justify-center rounded-md text-[var(--text-secondary)] transition-colors hover:bg-white/10 hover:text-[var(--text-primary)]"
                        onClick={onClose}
                        title="Close"
                        aria-label="Close object template matching settings"
                    >
                        <X size={28} />
                    </button>
                </header>

                <div className="min-h-0 flex-1 p-3 md:p-4">
                    <ObjectTemplateMatchTuner {...tunerProps} layout="dialog" />
                </div>
            </section>
        </div>,
        document.body,
    );
}
