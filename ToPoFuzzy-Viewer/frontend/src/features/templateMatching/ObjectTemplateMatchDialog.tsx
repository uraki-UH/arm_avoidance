import { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import { Maximize2, ScanSearch, X } from 'lucide-react';
import { ObjectTemplateMatchTuner } from './ObjectTemplateMatchTuner';
import {
    TemplateMatchConfig,
    TemplateMatchConfigResult,
    TemplateMatchTargets,
} from './types';

interface ObjectTemplateMatchDialogLauncherProps {
    isConnected: boolean;
    getConfig: (targets: TemplateMatchTargets) => Promise<TemplateMatchConfigResult>;
    applyConfig: (config: TemplateMatchConfig) => Promise<TemplateMatchConfigResult>;
}

export function ObjectTemplateMatchDialogLauncher(props: ObjectTemplateMatchDialogLauncherProps) {
    const [open, setOpen] = useState(true);

    useEffect(() => {
        if (!open) return;
        const handleKeyDown = (event: KeyboardEvent) => {
            if (event.key === 'Escape') setOpen(false);
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => window.removeEventListener('keydown', handleKeyDown);
    }, [open]);

    return (
        <>
            <div className="surface-soft flex items-center justify-between gap-3 p-3">
                <div className="flex min-w-0 items-center gap-2 text-sm font-semibold text-[var(--text-primary)]">
                    <ScanSearch size={16} className="shrink-0 text-[var(--accent-strong)]" />
                    <span className="truncate">Object template matching</span>
                </div>
                <button
                    className="btn-primary flex h-9 shrink-0 items-center gap-2 px-3 text-xs font-semibold"
                    onClick={() => setOpen(true)}
                >
                    <Maximize2 size={14} />
                    Open
                </button>
            </div>

            {open && createPortal(
                <div
                    className="fixed inset-0 z-[10000] bg-black/70 p-2 backdrop-blur-sm md:p-4"
                    onPointerDown={() => setOpen(false)}
                    role="presentation"
                >
                    <section
                        className="surface-panel flex h-full min-h-0 w-full flex-col overflow-hidden shadow-2xl ring-1 ring-white/10"
                        onPointerDown={(event) => event.stopPropagation()}
                        role="dialog"
                        aria-modal="true"
                        aria-labelledby="object-template-match-dialog-title"
                    >
                        <header className="flex h-14 shrink-0 items-center justify-between border-b border-white/10 bg-black/30 px-4 md:px-5">
                            <div className="flex min-w-0 items-center gap-2">
                                <ScanSearch size={18} className="shrink-0 text-[var(--accent-strong)]" />
                                <h2 id="object-template-match-dialog-title" className="truncate text-sm font-bold text-[var(--text-primary)]">
                                    Object Template Matching
                                </h2>
                            </div>
                            <button
                                className="flex h-9 w-9 items-center justify-center rounded-md text-[var(--text-secondary)] transition-colors hover:bg-white/10 hover:text-[var(--text-primary)]"
                                onClick={() => setOpen(false)}
                                title="Close"
                                aria-label="Close object template matching settings"
                            >
                                <X size={18} />
                            </button>
                        </header>

                        <div className="min-h-0 flex-1 p-3 md:p-4">
                            <ObjectTemplateMatchTuner {...props} layout="dialog" />
                        </div>
                    </section>
                </div>,
                document.body,
            )}
        </>
    );
}
