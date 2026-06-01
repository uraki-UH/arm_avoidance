import React from 'react';
import { ChevronLeft, SlidersHorizontal } from 'lucide-react';

interface SidebarProps {
    isOpen: boolean;
    onToggle: () => void;
    children: React.ReactNode;
    title?: string;
}

export const Sidebar: React.FC<SidebarProps> = ({
    isOpen,
    onToggle,
    children,
    title = "ToPo-FUZZY Viewer"
}) => {
    return (
        <>
            {isOpen && (
                <div
                    className="fixed inset-0 z-30 bg-black/55 backdrop-blur-sm md:hidden"
                    onClick={onToggle}
                />
            )}

            {!isOpen && (
                <button
                    onClick={onToggle}
                    className="surface-panel fixed left-4 top-4 z-50 flex items-center gap-2 px-3 py-2 text-sm font-semibold text-[var(--text-primary)] hover:bg-white/10"
                    aria-label="Open Sidebar"
                >
                    <SlidersHorizontal size={18} />
                    <span className="hidden sm:inline">Control Panel</span>
                </button>
            )}

            <div
                className={`fixed inset-y-0 left-0 z-40 w-[var(--sidebar-width)] transform px-3 py-3 transition-transform duration-300 ease-out ${isOpen ? 'translate-x-0' : '-translate-x-full'
                    }`}
            >
                <div className="surface-panel flex h-full min-h-0 w-full flex-col overflow-hidden">
                    <header className="flex items-center justify-between gap-3 border-b border-white/10 px-4 py-2.5">
                        <h1 className="min-w-0 truncate text-xl font-bold text-[var(--brand-color)]">
                            {title}
                        </h1>
                        <button
                            onClick={onToggle}
                            className="btn-secondary inline-flex h-8 w-8 shrink-0 items-center justify-center p-0 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                            aria-label="Close Sidebar"
                        >
                            <ChevronLeft size={18} />
                        </button>
                    </header>

                    <div className="min-h-0 flex-1 px-3 pb-3 pt-2">
                        {children}
                    </div>

                </div>
            </div>
        </>
    );
};
