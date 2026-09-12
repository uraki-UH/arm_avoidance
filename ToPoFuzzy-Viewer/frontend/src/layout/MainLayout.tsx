import React, { useEffect, useRef, useState } from 'react';
import { ChevronLeft, SlidersHorizontal, ChevronDown } from 'lucide-react';

interface MainLayoutProps {
    sidebar: React.ReactNode;
    isSidebarOpen: boolean;
    children: React.ReactNode;
}

export function MainLayout({ sidebar, isSidebarOpen, children }: MainLayoutProps) {
    return (
        <div className="relative flex h-screen w-full overflow-hidden text-[var(--text-primary)]">
            <div className="pointer-events-none absolute inset-0 bg-[radial-gradient(circle_at_0%_0%,rgba(41,200,168,0.12),transparent_38%),radial-gradient(circle_at_100%_0%,rgba(36,159,229,0.14),transparent_42%)]" />
            {sidebar}

            <main
                className={`relative z-10 flex min-h-0 flex-1 transition-[margin] duration-300 ease-out ${isSidebarOpen ? 'md:ml-[var(--sidebar-width)]' : 'ml-0'
                    }`}
            >
                <div className="relative min-h-0 flex-1">{children}</div>
            </main>
        </div>
    );
}

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

interface Tab {
    id: string;
    label: string;
    icon?: React.ReactNode;
    content: React.ReactNode;
    onActivate?: () => void;
}

interface TabsProps {
    tabs: Tab[];
    defaultTabId?: string;
    className?: string;
}

export function Tabs({ tabs, defaultTabId, className = '' }: TabsProps) {
    const [activeTabId, setActiveTabId] = useState(defaultTabId || tabs[0]?.id);
    const activeTab = tabs.find(t => t.id === activeTabId);

    return (
        <div className={`flex h-full min-h-0 flex-col ${className}`}>
            <div className="sticky top-0 z-20 -mx-1 mb-3 rounded-xl bg-[rgba(8,19,29,0.7)] px-1 pb-2 pt-3 backdrop-blur-sm">
                <div
                    className="grid gap-1.5"
                    style={{ gridTemplateColumns: `repeat(${tabs.length}, minmax(0, 1fr))` }}
                >
                    {tabs.map(tab => (
                        <button
                            key={tab.id}
                            onClick={() => {
                                setActiveTabId(tab.id);
                                tab.onActivate?.();
                            }}
                            className={`relative flex min-h-[40px] items-center justify-center gap-1 rounded-lg px-2 py-2 text-[11px] font-semibold uppercase tracking-[0.08em] transition-all ${activeTabId === tab.id
                                ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)] ring-1 ring-[var(--accent-color)]/40'
                                : 'text-[var(--text-secondary)] hover:bg-white/5 hover:text-[var(--text-primary)]'
                                }`}
                        >
                            {tab.icon && <span className="shrink-0">{tab.icon}</span>}
                            <span className="truncate">{tab.label}</span>
                        </button>
                    ))}
                </div>
            </div>

            <div className="min-h-0 flex-1 overflow-y-auto pb-3 pr-1 scrollbar-thin">
                {activeTab?.content}
            </div>
        </div>
    );
}

interface CollapsibleSectionProps {
    title: string;
    icon?: React.ReactNode;
    children: React.ReactNode;
    defaultOpen?: boolean;
    className?: string;
    headerClassName?: string;
    titleClassName?: string;
}

export const CollapsibleSection: React.FC<CollapsibleSectionProps> = ({
    title,
    icon,
    children,
    defaultOpen = false,
    className = "",
    headerClassName = "",
    titleClassName = "",
}) => {
    const [isOpen, setIsOpen] = useState(defaultOpen);
    const contentRef = useRef<HTMLDivElement>(null);
    const [maxHeight, setMaxHeight] = useState(defaultOpen ? 'none' : '0px');

    useEffect(() => {
        const el = contentRef.current;
        if (!el) return;

        const updateHeight = () => {
            if (isOpen) {
                setMaxHeight(`${el.scrollHeight}px`);
            } else {
                setMaxHeight('0px');
            }
        };

        updateHeight();
        window.addEventListener('resize', updateHeight);

        let observer: ResizeObserver | null = null;
        if (isOpen && typeof ResizeObserver !== 'undefined') {
            observer = new ResizeObserver(() => {
                setMaxHeight(`${el.scrollHeight}px`);
            });
            observer.observe(el);
        }

        return () => {
            window.removeEventListener('resize', updateHeight);
            observer?.disconnect();
        };
    }, [isOpen, children]);

    return (
        <section className={`surface-soft overflow-hidden ${className}`}>
            <button
                onClick={() => setIsOpen(!isOpen)}
                className={`flex w-full items-center justify-between px-4 py-3 text-left transition-colors ${headerClassName} ${isOpen ? 'bg-white/5' : 'hover:bg-white/5'
                    }`}
                aria-expanded={isOpen}
            >
                <div className={`flex items-center gap-2 text-sm font-semibold text-[var(--text-primary)] ${titleClassName}`}>
                    {icon && <span className="text-[var(--text-secondary)]">{icon}</span>}
                    <span>{title}</span>
                </div>
                <div className={`text-[var(--text-secondary)] transition-transform duration-200 ${isOpen ? 'rotate-180' : ''}`}>
                    <ChevronDown size={16} />
                </div>
            </button>
            <div
                className="overflow-hidden transition-[max-height,opacity] duration-300 ease-out"
                style={{ maxHeight, opacity: isOpen ? 1 : 0 }}
            >
                <div ref={contentRef} className="space-y-4 px-4 pb-4">
                    {children}
                </div>
            </div>
        </section>
    );
};
