import { useState } from 'react';

interface Tab {
    id: string;
    label: string;
    icon?: React.ReactNode;
    content: React.ReactNode;
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
                <div className="grid grid-cols-4 gap-1.5">
                    {tabs.map(tab => (
                        <button
                            key={tab.id}
                            onClick={() => setActiveTabId(tab.id)}
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
