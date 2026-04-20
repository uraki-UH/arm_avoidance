import React from 'react';

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
