import React, { useEffect, useRef, useState } from 'react';
import { ChevronDown } from 'lucide-react';

interface CollapsibleSectionProps {
    title: string;
    icon?: React.ReactNode;
    children: React.ReactNode;
    defaultOpen?: boolean;
    className?: string;
}

export const CollapsibleSection: React.FC<CollapsibleSectionProps> = ({
    title,
    icon,
    children,
    defaultOpen = false,
    className = ""
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
                className={`flex w-full items-center justify-between px-4 py-3 text-left transition-colors ${isOpen ? 'bg-white/5' : 'hover:bg-white/5'
                    }`}
                aria-expanded={isOpen}
            >
                <div className="flex items-center gap-2 text-sm font-semibold text-[var(--text-primary)]">
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
