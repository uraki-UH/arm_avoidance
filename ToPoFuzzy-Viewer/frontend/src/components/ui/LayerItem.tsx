import React from 'react';
import { Eye, EyeOff, Move, Trash2 } from 'lucide-react';
import { getDisplayTitle, getStatusLabel } from '../../utils/rosUtils';

interface LayerItemProps {
    id: string;
    type: 'cloud' | 'graph' | 'robot' | 'marker';
    visible: boolean;
    onToggleVisibility: () => void;
    onRemove: () => void;
    onOpenTransform?: () => void;
    statusLabel?: string;
    isSelected?: boolean;
    onSelect?: () => void;
    isActionDisabled?: boolean;
    children?: React.ReactNode;
    className?: string;
    headerOnly?: boolean;
}

export const LayerItem: React.FC<LayerItemProps> = ({
    id, type, visible, onToggleVisibility, onRemove, onOpenTransform,
    statusLabel, isSelected, onSelect, isActionDisabled, children,
    className = '', headerOnly = false
}) => {
    const displayTitle = getDisplayTitle(id, type);
    const finalStatusLabel = statusLabel || getStatusLabel(type);

    const content = (
        <div className="flex items-start justify-between gap-2">
            <div className="min-w-0 flex-1">
                <div className="mb-1 flex items-center gap-2">
                    <button
                        onClick={(e) => { e.stopPropagation(); if (!isActionDisabled) onToggleVisibility(); }}
                        disabled={isActionDisabled}
                        className={`inline-flex h-6 w-6 items-center justify-center rounded-md border transition-all ${
                            visible 
                            ? 'border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]' 
                            : 'border-white/10 bg-black/20 text-[var(--text-secondary)]'
                        }`}
                        title={visible ? 'Hide layer' : 'Show layer'}
                    >
                        {visible ? <Eye size={14} /> : <EyeOff size={14} />}
                    </button>
                    {onOpenTransform && (
                        <button
                            onClick={(e) => { e.stopPropagation(); onOpenTransform(); }}
                            className="inline-flex h-6 w-6 items-center justify-center rounded-md border border-white/10 bg-black/20 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                            title="Open transform dialog"
                        >
                            <Move size={12} />
                        </button>
                    )}
                    <span className="text-[9px] font-bold uppercase tracking-wider text-[var(--text-secondary)] opacity-50">
                        {finalStatusLabel}
                    </span>
                    <p className="truncate text-sm font-semibold text-[var(--text-primary)]" title={id}>{displayTitle}</p>
                </div>
                {children}
            </div>
            <button
                onClick={(e) => { e.stopPropagation(); if (!isActionDisabled) onRemove(); }}
                disabled={isActionDisabled}
                className="btn-icon btn-icon-danger disabled:cursor-not-allowed disabled:opacity-50"
                title={`Remove ${type} layer`}
            >
                <Trash2 size={13} />
            </button>
        </div>
    );

    if (headerOnly) return content;

    return (
        <div
            onClick={onSelect}
            className={`rounded-lg border p-2 transition-colors mb-2 ${
                isSelected
                ? 'border-[var(--accent-color)]/70 bg-[var(--accent-soft)]'
                : 'border-white/10 bg-white/5 hover:bg-white/10'
            } ${isActionDisabled ? 'opacity-70 cursor-not-allowed' : ''} ${className}`}
        >
            {content}
        </div>
    );
};
