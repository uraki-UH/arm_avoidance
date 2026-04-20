interface SelectionToolbarProps {
    isSelectionMode: boolean;
    onToggleSelectionMode: () => void;
    selectedCount: number;
    onDeleteSelected: () => void;
    onClearSelection: () => void;
    disabled?: boolean;
}

export function SelectionToolbar({
    isSelectionMode,
    onToggleSelectionMode,
    selectedCount,
    onDeleteSelected,
    onClearSelection,
    disabled = false,
}: SelectionToolbarProps) {
    return (
        <div className="space-y-3">
            <h3 className="panel-title">Point Deletion Tool</h3>

            <button
                onClick={onToggleSelectionMode}
                disabled={disabled}
                className={`w-full px-4 py-2 text-sm font-semibold ${isSelectionMode
                    ? 'btn-primary'
                    : 'btn-secondary text-[var(--text-secondary)] hover:text-[var(--text-primary)]'
                    } disabled:cursor-not-allowed disabled:opacity-45`}
            >
                {isSelectionMode ? 'Selection Mode Active' : 'Activate Selection'}
            </button>

            {isSelectionMode && (
                <div className="rounded-md border border-[var(--accent-color)]/40 bg-[var(--accent-soft)] px-3 py-2 text-xs text-[var(--text-primary)]">
                    <p className="mb-1 font-semibold">How to use</p>
                    <ol className="list-decimal list-inside space-y-1 text-[var(--text-secondary)]">
                        <li>Drag on the viewport to select points.</li>
                        <li>Release mouse to finish selection.</li>
                        <li>Delete selected points when confirmed.</li>
                    </ol>
                </div>
            )}

            {selectedCount > 0 && (
                <div className="space-y-2">
                    <div className="rounded-md border border-white/10 bg-white/5 p-2 text-sm text-[var(--text-secondary)]">
                        Selected: <span className="font-bold text-[var(--accent-strong)]">{selectedCount.toLocaleString()}</span> points
                    </div>

                    <div className="grid grid-cols-2 gap-2">
                        <button
                            onClick={onDeleteSelected}
                            disabled={disabled}
                            className="btn-danger px-3 py-2 text-sm font-semibold text-white disabled:cursor-not-allowed disabled:opacity-45"
                        >
                            Delete
                        </button>
                        <button
                            onClick={onClearSelection}
                            disabled={disabled}
                            className="btn-secondary px-3 py-2 text-sm font-semibold disabled:cursor-not-allowed disabled:opacity-45"
                        >
                            Clear
                        </button>
                    </div>
                </div>
            )}

            {!isSelectionMode && selectedCount === 0 && (
                <div className="text-center text-xs italic text-[var(--text-secondary)]">
                    Enable selection mode to remove points.
                </div>
            )}
        </div>
    );
}
