import { HeatmapMode, HeatmapSettings } from '../../types';

interface HeatmapControlsProps {
    settings: HeatmapSettings;
    onSettingsChange: (settings: HeatmapSettings) => void;
    bounds?: { minZ: number; maxZ: number; maxDist: number };
}

export function HeatmapControls({ settings, onSettingsChange, bounds }: HeatmapControlsProps) {
    const handleModeChange = (newMode: HeatmapMode) => {
        let newMin = settings.min;
        let newMax = settings.max;

        if (newMode === 'height' && bounds) {
            newMin = bounds.minZ;
            newMax = bounds.maxZ;
        } else if (newMode === 'distance' && bounds) {
            newMin = 0;
            newMax = bounds.maxDist;
        } else if (newMode === 'intensity') {
            newMin = 0;
            newMax = 1;
        }

        onSettingsChange({ ...settings, mode: newMode, min: newMin, max: newMax });
    };

    const isHeatmapMode = ['height', 'distance', 'intensity'].includes(settings.mode);

    return (
        <div className="space-y-4">
            <div className="surface-muted space-y-2 p-3">
                <label className="control-label">Color Mode</label>
                <div className="grid grid-cols-3 gap-1">
                    {(['simple', 'rgb', 'height'] as HeatmapMode[]).map((mode) => {
                        const label = mode === 'height' ? 'heatmap' : mode;
                        const active = mode === 'height' ? isHeatmapMode : settings.mode === mode;
                        return (
                            <button
                                key={mode}
                                className={`rounded-md px-2 py-1.5 text-xs font-semibold uppercase tracking-[0.06em] transition-colors ${active
                                    ? 'bg-[var(--accent-soft)] text-[var(--accent-strong)] ring-1 ring-[var(--accent-color)]/40'
                                    : 'text-[var(--text-secondary)] hover:bg-white/10 hover:text-[var(--text-primary)]'
                                    }`}
                                onClick={() => handleModeChange(mode)}
                            >
                                {label}
                            </button>
                        );
                    })}
                </div>
            </div>

            {settings.mode === 'simple' && (
                <div className="surface-muted space-y-2 p-3">
                    <label className="control-label">Simple Color</label>
                    <div className="flex items-center gap-2">
                        <input
                            type="color"
                            value={settings.simpleColor || '#ffffff'}
                            onChange={(e) => onSettingsChange({ ...settings, simpleColor: e.target.value })}
                            className="h-8 w-10 cursor-pointer rounded border border-white/20 bg-transparent"
                        />
                        <span className="text-xs text-[var(--text-secondary)]">{settings.simpleColor || '#ffffff'}</span>
                    </div>
                </div>
            )}

            {isHeatmapMode && (
                <>
                    <div className="surface-muted space-y-2 p-3">
                        <label className="control-label">Data Source</label>
                        <div className="grid grid-cols-3 gap-1">
                            {(['height', 'distance', 'intensity'] as HeatmapMode[]).map((mode) => (
                                <button
                                    key={mode}
                                    className={`rounded-md px-2 py-1.5 text-[11px] font-semibold uppercase tracking-[0.06em] transition-colors ${settings.mode === mode
                                        ? 'bg-white/20 text-[var(--text-primary)] ring-1 ring-white/20'
                                        : 'text-[var(--text-secondary)] hover:bg-white/10 hover:text-[var(--text-primary)]'
                                        }`}
                                    onClick={() => handleModeChange(mode)}
                                >
                                    {mode}
                                </button>
                            ))}
                        </div>

                        <label className="control-label pt-1">Color Scheme</label>
                        <select
                            value={settings.colorScheme}
                            onChange={(e) => onSettingsChange({ ...settings, colorScheme: e.target.value as HeatmapSettings['colorScheme'] })}
                            className="select-field"
                        >
                            <option value="viridis">Viridis</option>
                            <option value="plasma">Plasma</option>
                            <option value="magma">Magma</option>
                            <option value="inferno">Inferno</option>
                            <option value="jet">Jet</option>
                            <option value="grayscale">Grayscale</option>
                        </select>
                    </div>

                    <div className="surface-muted grid grid-cols-2 gap-3 p-3">
                        <div>
                            <label className="control-label mb-1 block">Min Value</label>
                            <input
                                type="number"
                                value={settings.min}
                                step={0.1}
                                onChange={(e) => onSettingsChange({ ...settings, min: parseFloat(e.target.value) })}
                                className="input-field"
                            />
                        </div>
                        <div>
                            <label className="control-label mb-1 block">Max Value</label>
                            <input
                                type="number"
                                value={settings.max}
                                step={0.1}
                                onChange={(e) => onSettingsChange({ ...settings, max: parseFloat(e.target.value) })}
                                className="input-field"
                            />
                        </div>
                    </div>
                </>
            )}
        </div>
    );
}
