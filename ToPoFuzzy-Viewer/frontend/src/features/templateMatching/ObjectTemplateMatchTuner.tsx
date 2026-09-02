import { ChangeEvent, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Check, Download, RefreshCw, RotateCcw, Upload } from 'lucide-react';
import { CollapsibleSection } from '../../components/ui/CollapsibleSection';
import {
    makeDefaultTemplateMatchConfig,
    TEMPLATE_MATCH_PARAMETER_DEFINITIONS,
    TemplateMatchConfig,
    TemplateMatchConfigResult,
    TemplateMatchParameterDefinition,
    TemplateMatchParameterValue,
    TemplateMatchTargets,
} from './types';

interface ObjectTemplateMatchTunerProps {
    isConnected: boolean;
    getConfig: (targets: TemplateMatchTargets) => Promise<TemplateMatchConfigResult>;
    applyConfig: (config: TemplateMatchConfig) => Promise<TemplateMatchConfigResult>;
}

const sectionOrder = ['Orientation', 'Node', 'Plane', 'Graph & Scale', 'Contradiction', 'Validation'] as const;

function clampNumber(value: number, definition: TemplateMatchParameterDefinition) {
    const minimum = definition.min ?? Number.NEGATIVE_INFINITY;
    const maximum = definition.max ?? Number.POSITIVE_INFINITY;
    const clamped = Math.max(minimum, Math.min(maximum, value));
    return definition.step === 1 ? Math.round(clamped) : clamped;
}

function mergeConfig(current: TemplateMatchConfig, incoming: Partial<TemplateMatchConfig>): TemplateMatchConfig {
    return {
        matcherNode: incoming.matcherNode || current.matcherNode,
        validatorNode: incoming.validatorNode || current.validatorNode,
        matcher: { ...current.matcher, ...(incoming.matcher || {}) },
        validator: { ...current.validator, ...(incoming.validator || {}) },
    };
}

export function ObjectTemplateMatchTuner({ isConnected, getConfig, applyConfig }: ObjectTemplateMatchTunerProps) {
    const [config, setConfig] = useState<TemplateMatchConfig>(() => makeDefaultTemplateMatchConfig());
    const [isBusy, setIsBusy] = useState(false);
    const [status, setStatus] = useState<{ kind: 'idle' | 'ok' | 'error'; text: string }>({
        kind: 'idle', text: 'Not loaded',
    });
    const profileInputRef = useRef<HTMLInputElement>(null);

    const refresh = useCallback(async () => {
        if (!isConnected) return;
        setIsBusy(true);
        try {
            const result = await getConfig({ matcherNode: config.matcherNode, validatorNode: config.validatorNode });
            setConfig((current) => mergeConfig(current, result));
            setStatus({ kind: 'ok', text: 'Current parameters loaded' });
        } catch (error) {
            setStatus({ kind: 'error', text: error instanceof Error ? error.message : 'Parameter load failed' });
        } finally {
            setIsBusy(false);
        }
    }, [config.matcherNode, config.validatorNode, getConfig, isConnected]);

    useEffect(() => {
        if (isConnected) void refresh();
    }, [isConnected]); // Load once for each connection.

    const definitionsBySection = useMemo(() => Object.fromEntries(sectionOrder.map((section) => [
        section,
        TEMPLATE_MATCH_PARAMETER_DEFINITIONS.filter((definition) => definition.section === section),
    ])) as Record<(typeof sectionOrder)[number], TemplateMatchParameterDefinition[]>, []);

    const setValue = (definition: TemplateMatchParameterDefinition, value: TemplateMatchParameterValue) => {
        setConfig((current) => ({
            ...current,
            [definition.node]: { ...current[definition.node], [definition.key]: value },
        }));
        setStatus({ kind: 'idle', text: 'Modified' });
    };

    const handleApply = async () => {
        setIsBusy(true);
        try {
            const result = await applyConfig(config);
            if (!result.success) throw new Error(result.reason || 'Parameter update rejected');
            setConfig((current) => mergeConfig(current, result));
            setStatus({ kind: 'ok', text: 'Applied' });
        } catch (error) {
            setStatus({ kind: 'error', text: error instanceof Error ? error.message : 'Parameter update failed' });
        } finally {
            setIsBusy(false);
        }
    };

    const handleReset = () => {
        const defaults = makeDefaultTemplateMatchConfig();
        defaults.matcherNode = config.matcherNode;
        defaults.validatorNode = config.validatorNode;
        setConfig(defaults);
        setStatus({ kind: 'idle', text: 'Defaults staged' });
    };

    const handleExport = () => {
        const blob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const anchor = document.createElement('a');
        anchor.href = url;
        anchor.download = 'object_template_matching_profile.json';
        anchor.click();
        window.setTimeout(() => URL.revokeObjectURL(url), 1000);
        setStatus({ kind: 'ok', text: 'Profile exported' });
    };

    const handleImport = async (event: ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        event.target.value = '';
        if (!file) return;
        try {
            const parsed = JSON.parse(await file.text()) as Partial<TemplateMatchConfig>;
            setConfig((current) => mergeConfig(current, parsed));
            setStatus({ kind: 'idle', text: 'Profile staged' });
        } catch (error) {
            setStatus({ kind: 'error', text: error instanceof Error ? error.message : 'Profile import failed' });
        }
    };

    const renderControl = (definition: TemplateMatchParameterDefinition) => {
        const value = config[definition.node][definition.key] ?? definition.defaultValue;
        if (definition.type === 'boolean') {
            return (
                <label key={`${definition.node}.${definition.key}`} className="flex min-h-10 items-center justify-between gap-3 border-b border-white/5 px-1 py-2 last:border-0">
                    <span className="text-[11px] font-medium text-[var(--text-primary)]">{definition.label}</span>
                    <input type="checkbox" checked={Boolean(value)} onChange={(event) => setValue(definition, event.target.checked)} className="h-4 w-4 accent-[var(--accent-color)]" />
                </label>
            );
        }

        const numberValue = Number(value);
        return (
            <div key={`${definition.node}.${definition.key}`} className="border-b border-white/5 px-1 py-2 last:border-0">
                <div className="mb-1.5 flex items-center justify-between gap-2">
                    <label className="min-w-0 text-[11px] font-medium text-[var(--text-primary)]">{definition.label}</label>
                    <input
                        type="number" min={definition.min} max={definition.max} step={definition.step} value={numberValue}
                        onChange={(event) => {
                            const next = Number(event.target.value);
                            if (Number.isFinite(next)) setValue(definition, clampNumber(next, definition));
                        }}
                        className="h-8 w-24 rounded border border-white/10 bg-black/20 px-2 text-right font-mono text-[11px] text-[var(--text-primary)]"
                    />
                </div>
                <input
                    type="range" min={definition.min} max={definition.max} step={definition.step} value={numberValue}
                    onChange={(event) => setValue(definition, clampNumber(Number(event.target.value), definition))}
                    className="h-5 w-full accent-[var(--accent-color)]"
                />
            </div>
        );
    };

    return (
        <div className="space-y-3">
            <div className="surface-soft space-y-2 p-3">
                <div className="grid grid-cols-1 gap-2">
                    <label className="space-y-1">
                        <span className="panel-title">Matcher node</span>
                        <input type="text" value={config.matcherNode} onChange={(event) => setConfig((current) => ({ ...current, matcherNode: event.target.value }))} className="h-9 w-full rounded border border-white/10 bg-black/20 px-2 font-mono text-[11px] text-[var(--text-primary)]" />
                    </label>
                    <label className="space-y-1">
                        <span className="panel-title">Validator node</span>
                        <input type="text" value={config.validatorNode} onChange={(event) => setConfig((current) => ({ ...current, validatorNode: event.target.value }))} className="h-9 w-full rounded border border-white/10 bg-black/20 px-2 font-mono text-[11px] text-[var(--text-primary)]" />
                    </label>
                </div>
                <div className="grid grid-cols-4 gap-1.5">
                    <button className="entity-btn" title="Reload parameters" disabled={!isConnected || isBusy} onClick={() => void refresh()}><RefreshCw size={14} className={isBusy ? 'animate-spin' : ''} /><span>Load</span></button>
                    <button className="entity-btn" title="Restore staged defaults" disabled={isBusy} onClick={handleReset}><RotateCcw size={14} /><span>Reset</span></button>
                    <button className="entity-btn" title="Import profile" disabled={isBusy} onClick={() => profileInputRef.current?.click()}><Upload size={14} /><span>Open</span></button>
                    <button className="entity-btn" title="Export profile" disabled={isBusy} onClick={handleExport}><Download size={14} /><span>Save</span></button>
                </div>
                <input ref={profileInputRef} type="file" accept=".json,application/json" className="hidden" onChange={(event) => void handleImport(event)} />
            </div>

            {sectionOrder.map((section, index) => (
                <CollapsibleSection key={section} title={section} defaultOpen={index < 2}>
                    <div className="surface-muted px-2 py-1">{definitionsBySection[section].map(renderControl)}</div>
                </CollapsibleSection>
            ))}

            <div className="sticky bottom-0 z-10 border-t border-white/10 bg-[rgba(8,19,29,0.94)] px-1 pb-1 pt-2 backdrop-blur-sm">
                <button className="btn-primary flex w-full items-center justify-center gap-2 px-4 py-2 text-sm font-semibold" disabled={!isConnected || isBusy} onClick={() => void handleApply()}>
                    <Check size={16} />Apply
                </button>
                <div className={`mt-1.5 min-h-5 truncate text-center font-mono text-[10px] ${status.kind === 'error' ? 'text-red-300' : status.kind === 'ok' ? 'text-emerald-300' : 'text-[var(--text-secondary)]'}`} title={status.text}>
                    {isConnected ? status.text : 'Viewer backend offline'}
                </div>
            </div>
        </div>
    );
}
