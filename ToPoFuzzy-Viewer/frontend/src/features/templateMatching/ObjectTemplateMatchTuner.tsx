import { ChangeEvent, useCallback, useMemo, useRef, useState } from 'react';
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
    layout?: 'sidebar' | 'dialog';
}

const sectionOrder = ['Orientation', 'Evidence', 'Rejection', 'Decision'] as const;

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

export function ObjectTemplateMatchTuner({
    isConnected,
    getConfig,
    applyConfig,
    layout = 'sidebar',
}: ObjectTemplateMatchTunerProps) {
    const [config, setConfig] = useState<TemplateMatchConfig>(() => makeDefaultTemplateMatchConfig());
    const [pendingAction, setPendingAction] = useState<'load' | 'apply' | null>(null);
    const [status, setStatus] = useState<{ kind: 'idle' | 'ok' | 'error'; text: string }>({
        kind: 'idle', text: 'Not loaded',
    });
    const profileInputRef = useRef<HTMLInputElement>(null);
    const isBusy = pendingAction !== null;

    const refresh = useCallback(async () => {
        if (!isConnected) return;
        setPendingAction('load');
        try {
            const result = await getConfig({ matcherNode: config.matcherNode, validatorNode: config.validatorNode });
            setConfig((current) => mergeConfig(current, result));
            setStatus({ kind: 'ok', text: 'Current parameters loaded' });
        } catch (error) {
            setStatus({ kind: 'error', text: error instanceof Error ? error.message : 'Parameter load failed' });
        } finally {
            setPendingAction(null);
        }
    }, [config.matcherNode, config.validatorNode, getConfig, isConnected]);

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
        setPendingAction('apply');
        try {
            const result = await applyConfig(config);
            if (!result.success) throw new Error(result.reason || 'Parameter update rejected');
            setConfig((current) => mergeConfig(current, result));
            setStatus({ kind: 'ok', text: 'Applied' });
        } catch (error) {
            setStatus({ kind: 'error', text: error instanceof Error ? error.message : 'Parameter update failed' });
        } finally {
            setPendingAction(null);
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

    const isDialog = layout === 'dialog';
    const actionIconSize = isDialog ? 24 : 14;

    const renderControl = (definition: TemplateMatchParameterDefinition) => {
        const value = config[definition.node][definition.key] ?? definition.defaultValue;
        if (definition.type === 'boolean') {
            return (
                <label key={`${definition.node}.${definition.key}`} className={`flex items-center justify-between gap-3 border-b border-white/5 px-1 last:border-0 ${isDialog ? 'min-h-16 py-3' : 'min-h-10 py-2'}`}>
                    <span className={`${isDialog ? 'text-[22px] leading-tight' : 'text-[11px]'} font-medium text-[var(--text-primary)]`}>{definition.label}</span>
                    <input type="checkbox" checked={Boolean(value)} onChange={(event) => setValue(definition, event.target.checked)} className={`${isDialog ? 'h-6 w-6' : 'h-4 w-4'} shrink-0 accent-[var(--accent-color)]`} />
                </label>
            );
        }

        const numberValue = Number(value);
        return (
            <div key={`${definition.node}.${definition.key}`} className={`border-b border-white/5 px-1 last:border-0 ${isDialog ? 'py-3' : 'py-2'}`}>
                <div className={`${isDialog ? 'mb-2.5' : 'mb-1.5'} flex items-center justify-between gap-2`}>
                    <label className={`min-w-0 font-medium leading-tight text-[var(--text-primary)] ${isDialog ? 'text-[22px]' : 'text-[11px]'}`}>{definition.label}</label>
                    <input
                        type="number" min={definition.min} max={definition.max} step={definition.step} value={numberValue}
                        onChange={(event) => {
                            const next = Number(event.target.value);
                            if (Number.isFinite(next)) setValue(definition, clampNumber(next, definition));
                        }}
                        className={`${isDialog ? 'h-12 w-32 px-3 text-[22px]' : 'h-8 w-24 px-2 text-[11px]'} shrink-0 rounded border border-white/10 bg-black/20 text-right font-mono text-[var(--text-primary)]`}
                    />
                </div>
                <input
                    type="range" min={definition.min} max={definition.max} step={definition.step} value={numberValue}
                    onChange={(event) => setValue(definition, clampNumber(Number(event.target.value), definition))}
                    className={`${isDialog ? 'h-7' : 'h-5'} w-full accent-[var(--accent-color)]`}
                />
            </div>
        );
    };

    const sectionGrid = sectionOrder.map((section, index) => (
        <CollapsibleSection
            key={section}
            title={section}
            defaultOpen={isDialog || index < 2}
            className={isDialog ? 'min-w-0' : ''}
            titleClassName={isDialog ? '!text-[28px]' : ''}
        >
            <div className="surface-muted px-2 py-1">{definitionsBySection[section].map(renderControl)}</div>
        </CollapsibleSection>
    ));

    return (
        <div className={isDialog ? 'flex h-full min-h-0 flex-col gap-3' : 'space-y-3'}>
            <div className={`surface-soft space-y-2 p-3 ${isDialog ? 'shrink-0' : ''}`}>
                <div className={`grid gap-2 ${isDialog ? 'md:grid-cols-2' : 'grid-cols-1'}`}>
                    <label className="space-y-1">
                        <span className={`panel-title ${isDialog ? '!text-[24px]' : ''}`}>Matcher node</span>
                        <input type="text" value={config.matcherNode} onChange={(event) => setConfig((current) => ({ ...current, matcherNode: event.target.value }))} className={`${isDialog ? 'h-14 px-4 text-[22px]' : 'h-9 px-2 text-[11px]'} w-full rounded border border-white/10 bg-black/20 font-mono text-[var(--text-primary)]`} />
                    </label>
                    <label className="space-y-1">
                        <span className={`panel-title ${isDialog ? '!text-[24px]' : ''}`}>Validator node</span>
                        <input type="text" value={config.validatorNode} onChange={(event) => setConfig((current) => ({ ...current, validatorNode: event.target.value }))} className={`${isDialog ? 'h-14 px-4 text-[22px]' : 'h-9 px-2 text-[11px]'} w-full rounded border border-white/10 bg-black/20 font-mono text-[var(--text-primary)]`} />
                    </label>
                </div>
                <div className={`grid grid-cols-4 gap-1.5 ${isDialog ? 'md:ml-auto md:w-[720px]' : ''}`}>
                    <button className={`entity-btn ${isDialog ? '!min-h-14 !text-[20px]' : ''}`} title="Reload parameters" disabled={!isConnected || isBusy} onClick={() => void refresh()}><RefreshCw size={actionIconSize} className={pendingAction === 'load' ? 'animate-spin' : ''} /><span>Load</span></button>
                    <button className={`entity-btn ${isDialog ? '!min-h-14 !text-[20px]' : ''}`} title="Restore staged defaults" disabled={isBusy} onClick={handleReset}><RotateCcw size={actionIconSize} /><span>Reset</span></button>
                    <button className={`entity-btn ${isDialog ? '!min-h-14 !text-[20px]' : ''}`} title="Import profile" disabled={isBusy} onClick={() => profileInputRef.current?.click()}><Upload size={actionIconSize} /><span>Open</span></button>
                    <button className={`entity-btn ${isDialog ? '!min-h-14 !text-[20px]' : ''}`} title="Export profile" disabled={isBusy} onClick={handleExport}><Download size={actionIconSize} /><span>Save</span></button>
                </div>
                <input ref={profileInputRef} type="file" accept=".json,application/json" className="hidden" onChange={(event) => void handleImport(event)} />
            </div>

            {isDialog ? (
                <div className="min-h-0 flex-1 overflow-y-auto pr-1 scrollbar-thin">
                    <div className="grid items-start gap-3 lg:grid-cols-2 2xl:grid-cols-3">
                        {sectionGrid}
                    </div>
                </div>
            ) : sectionGrid}

            <div className={`${isDialog ? 'shrink-0' : 'sticky bottom-0 z-10'} border-t border-white/10 bg-[rgba(8,19,29,0.94)] px-1 pb-1 pt-2 backdrop-blur-sm`}>
                <button className={`btn-primary flex items-center justify-center gap-2 px-4 font-semibold ${isDialog ? 'ml-auto w-full py-3 text-[28px] md:w-96' : 'w-full py-2 text-sm'}`} disabled={!isConnected || isBusy} onClick={() => void handleApply()}>
                    <Check size={isDialog ? 28 : 16} />Apply
                </button>
                <div className={`mt-1.5 min-h-5 truncate font-mono ${isDialog ? 'text-right text-[20px]' : 'text-center text-[10px]'} ${status.kind === 'error' ? 'text-red-300' : status.kind === 'ok' ? 'text-emerald-300' : 'text-[var(--text-secondary)]'}`} title={status.text}>
                    {isConnected ? status.text : 'Viewer backend offline'}
                </div>
            </div>
        </div>
    );
}
