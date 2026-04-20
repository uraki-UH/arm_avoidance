import { useCallback, useEffect, useState } from 'react';
import { GngStatus, NodeParameters, SetParameterResult } from '../../hooks/useWebSocket';

type ParamKey = 'all' | 'unknown' | 'human';

interface NumericParamState {
    name: string;
    label: string;
    min: number;
    max: number;
    step: number;
    value: number;
}

interface GngDownsamplingPanelProps {
    isConnected: boolean;
    getGngStatus: () => Promise<GngStatus>;
    getParameters: () => Promise<NodeParameters>;
    setParameter: (paramName: string, value: number | string | boolean) => Promise<SetParameterResult>;
}

const clampNumber = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

export function GngDownsamplingPanel({
    isConnected,
    getGngStatus,
    getParameters,
    setParameter
}: GngDownsamplingPanelProps) {
    const [gngStatus, setGngStatus] = useState<GngStatus>({ isRunning: false });
    const [paramState, setParamState] = useState<{
        all?: NumericParamState;
        unknown?: NumericParamState;
        human?: NumericParamState;
    } | null>(null);
    const [isParamLoading, setIsParamLoading] = useState(false);
    const [paramError, setParamError] = useState<string | null>(null);

    const refreshGngStatus = useCallback(async () => {
        if (!isConnected) return;
        try {
            const status = await getGngStatus();
            setGngStatus(status);
        } catch (err) {
            console.error('Failed to get GNG status:', err);
        }
    }, [isConnected, getGngStatus]);

    const refreshParams = useCallback(async () => {
        if (!isConnected) return;
        setIsParamLoading(true);
        setParamError(null);
        try {
            const result = await getParameters();
            const paramMap = new Map(result.parameters.map((p) => [p.name, p]));
            const buildNumericParam = (name: string, label: string): NumericParamState | undefined => {
                const param = paramMap.get(name);
                if (!param) return undefined;
                const rawValue = param.value;
                const numericValue = typeof rawValue === 'number' ? rawValue : Number(rawValue);
                if (!Number.isFinite(numericValue)) return undefined;
                return {
                    name: param.name,
                    label: param.description || label,
                    min: param.min,
                    max: param.max,
                    step: param.step,
                    value: numericValue
                };
            };

            const allParam = buildNumericParam('ds.all.num_max', 'All Clusters');
            const unknownParam = buildNumericParam('ds.unknown.num_max', 'Unknown Objects');
            const humanParam = buildNumericParam('ds.human.num_max', 'Human Detection');

            if (!allParam || !unknownParam || !humanParam) {
                setParamState(null);
                setParamError('GNG parameters are not available');
                return;
            }

            const safeUnknown = Math.min(unknownParam.value, allParam.value);
            const safeHuman = Math.min(humanParam.value, allParam.value);

            setParamState({
                all: allParam,
                unknown: { ...unknownParam, value: safeUnknown },
                human: { ...humanParam, value: safeHuman }
            });
        } catch (err) {
            setParamError(err instanceof Error ? err.message : 'Failed to load GNG parameters');
        } finally {
            setIsParamLoading(false);
        }
    }, [getParameters, isConnected]);

    useEffect(() => {
        if (isConnected) {
            refreshGngStatus();
            refreshParams();
            const interval = setInterval(refreshGngStatus, 5000);
            return () => clearInterval(interval);
        }

        setGngStatus({ isRunning: false });
        setParamState(null);
        setParamError(null);
    }, [isConnected, refreshGngStatus, refreshParams]);

    useEffect(() => {
        if (isConnected && gngStatus.isRunning) {
            refreshParams();
        }
    }, [gngStatus.isRunning, isConnected, refreshParams]);

    const canEditParams = isConnected && gngStatus.isRunning;

    const commitParamUpdate = useCallback(async (paramName: string, value: number) => {
        try {
            const result = await setParameter(paramName, value);
            if (!result.success) {
                setParamError(`Failed to set ${paramName}`);
            }
        } catch (err) {
            setParamError(err instanceof Error ? err.message : `Failed to set ${paramName}`);
        }
    }, [setParameter]);

    const updateParamValue = useCallback(async (key: ParamKey, nextValue: number, maxOverride?: number) => {
        if (!paramState || !paramState[key]) return;
        setParamError(null);
        const current = paramState[key]!;
        const effectiveMax = maxOverride !== undefined ? Math.min(current.max, maxOverride) : current.max;
        const clamped = clampNumber(nextValue, current.min, effectiveMax);

        setParamState((prev) => {
            if (!prev || !prev[key]) return prev;
            return { ...prev, [key]: { ...prev[key]!, value: clamped } };
        });

        if (canEditParams) {
            await commitParamUpdate(current.name, clamped);
        }
    }, [commitParamUpdate, canEditParams, paramState]);

    const handleAllParamChange = useCallback(async (nextValue: number) => {
        if (!paramState?.all) return;
        setParamError(null);
        const nextAll = clampNumber(nextValue, paramState.all.min, paramState.all.max);
        const nextUnknown = paramState.unknown
            ? clampNumber(Math.min(paramState.unknown.value, nextAll), paramState.unknown.min, Math.min(paramState.unknown.max, nextAll))
            : undefined;
        const nextHuman = paramState.human
            ? clampNumber(Math.min(paramState.human.value, nextAll), paramState.human.min, Math.min(paramState.human.max, nextAll))
            : undefined;

        setParamState((prev) => {
            if (!prev?.all) return prev;
            return {
                ...prev,
                all: { ...prev.all, value: nextAll },
                unknown: prev.unknown && nextUnknown !== undefined ? { ...prev.unknown, value: nextUnknown } : prev.unknown,
                human: prev.human && nextHuman !== undefined ? { ...prev.human, value: nextHuman } : prev.human
            };
        });

        if (!canEditParams) return;

        const updates: Promise<SetParameterResult>[] = [];
        if (nextAll !== paramState.all.value) {
            updates.push(setParameter(paramState.all.name, nextAll));
        }
        if (paramState.unknown && nextUnknown !== undefined && nextUnknown !== paramState.unknown.value) {
            updates.push(setParameter(paramState.unknown.name, nextUnknown));
        }
        if (paramState.human && nextHuman !== undefined && nextHuman !== paramState.human.value) {
            updates.push(setParameter(paramState.human.name, nextHuman));
        }

        try {
            const results = await Promise.all(updates);
            const failed = results.find((r) => !r.success);
            if (failed) {
                setParamError(`Failed to set ${failed.paramName}`);
            }
        } catch (err) {
            setParamError(err instanceof Error ? err.message : 'Failed to update parameters');
        }
    }, [canEditParams, paramState, setParameter]);

    const renderParamControl = (
        param: NumericParamState | undefined,
        maxOverride: number | undefined,
        onChange: (value: number) => void
    ) => {
        if (!param) return null;
        const effectiveMax = maxOverride !== undefined ? Math.min(param.max, maxOverride) : param.max;
        return (
            <div className="space-y-1.5">
                <div className="flex items-center justify-between gap-2">
                    <label className="control-label">{param.label}</label>
                    <input
                        type="number"
                        min={param.min}
                        max={effectiveMax}
                        step={param.step}
                        value={Math.round(param.value)}
                        disabled={!canEditParams}
                        onChange={(e) => {
                            const next = Number(e.target.value);
                            if (!Number.isFinite(next)) return;
                            onChange(next);
                        }}
                        className="input-field w-24 p-1.5 text-[11px] disabled:opacity-50"
                    />
                </div>
                <input
                    type="range"
                    min={param.min}
                    max={effectiveMax}
                    step={param.step}
                    value={Math.min(param.value, effectiveMax)}
                    disabled={!canEditParams}
                    onChange={(e) => onChange(Number(e.target.value))}
                    className="w-full disabled:opacity-50"
                />
            </div>
        );
    };

    if (!isConnected) {
        return (
            <div className="surface-muted p-3 text-xs italic text-[var(--text-secondary)]">
                Connect to the server to configure downsampling.
            </div>
        );
    }

    return (
        <div className="surface-muted space-y-2.5 p-3">
            <div className="flex items-center justify-between">
                <span className="panel-title">GNG Downsampling</span>
                <button
                    onClick={refreshParams}
                    disabled={isParamLoading}
                    className="btn-secondary px-2 py-1 text-[10px] font-semibold disabled:opacity-50"
                    title="Refresh parameters"
                >
                    {isParamLoading ? 'Loading...' : 'Refresh'}
                </button>
            </div>

            {isParamLoading ? (
                <div className="text-[11px] text-[var(--text-secondary)]">Loading parameters...</div>
            ) : paramState ? (
                <div className="space-y-2">
                    {renderParamControl(paramState.all, undefined, (value) => { void handleAllParamChange(value); })}
                    {renderParamControl(paramState.unknown, paramState.all?.value, (value) => { void updateParamValue('unknown', value, paramState.all?.value); })}
                    {renderParamControl(paramState.human, paramState.all?.value, (value) => { void updateParamValue('human', value, paramState.all?.value); })}
                </div>
            ) : (
                <div className="text-[11px] text-[var(--text-secondary)]">No parameters available.</div>
            )}

            {!canEditParams && (
                <div className="text-[11px] text-[var(--text-secondary)]">Start GNG in Data tab to apply parameter changes.</div>
            )}
            {paramError && (
                <div className="rounded border border-red-500/30 bg-red-500/10 px-2 py-1 text-[11px] text-red-300">{paramError}</div>
            )}
        </div>
    );
}
