import { useState, useEffect, useCallback } from 'react';
import { DataSource, GngStatus, GngParams, GngConfigInfo } from '../../hooks/useWebSocket';

interface SourceSelectorProps {
    isConnected: boolean;
    getSources: () => Promise<DataSource[]>;
    subscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string }>;
    unsubscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string }>;
    startGng: (params: GngParams) => Promise<{ success: boolean; pid?: number; inputTopic?: string }>;
    stopGng: () => Promise<{ success: boolean }>;
    getGngStatus: () => Promise<GngStatus>;
    listGngConfigs: () => Promise<GngConfigInfo[]>;
    onSourceToggled?: (sourceId: string, active: boolean) => void;
}

export function SourceSelector({
    isConnected,
    getSources,
    subscribeSource,
    unsubscribeSource,
    startGng,
    stopGng,
    getGngStatus,
    listGngConfigs,
    onSourceToggled
}: SourceSelectorProps) {
    const [sources, setSources] = useState<DataSource[]>([]);
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [processingSource, setProcessingSource] = useState<string | null>(null);
    const [gngStatus, setGngStatus] = useState<GngStatus>({ isRunning: false });
    const [configs, setConfigs] = useState<GngConfigInfo[]>([]);
    const [selectedConfig, setSelectedConfig] = useState<string>('');

    const refreshSources = useCallback(async () => {
        if (!isConnected) return;

        setIsLoading(true);
        setError(null);
        try {
            const newSources = await getSources();
            setSources(newSources);
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Failed to get sources');
        } finally {
            setIsLoading(false);
        }
    }, [isConnected, getSources]);

    const refreshGngStatus = useCallback(async () => {
        if (!isConnected) return;
        try {
            const status = await getGngStatus();
            setGngStatus(status);
        } catch (err) {
            console.error('Failed to get GNG status:', err);
        }
    }, [isConnected, getGngStatus]);

    const refreshConfigs = useCallback(async () => {
        if (!isConnected) return;
        try {
            const configList = await listGngConfigs();
            setConfigs(configList);
            setSelectedConfig((prev) => {
                if (configList.length > 0 && !prev) {
                    return configList[0].path;
                }
                return prev;
            });
        } catch (err) {
            console.error('Failed to list GNG configs:', err);
        }
    }, [isConnected, listGngConfigs]);

    useEffect(() => {
        if (isConnected) {
            refreshSources();
            refreshGngStatus();
            refreshConfigs();
            const interval = setInterval(refreshGngStatus, 5000);
            return () => clearInterval(interval);
        }

        setSources([]);
        setGngStatus({ isRunning: false });
        setConfigs([]);
    }, [isConnected, refreshSources, refreshGngStatus, refreshConfigs]);

    const handleToggleSource = async (source: DataSource) => {
        try {
            if (source.active) {
                await unsubscribeSource(source.id);
            } else {
                await subscribeSource(source.id);
            }

            const nextActive = !source.active;
            onSourceToggled?.(source.id, nextActive);
            await refreshSources();
        } catch (err) {
            console.error('Failed to toggle source:', err);
        }
    };

    const handleStartGng = async (e: React.MouseEvent, sourceId: string) => {
        e.stopPropagation();
        setProcessingSource(sourceId);
        setError(null);
        try {
            const params: GngParams = { inputTopic: sourceId };
            if (selectedConfig) {
                params.configFile = selectedConfig;
            }
            const result = await startGng(params);
            if (result.success) {
                await refreshGngStatus();
            } else {
                setError('Failed to start GNG');
            }
        } catch (err) {
            console.error('Failed to start GNG:', err);
            setError(err instanceof Error ? err.message : 'Failed to start GNG');
        } finally {
            setProcessingSource(null);
        }
    };

    const handleStopGng = async () => {
        setError(null);
        try {
            const result = await stopGng();
            if (result.success) {
                await refreshGngStatus();
            } else {
                setError('Failed to stop GNG');
            }
        } catch (err) {
            console.error('Failed to stop GNG:', err);
            setError(err instanceof Error ? err.message : 'Failed to stop GNG');
        }
    };

    if (!isConnected) {
        return (
            <div className="surface-muted p-3 text-xs italic text-[var(--text-secondary)]">
                Connect to the server to view available point cloud topics.
            </div>
        );
    }

    return (
        <div className="space-y-3">
            <div className="flex items-center justify-between">
                <h3 className="panel-title">PointCloud Topics</h3>
                <button
                    onClick={refreshSources}
                    disabled={isLoading}
                    className="btn-secondary px-2.5 py-1 text-[11px] font-semibold uppercase tracking-[0.08em] disabled:opacity-50"
                >
                    {isLoading ? 'Loading...' : 'Refresh'}
                </button>
            </div>

            {configs.length > 0 && (
                <div className="surface-muted space-y-1.5 p-3">
                    <label className="control-label">GNG Config</label>
                    <select
                        value={selectedConfig}
                        onChange={(e) => setSelectedConfig(e.target.value)}
                        disabled={gngStatus.isRunning}
                        className="select-field disabled:opacity-50"
                    >
                        <option value="">Default (no config)</option>
                        {configs.map((config) => (
                            <option key={config.path} value={config.path}>
                                {config.name}
                            </option>
                        ))}
                    </select>
                </div>
            )}

            <div className={`rounded-lg border px-3 py-2 text-xs ${gngStatus.isRunning ? 'border-green-500/40 bg-green-500/10' : 'border-white/20 bg-black/25'}`}>
                <div className="flex items-center justify-between gap-2">
                    <span className="text-[var(--text-primary)]">
                        GNG: {gngStatus.isRunning ? (
                            <span className="text-green-400">Running ({gngStatus.inputTopic})</span>
                        ) : (
                            <span className="text-[var(--text-secondary)]">Stopped</span>
                        )}
                    </span>
                    {gngStatus.isRunning && (
                        <button
                            onClick={handleStopGng}
                            className="btn-danger px-2 py-1 text-[11px] font-semibold"
                            title="Stop GNG"
                        >
                            Stop
                        </button>
                    )}
                </div>
            </div>

            {error && (
                <div className="rounded border border-red-500/30 bg-red-500/10 p-2 text-xs text-red-300">
                    {error}
                </div>
            )}

            {sources.length === 0 ? (
                <div className="rounded-lg border border-white/10 bg-black/20 py-3 text-center text-xs italic text-[var(--text-secondary)]">
                    No PointCloud2 topics found.
                </div>
            ) : (
                <div className="max-h-60 space-y-2 overflow-y-auto pr-1 scrollbar-thin">
                    {sources.map((source) => {
                        const isRunningSource = gngStatus.isRunning && gngStatus.inputTopic === source.id;
                        return (
                            <div key={source.id} className="flex items-center gap-2">
                                <label className="flex min-w-0 flex-1 cursor-pointer items-center gap-3 rounded-lg border border-white/10 bg-white/5 p-3 transition-colors hover:bg-white/10">
                                    <input
                                        type="checkbox"
                                        checked={source.active}
                                        onChange={() => handleToggleSource(source)}
                                        className="h-4 w-4 rounded border-gray-500 bg-transparent text-[var(--accent-color)] focus:ring-[var(--accent-color)]"
                                    />
                                    <span className="flex-1 truncate text-sm text-[var(--text-primary)]" title={source.id}>
                                        {source.name}
                                    </span>
                                    {source.active && (
                                        <span className="text-[10px] font-semibold uppercase tracking-[0.08em] text-green-400">Active</span>
                                    )}
                                </label>

                                <button
                                    onClick={(e) => handleStartGng(e, source.id)}
                                    disabled={processingSource === source.id || gngStatus.isRunning}
                                    className={`rounded-lg border px-3 py-2 text-[11px] font-semibold transition-colors ${isRunningSource
                                        ? 'border-green-500/50 bg-green-500/20 text-green-300'
                                        : 'border-sky-400/30 bg-sky-500/10 text-sky-300 hover:bg-sky-500/20 disabled:opacity-45'
                                        }`}
                                    title={gngStatus.isRunning ? (isRunningSource ? 'GNG running on this source' : 'GNG is already running') : 'Start GNG on this source'}
                                >
                                    {processingSource === source.id ? '...' : isRunningSource ? 'Running' : 'Start GNG'}
                                </button>
                            </div>
                        );
                    })}
                </div>
            )}

            <div className="text-right text-xs text-[var(--text-secondary)]">
                {sources.filter((s) => s.active).length} / {sources.length} active
            </div>
        </div>
    );
}
