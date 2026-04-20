import { useState, useEffect, useCallback } from 'react';
import { RosbagInfo, PlaybackStatus } from '../../hooks/useWebSocket';

interface RosbagPlayerProps {
    isConnected: boolean;
    listRosbags: () => Promise<RosbagInfo[]>;
    playRosbag: (path: string, remaps: string[], loop: boolean) => Promise<{ success: boolean }>;
    stopRosbag: () => Promise<{ success: boolean }>;
    getRosbagStatus: () => Promise<PlaybackStatus>;
}

export function RosbagPlayer({
    isConnected,
    listRosbags,
    playRosbag,
    stopRosbag,
    getRosbagStatus
}: RosbagPlayerProps) {
    const [bags, setBags] = useState<RosbagInfo[]>([]);
    const [selectedBag, setSelectedBag] = useState<string | null>(null);
    const [status, setStatus] = useState<PlaybackStatus>({ isPlaying: false, currentBag: '' });
    const [isLoading, setIsLoading] = useState(false);
    const [remaps, setRemaps] = useState<{ from: string; to: string }[]>([{ from: '/scan', to: '/input_scan' }]);
    const [isLoop, setIsLoop] = useState(true);

    const refreshBags = useCallback(async () => {
        if (!isConnected) return;
        setIsLoading(true);
        try {
            const list = await listRosbags();
            setBags(list);
        } catch (e) {
            console.error(e);
        } finally {
            setIsLoading(false);
        }
    }, [isConnected, listRosbags]);

    const refreshStatus = useCallback(async () => {
        if (!isConnected) return;
        try {
            const s = await getRosbagStatus();
            setStatus(s);
        } catch (e) {
            console.error(e);
        }
    }, [isConnected, getRosbagStatus]);

    useEffect(() => {
        if (isConnected) {
            refreshBags();
            const interval = setInterval(refreshStatus, 2000);
            return () => clearInterval(interval);
        }
    }, [isConnected, refreshBags, refreshStatus]);

    const handlePlay = async () => {
        if (!selectedBag) return;

        const remapArgs = remaps
            .filter((r) => r.from && r.to)
            .map((r) => `${r.from}:=${r.to}`);

        await playRosbag(selectedBag, remapArgs, isLoop);
        refreshStatus();
    };

    const handleStop = async () => {
        await stopRosbag();
        refreshStatus();
    };

    const addRemap = () => {
        setRemaps([...remaps, { from: '', to: '' }]);
    };

    const removeRemap = (index: number) => {
        setRemaps(remaps.filter((_, i) => i !== index));
    };

    const updateRemap = (index: number, field: 'from' | 'to', value: string) => {
        const newRemaps = [...remaps];
        newRemaps[index] = { ...newRemaps[index], [field]: value };
        setRemaps(newRemaps);
    };

    if (!isConnected) {
        return (
            <div className="surface-muted p-3 text-xs italic text-[var(--text-secondary)]">
                Connect to the server to play rosbags.
            </div>
        );
    }

    return (
        <div className="space-y-3">
            <div className={`rounded-lg border p-3 text-xs ${status.isPlaying ? 'border-green-500/40 bg-green-500/10' : 'border-white/20 bg-black/20'}`}>
                <div className="flex items-center justify-between gap-2">
                    <div className="min-w-0">
                        <p className={`font-semibold ${status.isPlaying ? 'text-green-300' : 'text-[var(--text-secondary)]'}`}>
                            {status.isPlaying ? 'Playing' : 'Stopped'}
                        </p>
                        {status.isPlaying && (
                            <p className="truncate text-[11px] text-[var(--text-secondary)]" title={status.currentBag}>
                                {status.currentBag}
                            </p>
                        )}
                    </div>
                    {status.isPlaying && (
                        <button onClick={handleStop} className="btn-danger px-3 py-1.5 text-[11px] font-semibold text-white">
                            Stop
                        </button>
                    )}
                </div>
            </div>

            <div className="flex items-center justify-between">
                <span className="panel-title">Available Bags</span>
                <button onClick={refreshBags} className="btn-secondary px-2.5 py-1 text-[11px] font-semibold">
                    Refresh
                </button>
            </div>

            <div className="surface-muted max-h-44 overflow-y-auto p-1 scrollbar-thin">
                {bags.length === 0 ? (
                    <div className="p-2 text-center text-xs text-[var(--text-secondary)]">
                        {isLoading ? 'Loading bags...' : 'No bags found'}
                    </div>
                ) : (
                    bags.map((bag) => (
                        <button
                            key={bag.relativePath}
                            onClick={() => setSelectedBag(bag.relativePath)}
                            className={`mb-1 w-full truncate rounded-md border px-2 py-2 text-left text-xs transition-colors last:mb-0 ${selectedBag === bag.relativePath
                                ? 'border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]'
                                : 'border-transparent text-[var(--text-primary)] hover:bg-white/10'
                                }`}
                            title={bag.path}
                        >
                            {bag.relativePath}
                        </button>
                    ))
                )}
            </div>

            <div className="surface-muted space-y-3 p-3">
                <div className="flex items-center justify-between">
                    <label className="inline-flex cursor-pointer items-center gap-2 text-xs text-[var(--text-primary)]">
                        <input
                            type="checkbox"
                            checked={isLoop}
                            onChange={(e) => setIsLoop(e.target.checked)}
                            className="h-4 w-4 rounded border-white/30 bg-transparent text-[var(--accent-color)]"
                        />
                        Loop playback
                    </label>
                    <button
                        onClick={handlePlay}
                        disabled={!selectedBag || status.isPlaying}
                        className="btn-primary px-3 py-1.5 text-xs disabled:cursor-not-allowed disabled:opacity-45"
                    >
                        Play Selected
                    </button>
                </div>

                <div className="space-y-2">
                    <div className="flex items-center justify-between">
                        <span className="panel-title">Topic Remapping</span>
                        <button onClick={addRemap} className="btn-secondary px-2 py-1 text-[11px] font-semibold">
                            Add
                        </button>
                    </div>
                    {remaps.map((remap, idx) => (
                        <div key={idx} className="grid grid-cols-[1fr_auto_1fr_auto] items-center gap-1.5">
                            <input
                                type="text"
                                placeholder="/scan"
                                value={remap.from}
                                onChange={(e) => updateRemap(idx, 'from', e.target.value)}
                                className="input-field px-2 py-1.5 text-[11px]"
                            />
                            <span className="text-[var(--text-secondary)]">:=</span>
                            <input
                                type="text"
                                placeholder="/points"
                                value={remap.to}
                                onChange={(e) => updateRemap(idx, 'to', e.target.value)}
                                className="input-field px-2 py-1.5 text-[11px]"
                            />
                            <button onClick={() => removeRemap(idx)} className="btn-danger px-2 py-1 text-[11px]">
                                Del
                            </button>
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
}
