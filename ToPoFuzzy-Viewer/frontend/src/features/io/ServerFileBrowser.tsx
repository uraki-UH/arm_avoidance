import { useState, useEffect, useCallback } from 'react';

export interface PointCloudFileInfo {
    path: string;
    name: string;
    relativePath: string;
    format: string;
    fileSize: number;
}

interface ServerFileBrowserProps {
    isConnected: boolean;
    listPointCloudFiles: () => Promise<PointCloudFileInfo[]>;
    loadPointCloudFile: (path: string) => Promise<{ success: boolean; pointCount?: number }>;
}

export function ServerFileBrowser({
    isConnected,
    listPointCloudFiles,
    loadPointCloudFile
}: ServerFileBrowserProps) {
    const [files, setFiles] = useState<PointCloudFileInfo[]>([]);
    const [selectedFile, setSelectedFile] = useState<string | null>(null);
    const [isLoading, setIsLoading] = useState(false);
    const [loadStatus, setLoadStatus] = useState<{ success: boolean; message?: string } | null>(null);

    const refreshFiles = useCallback(async () => {
        if (!isConnected) return;
        setIsLoading(true);
        try {
            const list = await listPointCloudFiles();
            setFiles(list);
        } catch (e) {
            console.error('Failed to list files:', e);
        } finally {
            setIsLoading(false);
        }
    }, [isConnected, listPointCloudFiles]);

    useEffect(() => {
        if (isConnected) {
            refreshFiles();
        }
    }, [isConnected, refreshFiles]);

    const handleLoad = async () => {
        if (!selectedFile) return;

        setIsLoading(true);
        setLoadStatus(null);
        try {
            const result = await loadPointCloudFile(selectedFile);
            setLoadStatus({
                success: result.success,
                message: result.success ? `Loaded ${result.pointCount?.toLocaleString()} points` : 'Failed to load'
            });
        } catch (e) {
            setLoadStatus({
                success: false,
                message: e instanceof Error ? e.message : 'Load failed'
            });
        } finally {
            setIsLoading(false);
        }
    };

    const formatFileSize = (bytes: number): string => {
        if (bytes < 1024) return `${bytes} B`;
        if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
        return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
    };

    if (!isConnected) {
        return (
            <div className="surface-muted p-3 text-xs italic text-[var(--text-secondary)]">
                Connect to the server to browse point cloud files.
            </div>
        );
    }

    return (
        <div className="space-y-3 pt-2">
            <div className="flex items-center justify-between">
                <span className="panel-title">Server Files</span>
                <button onClick={refreshFiles} className="btn-secondary px-2.5 py-1 text-[11px] font-semibold">
                    Refresh
                </button>
            </div>

            <div className="surface-muted max-h-52 overflow-y-auto p-1 scrollbar-thin">
                {files.length === 0 ? (
                    <div className="p-2 text-center text-xs text-[var(--text-secondary)]">
                        {isLoading ? 'Loading files...' : 'No files found'}
                    </div>
                ) : (
                    files.map((file) => (
                        <button
                            key={file.relativePath}
                            onClick={() => setSelectedFile(file.relativePath)}
                            className={`mb-1 flex w-full items-center justify-between rounded-md px-2 py-2 text-left text-xs transition-colors last:mb-0 ${selectedFile === file.relativePath
                                ? 'border border-[var(--accent-color)]/50 bg-[var(--accent-soft)] text-[var(--accent-strong)]'
                                : 'border border-transparent text-[var(--text-primary)] hover:bg-white/10'
                                }`}
                            title={file.path}
                        >
                            <div className="min-w-0">
                                <div className="flex items-center gap-1.5">
                                    <span className="rounded bg-white/10 px-1 py-0.5 text-[10px] uppercase tracking-[0.08em]">{file.format}</span>
                                    <span className="truncate">{file.name}</span>
                                </div>
                                <p className="mt-0.5 truncate text-[10px] text-[var(--text-secondary)]">{file.relativePath}</p>
                            </div>
                            <span className="ml-2 shrink-0 text-[10px] text-[var(--text-secondary)]">{formatFileSize(file.fileSize)}</span>
                        </button>
                    ))
                )}
            </div>

            <button
                onClick={handleLoad}
                disabled={!selectedFile || isLoading}
                className="btn-primary w-full px-4 py-2 text-sm disabled:cursor-not-allowed disabled:opacity-45"
            >
                {isLoading ? 'Loading...' : 'Load Selected File'}
            </button>

            {loadStatus && (
                <div className={`rounded-md border px-3 py-2 text-xs ${loadStatus.success
                    ? 'border-green-500/40 bg-green-500/10 text-green-200'
                    : 'border-red-500/40 bg-red-500/10 text-red-200'
                    }`}>
                    {loadStatus.message}
                </div>
            )}

        </div>
    );
}
