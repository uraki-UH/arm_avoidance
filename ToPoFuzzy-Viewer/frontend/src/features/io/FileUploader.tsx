import { useState, useCallback } from 'react';
import { loadPointCloudFile } from '../../utils/fileLoader';
import { PointCloudData } from '../../types';

interface FileUploaderProps {
    onLoad: (data: PointCloudData) => void;
    onPublish?: (data: PointCloudData) => Promise<{ success: boolean; message?: string }>;
}

export function FileUploader({ onLoad, onPublish }: FileUploaderProps) {
    const [isDragging, setIsDragging] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const [isPublishing, setIsPublishing] = useState(false);
    const [publishStatus, setPublishStatus] = useState<{ success: boolean; message?: string } | null>(null);
    const [error, setError] = useState<string | null>(null);
    const [autoPublish, setAutoPublish] = useState(false);

    const handleFile = useCallback(async (file: File) => {
        setIsLoading(true);
        setError(null);
        setPublishStatus(null);

        try {
            const data = await loadPointCloudFile(file);
            onLoad(data);

            if (autoPublish && onPublish) {
                setIsPublishing(true);
                try {
                    const result = await onPublish(data);
                    setPublishStatus(result);
                    if (!result.success) {
                        setError(`Publish failed: ${result.message}`);
                    }
                } catch (err) {
                    setError('Publish failed due to connection error');
                } finally {
                    setIsPublishing(false);
                }
            }
        } catch (err) {
            setError(err instanceof Error ? err.message : 'Failed to load file');
            console.error('File load error:', err);
        } finally {
            setIsLoading(false);
        }
    }, [onLoad, onPublish, autoPublish]);

    const handleDrop = useCallback((e: React.DragEvent) => {
        e.preventDefault();
        setIsDragging(false);

        const file = e.dataTransfer.files[0];
        if (file) {
            handleFile(file);
        }
    }, [handleFile]);

    const handleDragOver = useCallback((e: React.DragEvent) => {
        e.preventDefault();
        setIsDragging(true);
    }, []);

    const handleDragLeave = useCallback(() => {
        setIsDragging(false);
    }, []);

    const handleFileInput = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (file) {
            handleFile(file);
        }
    }, [handleFile]);

    return (
        <div
            onDrop={handleDrop}
            onDragOver={handleDragOver}
            onDragLeave={handleDragLeave}
            className={`
        p-6 rounded-lg border-2 border-dashed
        transition-all duration-200 bg-black/70 backdrop-blur-sm
        ${isDragging ? 'border-blue-500 bg-blue-500/20' : 'border-gray-600'}
        ${isLoading ? 'opacity-50 pointer-events-none' : ''}
      `}
        >
            <div className="text-white text-center">
                <div className="mb-2 font-semibold">
                    {isLoading ? 'Loading...' : isPublishing ? 'Publishing...' : 'Drop Point Cloud File'}
                </div>
                <div className="text-sm text-gray-400 mb-3">
                    Supports: PCD, LAS, PLY, LandXML (.xml/.landxml)
                </div>

                {onPublish && (
                    <div className="mb-4 flex items-center justify-center gap-2">
                        <label className="flex items-center gap-2 text-xs text-[var(--accent-color)] cursor-pointer hover:text-white transition-colors">
                            <input
                                type="checkbox"
                                checked={autoPublish}
                                onChange={(e) => {
                                    setAutoPublish(e.target.checked);
                                    setPublishStatus(null);
                                }}
                                className="w-3.5 h-3.5 rounded border-gray-500 bg-transparent text-[var(--accent-color)] focus:ring-[var(--accent-color)]"
                            />
                            Auto-publish to ROS
                        </label>
                    </div>
                )}

                <label className="cursor-pointer inline-block px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded transition-colors shadow-lg shadow-blue-900/200">
                    <span>Browse Files</span>
                    <input
                        type="file"
                        accept=".pcd,.las,.laz,.ply,.xml,.landxml,.landxm"
                        onChange={handleFileInput}
                        className="hidden"
                    />
                </label>

                {publishStatus && (
                    <div className={`mt-3 text-xs p-1 rounded ${publishStatus.success ? 'text-green-400 bg-green-900/20' : 'text-red-400 bg-red-900/20'}`}>
                        {publishStatus.success ? '✅ Published to ROS' : '❌ Publish Failed'}
                    </div>
                )}

                {error && (
                    <div className="mt-3 text-red-400 text-sm">
                        {error}
                    </div>
                )}
            </div>
        </div>
    );
}
