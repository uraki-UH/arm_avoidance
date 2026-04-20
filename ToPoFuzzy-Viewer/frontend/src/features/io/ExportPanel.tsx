import { useState, useEffect } from 'react';
import { PointCloudData } from '../../types';
import { exportToPCD } from '../../utils/exporters/pcdExporter';
import { exportToPLY } from '../../utils/exporters/plyExporter';
import { exportToLAS } from '../../utils/exporters/lasExporter';

interface ExportPanelProps {
    pointClouds: PointCloudData[];
    selectedLayerId: string | null;
}

type ExportFormat = 'pcd-ascii' | 'pcd-binary' | 'ply-ascii' | 'ply-binary' | 'las';

export function ExportPanel({ pointClouds, selectedLayerId }: ExportPanelProps) {
    const [format, setFormat] = useState<ExportFormat>('pcd-ascii');
    const [includeTransform, setIncludeTransform] = useState(false);
    const [layerId, setLayerId] = useState<string>(selectedLayerId || '');

    useEffect(() => {
        if (selectedLayerId) {
            setLayerId(selectedLayerId);
        }
    }, [selectedLayerId]);

    const handleExport = () => {
        const cloud = pointClouds.find((pc) => pc.id === layerId);
        if (!cloud) {
            alert('Please select a layer to export');
            return;
        }

        if (cloud.count === 0) {
            alert('Selected layer has no points');
            return;
        }

        let blob: Blob;
        let extension: string;

        try {
            switch (format) {
                case 'pcd-ascii':
                    blob = exportToPCD(cloud, { format: 'ascii', includeTransform });
                    extension = '.pcd';
                    break;
                case 'pcd-binary':
                    blob = exportToPCD(cloud, { format: 'binary', includeTransform });
                    extension = '.pcd';
                    break;
                case 'ply-ascii':
                    blob = exportToPLY(cloud, { format: 'ascii', includeTransform });
                    extension = '.ply';
                    break;
                case 'ply-binary':
                    blob = exportToPLY(cloud, { format: 'binary', includeTransform });
                    extension = '.ply';
                    break;
                case 'las':
                    blob = exportToLAS(cloud, { includeTransform });
                    extension = '.las';
                    break;
                default:
                    throw new Error('Unknown format');
            }

            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `${cloud.name}${extension}`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);

            console.log(`Exported ${cloud.name} as ${format} (${blob.size} bytes)`);
        } catch (error) {
            console.error('Export failed:', error);
            alert(`Export failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    };

    return (
        <div className="space-y-4">
            <h3 className="panel-title">Export Point Cloud</h3>

            <div className="surface-muted space-y-1.5 p-3">
                <label className="control-label">Layer</label>
                <select
                    value={layerId}
                    onChange={(e) => setLayerId(e.target.value)}
                    className="select-field"
                >
                    <option value="">Select a layer...</option>
                    {pointClouds.map((pc) => (
                        <option key={pc.id} value={pc.id}>
                            {pc.name} ({pc.count.toLocaleString()} points)
                        </option>
                    ))}
                </select>
            </div>

            <div className="surface-muted space-y-1.5 p-3">
                <label className="control-label">Format</label>
                <select
                    value={format}
                    onChange={(e) => setFormat(e.target.value as ExportFormat)}
                    className="select-field"
                >
                    <optgroup label="PCD">
                        <option value="pcd-ascii">PCD (ASCII)</option>
                        <option value="pcd-binary">PCD (Binary)</option>
                    </optgroup>
                    <optgroup label="PLY">
                        <option value="ply-ascii">PLY (ASCII)</option>
                        <option value="ply-binary">PLY (Binary)</option>
                    </optgroup>
                    <optgroup label="LAS">
                        <option value="las">LAS 1.2</option>
                    </optgroup>
                </select>
            </div>

            <label className="inline-flex items-center gap-2 text-xs text-[var(--text-secondary)]">
                <input
                    type="checkbox"
                    id="include-transform"
                    checked={includeTransform}
                    onChange={(e) => setIncludeTransform(e.target.checked)}
                    className="h-4 w-4 rounded border-white/30 bg-white/10 text-[var(--accent-color)]"
                />
                Include transform (position, rotation, scale)
            </label>

            <button
                onClick={handleExport}
                disabled={!layerId}
                className="btn-primary w-full px-4 py-2 text-sm disabled:cursor-not-allowed disabled:opacity-45"
            >
                Export
            </button>

            {layerId && (
                <div className="text-center text-xs text-[var(--text-secondary)]">
                    {pointClouds.find((pc) => pc.id === layerId)?.count.toLocaleString() || 0} points will be exported.
                </div>
            )}
        </div>
    );
}
