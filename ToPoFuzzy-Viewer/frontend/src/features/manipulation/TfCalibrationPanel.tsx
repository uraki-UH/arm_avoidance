import { useCallback, useEffect, useState } from 'react';
import { NodeParameters, SetParameterResult } from '../../types';

interface TfCalibrationPanelProps {
    isConnected: boolean;
    getParameters: () => Promise<NodeParameters>;
    setParameter: (paramName: string, value: number | string | boolean) => Promise<SetParameterResult>;
}

interface ParamState {
    name: string;
    label: string;
    value: number;
    min: number;
    max: number;
    step: number;
}

export function TfCalibrationPanel({
    isConnected,
    getParameters,
    setParameter
}: TfCalibrationPanelProps) {
    const [params, setParams] = useState<Record<string, ParamState>>({});
    const [isLoading, setIsLoading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const refreshParams = useCallback(async () => {
        if (!isConnected) return;
        setIsLoading(true);
        setError(null);
        try {
            const result = await getParameters();
            // Filter parameters belonging to pointcloud_transformer
            // Note: In some ROS2 versions, parameters might not have the node prefix in the name
            // but our viewer bridge might be filtering them.
            const tfParams = result.parameters.filter(p => 
                ['x', 'y', 'z', 'roll', 'pitch', 'yaw'].includes(p.name)
            );

            if (tfParams.length === 0) {
                setError('No TF calibration parameters found. Is transform_pointcloud.py running?');
                return;
            }

            const nextParams: Record<string, ParamState> = {};
            tfParams.forEach(p => {
                nextParams[p.name] = {
                    name: p.name,
                    label: p.name.toUpperCase(),
                    value: typeof p.value === 'number' ? p.value : Number(p.value),
                    min: p.name.length === 1 ? -10 : -180, // Guess limits if not provided
                    max: p.name.length === 1 ? 10 : 180,
                    step: p.name.length === 1 ? 0.001 : 0.1
                };
            });
            setParams(nextParams);
        } catch (err) {
            setError('Failed to load parameters');
        } finally {
            setIsLoading(false);
        }
    }, [isConnected, getParameters]);

    useEffect(() => {
        if (isConnected) {
            refreshParams();
        } else {
            setParams({});
            setError(null);
        }
    }, [isConnected, refreshParams]);

    const handleParamChange = async (name: string, value: number) => {
        setParams(prev => ({
            ...prev,
            [name]: { ...prev[name], value }
        }));
        
        try {
            await setParameter(name, value);
        } catch (err) {
            setError(`Failed to set ${name}`);
        }
    };

    if (!isConnected) {
        return <div className="p-3 text-xs italic text-[var(--text-secondary)]">Connect to calibrate TF.</div>;
    }

    return (
        <div className="surface-muted space-y-3 p-3">
            <div className="flex items-center justify-between">
                <span className="panel-title">Real-time TF Calibration</span>
                <button onClick={refreshParams} disabled={isLoading} className="btn-secondary px-2 py-1 text-[10px]">
                    {isLoading ? '...' : 'Refresh'}
                </button>
            </div>

            {error && <div className="text-[10px] text-red-400">{error}</div>}

            <div className="space-y-4">
                {['x', 'y', 'z'].map(key => params[key] && (
                    <div key={key} className="space-y-1">
                        <div className="flex justify-between text-[10px]">
                            <span>{params[key].label} (m)</span>
                            <span className="font-mono">{params[key].value.toFixed(3)}</span>
                        </div>
                        <input
                            type="range"
                            min="-5"
                            max="5"
                            step="0.001"
                            value={params[key].value}
                            onChange={e => handleParamChange(key, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}

                <div className="h-px bg-white/5 my-2" />

                {['roll', 'pitch', 'yaw'].map(key => params[key] && (
                    <div key={key} className="space-y-1">
                        <div className="flex justify-between text-[10px]">
                            <span>{params[key].label} (deg)</span>
                            <span className="font-mono">{params[key].value.toFixed(2)}°</span>
                        </div>
                        <input
                            type="range"
                            min="-180"
                            max="180"
                            step="0.1"
                            value={params[key].value}
                            onChange={e => handleParamChange(key, parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>
                ))}
            </div>
            
            <p className="text-[9px] text-[var(--text-secondary)] leading-tight italic mt-2">
                * Adjust parameters to align point cloud with robot model.
                Values are applied in real-time to the pointcloud_transformer node.
            </p>
        </div>
    );
}
