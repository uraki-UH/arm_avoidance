import { useState, useEffect, useCallback, useRef } from 'react';
import { deserializePointCloud } from '../utils/protocol';
import {
    PointCloudData,
    GraphData,
    RobotData,
    DataSource,
    RosbagInfo,
    PlaybackStatus,
    PointCloudFileInfo,
    GngStatus,
    GngParams,
    GngConfigInfo,
    ContinuousPublishStatus,
    NodeParameters,
    SetParameterResult,
    EditRegion,
    EditSessionInfo,
    EditJobEvent,
} from '../types';
import { generateUUID } from '../utils/uuid';

export type {
    DataSource,
    RosbagInfo,
    PlaybackStatus,
    PointCloudFileInfo,
    GngStatus,
    GngParams,
    GngConfigInfo,
    ContinuousPublishStatus,
    NodeParameters,
    SetParameterResult,
    EditRegion,
    EditSessionInfo,
    EditJobEvent,
};

interface PendingRequest {
    resolve: (value: unknown) => void;
    reject: (reason?: unknown) => void;
    timer: number;
}

interface UseWebSocketReturn {
    pointCloud: PointCloudData | null;
    graphData: Record<string, GraphData>;
    robotData: RobotData | null;
    lastJobEvent: EditJobEvent | null;
    isConnected: boolean;
    error: string | null;
    connect: () => void;
    disconnect: () => void;
    clearGraphLayer: (tag: string) => void;

    getSources: () => Promise<DataSource[]>;
    subscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string; active: boolean }>;
    unsubscribeSource: (sourceId: string) => Promise<{ success: boolean; sourceId: string; active: boolean }>;

    listRosbags: () => Promise<RosbagInfo[]>;
    playRosbag: (path: string, remaps: string[], loop: boolean) => Promise<{ success: boolean }>;
    stopRosbag: () => Promise<{ success: boolean }>;
    getRosbagStatus: () => Promise<PlaybackStatus>;

    listPointCloudFiles: () => Promise<PointCloudFileInfo[]>;
    loadPointCloudFile: (path: string) => Promise<{ success: boolean; pointCount?: number; frameId?: string }>;

    startGng: (params: GngParams) => Promise<{ success: boolean; pid?: number; inputTopic?: string }>;
    stopGng: () => Promise<{ success: boolean }>;
    getGngStatus: () => Promise<GngStatus>;
    listGngConfigs: () => Promise<GngConfigInfo[]>;

    getParameters: () => Promise<NodeParameters>;
    setParameter: (paramName: string, value: number | string | boolean) => Promise<SetParameterResult>;

    startContinuousPublish: (topic: string, rateHz: number) => Promise<{ success: boolean; topic?: string; rateHz?: number }>;
    stopContinuousPublish: () => Promise<{ success: boolean }>;
    getContinuousPublishStatus: () => Promise<ContinuousPublishStatus>;

    openEditSession: (sourceTopic: string, targetFrame?: string) => Promise<EditSessionInfo>;
    addEditRegion: (
        sessionId: string,
        min: [number, number, number],
        max: [number, number, number],
        frameId?: string
    ) => Promise<{ sessionId: string; region: EditRegion; regionCount: number }>;
    removeEditRegion: (sessionId: string, regionId: string) => Promise<{ sessionId: string; regionId: string; removed: boolean; regionCount: number }>;
    clearEditRegions: (sessionId: string) => Promise<{ sessionId: string; regionCount: number }>;
    getEditSession: (sessionId: string) => Promise<EditSessionInfo>;
    commitEditSession: (
        sessionId: string,
        transform: {
            position: [number, number, number];
            rotation: [number, number, number];
            scale: [number, number, number];
        },
        outputTopic?: string
    ) => Promise<{ accepted: boolean; sessionId: string; jobId: string; outputTopic: string }>;
    cancelEditSession: (sessionId: string) => Promise<{ success: boolean; sessionId: string }>;
}

export function useWebSocket(url: string): UseWebSocketReturn {
    const [pointCloud, setPointCloud] = useState<PointCloudData | null>(null);
    const [graphData, setGraphData] = useState<Record<string, GraphData>>({});
    const [robotData, setRobotData] = useState<RobotData | null>(null);
    const [lastJobEvent, setLastJobEvent] = useState<EditJobEvent | null>(null);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [ws, setWs] = useState<WebSocket | null>(null);

    const pendingTopicQueueRef = useRef<string[]>([]);
    const pendingRequestsRef = useRef<Map<string, PendingRequest>>(new Map());
    const intentionalCloseRef = useRef(false);

    const flushPendingWithError = useCallback((message: string) => {
        for (const [, pending] of pendingRequestsRef.current) {
            window.clearTimeout(pending.timer);
            pending.reject(new Error(message));
        }
        pendingRequestsRef.current.clear();
    }, []);

    const clearGraphLayer = useCallback((tag: string) => {
        setGraphData((prev) => {
            if (!prev[tag]) {
                return prev;
            }
            const next = { ...prev };
            delete next[tag];
            return next;
        });
    }, []);

    const connect = useCallback(() => {
        if (ws) {
            intentionalCloseRef.current = true;
            ws.close();
        }

        try {
            pendingTopicQueueRef.current = [];
            flushPendingWithError('WebSocket reconnected');

            const socket = new WebSocket(url);
            socket.binaryType = 'arraybuffer';

            socket.onopen = () => {
                setIsConnected(true);
                setError(null);
            };

            socket.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    try {
                        const deserialized = deserializePointCloud(event.data);
                        const queuedTopic = pendingTopicQueueRef.current.shift();
                        const layerId = queuedTopic || '__stream_fallback__';

                        const data: PointCloudData = {
                            id: layerId,
                            name: queuedTopic || 'Streamed Point Cloud',
                            points: deserialized.positions,
                            colors: deserialized.colors ? convertToFloat32RGB(deserialized.colors) : undefined,
                            intensities: deserialized.intensities,
                            count: deserialized.pointCount,
                        };
                        setPointCloud(data);
                    } catch (parseError) {
                        console.error('Failed to parse binary point cloud:', parseError);
                    }
                    return;
                }

                if (typeof event.data !== 'string') {
                    return;
                }

                try {
                    const payload = JSON.parse(event.data);

                    if (payload && typeof payload.id === 'string' && pendingRequestsRef.current.has(payload.id)) {
                        const pending = pendingRequestsRef.current.get(payload.id)!;
                        pendingRequestsRef.current.delete(payload.id);
                        window.clearTimeout(pending.timer);

                        if (payload.ok === true) {
                            pending.resolve(payload.result ?? {});
                        } else {
                            const message = payload.error?.message || 'RPC error';
                            pending.reject(new Error(message));
                        }
                        return;
                    }

                    if (payload.type === 'stream.pointcloud.meta' && typeof payload.topic === 'string') {
                        pendingTopicQueueRef.current.push(payload.topic);
                        return;
                    }

                    if (payload.type === 'stream.graph' && payload.graph) {
                        const tag = payload.tag || 'default';
                        setGraphData((prev) => ({
                            ...prev,
                            [tag]: payload.graph as GraphData,
                        }));
                        return;
                    }

                    if (payload.type === 'stream.graph.delete' && typeof payload.tag === 'string') {
                        clearGraphLayer(payload.tag);
                        return;
                    }

                    if (payload.type === 'stream.robot' && payload.robot) {
                        setRobotData(payload.robot as RobotData);
                        return;
                    }

                    if (payload.type === 'job.progress' || payload.type === 'job.completed' || payload.type === 'job.failed') {
                        setLastJobEvent(payload as EditJobEvent);
                        return;
                    }
                } catch (jsonError) {
                    console.error('Failed to parse text message:', jsonError);
                }
            };

            socket.onerror = () => {
                setError('WebSocket connection error');
            };

            socket.onclose = (event) => {
                setIsConnected(false);
                pendingTopicQueueRef.current = [];
                flushPendingWithError('WebSocket closed');
                if (intentionalCloseRef.current) {
                    intentionalCloseRef.current = false;
                    return;
                }
                if (event.code !== 1000) {
                    setError(`WebSocket disconnected (code=${event.code}${event.reason ? `, reason=${event.reason}` : ''})`);
                }
            };

            setWs(socket);
        } catch (createError) {
            setError(createError instanceof Error ? createError.message : 'Connection failed');
        }
    }, [flushPendingWithError, url, ws]);

    const disconnect = useCallback(() => {
        if (ws) {
            intentionalCloseRef.current = true;
            ws.close();
            setWs(null);
        }
    }, [ws]);

    useEffect(() => {
        return () => {
            intentionalCloseRef.current = true;
            flushPendingWithError('WebSocket hook disposed');
            if (ws) {
                ws.close();
            }
        };
    }, [flushPendingWithError, ws]);

    const sendRpc = useCallback(
        <T,>(method: string, params: Record<string, unknown> = {}, timeoutMs = 10000): Promise<T> => {
            return new Promise((resolve, reject) => {
                if (!ws || ws.readyState !== WebSocket.OPEN) {
                    reject(new Error('WebSocket not connected'));
                    return;
                }

                const requestId = generateUUID();
                const request = {
                    id: requestId,
                    method,
                    params,
                };

                const timer = window.setTimeout(() => {
                    pendingRequestsRef.current.delete(requestId);
                    reject(new Error(`Request timeout: ${method}`));
                }, timeoutMs);

                pendingRequestsRef.current.set(requestId, {
                    resolve: (value) => resolve(value as T),
                    reject,
                    timer,
                });

                ws.send(JSON.stringify(request));
            });
        },
        [ws]
    );

    const getSources = useCallback(async (): Promise<DataSource[]> => {
        const result = await sendRpc<{ sources: DataSource[] }>('sources.list');
        return result.sources;
    }, [sendRpc]);

    const subscribeSource = useCallback(async (sourceId: string): Promise<{ success: boolean; sourceId: string; active: boolean }> => {
        return sendRpc('sources.setActive', { sourceId, active: true });
    }, [sendRpc]);

    const unsubscribeSource = useCallback(async (sourceId: string): Promise<{ success: boolean; sourceId: string; active: boolean }> => {
        return sendRpc('sources.setActive', { sourceId, active: false });
    }, [sendRpc]);

    const listRosbags = useCallback(async (): Promise<RosbagInfo[]> => {
        try {
            const result = await sendRpc<{ bags: RosbagInfo[] }>('rosbag.list');
            return result.bags || [];
        } catch (rpcError) {
            console.error('Failed to list rosbags:', rpcError);
            return [];
        }
    }, [sendRpc]);

    const playRosbag = useCallback(async (path: string, remaps: string[], loop: boolean): Promise<{ success: boolean }> => {
        return sendRpc('rosbag.play', { path, remaps, loop });
    }, [sendRpc]);

    const stopRosbag = useCallback(async (): Promise<{ success: boolean }> => {
        return sendRpc('rosbag.stop');
    }, [sendRpc]);

    const getRosbagStatus = useCallback(async (): Promise<PlaybackStatus> => {
        return sendRpc('rosbag.status');
    }, [sendRpc]);

    const listPointCloudFiles = useCallback(async (): Promise<PointCloudFileInfo[]> => {
        try {
            const result = await sendRpc<{ files: PointCloudFileInfo[] }>('files.list');
            return result.files || [];
        } catch (rpcError) {
            console.error('Failed to list files:', rpcError);
            return [];
        }
    }, [sendRpc]);

    const loadPointCloudFile = useCallback(async (path: string): Promise<{ success: boolean; pointCount?: number; frameId?: string }> => {
        return sendRpc('files.load', { path });
    }, [sendRpc]);

    const startGng = useCallback(async (params: GngParams): Promise<{ success: boolean; pid?: number; inputTopic?: string }> => {
        return sendRpc('gng.start', params);
    }, [sendRpc]);

    const stopGng = useCallback(async (): Promise<{ success: boolean }> => {
        return sendRpc('gng.stop');
    }, [sendRpc]);

    const getGngStatus = useCallback(async (): Promise<GngStatus> => {
        try {
            return await sendRpc<GngStatus>('gng.status');
        } catch (rpcError) {
            console.error('Failed to get gng status:', rpcError);
            return { isRunning: false };
        }
    }, [sendRpc]);

    const listGngConfigs = useCallback(async (): Promise<GngConfigInfo[]> => {
        try {
            const result = await sendRpc<{ configs: GngConfigInfo[] }>('gng.listConfigs');
            return result.configs || [];
        } catch (rpcError) {
            console.error('Failed to list gng configs:', rpcError);
            return [];
        }
    }, [sendRpc]);

    const getParameters = useCallback(async (): Promise<NodeParameters> => {
        return sendRpc('params.get');
    }, [sendRpc]);

    const setParameter = useCallback(async (paramName: string, value: number | string | boolean): Promise<SetParameterResult> => {
        return sendRpc('params.set', { paramName, value });
    }, [sendRpc]);

    const startContinuousPublish = useCallback(async (
        topic: string,
        rateHz: number
    ): Promise<{ success: boolean; topic?: string; rateHz?: number }> => {
        return sendRpc('publish.startContinuous', { topic, rateHz });
    }, [sendRpc]);

    const stopContinuousPublish = useCallback(async (): Promise<{ success: boolean }> => {
        return sendRpc('publish.stopContinuous');
    }, [sendRpc]);

    const getContinuousPublishStatus = useCallback(async (): Promise<ContinuousPublishStatus> => {
        try {
            return await sendRpc<ContinuousPublishStatus>('publish.status');
        } catch (rpcError) {
            console.error('Failed to get continuous publish status:', rpcError);
            return { isPublishing: false, topic: '', rateHz: 0, pointCount: 0 };
        }
    }, [sendRpc]);

    const openEditSession = useCallback(async (sourceTopic: string, targetFrame: string = 'map'): Promise<EditSessionInfo> => {
        return sendRpc('edit.openSession', { sourceTopic, targetFrame });
    }, [sendRpc]);

    const addEditRegion = useCallback(async (
        sessionId: string,
        min: [number, number, number],
        max: [number, number, number],
        frameId: string = 'map'
    ): Promise<{ sessionId: string; region: EditRegion; regionCount: number }> => {
        return sendRpc('edit.addRegion', { sessionId, min, max, frameId });
    }, [sendRpc]);

    const removeEditRegion = useCallback(async (
        sessionId: string,
        regionId: string
    ): Promise<{ sessionId: string; regionId: string; removed: boolean; regionCount: number }> => {
        return sendRpc('edit.removeRegion', { sessionId, regionId });
    }, [sendRpc]);

    const clearEditRegions = useCallback(async (sessionId: string): Promise<{ sessionId: string; regionCount: number }> => {
        return sendRpc('edit.clearRegions', { sessionId });
    }, [sendRpc]);

    const getEditSession = useCallback(async (sessionId: string): Promise<EditSessionInfo> => {
        return sendRpc('edit.getSession', { sessionId });
    }, [sendRpc]);

    const commitEditSession = useCallback(async (
        sessionId: string,
        transform: {
            position: [number, number, number];
            rotation: [number, number, number];
            scale: [number, number, number];
        },
        outputTopic?: string
    ): Promise<{ accepted: boolean; sessionId: string; jobId: string; outputTopic: string }> => {
        return sendRpc('edit.commit', { sessionId, transform, outputTopic }, 60000);
    }, [sendRpc]);

    const cancelEditSession = useCallback(async (sessionId: string): Promise<{ success: boolean; sessionId: string }> => {
        return sendRpc('edit.cancelSession', { sessionId });
    }, [sendRpc]);

    return {
        pointCloud,
        graphData,
        robotData,
        lastJobEvent,
        isConnected,
        error,
        connect,
        disconnect,
        clearGraphLayer,
        getSources,
        subscribeSource,
        unsubscribeSource,
        listRosbags,
        playRosbag,
        stopRosbag,
        getRosbagStatus,
        listPointCloudFiles,
        loadPointCloudFile,
        startGng,
        stopGng,
        getGngStatus,
        listGngConfigs,
        getParameters,
        setParameter,
        startContinuousPublish,
        stopContinuousPublish,
        getContinuousPublishStatus,
        openEditSession,
        addEditRegion,
        removeEditRegion,
        clearEditRegions,
        getEditSession,
        commitEditSession,
        cancelEditSession,
    };
}

function convertToFloat32RGB(uint8Colors: Uint8Array): Float32Array {
    const float32Colors = new Float32Array(uint8Colors.length);
    for (let i = 0; i < uint8Colors.length; i++) {
        float32Colors[i] = uint8Colors[i] / 255.0;
    }
    return float32Colors;
}
