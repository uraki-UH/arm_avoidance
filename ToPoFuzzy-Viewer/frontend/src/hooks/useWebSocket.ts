import { useState, useEffect, useCallback, useRef } from 'react';
import { deserializePointCloud } from '../utils/protocol';
import {
    PointCloudData,
    MarkerArrayData,
    GraphData,
    RobotData,
    TransformData,
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
    VoxelData,
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

type QueuedGraphUpdate = {
    tag: string;
    graph: GraphData;
};

type QueuedVoxelUpdate = {
    tag: string;
    voxel: VoxelData;
};

type QueuedRobotPoseUpdate = {
    tag: string;
    robot: RobotData;
};

function graphHasChanged(prev: GraphData, next: GraphData): boolean {
    if (
        prev.timestamp !== next.timestamp ||
        prev.nodes.length !== next.nodes.length ||
        prev.edges.length !== next.edges.length ||
        prev.clusters.length !== next.clusters.length
    ) {
        return true;
    }

    for (let i = 0; i < next.nodes.length; i++) {
        const a = prev.nodes[i];
        const b = next.nodes[i];
        if (
            !a ||
            !b ||
            a.x !== b.x ||
            a.y !== b.y ||
            a.z !== b.z ||
            a.nx !== b.nx ||
            a.ny !== b.ny ||
            a.nz !== b.nz ||
            a.label !== b.label ||
            a.age !== b.age ||
            a.winnerPointCount !== b.winnerPointCount
        ) {
            return true;
        }

        const aMean = a.winnerPointMean || [0, 0, 0];
        const bMean = b.winnerPointMean || [0, 0, 0];
        if (aMean[0] !== bMean[0] || aMean[1] !== bMean[1] || aMean[2] !== bMean[2]) {
            return true;
        }

        const aCov = a.winnerPointCovariance || [0, 0, 0, 0, 0, 0, 0, 0, 0];
        const bCov = b.winnerPointCovariance || [0, 0, 0, 0, 0, 0, 0, 0, 0];
        for (let j = 0; j < aCov.length; j++) {
            if (aCov[j] !== bCov[j]) {
                return true;
            }
        }
    }

    for (let i = 0; i < next.edges.length; i++) {
        if (prev.edges[i] !== next.edges[i]) {
            return true;
        }
    }

    for (let i = 0; i < next.clusters.length; i++) {
        const a = prev.clusters[i];
        const b = next.clusters[i];
        if (
            !a ||
            !b ||
            a.id !== b.id ||
            a.label !== b.label ||
            a.pos[0] !== b.pos[0] ||
            a.pos[1] !== b.pos[1] ||
            a.pos[2] !== b.pos[2] ||
            a.scale[0] !== b.scale[0] ||
            a.scale[1] !== b.scale[1] ||
            a.scale[2] !== b.scale[2] ||
            a.quat[0] !== b.quat[0] ||
            a.quat[1] !== b.quat[1] ||
            a.quat[2] !== b.quat[2] ||
            a.quat[3] !== b.quat[3] ||
            a.match !== b.match ||
            a.reliability !== b.reliability ||
            a.velocity[0] !== b.velocity[0] ||
            a.velocity[1] !== b.velocity[1] ||
            a.velocity[2] !== b.velocity[2] ||
            a.nodeIds.length !== b.nodeIds.length
        ) {
            return true;
        }

        for (let j = 0; j < a.nodeIds.length; j++) {
            if (a.nodeIds[j] !== b.nodeIds[j]) {
                return true;
            }
        }
    }

    return false;
}

function robotHasChanged(prev: RobotData, next: RobotData): boolean {
    if (prev.timestamp !== next.timestamp) {
        return true;
    }
    if (prev.frameId !== next.frameId) {
        return true;
    }
    if ((prev.urdf || '') !== (next.urdf || '')) {
        return true;
    }
    if (prev.jointNames.length !== next.jointNames.length || prev.jointValues.length !== next.jointValues.length) {
        return true;
    }
    for (let i = 0; i < next.jointNames.length; i++) {
        if (prev.jointNames[i] !== next.jointNames[i]) {
            return true;
        }
    }
    for (let i = 0; i < next.jointValues.length; i++) {
        if (prev.jointValues[i] !== next.jointValues[i]) {
            return true;
        }
    }
    if ((prev.positions?.length || 0) !== (next.positions?.length || 0)) {
        return true;
    }
    if ((prev.orientations?.length || 0) !== (next.orientations?.length || 0)) {
        return true;
    }
    return false;
}

interface UseWebSocketReturn {
    pointClouds: Record<string, PointCloudData>;
    markerData: Record<string, MarkerArrayData>;
    graphData: Record<string, GraphData>;
    robotData: Record<string, RobotData>;
    voxelData: Record<string, VoxelData>;
    transforms: Record<string, TransformData>;
    lastJobEvent: EditJobEvent | null;
    isConnected: boolean;
    error: string | null;
    connect: () => void;
    disconnect: () => void;
    clearGraphLayer: (tag: string) => void;
    deleteGraphLayer: (tag: string) => void;

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
    const [pointClouds, setPointClouds] = useState<Record<string, PointCloudData>>({});
    const [markerData, setMarkerData] = useState<Record<string, MarkerArrayData>>({});
    const [graphData, setGraphData] = useState<Record<string, GraphData>>({});
    const [robotData, setRobotData] = useState<Record<string, RobotData>>({});
    const [voxelData, setVoxelData] = useState<Record<string, VoxelData>>({});
    const [transforms, setTransforms] = useState<Record<string, TransformData>>({});
    const [lastJobEvent, setLastJobEvent] = useState<EditJobEvent | null>(null);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [ws, setWs] = useState<WebSocket | null>(null);

    const pendingTopicQueueRef = useRef<string[]>([]);
    const pendingRequestsRef = useRef<Map<string, PendingRequest>>(new Map());
    const pendingGraphUpdatesRef = useRef<Map<string, QueuedGraphUpdate>>(new Map());
    const pendingVoxelUpdatesRef = useRef<Map<string, QueuedVoxelUpdate>>(new Map());
    const pendingRobotPoseUpdatesRef = useRef<Map<string, QueuedRobotPoseUpdate>>(new Map());
    const flushScheduledRef = useRef<number | null>(null);
    const intentionalCloseRef = useRef(false);
    const reconnectCountRef = useRef(0);
    const reconnectTimerRef = useRef<number | null>(null);

    const flushBufferedStreams = useCallback(() => {
        flushScheduledRef.current = null;

        if (pendingGraphUpdatesRef.current.size > 0) {
            const graphBatch = Array.from(pendingGraphUpdatesRef.current.values());
            pendingGraphUpdatesRef.current.clear();
            setGraphData((prev) => {
                let next = prev;
                let changed = false;

                for (const { tag, graph } of graphBatch) {
                    const existing = next[tag];
                    if (existing && !graphHasChanged(existing, graph)) {
                        continue;
                    }
                    if (next === prev) {
                        next = { ...prev };
                    }
                    next[tag] = graph;
                    changed = true;
                }

                return changed ? next : prev;
            });
        }

        if (pendingVoxelUpdatesRef.current.size > 0) {
            const voxelBatch = Array.from(pendingVoxelUpdatesRef.current.values());
            pendingVoxelUpdatesRef.current.clear();
            setVoxelData((prev) => {
                let next = prev;
                let changed = false;

                for (const { tag, voxel } of voxelBatch) {
                    if (next === prev) {
                        next = { ...prev };
                    }
                    next[tag] = voxel;
                    changed = true;
                }

                return changed ? next : prev;
            });
        }

        if (pendingRobotPoseUpdatesRef.current.size > 0) {
            const robotBatch = Array.from(pendingRobotPoseUpdatesRef.current.values());
            pendingRobotPoseUpdatesRef.current.clear();
            setRobotData((prev) => {
                let next = prev;
                let changed = false;

                for (const { tag, robot } of robotBatch) {
                    const existing = next[tag];
                    const mergedRobot: RobotData = existing
                        ? {
                            ...existing,
                            ...robot,
                            urdf: existing.urdf ?? robot.urdf,
                            jointNames: robot.jointNames?.length ? robot.jointNames : existing.jointNames,
                            jointValues: robot.jointValues?.length ? robot.jointValues : existing.jointValues,
                        }
                        : robot;

                    if (existing && !robotHasChanged(existing, mergedRobot)) {
                        continue;
                    }

                    if (next === prev) {
                        next = { ...prev };
                    }
                    next[tag] = mergedRobot;
                    changed = true;
                }

                return changed ? next : prev;
            });
        }
    }, []);

    const scheduleStreamFlush = useCallback(() => {
        if (flushScheduledRef.current !== null) {
            return;
        }
        flushScheduledRef.current = window.requestAnimationFrame(() => {
            flushBufferedStreams();
        });
    }, [flushBufferedStreams]);

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

    const deleteGraphLayer = useCallback((tag: string) => {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            clearGraphLayer(tag);
            return;
        }

        clearGraphLayer(tag);
        ws.send(JSON.stringify({
            type: 'stream.graph.delete',
            tag,
        }));
    }, [ws, clearGraphLayer]);

    const connect = useCallback(() => {
        if (ws) {
            intentionalCloseRef.current = true;
            ws.close();
        }

        try {
            pendingTopicQueueRef.current = [];
            flushPendingWithError('WebSocket reconnected');
            pendingGraphUpdatesRef.current.clear();
            pendingVoxelUpdatesRef.current.clear();
            pendingRobotPoseUpdatesRef.current.clear();
            if (flushScheduledRef.current !== null) {
                window.cancelAnimationFrame(flushScheduledRef.current);
                flushScheduledRef.current = null;
            }

            const socket = new WebSocket(url);
            socket.binaryType = 'arraybuffer';

            socket.onopen = () => {
                setIsConnected(true);
                setError(null);
                reconnectCountRef.current = 0;
                if (reconnectTimerRef.current !== null) {
                    window.clearTimeout(reconnectTimerRef.current);
                    reconnectTimerRef.current = null;
                }
                // Challenge: periodically request state until data arrives
                const challengeInterval = window.setInterval(() => {
                    if (socket.readyState === WebSocket.OPEN) {
                        socket.send(JSON.stringify({ type: 'request.state' }));
                    } else {
                        window.clearInterval(challengeInterval);
                    }
                }, 3000);

                // Stop challenging as soon as we get any data
                const stopChallenge = () => window.clearInterval(challengeInterval);
                socket.addEventListener('message', (e) => {
                    if (typeof e.data === 'string' && (e.data.includes('stream.graph') || e.data.includes('stream.robot') || e.data.includes('stream.marker_array'))) {
                        stopChallenge();
                    }
                }, { once: true });

                // Safety timeout: stop challenging after 15s regardless
                window.setTimeout(stopChallenge, 15000);
                
                // Send immediately on connect too
                console.log('WebSocket connected. Sending initial request.state challenge...');
                socket.send(JSON.stringify({ type: 'request.state' }));
            };

            socket.onmessage = (event) => {
                if (event.data instanceof ArrayBuffer) {
                    try {
                        const buffer = event.data;
                        const view = new DataView(buffer);
                        
                        // Read topic name prefix (1 byte length + topic name string)
                        const topicLen = view.getUint8(0);
                        const topicBytes = new Uint8Array(buffer, 1, topicLen);
                        const topicName = new TextDecoder().decode(topicBytes);
                        
                        // Remaining buffer is the PCDX data
                        const pcdBuffer = buffer.slice(1 + topicLen);
                        const deserialized = deserializePointCloud(pcdBuffer);
                        
                        // Use the extracted topic name as the layerId
                        const layerId = topicName || '__stream_fallback__';

                        // Sync the queue just in case other logic depends on it
                        if (pendingTopicQueueRef.current.length > 0 && pendingTopicQueueRef.current[0] === layerId) {
                            pendingTopicQueueRef.current.shift();
                        }

                        const data: PointCloudData = {
                            id: layerId,
                            name: layerId,
                            points: deserialized.positions,
                            colors: deserialized.colors ? convertToFloat32RGB(deserialized.colors) : undefined,
                            intensities: deserialized.intensities,
                            count: deserialized.pointCount,
                        };
                        setPointClouds((prev) => ({
                            ...prev,
                            [layerId]: data
                        }));
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

                    // Handle RPC responses
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

                    // Dispatcher for streaming messages
                    const type = payload.type as string;
                    const tag = payload.tag || 'default';

                    const handlers: Record<string, (p: any) => void> = {
                        'stream.pointcloud.meta': (p) => {
                            if (typeof p.topic === 'string') pendingTopicQueueRef.current.push(p.topic);
                        },
                        'stream.marker_array': (p) => {
                            if (!Array.isArray(p.markers)) return;
                            setMarkerData(prev => ({
                                ...prev,
                                [tag]: {
                                    id: tag, name: p.name || tag, tag,
                                    frameId: p.frameId || undefined,
                                    markers: p.markers, count: p.markers.length,
                                    visible: prev[tag]?.visible ?? true,
                                } as MarkerArrayData,
                            }));
                        },
                        'stream.marker': (p) => {
                            if (!p.marker) return;
                            setMarkerData(prev => ({
                                ...prev,
                                [tag]: {
                                    id: tag, name: p.name || tag, tag,
                                    frameId: p.frameId || undefined,
                                    markers: [p.marker], count: 1,
                                    visible: prev[tag]?.visible ?? true,
                                } as MarkerArrayData,
                            }));
                        },
                        'stream.graph': (p) => {
                            if (!p.graph) return;
                            pendingGraphUpdatesRef.current.set(tag, {
                                tag,
                                graph: p.graph as GraphData,
                            });
                            scheduleStreamFlush();
                        },
                        'stream.graph.delete': (p) => {
                            if (typeof p.tag === 'string') clearGraphLayer(p.tag);
                        },
                        'stream.robot.description': (p) => {
                            if (p.robot) setRobotData(prev => ({ ...prev, [tag]: p.robot as RobotData }));
                        },
                        'stream.robot.pose': (p) => {
                            if (!p.robot) return;
                            pendingRobotPoseUpdatesRef.current.set(tag, {
                                tag,
                                robot: p.robot as RobotData,
                            });
                            scheduleStreamFlush();
                        },
                        'stream.voxel': (p) => {
                            if (p.data) {
                                pendingVoxelUpdatesRef.current.set(tag, {
                                    tag,
                                    voxel: { id: tag, tag, data: p.data, layout: p.layout, frameId: p.frameId } as VoxelData,
                                });
                                scheduleStreamFlush();
                            }
                        },
                        'stream.tf': (p) => {
                            if (p.transforms) {
                                setTransforms(prev => {
                                    const next = { ...prev };
                                    p.transforms.forEach((ts: TransformData) => { next[ts.childFrameId] = ts; });
                                    return next;
                                });
                            }
                        },
                        'stream.robot.delete': (p) => {
                            if (typeof p.tag === 'string') setRobotData(prev => {
                                const next = { ...prev };
                                delete next[p.tag];
                                return next;
                            });
                        },
                        'stream.delete': (p) => {
                            const targetId = p.topic || p.tag || p.id;
                            if (targetId) setPointClouds(prev => {
                                const next = { ...prev };
                                delete next[targetId];
                                return next;
                            });
                        },
                        'stream.pointcloud.delete': (p) => handlers['stream.delete'](p),
                        'stream.remove_layer': (p) => handlers['stream.delete'](p),
                        'stream.robot': (p) => handlers['stream.robot.description'](p), // Legacy support
                        'job.progress': (p) => setLastJobEvent(p as EditJobEvent),
                        'job.completed': (p) => setLastJobEvent(p as EditJobEvent),
                        'job.failed': (p) => setLastJobEvent(p as EditJobEvent),
                    };

                    if (handlers[type]) {
                        handlers[type](payload);
                    }
                } catch (jsonError) {
                    console.error('Failed to parse text message:', jsonError);
                }
            };

            socket.onerror = (e) => {
                console.error('WebSocket Error:', e);
                setError('WebSocket connection error');
            };

            socket.onclose = () => {
                setIsConnected(false);
                pendingTopicQueueRef.current = [];
                pendingGraphUpdatesRef.current.clear();
                pendingVoxelUpdatesRef.current.clear();
                pendingRobotPoseUpdatesRef.current.clear();
                if (flushScheduledRef.current !== null) {
                    window.cancelAnimationFrame(flushScheduledRef.current);
                    flushScheduledRef.current = null;
                }
                flushPendingWithError('WebSocket closed');
                
                if (intentionalCloseRef.current) {
                    intentionalCloseRef.current = false;
                    return;
                }
                
                // Fixed 2s interval is better for detecting backend startup
                const delay = 2000;
                setError(`Disconnected. Reconnecting...`);
                
                if (reconnectTimerRef.current !== null) {
                    window.clearTimeout(reconnectTimerRef.current);
                }
                
                reconnectTimerRef.current = window.setTimeout(() => {
                    console.log('Attempting to reconnect WebSocket...');
                    connect();
                }, delay);
            };

            setWs(socket);
        } catch (createError) {
            setError(createError instanceof Error ? createError.message : 'Connection failed');
        }
    }, [flushPendingWithError, scheduleStreamFlush, url]);

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
            pendingGraphUpdatesRef.current.clear();
            pendingVoxelUpdatesRef.current.clear();
            pendingRobotPoseUpdatesRef.current.clear();
            if (flushScheduledRef.current !== null) {
                window.cancelAnimationFrame(flushScheduledRef.current);
                flushScheduledRef.current = null;
            }
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
        const result = await sendRpc<{ success: boolean; sourceId: string; active: boolean }>('sources.setActive', { sourceId, active: false });
        if (result.success) {
            // Immediately clear local data to prevent App.tsx from re-initializing it
            setPointClouds((prev) => { const next = { ...prev }; delete next[sourceId]; return next; });
            setVoxelData((prev) => { const next = { ...prev }; delete next[sourceId]; return next; });
            setMarkerData((prev) => { const next = { ...prev }; delete next[sourceId]; return next; });
        }
        return result;
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
        pointClouds,
        markerData,
        graphData,
        robotData,
        voxelData,
        transforms,
        lastJobEvent,
        isConnected,
        error,
        connect,
        disconnect,
        clearGraphLayer,
        deleteGraphLayer,
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
