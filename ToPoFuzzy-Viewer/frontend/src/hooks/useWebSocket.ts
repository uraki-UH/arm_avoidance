import { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import { deserializePointCloud } from '../utils/protocol';
import {
    PointCloudData,
    MarkerArrayData,
    GraphData,
    RobotData,
    RobotPoseInstance,
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

/** Returns true for errors that are expected on component unmount (RPC cancelled). */
const isUnmountCancellation = (e: unknown): boolean =>
    e instanceof Error && (
        e.message === 'WebSocket hook disposed' ||
        e.message === 'WebSocket not connected'
    );

const composeTransforms = (parent: TransformData, child: TransformData): TransformData => {
    const [px, py, pz, pw] = parent.quat;
    const [cx, cy, cz, cw] = child.quat;
    const [x, y, z] = child.pos;
    const tx = 2 * (py * z - pz * y);
    const ty = 2 * (pz * x - px * z);
    const tz = 2 * (px * y - py * x);
    const quat: [number, number, number, number] = [
        pw * cx + px * cw + py * cz - pz * cy,
        pw * cy - px * cz + py * cw + pz * cx,
        pw * cz + px * cy - py * cx + pz * cw,
        pw * cw - px * cx - py * cy - pz * cz,
    ];
    const norm = Math.hypot(...quat) || 1;

    return {
        frameId: parent.frameId,
        childFrameId: child.childFrameId,
        pos: [
            parent.pos[0] + x + pw * tx + py * tz - pz * ty,
            parent.pos[1] + y + pw * ty + pz * tx - px * tz,
            parent.pos[2] + z + pw * tz + px * ty - py * tx,
        ],
        quat: quat.map(value => value / norm) as [number, number, number, number],
    };
};

const resolveTransformTree = (
    relative: Record<string, TransformData>
): Record<string, TransformData> => {
    const resolved: Record<string, TransformData> = {};
    const resolving = new Set<string>();

    const resolve = (childFrameId: string): TransformData | null => {
        if (resolved[childFrameId]) return resolved[childFrameId];
        const local = relative[childFrameId];
        if (!local || resolving.has(childFrameId)) return null;

        resolving.add(childFrameId);
        const parent = relative[local.frameId] ? resolve(local.frameId) : null;
        resolving.delete(childFrameId);
        resolved[childFrameId] = parent ? composeTransforms(parent, local) : local;
        return resolved[childFrameId];
    };

    Object.keys(relative).forEach(resolve);
    return resolved;
};

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

type QueuedPointCloudUpdate = {
    layerId: string;
    buffer: ArrayBuffer;
};

const pointCloudTopicDecoder = new TextDecoder();

function readPointCloudLayerId(buffer: ArrayBuffer): string {
    if (buffer.byteLength < 1) {
        throw new Error('Point cloud packet is empty');
    }
    const topicLength = new DataView(buffer).getUint8(0);
    if (buffer.byteLength < 1 + topicLength) {
        throw new Error('Point cloud topic prefix is truncated');
    }
    const topicBytes = new Uint8Array(buffer, 1, topicLength);
    return pointCloudTopicDecoder.decode(topicBytes) || '__stream_fallback__';
}

function decodePointCloudUpdate(update: QueuedPointCloudUpdate): PointCloudData {
    const topicLength = new DataView(update.buffer).getUint8(0);
    const pcdBuffer = update.buffer.slice(1 + topicLength);
    const deserialized = deserializePointCloud(pcdBuffer);
    return {
        id: update.layerId,
        name: update.layerId,
        points: deserialized.positions,
        colors: deserialized.colors ? convertToFloat32RGB(deserialized.colors) : undefined,
        intensities: deserialized.intensities,
        count: deserialized.pointCount,
    };
}

type GraphStreamPayload = GraphData & {
    node_features?: Array<Record<string, unknown>>;
    cluster_features?: Array<Record<string, unknown>>;
};

function mergeGraphFeatures(graph: GraphStreamPayload): GraphData {
    const nodeFeatureById = new Map<number, Record<string, unknown>>();
    for (const feature of graph.node_features || []) {
        const rawId = feature.node_id ?? feature.id;
        const nodeId = typeof rawId === 'number' ? rawId : Number(rawId);
        if (!Number.isFinite(nodeId)) continue;
        nodeFeatureById.set(nodeId, feature);
    }

    const clusterFeatureById = new Map<number, Record<string, unknown>>();
    for (const feature of graph.cluster_features || []) {
        const rawId = feature.cluster_id ?? feature.id;
        const clusterId = typeof rawId === 'number' ? rawId : Number(rawId);
        if (!Number.isFinite(clusterId)) continue;
        clusterFeatureById.set(clusterId, feature);
    }

    return {
        ...graph,
        nodes: graph.nodes.map((node) => {
            const feature = nodeFeatureById.get(Number(node.id));
            if (!feature) return node;
            return {
                ...node,
                isGoal: feature.is_goal as boolean | undefined,
                manipValid: feature.manip_valid as boolean | undefined,
                manipValue: feature.manip_value as number | undefined,
                manipConditionNumber: feature.manip_condition_number as number | undefined,
                manipScale: feature.manip_scale as [number, number, number] | undefined,
                manipOrientation: feature.manip_orientation as [number, number, number, number] | undefined,
                rotationalManipValid: feature.rotational_manip_valid as boolean | undefined,
                rotationalManipValue: feature.rotational_manip_value as number | undefined,
                rotationalManipConditionNumber: feature.rotational_manip_condition_number as number | undefined,
                rotationalManipScale: feature.rotational_manip_scale as [number, number, number] | undefined,
                rotationalManipOrientation: feature.rotational_manip_orientation as [number, number, number, number] | undefined,
            };
        }),
        clusters: graph.clusters.map((cluster) => {
            const feature = clusterFeatureById.get(Number(cluster.id));
            if (!feature) return cluster;
            return {
                ...cluster,
                hasVelocityObservation: feature.has_velocity_observation as boolean | undefined,
                velCovXx: feature.vel_cov_xx as number | undefined,
                velCovXy: feature.vel_cov_xy as number | undefined,
                velCovYy: feature.vel_cov_yy as number | undefined,
            } as GraphData['clusters'][number];
        }),
    };
}

function robotPoseInstanceHasChanged(prev: RobotPoseInstance, next: RobotPoseInstance): boolean {
    if ((prev.opacity ?? 1) !== (next.opacity ?? 1)) {
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
    const prevBasePos = prev.basePosition || [0, 0, 0];
    const nextBasePos = next.basePosition || [0, 0, 0];
    if (
        prevBasePos[0] !== nextBasePos[0] ||
        prevBasePos[1] !== nextBasePos[1] ||
        prevBasePos[2] !== nextBasePos[2]
    ) {
        return true;
    }
    const prevBaseQuat = prev.baseOrientation || [0, 0, 0, 1];
    const nextBaseQuat = next.baseOrientation || [0, 0, 0, 1];
    if (
        prevBaseQuat[0] !== nextBaseQuat[0] ||
        prevBaseQuat[1] !== nextBaseQuat[1] ||
        prevBaseQuat[2] !== nextBaseQuat[2] ||
        prevBaseQuat[3] !== nextBaseQuat[3]
    ) {
        return true;
    }
    return false;
}

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
            a.semanticLabel !== b.semanticLabel ||
            a.semanticReliability !== b.semanticReliability ||
            a.age !== b.age ||
            a.winnerPointCount !== b.winnerPointCount ||
            a.isGoal !== b.isGoal ||
            a.manipValid !== b.manipValid ||
            a.manipValue !== b.manipValue ||
            a.manipConditionNumber !== b.manipConditionNumber ||
            a.manipScale?.[0] !== b.manipScale?.[0] ||
            a.manipScale?.[1] !== b.manipScale?.[1] ||
            a.manipScale?.[2] !== b.manipScale?.[2] ||
            a.manipOrientation?.[0] !== b.manipOrientation?.[0] ||
            a.manipOrientation?.[1] !== b.manipOrientation?.[1] ||
            a.manipOrientation?.[2] !== b.manipOrientation?.[2] ||
            a.manipOrientation?.[3] !== b.manipOrientation?.[3] ||
            a.rotationalManipValid !== b.rotationalManipValid ||
            a.rotationalManipValue !== b.rotationalManipValue ||
            a.rotationalManipConditionNumber !== b.rotationalManipConditionNumber ||
            a.rotationalManipScale?.[0] !== b.rotationalManipScale?.[0] ||
            a.rotationalManipScale?.[1] !== b.rotationalManipScale?.[1] ||
            a.rotationalManipScale?.[2] !== b.rotationalManipScale?.[2] ||
            a.rotationalManipOrientation?.[0] !== b.rotationalManipOrientation?.[0] ||
            a.rotationalManipOrientation?.[1] !== b.rotationalManipOrientation?.[1] ||
            a.rotationalManipOrientation?.[2] !== b.rotationalManipOrientation?.[2] ||
            a.rotationalManipOrientation?.[3] !== b.rotationalManipOrientation?.[3]
        ) {
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
            a.semanticLabel !== b.semanticLabel ||
            a.semanticReliability !== b.semanticReliability ||
            a.velocity[0] !== b.velocity[0] ||
            a.velocity[1] !== b.velocity[1] ||
            a.velocity[2] !== b.velocity[2] ||
            a.nodeIds.length !== b.nodeIds.length ||
            a.hasVelocityObservation !== b.hasVelocityObservation ||
            a.velCovXx !== b.velCovXx ||
            a.velCovXy !== b.velCovXy ||
            a.velCovYy !== b.velCovYy
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
    if ((prev.frameId || '') !== (next.frameId || '')) {
        return true;
    }
    if ((prev.opacity ?? 1) !== (next.opacity ?? 1)) {
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
    if ((prev.instances?.length || 0) !== (next.instances?.length || 0)) {
        return true;
    }
    if ((prev.linkNames?.length || 0) !== (next.linkNames?.length || 0)) {
        return true;
    }
    for (let i = 0; i < (next.linkNames?.length || 0); i++) {
        if (prev.linkNames?.[i] !== next.linkNames?.[i]) {
            return true;
        }
    }
    if ((prev.linkManipulabilities?.length || 0) !== (next.linkManipulabilities?.length || 0)) {
        return true;
    }
    for (let i = 0; i < (next.linkManipulabilities?.length || 0); i++) {
        const a = prev.linkManipulabilities?.[i];
        const b = next.linkManipulabilities?.[i];
        if (!a || !b) return true;
        if (a.linkName !== b.linkName ||
            a.manipValid !== b.manipValid ||
            a.manipValue !== b.manipValue ||
            a.manipConditionNumber !== b.manipConditionNumber ||
            a.manipCenter?.[0] !== b.manipCenter?.[0] ||
            a.manipCenter?.[1] !== b.manipCenter?.[1] ||
            a.manipCenter?.[2] !== b.manipCenter?.[2] ||
            a.manipScale?.[0] !== b.manipScale?.[0] ||
            a.manipScale?.[1] !== b.manipScale?.[1] ||
            a.manipScale?.[2] !== b.manipScale?.[2] ||
            a.manipOrientation?.[0] !== b.manipOrientation?.[0] ||
            a.manipOrientation?.[1] !== b.manipOrientation?.[1] ||
            a.manipOrientation?.[2] !== b.manipOrientation?.[2] ||
            a.manipOrientation?.[3] !== b.manipOrientation?.[3] ||
            a.rotationalManipValid !== b.rotationalManipValid ||
            a.rotationalManipValue !== b.rotationalManipValue ||
            a.rotationalManipConditionNumber !== b.rotationalManipConditionNumber ||
            a.rotationalManipScale?.[0] !== b.rotationalManipScale?.[0] ||
            a.rotationalManipScale?.[1] !== b.rotationalManipScale?.[1] ||
            a.rotationalManipScale?.[2] !== b.rotationalManipScale?.[2] ||
            a.rotationalManipOrientation?.[0] !== b.rotationalManipOrientation?.[0] ||
            a.rotationalManipOrientation?.[1] !== b.rotationalManipOrientation?.[1] ||
            a.rotationalManipOrientation?.[2] !== b.rotationalManipOrientation?.[2] ||
            a.rotationalManipOrientation?.[3] !== b.rotationalManipOrientation?.[3]) {
            return true;
        }
    }
    for (let i = 0; i < (next.instances?.length || 0); i++) {
        const a = prev.instances?.[i];
        const b = next.instances?.[i];
        if (!a || !b || robotPoseInstanceHasChanged(a, b)) {
            return true;
        }
    }
    return false;
}

export interface UseWebSocketReturn {
    sources: DataSource[];
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
    unsubscribeSource: (sourceId: string, removeLayer?: boolean) => Promise<{ success: boolean; sourceId: string; active: boolean }>;

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

type SendRpc = <T>(
    method: string,
    params?: Record<string, unknown>,
    timeoutMs?: number
) => Promise<T>;

function createViewerRpcApi(sendRpc: SendRpc, updateSources: (sources: DataSource[]) => void) {
    return {
        getSources: async (): Promise<DataSource[]> => {
            const result = await sendRpc<{ sources: DataSource[] }>('sources.list');
            updateSources(result.sources);
            return result.sources;
        },
        subscribeSource: (sourceId: string): Promise<{ success: boolean; sourceId: string; active: boolean }> =>
            sendRpc('sources.setActive', { sourceId, active: true }),
        unsubscribeSource: (
            sourceId: string,
            removeLayer = false
        ): Promise<{ success: boolean; sourceId: string; active: boolean }> =>
            sendRpc('sources.setActive', { sourceId, active: false, removeLayer }),
        listRosbags: async (): Promise<RosbagInfo[]> => {
            try {
                const result = await sendRpc<{ bags: RosbagInfo[] }>('rosbag.list');
                return result.bags || [];
            } catch (rpcError) {
                if (!isUnmountCancellation(rpcError)) console.error('Failed to list rosbags:', rpcError);
                return [];
            }
        },
        playRosbag: (path: string, remaps: string[], loop: boolean): Promise<{ success: boolean }> =>
            sendRpc('rosbag.play', { path, remaps, loop }),
        stopRosbag: (): Promise<{ success: boolean }> => sendRpc('rosbag.stop'),
        getRosbagStatus: (): Promise<PlaybackStatus> => sendRpc('rosbag.status'),
        listPointCloudFiles: async (): Promise<PointCloudFileInfo[]> => {
            try {
                const result = await sendRpc<{ files: PointCloudFileInfo[] }>('files.list');
                return result.files || [];
            } catch (rpcError) {
                if (!isUnmountCancellation(rpcError)) console.error('Failed to list files:', rpcError);
                return [];
            }
        },
        loadPointCloudFile: (
            path: string
        ): Promise<{ success: boolean; pointCount?: number; frameId?: string }> =>
            sendRpc('files.load', { path }),
        startGng: (
            params: GngParams
        ): Promise<{ success: boolean; pid?: number; inputTopic?: string }> =>
            sendRpc('gng.start', params),
        stopGng: (): Promise<{ success: boolean }> => sendRpc('gng.stop'),
        getGngStatus: async (): Promise<GngStatus> => {
            try {
                return await sendRpc<GngStatus>('gng.status');
            } catch (rpcError) {
                if (!isUnmountCancellation(rpcError)) console.error('Failed to get gng status:', rpcError);
                return { isRunning: false };
            }
        },
        listGngConfigs: async (): Promise<GngConfigInfo[]> => {
            try {
                const result = await sendRpc<{ configs: GngConfigInfo[] }>('gng.listConfigs');
                return result.configs || [];
            } catch (rpcError) {
                if (!isUnmountCancellation(rpcError)) console.error('Failed to list gng configs:', rpcError);
                return [];
            }
        },
        getParameters: (): Promise<NodeParameters> => sendRpc('params.get'),
        setParameter: (
            paramName: string,
            value: number | string | boolean
        ): Promise<SetParameterResult> => sendRpc('params.set', { paramName, value }),
        startContinuousPublish: (
            topic: string,
            rateHz: number
        ): Promise<{ success: boolean; topic?: string; rateHz?: number }> =>
            sendRpc('publish.startContinuous', { topic, rateHz }),
        stopContinuousPublish: (): Promise<{ success: boolean }> => sendRpc('publish.stopContinuous'),
        getContinuousPublishStatus: async (): Promise<ContinuousPublishStatus> => {
            try {
                return await sendRpc<ContinuousPublishStatus>('publish.status');
            } catch (rpcError) {
                if (!isUnmountCancellation(rpcError)) {
                    console.error('Failed to get continuous publish status:', rpcError);
                }
                return { isPublishing: false, topic: '', rateHz: 0, pointCount: 0 };
            }
        },
        openEditSession: (
            sourceTopic: string,
            targetFrame = 'map'
        ): Promise<EditSessionInfo> => sendRpc('edit.openSession', { sourceTopic, targetFrame }),
        addEditRegion: (
            sessionId: string,
            min: [number, number, number],
            max: [number, number, number],
            frameId = 'map'
        ): Promise<{ sessionId: string; region: EditRegion; regionCount: number }> =>
            sendRpc('edit.addRegion', { sessionId, min, max, frameId }),
        removeEditRegion: (
            sessionId: string,
            regionId: string
        ): Promise<{ sessionId: string; regionId: string; removed: boolean; regionCount: number }> =>
            sendRpc('edit.removeRegion', { sessionId, regionId }),
        clearEditRegions: (
            sessionId: string
        ): Promise<{ sessionId: string; regionCount: number }> =>
            sendRpc('edit.clearRegions', { sessionId }),
        getEditSession: (sessionId: string): Promise<EditSessionInfo> =>
            sendRpc('edit.getSession', { sessionId }),
        commitEditSession: (
            sessionId: string,
            transform: {
                position: [number, number, number];
                rotation: [number, number, number];
                scale: [number, number, number];
            },
            outputTopic?: string
        ): Promise<{ accepted: boolean; sessionId: string; jobId: string; outputTopic: string }> =>
            sendRpc('edit.commit', { sessionId, transform, outputTopic }, 60000),
        cancelEditSession: (
            sessionId: string
        ): Promise<{ success: boolean; sessionId: string }> =>
            sendRpc('edit.cancelSession', { sessionId }),
    };
}

export function useWebSocket(url: string): UseWebSocketReturn {
    const [sources, setSources] = useState<DataSource[]>([]);
    const [pointClouds, setPointClouds] = useState<Record<string, PointCloudData>>({});
    const [markerData, setMarkerData] = useState<Record<string, MarkerArrayData>>({});
    const [graphData, setGraphData] = useState<Record<string, GraphData>>({});
    const [robotData, setRobotData] = useState<Record<string, RobotData>>({});
    const [voxelData, setVoxelData] = useState<Record<string, VoxelData>>({});
    const [transforms, setTransforms] = useState<Record<string, TransformData>>({});
    const [lastJobEvent, setLastJobEvent] = useState<EditJobEvent | null>(null);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const wsRef = useRef<WebSocket | null>(null);
    const relativeTransformsRef = useRef<Record<string, TransformData>>({});

    const pendingTopicQueueRef = useRef<string[]>([]);
    const pendingRequestsRef = useRef<Map<string, PendingRequest>>(new Map());
    const pendingGraphUpdatesRef = useRef<Map<string, QueuedGraphUpdate>>(new Map());
    const pendingVoxelUpdatesRef = useRef<Map<string, QueuedVoxelUpdate>>(new Map());
    const pendingRobotPoseUpdatesRef = useRef<Map<string, QueuedRobotPoseUpdate>>(new Map());
    const pendingPointCloudUpdatesRef = useRef<Map<string, QueuedPointCloudUpdate>>(new Map());
    const flushScheduledRef = useRef<number | null>(null);
    const intentionalCloseRef = useRef(false);
    const reconnectCountRef = useRef(0);
    const reconnectTimerRef = useRef<number | null>(null);

    const flushBufferedStreams = useCallback(() => {
        flushScheduledRef.current = null;

        if (pendingPointCloudUpdatesRef.current.size > 0) {
            const pointCloudBatch = Array.from(pendingPointCloudUpdatesRef.current.values());
            pendingPointCloudUpdatesRef.current.clear();
            const decoded: Record<string, PointCloudData> = {};
            for (const update of pointCloudBatch) {
                try {
                    decoded[update.layerId] = decodePointCloudUpdate(update);
                } catch (parseError) {
                    console.error('Failed to parse binary point cloud:', parseError);
                }
            }
            if (Object.keys(decoded).length > 0) {
                setPointClouds((prev) => ({ ...prev, ...decoded }));
            }
        }

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
        pendingGraphUpdatesRef.current.delete(tag);
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
        const socket = wsRef.current;
        if (!socket || socket.readyState !== WebSocket.OPEN) {
            clearGraphLayer(tag);
            return;
        }

        clearGraphLayer(tag);
        socket.send(JSON.stringify({
            type: 'stream.graph.delete',
            tag,
        }));
    }, [clearGraphLayer]);

    const connect = useCallback(() => {
        const currentSocket = wsRef.current;
        if (currentSocket) {
            intentionalCloseRef.current = true;
            currentSocket.close();
        }

        try {
            pendingTopicQueueRef.current = [];
            flushPendingWithError('WebSocket reconnected');
            pendingGraphUpdatesRef.current.clear();
            pendingVoxelUpdatesRef.current.clear();
            pendingRobotPoseUpdatesRef.current.clear();
            pendingPointCloudUpdatesRef.current.clear();
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
                        const layerId = readPointCloudLayerId(buffer);

                        // Sync the queue just in case other logic depends on it
                        if (pendingTopicQueueRef.current.length > 0 && pendingTopicQueueRef.current[0] === layerId) {
                            pendingTopicQueueRef.current.shift();
                        }

                        pendingPointCloudUpdatesRef.current.set(layerId, { layerId, buffer });
                        scheduleStreamFlush();
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

                    if (payload && payload.id === 'sync_sources' && payload.ok === true) {
                        const newSources = payload.result?.sources;
                        if (Array.isArray(newSources)) {
                            setSources(newSources);
                        }
                        return;
                    }

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
                            const mergedGraph = mergeGraphFeatures(p.graph as GraphStreamPayload);
                            pendingGraphUpdatesRef.current.set(tag, {
                                tag,
                                graph: mergedGraph,
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
                                    voxel: {
                                        id: tag,
                                        tag,
                                        data: p.data,
                                        labels: Array.isArray(p.labels) ? p.labels : undefined,
                                        layout: p.layout,
                                        frameId: p.frameId,
                                    } as VoxelData,
                                });
                                scheduleStreamFlush();
                            }
                        },
                        'stream.tf': (p) => {
                            if (Array.isArray(p.transforms)) {
                                p.transforms.forEach((ts: TransformData) => {
                                    relativeTransformsRef.current[ts.childFrameId] = ts;
                                });
                                setTransforms(resolveTransformTree(relativeTransformsRef.current));
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
                            if (!targetId) return;
                            pendingPointCloudUpdatesRef.current.delete(targetId);
                            pendingVoxelUpdatesRef.current.delete(targetId);
                            setPointClouds(prev => {
                                const next = { ...prev };
                                delete next[targetId];
                                return next;
                            });
                            setVoxelData(prev => {
                                const next = { ...prev };
                                delete next[targetId];
                                return next;
                            });
                            setMarkerData(prev => {
                                const next = { ...prev };
                                delete next[targetId];
                                return next;
                            });
                            clearGraphLayer(targetId);
                        },
                        'stream.reset': (p) => handlers['stream.delete'](p),
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

            socket.onerror = () => {
                setError('WebSocket connection error');
            };

            socket.onclose = () => {
                if (wsRef.current === socket) {
                    wsRef.current = null;
                }
                setIsConnected(false);
                setSources([]);
                pendingTopicQueueRef.current = [];
                pendingGraphUpdatesRef.current.clear();
                pendingVoxelUpdatesRef.current.clear();
                pendingRobotPoseUpdatesRef.current.clear();
                pendingPointCloudUpdatesRef.current.clear();
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
                    connect();
                }, delay);
            };

            wsRef.current = socket;
        } catch (createError) {
            setError(createError instanceof Error ? createError.message : 'Connection failed');
        }
    }, [clearGraphLayer, flushPendingWithError, scheduleStreamFlush, url]);

    const disconnect = useCallback(() => {
        const socket = wsRef.current;
        if (socket) {
            intentionalCloseRef.current = true;
            wsRef.current = null;
            socket.close();
        }
    }, []);

    useEffect(() => {
        const pendingGraphUpdates = pendingGraphUpdatesRef.current;
        const pendingVoxelUpdates = pendingVoxelUpdatesRef.current;
        const pendingRobotPoseUpdates = pendingRobotPoseUpdatesRef.current;
        const pendingPointCloudUpdates = pendingPointCloudUpdatesRef.current;

        return () => {
            intentionalCloseRef.current = true;
            flushPendingWithError('WebSocket hook disposed');
            pendingGraphUpdates.clear();
            pendingVoxelUpdates.clear();
            pendingRobotPoseUpdates.clear();
            pendingPointCloudUpdates.clear();
            if (flushScheduledRef.current !== null) {
                window.cancelAnimationFrame(flushScheduledRef.current);
                flushScheduledRef.current = null;
            }
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, [flushPendingWithError]);

    const sendRpc = useCallback(
        <T,>(method: string, params: Record<string, unknown> = {}, timeoutMs = 10000): Promise<T> => {
            return new Promise((resolve, reject) => {
                const socket = wsRef.current;
                if (!socket || socket.readyState !== WebSocket.OPEN) {
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

                socket.send(JSON.stringify(request));
            });
        },
        []
    );

    const rpcApi = useMemo(
        () => createViewerRpcApi(sendRpc, setSources),
        [sendRpc]
    );

    return {
        sources,
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
        ...rpcApi,
    };
}

function convertToFloat32RGB(uint8Colors: Uint8Array): Float32Array {
    const float32Colors = new Float32Array(uint8Colors.length);
    for (let i = 0; i < uint8Colors.length; i++) {
        float32Colors[i] = uint8Colors[i] / 255.0;
    }
    return float32Colors;
}
