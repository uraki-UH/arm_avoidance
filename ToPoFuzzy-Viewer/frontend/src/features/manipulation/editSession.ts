import { useEffect, useState } from 'react';
import type { UseWebSocketReturn } from '../../hooks/useWebSocket';
import type { EditJobEvent, EditRegion, PointCloudData } from '../../types';

export interface DraftRegion {
    center: [number, number, number];
    size: [number, number, number];
}

export interface EditJobStatus {
    isRunning: boolean;
    jobId: string;
    progress: number;
    stage: string;
    error?: string;
}

type EditApi = Pick<
    UseWebSocketReturn,
    | 'subscribeSource'
    | 'openEditSession'
    | 'addEditRegion'
    | 'removeEditRegion'
    | 'clearEditRegions'
    | 'commitEditSession'
    | 'cancelEditSession'
>;

interface UseEditSessionOptions {
    pointClouds: PointCloudData[];
    selectedLayerId: string | null;
    onSelectLayer: (layerId: string | null) => void;
    isConnected: boolean;
    lastJobEvent: EditJobEvent | null;
    api: EditApi;
}

const DEFAULT_DRAFT_REGION: DraftRegion = {
    center: [0, 0, 0],
    size: [1, 1, 1],
};

function cloneVec3(
    value: [number, number, number] | undefined,
    fallback: [number, number, number]
): [number, number, number] {
    if (!value) return fallback;
    return [value[0], value[1], value[2]];
}

function createDraftRegionFromCloud(cloud: PointCloudData): DraftRegion {
    if (!cloud.points || cloud.count === 0) {
        return DEFAULT_DRAFT_REGION;
    }

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;

    for (let i = 0; i < cloud.count; i++) {
        const x = cloud.points[i * 3];
        const y = cloud.points[i * 3 + 1];
        const z = cloud.points[i * 3 + 2];
        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        minZ = Math.min(minZ, z);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
        maxZ = Math.max(maxZ, z);
    }

    return {
        center: [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2],
        size: [
            Math.max(0.5, (maxX - minX) * 0.2),
            Math.max(0.5, (maxY - minY) * 0.2),
            Math.max(0.5, (maxZ - minZ) * 0.2),
        ],
    };
}

function draftToMinMax(draft: DraftRegion): {
    min: [number, number, number];
    max: [number, number, number];
} {
    const halfX = draft.size[0] / 2;
    const halfY = draft.size[1] / 2;
    const halfZ = draft.size[2] / 2;

    return {
        min: [draft.center[0] - halfX, draft.center[1] - halfY, draft.center[2] - halfZ],
        max: [draft.center[0] + halfX, draft.center[1] + halfY, draft.center[2] + halfZ],
    };
}

export function useEditSession({
    pointClouds,
    selectedLayerId,
    onSelectLayer,
    isConnected,
    lastJobEvent,
    api,
}: UseEditSessionOptions) {
    const [isEditMode, setIsEditMode] = useState(false);
    const [editLayerId, setEditLayerId] = useState<string | null>(null);
    const [editSessionId, setEditSessionId] = useState<string | null>(null);
    const [editRegions, setEditRegions] = useState<EditRegion[]>([]);
    const [draftRegion, setDraftRegion] = useState<DraftRegion>(DEFAULT_DRAFT_REGION);
    const [regionGizmoMode, setRegionGizmoMode] = useState<'translate' | 'scale'>('translate');
    const [editJobStatus, setEditJobStatus] = useState<EditJobStatus | null>(null);

    const {
        subscribeSource,
        openEditSession,
        addEditRegion,
        removeEditRegion,
        clearEditRegions,
        commitEditSession,
        cancelEditSession,
    } = api;
    const activeJobId = editJobStatus?.jobId;

    useEffect(() => {
        if (!lastJobEvent || !activeJobId || lastJobEvent.jobId !== activeJobId) return;

        if (lastJobEvent.type === 'job.progress') {
            setEditJobStatus((current) => current ? {
                ...current,
                progress: lastJobEvent.progress,
                stage: lastJobEvent.stage,
            } : current);
            return;
        }

        if (lastJobEvent.type === 'job.failed') {
            setEditJobStatus((current) => current ? {
                ...current,
                isRunning: false,
                stage: 'failed',
                error: `${lastJobEvent.error.code}: ${lastJobEvent.error.message}`,
            } : current);
            return;
        }

        if (lastJobEvent.type === 'job.completed') {
            void (async () => {
                try {
                    await subscribeSource(lastJobEvent.publishedTopic);
                } catch (subscribeError) {
                    console.warn('Failed to subscribe edited topic:', subscribeError);
                }

                onSelectLayer(lastJobEvent.publishedTopic);
                setIsEditMode(false);
                setEditLayerId(null);
                setEditSessionId(null);
                setEditRegions([]);
                setEditJobStatus((current) => current ? {
                    ...current,
                    isRunning: false,
                    progress: 100,
                    stage: 'completed',
                } : current);
            })();
        }
    }, [activeJobId, lastJobEvent, onSelectLayer, subscribeSource]);

    const startEdit = async () => {
        if (!selectedLayerId || isEditMode) return;
        const cloud = pointClouds.find((pointCloud) => pointCloud.id === selectedLayerId);
        if (!cloud || cloud.id.endsWith('/edited')) return;
        if (!cloud.id.startsWith('/')) {
            alert('Only ROS topic layers can be edited in backend session mode.');
            return;
        }

        const maxRetries = 3;
        for (let attempt = 0; attempt <= maxRetries; attempt++) {
            try {
                const session = await openEditSession(cloud.id, 'map');
                setEditSessionId(session.sessionId);
                setEditLayerId(cloud.id);
                setIsEditMode(true);
                onSelectLayer(cloud.id);
                setEditRegions(session.regions || []);
                setDraftRegion(createDraftRegionFromCloud(cloud));
                setRegionGizmoMode('translate');
                setEditJobStatus(null);
                return;
            } catch (startError) {
                const message = startError instanceof Error ? startError.message : '';
                const snapshotNotReady = message.includes('snapshot') || message.includes('Retry');
                if (snapshotNotReady && attempt < maxRetries) {
                    console.log(`Edit session snapshot not ready, retrying (${attempt + 1}/${maxRetries})...`);
                    await new Promise((resolve) => setTimeout(resolve, 1000));
                    continue;
                }
                console.error('Failed to start edit session:', startError);
                alert(`Failed to start edit session: ${message || 'Unknown error'}`);
            }
        }
    };

    const cancelEdit = async () => {
        if (editSessionId) {
            try {
                await cancelEditSession(editSessionId);
            } catch (cancelError) {
                console.warn('Failed to cancel edit session on backend:', cancelError);
            }
        }

        setIsEditMode(false);
        setEditLayerId(null);
        setEditSessionId(null);
        setEditRegions([]);
        setEditJobStatus(null);
    };

    const addRegion = async () => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;
        const { min, max } = draftToMinMax(draftRegion);

        try {
            const result = await addEditRegion(editSessionId, min, max, 'map');
            setEditRegions((current) => [...current, result.region]);
        } catch (addError) {
            console.error('Failed to add region:', addError);
            alert(`Failed to add region: ${addError instanceof Error ? addError.message : 'Unknown error'}`);
        }
    };

    const removeRegion = async (regionId: string) => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;

        try {
            await removeEditRegion(editSessionId, regionId);
            setEditRegions((current) => current.filter((region) => region.regionId !== regionId));
        } catch (removeError) {
            console.error('Failed to remove region:', removeError);
            alert(`Failed to remove region: ${removeError instanceof Error ? removeError.message : 'Unknown error'}`);
        }
    };

    const clearRegions = async () => {
        if (!isEditMode || !editSessionId || editJobStatus?.isRunning) return;

        try {
            await clearEditRegions(editSessionId);
            setEditRegions([]);
        } catch (clearError) {
            console.error('Failed to clear regions:', clearError);
            alert(`Failed to clear regions: ${clearError instanceof Error ? clearError.message : 'Unknown error'}`);
        }
    };

    const publishEditedCloud = async () => {
        if (!isEditMode || !editSessionId || !isConnected) return;
        const cloud = pointClouds.find((pointCloud) => pointCloud.id === editLayerId);
        if (!cloud) return;

        try {
            const result = await commitEditSession(editSessionId, {
                position: cloneVec3(cloud.position, [0, 0, 0]),
                rotation: cloneVec3(cloud.rotation, [0, 0, 0]),
                scale: cloneVec3(cloud.scale, [1, 1, 1]),
            });
            setEditJobStatus({
                isRunning: true,
                jobId: result.jobId,
                progress: 0,
                stage: 'queued',
            });
        } catch (commitError) {
            console.error('Failed to commit edit session:', commitError);
            alert(`Failed to commit edit session: ${commitError instanceof Error ? commitError.message : 'Unknown error'}`);
        }
    };

    const selectedCloud = pointClouds.find((pointCloud) => pointCloud.id === selectedLayerId);
    const selectedIsEditedLayer = selectedCloud?.id.endsWith('/edited') ?? false;
    const selectedIsTopicLayer = selectedCloud?.id.startsWith('/') ?? false;
    const canStartEdit = Boolean(selectedCloud) && selectedIsTopicLayer && !selectedIsEditedLayer && !isEditMode;
    const startEditDisabledReason = !selectedCloud
        ? 'Select a layer to edit.'
        : !selectedIsTopicLayer
            ? 'Only ROS topic layers can be edited.'
            : selectedIsEditedLayer
                ? 'Select the original topic layer (not /edited).'
                : undefined;

    return {
        isEditMode,
        editLayerId,
        editRegions,
        draftRegion,
        regionGizmoMode,
        setRegionGizmoMode,
        editJobStatus,
        canStartEdit,
        startEditDisabledReason,
        startEdit,
        cancelEdit,
        addRegion,
        removeRegion,
        clearRegions,
        publishEditedCloud,
        updateDraftRegion: (center: DraftRegion['center'], size: DraftRegion['size']) => {
            setDraftRegion({ center, size });
        },
    };
}
