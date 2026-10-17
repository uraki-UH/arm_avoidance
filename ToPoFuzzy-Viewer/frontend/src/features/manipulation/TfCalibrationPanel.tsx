import { useCallback, useEffect, useRef, useState } from 'react';
import type { PointCloudData, sensor_calibration_state } from '../../types';
import { generateUUID } from '../../utils/uuid';

interface TfCalibrationPanelProps {
    is_connected: boolean;
    cloud: PointCloudData;
    get_calibration: (source_frame: string) => Promise<sensor_calibration_state>;
    apply_calibration: (mount_id: string, revision: number, source_frame: string,
        delta: { pos: number[]; rot_deg: number[]; scl: number[] }, operation_id: string) => Promise<sensor_calibration_state>;
    on_applied: (cloud_id: string) => void;
}

export function TfCalibrationPanel({ is_connected, cloud, get_calibration, apply_calibration, on_applied }: TfCalibrationPanelProps) {
    const [state, set_state] = useState<sensor_calibration_state | null>(null);
    const [mount_id, set_mount_id] = useState('');
    const [is_busy, set_is_busy] = useState(false);
    const [error, set_error] = useState('');
    const on_applied_ref = useRef(on_applied);
    on_applied_ref.current = on_applied;
    const baseline_ref = useRef<{ mount_id: string; source_frame: string; revision: number } | null>(null);
    const source_frame = cloud.frameId ?? '';
    const source_ref = useRef(source_frame);
    source_ref.current = source_frame;
    const pending_ref = useRef<{ operation_id: string; cloud_id: string; mount_id: string } | null>(null);
    const refresh = useCallback(async () => {
        if (!is_connected || !source_frame) return;
        try {
            const next = await get_calibration(source_frame);
            if (source_ref.current !== source_frame) return;
            const pending = pending_ref.current;
            if (pending && next.mounts.some(item => item.mount_id === pending.mount_id && item.last_operation_id === pending.operation_id && item.is_tf_applied)) {
                pending_ref.current = null;
                on_applied_ref.current(pending.cloud_id);
            }
            set_state(next);
            set_mount_id(current => next.mounts.some(mount => mount.mount_id === current) ? current :
                (next.mounts.find(mount => mount.can_apply) ?? next.mounts[0])?.mount_id ?? '');
            set_error('');
        } catch (value) {
            if (source_ref.current === source_frame) set_error(value instanceof Error ? value.message : '校正情報の取得失敗');
        }
    }, [get_calibration, is_connected, source_frame]);
    useEffect(() => {
        set_state(null);
        void refresh();
        const timer = window.setInterval(() => void refresh(), 2000);
        return () => window.clearInterval(timer);
    }, [refresh]);
    const mount = state?.mounts.find(value => value.mount_id === mount_id);
    const has_adjustment = [...(cloud.position ?? [0, 0, 0]), ...(cloud.rotation ?? [0, 0, 0])].some(value => value !== 0);
    const has_scale = (cloud.scale ?? [1, 1, 1]).some(value => value !== 1);
    useEffect(() => {
        if (!has_adjustment) baseline_ref.current = null;
        else if (mount && (!baseline_ref.current || baseline_ref.current.mount_id !== mount.mount_id || baseline_ref.current.source_frame !== source_frame)) {
            baseline_ref.current = { mount_id: mount.mount_id, source_frame, revision: mount.revision };
        }
    }, [has_adjustment, mount, source_frame]);
    const is_stale_preview = has_adjustment && baseline_ref.current !== null && mount !== undefined &&
        baseline_ref.current.mount_id === mount.mount_id && baseline_ref.current.revision !== mount.revision;

    const apply = async () => {
        if (!mount || !source_frame) return;
        const operation = { operation_id: generateUUID(), cloud_id: cloud.id, mount_id };
        pending_ref.current = operation;
        set_is_busy(true);
        set_error('');
        try {
            const next = await apply_calibration(mount_id, baseline_ref.current?.revision ?? mount.revision, source_frame, {
                pos: cloud.position ?? [0, 0, 0],
                rot_deg: (cloud.rotation ?? [0, 0, 0]).map(value => value * 180 / Math.PI),
                scl: cloud.scale ?? [1, 1, 1],
            }, operation.operation_id);
            if (!next.is_tf_applied) throw new Error('ROS側のTF反映が未確認です');
            on_applied(operation.cloud_id);
            pending_ref.current = null;
            set_state(next);
        } catch (value) {
            // 応答消失時の保存済み操作ID照合。補正の二重適用回避
            try {
                const next = await get_calibration(source_frame);
                if (next.mounts.some(item => item.mount_id === operation.mount_id && item.last_operation_id === operation.operation_id && item.is_tf_applied)) {
                    on_applied(operation.cloud_id);
                    pending_ref.current = null;
                    set_state(next);
                    return;
                }
            } catch { /* 接続復旧までの調整値保持 */ }
            set_error(value instanceof Error ? value.message : 'ROS側への反映失敗');
        } finally {
            set_is_busy(false);
        }
    };
    return <div className="space-y-2">
        <div className="flex gap-2">
            <select className="min-w-0 flex-1" value={mount_id} disabled={!is_connected || is_busy}
                onChange={event => set_mount_id(event.target.value)}>
                {!state?.mounts.length && <option value="">校正対象なし</option>}
                {state?.mounts.map(value => <option key={value.mount_id} value={value.mount_id}>
                    {value.mount_id} · v{value.revision}
                </option>)}
            </select>
            <button className="btn-secondary px-2 py-1" disabled={!is_connected || is_busy} onClick={() => void refresh()}>更新</button>
        </div>
        {mount && <div className="text-xs text-[var(--text-secondary)]">{mount.parent_frame} → {mount.child_frame}</div>}
        <button className="btn-primary w-full px-3 py-2" onClick={() => void apply()}
            disabled={!is_connected || is_busy || !mount?.can_apply || !has_adjustment || has_scale || is_stale_preview}>
            {is_busy ? '反映中…' : 'ROS TFへ適用・保存'}
        </button>
        {(error || mount?.reason || has_scale || is_stale_preview) && <div className="text-xs text-amber-400">
            {error || (is_stale_preview ? 'ROS側の基準姿勢が変更されています。調整をリセットして位置合わせをやり直してください' : has_scale ? 'TFへ反映できるのは位置・回転のみです' : mount?.reason)}
        </div>}
    </div>;
}
