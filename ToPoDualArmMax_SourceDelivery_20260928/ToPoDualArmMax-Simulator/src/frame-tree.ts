import * as THREE from 'three';
import type { TransformData } from '@viewer/types';
export interface frame_robot { readonly links: Readonly<Record<string, THREE.Object3D>>; readonly ros_frame_prefix?: string }
interface cached_transform { parent: string; matrix: THREE.Matrix4; received_ms: number; is_static: boolean }

// ROSの親子TFとSimulatorの実配置を結ぶ表示座標系
export class frame_tree {
    private root_frame = 'world';
    private transforms = new Map<string, cached_transform>();
    private unresolved = new Set<string>();
    private cache = new Map<string, THREE.Matrix4 | null>();
    private now_ms = 0;
    constructor(private robot: () => frame_robot) {}
    get fixed_frame() { return this.root_frame; }
    set fixed_frame(frame: string) {
        const root_frame = frame.trim() || 'world';
        if (this.root_frame === root_frame) return;
        this.root_frame = root_frame;
        this.cache.clear();
        this.unresolved.clear();
    }
    get unresolved_frames(): ReadonlySet<string> { return new Set(this.unresolved); }
    clear() { this.transforms.clear(); this.cache.clear(); this.unresolved.clear(); }
    update(items: TransformData[], is_static: boolean) {
        const pending = new Map<string, cached_transform>();
        const received_ms = performance.now();
        for (const item of items) {
            if (!item.frameId || !item.childFrameId || item.pos.length !== 3 || item.quat.length !== 4 ||
                ![...item.pos, ...item.quat].every(Number.isFinite) || Math.hypot(...item.quat) < 1e-9) continue;
            pending.set(item.childFrameId, { parent: item.frameId, is_static, received_ms,
                matrix: new THREE.Matrix4().compose(new THREE.Vector3().fromArray(item.pos),
                    new THREE.Quaternion().fromArray(item.quat).normalize(), new THREE.Vector3(1, 1, 1)) });
        }
        const num_new_frames = [...pending.keys()].filter(frame => !this.transforms.has(frame)).length;
        if (this.transforms.size + num_new_frames > 8192) throw Error('TFフレーム数超過');
        for (const [frame, transform] of pending) this.transforms.set(frame, transform);
        this.cache.clear();
    }
    begin_frame(now_ms: number) { this.now_ms = now_ms; this.cache.clear(); this.unresolved.clear(); }
    // 合成結果の内部再利用と、呼出し側の行列変更からの隔離
    resolve(frame: string): THREE.Matrix4 | null { return this.resolve_cached(frame, new Set())?.clone() ?? null; }
    private resolve_cached(frame: string, seen: Set<string>): THREE.Matrix4 | null {
        if (!frame || seen.has(frame)) return null;
        if (this.cache.has(frame)) return this.cache.get(frame)!;
        let matrix: THREE.Matrix4 | null = null;
        if (frame === this.fixed_frame) matrix = new THREE.Matrix4();
        else {
            const robot = this.robot();
            const prefix = robot.ros_frame_prefix ? `${robot.ros_frame_prefix}/` : '';
            const link = prefix && frame.startsWith(prefix) ? robot.links[frame.slice(prefix.length)] : undefined;
            // 選択モデルの名前空間はSimulatorの実リンクへ明示対応。ROS姿勢・期限切れTFとの混在なし。
            if (link && this.fixed_frame === 'world') {
                link.updateWorldMatrix(true, false);
                matrix = link.matrixWorld.clone();
            } else {
                seen.add(frame);
                const transform = this.transforms.get(frame);
                if (transform && (transform.is_static || this.now_ms - transform.received_ms <= 2000)) {
                    const parent = this.resolve_cached(transform.parent, seen);
                    if (parent) matrix = parent.clone().multiply(transform.matrix);
                } else if (!transform && frame === 'base_footprint' && this.fixed_frame === 'world') {
                    const base = robot.links.base_footprint;
                    if (base) { base.updateWorldMatrix(true, false); matrix = base.matrixWorld.clone(); }
                }
            }
        }
        if (!matrix) this.unresolved.add(frame);
        this.cache.set(frame, matrix);
        return matrix;
    }
}
