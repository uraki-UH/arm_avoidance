import * as THREE from 'three';
import type { TransformData } from '@viewer/types';
export interface frame_robot { links: Record<string, THREE.Object3D>; ros_frame_prefix?: string }
interface cached_transform { parent: string; matrix: THREE.Matrix4; received_ms: number; is_static: boolean }

// ROSの親子TFとSimulatorの実配置を結ぶ表示座標系。
export class frame_tree {
    fixed_frame = 'world';
    readonly transforms = new Map<string, cached_transform>();
    readonly unresolved_frames = new Set<string>();
    private cache = new Map<string, THREE.Matrix4 | null>();
    private now_ms = 0;
    constructor(private robot: () => frame_robot) {}
    clear() { this.transforms.clear(); this.cache.clear(); this.unresolved_frames.clear(); }
    update(items: TransformData[], is_static: boolean) {
        for (const item of items) {
            if (!item.frameId || !item.childFrameId || ![...item.pos, ...item.quat].every(Number.isFinite) || Math.hypot(...item.quat) < 1e-9) continue;
            this.transforms.set(item.childFrameId, { parent: item.frameId, is_static, received_ms: performance.now(),
                matrix: new THREE.Matrix4().compose(new THREE.Vector3().fromArray(item.pos),
                    new THREE.Quaternion().fromArray(item.quat).normalize(), new THREE.Vector3(1, 1, 1)) });
        }
        if (this.transforms.size > 8192) throw Error('TFフレーム数超過');
        this.cache.clear();
    }
    begin_frame(now_ms: number) { this.now_ms = now_ms; this.cache.clear(); this.unresolved_frames.clear(); }
    resolve(frame: string, seen = new Set<string>()): THREE.Matrix4 | null {
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
                    const parent = this.resolve(transform.parent, seen);
                    if (parent) matrix = parent.clone().multiply(transform.matrix);
                } else if (!transform && frame === 'base_footprint' && this.fixed_frame === 'world') {
                    const base = robot.links.base_footprint;
                    if (base) { base.updateWorldMatrix(true, false); matrix = base.matrixWorld.clone(); }
                }
            }
        }
        if (!matrix) this.unresolved_frames.add(frame);
        this.cache.set(frame, matrix);
        return matrix;
    }
}
