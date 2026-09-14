import { useEffect, useMemo, useRef, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { graph_bounds, graph_selection, LayerSettings, Transform } from '../../types';
import { DisplayFrame } from './SharedRenderers';

const bounds_key = (bounds: graph_bounds) => JSON.stringify([bounds.source_id, bounds.selection]);
const bounds_padding = (bounds: graph_bounds, settings: Record<string, LayerSettings>) =>
    0.003 + (bounds.selection.kind === 'marker' ? (bounds.node_diameter ?? 0) / 2 : (settings[bounds.source_id]?.nodeScale ?? 0.003));

export function CandidateHoverFrame({ is_enabled, get_bounds, on_inspect, transforms, layer_settings, marker_settings }: {
    is_enabled: boolean;
    get_bounds: (source_id: string) => Promise<graph_bounds[]>;
    on_inspect: (source_id: string, selection: graph_selection) => void;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    layer_settings: Record<string, LayerSettings>;
    marker_settings: Record<string, { transform?: Transform }>;
}) {
    const { gl, scene, camera, invalidate } = useThree();
    const [bounds, set_bounds] = useState<graph_bounds | null>(null);
    const settings_ref = useRef({ transforms, layer_settings, marker_settings });
    settings_ref.current = { transforms, layer_settings, marker_settings };
    const geometry = useMemo(() => {
        const box = new THREE.BoxGeometry(1, 1, 1);
        const edges = new THREE.EdgesGeometry(box);
        box.dispose();
        return edges;
    }, []);
    const material = useMemo(() => new THREE.LineBasicMaterial({
        color: '#ffe8a1', depthTest: false, depthWrite: false, toneMapped: false,
    }), []);
    useEffect(() => () => { geometry.dispose(); material.dispose(); }, [geometry, material]);
    useEffect(() => { invalidate(); }, [bounds, invalidate]);

    useEffect(() => {
        if (!is_enabled) return;
        const canvas = gl.domElement;
        const raycaster = new THREE.Raycaster();
        const local_ray = new THREE.Ray(), box = new THREE.Box3();
        const matrix = new THREE.Matrix4(), manual_matrix = new THREE.Matrix4();
        const position = new THREE.Vector3(), scale = new THREE.Vector3(), intersection = new THREE.Vector3();
        const rotation = new THREE.Quaternion(), euler = new THREE.Euler();
        const pointer = { x: 0, y: 0, is_active: false };
        const unrequested = Symbol();
        const cache = new Map<string, { revision: unknown; bounds: graph_bounds[]; last_request_ms: number; failed_since_ms?: number }>();
        let is_alive = true, is_pending = false, has_cursor = false;
        let selected: graph_bounds | null = null;
        let press: { x: number; y: number; has_dragged: boolean } | null = null;
        let previous_cursor = '', last_hit_ms = 0, next_request_ms = 0, suspend_until_ms = 0;
        const pick = (client_x: number, client_y: number) => {
            const rect = canvas.getBoundingClientRect();
            raycaster.setFromCamera(new THREE.Vector2((client_x - rect.left) / rect.width * 2 - 1,
                -(client_y - rect.top) / rect.height * 2 + 1), camera);
            let nearest: graph_bounds | null = null, min_dist = Infinity;
            const settings = settings_ref.current;
            // 点・球メッシュへのraycastなし。表示と同じTF・手動変換を適用したAABBとの交差のみ。
            for (const entry of cache.values()) for (const candidate of entry.bounds) {
                const tf = candidate.frame_id === 'world' ? undefined : settings.transforms[candidate.frame_id];
                const manual = candidate.selection.kind === 'marker' ? settings.marker_settings[candidate.source_id]?.transform :
                    settings.layer_settings[candidate.source_id]?.graphTransform;
                matrix.compose(position.fromArray(tf?.pos ?? [0, 0, 0]), rotation.fromArray(tf?.quat ?? [0, 0, 0, 1]), scale.set(1, 1, 1));
                manual_matrix.compose(position.fromArray(manual?.position ?? [0, 0, 0]),
                    rotation.setFromEuler(euler.fromArray(manual?.rotation ?? [0, 0, 0])), scale.fromArray(manual?.scale ?? [1, 1, 1]));
                matrix.multiply(manual_matrix);
                if (Math.abs(matrix.determinant()) < 1e-12) continue;
                local_ray.copy(raycaster.ray).applyMatrix4(manual_matrix.copy(matrix).invert());
                box.set(position.fromArray(candidate.min_position), scale.fromArray(candidate.max_position))
                    .expandByScalar(bounds_padding(candidate, settings.layer_settings));
                if (!local_ray.intersectBox(box, intersection)) continue;
                const dist = intersection.applyMatrix4(matrix).distanceTo(raycaster.ray.origin);
                if (dist < min_dist || (dist === min_dist && selected && bounds_key(candidate) === bounds_key(selected))) {
                    nearest = candidate;
                    min_dist = dist;
                }
            }
            return nearest;
        };
        const clear = () => {
            selected = null;
            set_bounds(null);
            if (has_cursor) canvas.style.cursor = previous_cursor;
            has_cursor = false;
        };
        const move = (event: PointerEvent) => {
            if (press && Math.hypot(event.clientX - press.x, event.clientY - press.y) > 5) press.has_dragged = true;
            pointer.x = event.clientX;
            pointer.y = event.clientY;
            pointer.is_active = event.buttons === 0 && event.isPrimary;
            if (!pointer.is_active) clear();
        };
        const leave = () => { pointer.is_active = false; press = null; clear(); };
        const down = (event: PointerEvent) => {
            leave();
            if (event.button === 0 && event.isPrimary) press = { x: event.clientX, y: event.clientY, has_dragged: false };
        };
        const click = (event: MouseEvent) => {
            const can_inspect = press && !press.has_dragged && event.button === 0 &&
                Math.hypot(event.clientX - press.x, event.clientY - press.y) <= 5;
            press = null;
            const candidate = can_inspect ? pick(event.clientX, event.clientY) : null;
            if (!candidate) return;
            // 枠内の空隙からの選択と、既存ノードクリックとの二重発火防止
            event.stopImmediatePropagation();
            on_inspect(candidate.source_id, candidate.selection);
        };
        const wheel = () => { suspend_until_ms = performance.now() + 180; clear(); };
        canvas.addEventListener('pointermove', move);
        canvas.addEventListener('pointerup', move);
        canvas.addEventListener('pointerdown', down);
        canvas.addEventListener('click', click, true);
        canvas.addEventListener('pointercancel', leave);
        canvas.addEventListener('pointerleave', leave);
        canvas.addEventListener('wheel', wheel, { passive: true });
        window.addEventListener('blur', leave);

        const timer = window.setInterval(() => {
            const now = performance.now();
            if (!pointer.is_active || now < suspend_until_ms || document.elementFromPoint(pointer.x, pointer.y) !== canvas) {
                clear(); return;
            }
            const sources = new Map<string, unknown>();
            scene.traverseVisible(object => {
                const source = object.userData.inspection_source;
                if (source && !sources.has(source)) sources.set(source, object.userData.inspection_revision);
            });
            for (const source of cache.keys()) if (!sources.has(source)) cache.delete(source);
            for (const source of sources.keys()) if (!cache.has(source)) cache.set(source, { revision: unrequested, bounds: [], last_request_ms: 0 });
            const selected_key = selected ? bounds_key(selected) : '';
            if (selected && !cache.get(selected.source_id)?.bounds.some(item => bounds_key(item) === selected_key)) clear();

            // 可視ソースの更新フレームごとに一括取得。同時1件・全体4 Hz、古い要求順の更新。
            if (!is_pending && now >= next_request_ms) {
                const next = [...cache].filter(([source, entry]) => entry.revision !== sources.get(source))
                    .sort((a, b) => a[1].last_request_ms - b[1].last_request_ms)[0];
                if (next) {
                    const [source, entry] = next;
                    const revision = sources.get(source);
                    is_pending = true;
                    entry.last_request_ms = now;
                    next_request_ms = now + 250;
                    void get_bounds(source).then(result => {
                        if (!is_alive || cache.get(source) !== entry) return;
                        entry.bounds = result;
                        entry.revision = revision;
                        entry.failed_since_ms = undefined;
                    }).catch(() => {
                        if (!is_alive || cache.get(source) !== entry) return;
                        entry.revision = unrequested;
                        entry.failed_since_ms ??= performance.now();
                        if (performance.now() - entry.failed_since_ms >= 600) entry.bounds = [];
                    }).finally(() => { is_pending = false; });
                }
            }

            const nearest = pick(pointer.x, pointer.y);
            if (!nearest) { if (now - last_hit_ms >= 350) clear(); return; }
            last_hit_ms = now;
            selected = nearest;
            set_bounds(nearest);
            if (!has_cursor) { previous_cursor = canvas.style.cursor; canvas.style.cursor = 'pointer'; has_cursor = true; }
        }, 100);
        return () => {
            is_alive = false;
            clear();
            window.clearInterval(timer);
            canvas.removeEventListener('pointermove', move);
            canvas.removeEventListener('pointerup', move);
            canvas.removeEventListener('pointerdown', down);
            canvas.removeEventListener('click', click, true);
            canvas.removeEventListener('pointercancel', leave);
            canvas.removeEventListener('pointerleave', leave);
            canvas.removeEventListener('wheel', wheel);
            window.removeEventListener('blur', leave);
        };
    }, [is_enabled, get_bounds, on_inspect, gl, scene, camera]);

    if (!is_enabled || !bounds) return null;
    const transform = bounds.selection.kind === 'marker' ? marker_settings[bounds.source_id]?.transform : layer_settings[bounds.source_id]?.graphTransform;
    const padding = bounds_padding(bounds, layer_settings);
    const size = bounds.max_position.map((value, idx) => value - bounds.min_position[idx] + padding * 2) as [number, number, number];
    const center = bounds.max_position.map((value, idx) => (value + bounds.min_position[idx]) / 2) as [number, number, number];
    return <DisplayFrame tf={bounds.frame_id !== 'world' ? transforms[bounds.frame_id] : null} manual_transform={transform}>
        <lineSegments name="candidate-hover-frame" geometry={geometry} material={material}
            position={center} scale={size} renderOrder={2000} dispose={null} />
    </DisplayFrame>;
}
