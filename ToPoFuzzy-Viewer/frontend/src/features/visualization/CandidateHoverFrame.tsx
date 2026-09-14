import { useEffect, useMemo, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { graph_bounds, graph_selection, LayerSettings, Transform } from '../../types';
import { DisplayFrame } from './SharedRenderers';

export function CandidateHoverFrame({ is_enabled, get_bounds, transforms, layer_settings, marker_settings }: {
    is_enabled: boolean;
    get_bounds: (source_id: string, selection: graph_selection) => Promise<graph_bounds>;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    layer_settings: Record<string, LayerSettings>;
    marker_settings: Record<string, { transform?: Transform }>;
}) {
    const { gl, scene, camera, invalidate } = useThree();
    const [bounds, set_bounds] = useState<graph_bounds | null>(null);
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
        const pointer = { x: 0, y: 0, is_active: false };
        let is_alive = true, is_pending = false, has_cursor = false;
        let target_key = '', request_key = '', previous_cursor = '';
        let revision: unknown;
        let generation = 0, next_request_ms = 0, next_pick_ms = 0, suspend_until_ms = 0;
        const clear = () => {
            if (target_key) ++generation;
            target_key = '';
            request_key = '';
            set_bounds(null);
            if (has_cursor) canvas.style.cursor = previous_cursor;
            has_cursor = false;
        };
        const move = (event: PointerEvent) => {
            pointer.x = event.clientX;
            pointer.y = event.clientY;
            pointer.is_active = event.buttons === 0 && event.isPrimary;
            if (!pointer.is_active) clear();
        };
        const leave = () => { pointer.is_active = false; clear(); };
        const wheel = () => { suspend_until_ms = performance.now() + 180; clear(); };
        canvas.addEventListener('pointermove', move);
        canvas.addEventListener('pointerup', move);
        canvas.addEventListener('pointerdown', leave);
        canvas.addEventListener('pointerleave', leave);
        canvas.addEventListener('wheel', wheel, { passive: true });
        window.addEventListener('blur', leave);

        // ホバー対象だけを最大10 Hzで判定。重いフレームでは測定時間に応じて間隔を拡大。
        const timer = window.setInterval(() => {
            const now = performance.now();
            if (!pointer.is_active || now < suspend_until_ms ||
                document.elementFromPoint(pointer.x, pointer.y) !== canvas) { clear(); return; }
            if (now < next_pick_ms) return;
            const rect = canvas.getBoundingClientRect();
            raycaster.setFromCamera(new THREE.Vector2(
                (pointer.x - rect.left) / rect.width * 2 - 1,
                -(pointer.y - rect.top) / rect.height * 2 + 1), camera);
            const objects: THREE.Object3D[] = [];
            scene.traverseVisible(object => { if (object.userData.inspection_source) objects.push(object); });
            const hit = raycaster.intersectObjects(objects, false)[0];
            next_pick_ms = now + Math.max(100, (performance.now() - now) * 8);
            if (!hit) { clear(); return; }
            const metadata = hit.object.userData;
            const node = metadata.pick_nodes?.[hit.instanceId ?? -1];
            const selection: graph_selection | undefined = metadata.inspection_selection ??
                (node ? { kind: 'node', id: node.id } : undefined);
            if (!selection) { clear(); return; }
            const key = JSON.stringify([metadata.inspection_source, selection]);
            if (key !== target_key) { clear(); target_key = key; }
            if (!has_cursor) { previous_cursor = canvas.style.cursor; canvas.style.cursor = 'pointer'; has_cursor = true; }
            // 同時取得1件、最大4 Hz。受信フレームも対象も同じ場合は再取得なし。
            if (is_pending || now < next_request_ms ||
                (request_key === key && revision === metadata.inspection_revision)) return;
            is_pending = true;
            next_request_ms = now + 250;
            request_key = key;
            revision = metadata.inspection_revision;
            const request_generation = generation;
            void get_bounds(metadata.inspection_source, selection).then(result => {
                if (is_alive && generation === request_generation && target_key === key) set_bounds(result);
            }).catch(() => {
                if (is_alive && generation === request_generation) set_bounds(null);
            }).finally(() => { is_pending = false; });
        }, 100);
        return () => {
            is_alive = false;
            clear();
            window.clearInterval(timer);
            canvas.removeEventListener('pointermove', move);
            canvas.removeEventListener('pointerup', move);
            canvas.removeEventListener('pointerdown', leave);
            canvas.removeEventListener('pointerleave', leave);
            canvas.removeEventListener('wheel', wheel);
            window.removeEventListener('blur', leave);
        };
    }, [is_enabled, get_bounds, gl, scene, camera]);

    if (!is_enabled || !bounds) return null;
    const is_marker = bounds.selection.kind === 'marker';
    const transform = is_marker ? marker_settings[bounds.source_id]?.transform : layer_settings[bounds.source_id]?.graphTransform;
    const padding = 0.003 + (is_marker ? (bounds.node_diameter ?? 0) / 2 : (layer_settings[bounds.source_id]?.nodeScale ?? 0.003));
    const size = bounds.max_position.map((value, idx) => value - bounds.min_position[idx] + padding * 2) as [number, number, number];
    const center = bounds.max_position.map((value, idx) => (value + bounds.min_position[idx]) / 2) as [number, number, number];
    return <DisplayFrame tf={bounds.frame_id !== 'world' ? transforms[bounds.frame_id] : null} manual_transform={transform}>
        <lineSegments name="candidate-hover-frame" geometry={geometry} material={material}
            position={center} scale={size} renderOrder={2000} dispose={null} />
    </DisplayFrame>;
}
