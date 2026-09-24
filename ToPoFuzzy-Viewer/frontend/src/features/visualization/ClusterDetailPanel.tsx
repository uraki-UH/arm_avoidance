import { VehicleRegistrationControls, VehicleRegistrationLayer } from '../vehicleRegistration/VehicleRegistration';
import { useVehicleRegistration } from '../vehicleRegistration/use_vehicle_registration';
import type { register_vehicle as register_vehicle_fn } from '../vehicleRegistration/types';
import { memo, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import { graph_snapshot, LayerSettings } from '../../types';
import { GraphRenderer } from './GraphRenderer';
import { createDefaultGraphLayerSettings } from './graphLayerSettings';
import { marker_color } from './arrows';
import { WebGLErrorBoundary } from '../../components/WebGLErrorBoundary';

export type ClusterSnapshot = graph_snapshot & { settings?: LayerSettings };

function InspectionCamera({ snapshot, reset_count }: { snapshot: graph_snapshot; reset_count: number }) {
    const { camera, controls, size, invalidate } = useThree();
    useLayoutEffect(() => {
        if (!(camera instanceof THREE.PerspectiveCamera) || !controls) return;
        const orbit = controls as unknown as { target: THREE.Vector3; update: () => void };
        const min = new THREE.Vector3(...snapshot.min_position);
        const max = new THREE.Vector3(...snapshot.max_position);
        const center = min.clone().add(max).multiplyScalar(0.5);
        const radius = Math.max(0.01, min.distanceTo(max) / 2 + (snapshot.node_diameter ?? 0) / 2);
        const vertical_fov = THREE.MathUtils.degToRad(camera.fov);
        const horizontal_fov = 2 * Math.atan(Math.tan(vertical_fov / 2) * size.width / size.height);
        const dist = 1.3 * radius / Math.sin(Math.min(vertical_fov, horizontal_fov) / 2);
        camera.up.set(0, 0, 1);
        camera.position.copy(center).add(new THREE.Vector3(1, -1, 0.8).normalize().multiplyScalar(dist));
        camera.near = Math.max(0.00001, radius / 1000);
        camera.far = Math.max(100, dist * 100);
        camera.updateProjectionMatrix();
        orbit.target.copy(center);
        orbit.update();
        invalidate();
    }, [snapshot, reset_count, camera, controls, size.width, size.height, invalidate]);
    return <OrbitControls makeDefault enableDamping={false} onChange={() => invalidate()} />;
}

function ClusterDetailPanelInner({ snapshot, onClose, on_refresh, is_loading, error, register_vehicle }: {
    register_vehicle?: register_vehicle_fn;
    snapshot: ClusterSnapshot;
    onClose: () => void;
    on_refresh: () => void;
    is_loading: boolean;
    error: string | null;
}) {
    const registration = useVehicleRegistration(snapshot, register_vehicle);
    const camera_snapshot = useMemo(() => registration.candidate ? { ...snapshot,
        min_position: registration.candidate.min_position, max_position: registration.candidate.max_position } : snapshot,
        [snapshot, registration.candidate]);
    const [enable_nodes, set_enable_nodes] = useState(true);
    const [enable_edges, set_enable_edges] = useState(true);
    const [enable_normals, set_enable_normals] = useState(false);
    const [enable_axes, set_enable_axes] = useState(false);
    const [enable_bounding_box, set_enable_bounding_box] = useState(false);
    const [reset_count, set_reset_count] = useState(0);
    const [position, set_position] = useState<{ x: number; y: number } | null>(null);
    const drag = useRef<{ x: number; y: number; left: number; top: number; max_x: number; max_y: number } | null>(null);
    const settings = useMemo(() => {
        const defaults = createDefaultGraphLayerSettings(snapshot.source_id, snapshot.graph);
        return { ...defaults, ...snapshot.settings,
            visible: true, visibleLabels: defaults.visibleLabels, graphTransform: defaults.graphTransform,
            showNodes: enable_nodes, showEdges: enable_edges, showNormals: enable_normals,
            showClusters: false, showVelocity: false,
            showManipulabilityEllipsoids: false, showCovarianceEllipsoids: false,
            ...(snapshot.node_color ? { nodeOpacity: marker_color(snapshot.node_color).opacity } : {}),
            ...((snapshot.node_diameter ?? 0) > 0 ? { nodeScale: snapshot.node_diameter! / 2 } : {}),
        };
    }, [snapshot, enable_nodes, enable_edges, enable_normals]);
    const extent = snapshot.max_position.map((value, idx) => value - snapshot.min_position[idx]);
    const center = snapshot.max_position.map((value, idx) => (value + snapshot.min_position[idx]) / 2) as [number, number, number];
    const bounds = useMemo(() => new THREE.Box3(
        new THREE.Vector3(...snapshot.min_position), new THREE.Vector3(...snapshot.max_position)), [snapshot]);

    return <section role="dialog" aria-label="候補の独立3Dビュー"
        className="surface-panel absolute z-50 flex min-h-0 flex-col overflow-hidden"
        style={{ left: position?.x, right: position ? undefined : 16, top: position?.y ?? 72,
            width: 'min(620px, calc(100% - 32px))', height: register_vehicle ? 680 : 460,
            minWidth: 'min(320px, calc(100% - 32px))', minHeight: 320,
            maxWidth: 'calc(100% - 32px)', maxHeight: '85vh', resize: 'both' }}>
        <header className="flex shrink-0 cursor-grab items-center justify-between gap-2 border-b border-white/10 bg-black/25 p-2"
            style={{ touchAction: 'none' }}
            onPointerDown={event => {
                if (event.button !== 0 || (event.target as HTMLElement).closest('button')) return;
                const panel = event.currentTarget.parentElement!;
                const bounds = panel.getBoundingClientRect();
                const parent = panel.offsetParent!.getBoundingClientRect();
                drag.current = { x: event.clientX, y: event.clientY, left: bounds.left - parent.left,
                    top: bounds.top - parent.top, max_x: parent.width - 100, max_y: parent.height - 48 };
                event.currentTarget.setPointerCapture(event.pointerId);
            }}
            onPointerMove={event => {
                if (!drag.current) return;
                set_position({ x: Math.max(0, Math.min(drag.current.max_x,
                    drag.current.left + event.clientX - drag.current.x)),
                y: Math.max(0, Math.min(drag.current.max_y,
                    drag.current.top + event.clientY - drag.current.y)) });
            }}
            onPointerUp={event => {
                drag.current = null;
                if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
            }}
            onPointerCancel={() => { drag.current = null; }}>
            <div className="min-w-0">
                <h3 className="text-sm font-bold">{snapshot.title.replace(/^nonplane_components #/, 'nonplane_')}</h3>
                <p className="truncate text-[10px] opacity-60" title={snapshot.source_id}>
                    {snapshot.source_id.replace(/(^|\/)nonplane_components(?=\/|$)/g, '$1nonplane_')}</p>
            </div>
            <button className="btn-secondary px-2" onClick={onClose} aria-label="候補ビューを閉じる">閉じる</button>
        </header>
        <div className="flex shrink-0 flex-wrap items-center gap-3 border-b border-white/10 p-2 text-xs">
            <label><input type="checkbox" checked={enable_nodes} onChange={e => set_enable_nodes(e.target.checked)} /> ノード</label>
            <label><input type="checkbox" checked={enable_edges} disabled={!snapshot.graph.edges.length}
                onChange={e => set_enable_edges(e.target.checked)} /> エッジ</label>
            <label><input type="checkbox" checked={enable_normals} disabled={snapshot.selection.kind === 'marker'}
                onChange={e => set_enable_normals(e.target.checked)} /> 法線</label>
            <label><input type="checkbox" checked={enable_axes} onChange={e => set_enable_axes(e.target.checked)} /> XYZ軸</label>
            <label><input type="checkbox" checked={enable_bounding_box} onChange={e => set_enable_bounding_box(e.target.checked)} /> Bbox</label>
            <button className="btn-secondary px-2 py-1" onClick={() => set_reset_count(value => value + 1)}>全体表示</button>
            <button className="btn-secondary px-2 py-1" disabled={is_loading} onClick={on_refresh}>
                {is_loading ? '取得中' : '最新を取得'}</button>
        </div>
        {register_vehicle && snapshot.selection.kind !== 'node' && snapshot.selection.kind !== 'marker' &&
            <VehicleRegistrationControls state={registration} num_nodes={snapshot.graph.nodes.length} />}
        {error && <p role="alert" className="px-2 py-1 text-xs text-red-300">{error}</p>}
        <div className="relative min-h-0 flex-1 bg-black/20">
            <WebGLErrorBoundary>
                <Canvas frameloop="demand" dpr={1} gl={{ antialias: false }}
                    camera={{ up: [0, 0, 1], fov: 45 }}>
                    <ambientLight intensity={1.5} />
                    <directionalLight position={[3, -3, 5]} intensity={2} />
                    <InspectionCamera snapshot={camera_snapshot} reset_count={reset_count} />
                    <GraphRenderer tag={snapshot.source_id} data={snapshot.graph} settings={settings} enableClusterSelection={false}
                        uniform_node_color={snapshot.node_color ? marker_color(snapshot.node_color).color.getStyle() : undefined} />
                    <VehicleRegistrationLayer candidate={registration.candidate} />
                    {enable_axes && <axesHelper args={[Math.max(...extent, 0.03) * 0.4]} position={center} />}
                    {enable_bounding_box && <box3Helper args={[bounds, '#ffe8a1']} renderOrder={2000}
                        material-depthTest={false} material-depthWrite={false} material-toneMapped={false} />}
                </Canvas>
            </WebGLErrorBoundary>
        </div>
        <footer className="shrink-0 overflow-x-auto whitespace-nowrap border-t border-white/10 p-2 text-[11px] text-[var(--text-secondary)]">
            <p>{snapshot.graph.nodes.length} nodes / {snapshot.graph.edges.length / 2} edges / XYZ寸法: {extent.map(value => value.toFixed(3)).join(' / ')} m</p>
        </footer>
    </section>;
}

export const ClusterDetailPanel = memo(ClusterDetailPanelInner);
export default ClusterDetailPanel;
