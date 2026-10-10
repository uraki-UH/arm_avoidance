import * as THREE from 'three';
import { createRoot, events, extend, type RootState } from '@react-three/fiber';
import { createElement, useLayoutEffect } from 'react';
import { ViewerEnvironment, type scene_surface_props, type viewer_environment } from '@viewer/embedding';
import { frame_tree, type frame_robot } from './frame-tree';

export interface scene_options {
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera | THREE.OrthographicCamera;
    renderer: THREE.WebGLRenderer;
    controls?: { target: THREE.Vector3; update: () => void; maxDistance?: number };
    robot: () => frame_robot;
}

// Fiberはシーングラフと入力イベントだけを担当。WebGL描画・コンテキスト所有はSimulator側。
export class graph_scene {
    readonly root_scene = new THREE.Scene();
    readonly frames: frame_tree;
    private root: ReturnType<typeof createRoot>;
    private state: RootState;
    private clipping_planes: THREE.Plane[] = [];
    private resize_observer: ResizeObserver;
    private is_dirty = true;
    private has_disposed = false;
    private has_focus_request = false;
    private inspection_picker: ((client_x: number, client_y: number) => boolean) | null = null;
    constructor(private options: scene_options) {
        extend(THREE);
        this.frames = new frame_tree(options.robot);
        this.root_scene.name = 'ROS Viewer';
        options.scene.add(this.root_scene);
        options.camera.layers.enable(3);
        // WebGLRendererを借用しないことで、Fiber終了時のコンテキスト破棄・二重描画を防止。
        const renderer = { domElement: options.renderer.domElement, render() {}, setSize() {}, setPixelRatio() {} };
        const canvas = options.renderer.domElement;
        const rect = canvas.getBoundingClientRect();
        this.root = createRoot(document.createElement('canvas'));
        this.root.configure({ gl: renderer as unknown as THREE.WebGLRenderer, scene: this.root_scene,
            camera: options.camera, events, frameloop: 'never', dpr: 1,
            // 描画キャンバスへの標準選択イベント接続。切り離したFiber所有canvasとの分離。
            onCreated: state => { state.events.connect?.(canvas); },
            size: { width: rect.width, height: rect.height, top: rect.top, left: rect.left, updateStyle: false } });
        this.state = this.root.render(null).getState();
        this.state.raycaster.layers.set(3);
        this.state.set({ controls: options.controls as unknown as RootState['controls'], invalidate: () => { this.is_dirty = true; } });
        this.resize_observer = new ResizeObserver(() => {
            const rect = canvas.getBoundingClientRect();
            this.state.setSize(rect.width, rect.height, false, rect.top, rect.left);
        });
        this.resize_observer.observe(canvas);
    }
    surface(environment: viewer_environment) {
        const host = this;
        return function scene_surface({ children, clipping_planes }: scene_surface_props) {
            useLayoutEffect(() => {
                host.clipping_planes = clipping_planes;
                host.is_dirty = true;
                host.root.render(createElement(ViewerEnvironment.Provider, { value: environment }, children));
            }, [children, clipping_planes]);
            useLayoutEffect(() => () => { host.root.render(null); }, []);
            return null;
        };
    }
    set_inspection_picker(picker: ((client_x: number, client_y: number) => boolean) | null) { this.inspection_picker = picker; }
    has_inspection_at(client_x: number, client_y: number) { return this.inspection_picker?.(client_x, client_y) ?? false; }
    request_focus() { this.has_focus_request = true; }
    private focus_visible() {
        const bounds = new THREE.Box3();
        this.root_scene.updateMatrixWorld(true);
        // 可視オブジェクトだけの境界。全ノードの位置配列への変換なし。
        this.root_scene.traverseVisible(object => {
            if (object instanceof THREE.InstancedMesh) object.computeBoundingBox();
            if (object instanceof THREE.Mesh || object instanceof THREE.Points || object instanceof THREE.Line || object instanceof THREE.Sprite) bounds.expandByObject(object);
        });
        if (bounds.isEmpty()) return;
        const center = bounds.getCenter(new THREE.Vector3());
        const radius = Math.max(.01, bounds.getSize(new THREE.Vector3()).length() / 2);
        const camera = this.options.camera;
        const direction = camera.position.clone().sub(this.options.controls?.target ?? center);
        if (direction.lengthSq() < 1e-8) direction.set(1, -1, 1);
        direction.normalize();
        let dist = radius * 3;
        if (camera instanceof THREE.PerspectiveCamera) {
            const vertical = THREE.MathUtils.degToRad(camera.fov);
            const horizontal = 2 * Math.atan(Math.tan(vertical / 2) * camera.aspect);
            dist = 1.2 * radius / Math.sin(Math.min(vertical, horizontal) / 2);
        } else {
            camera.zoom = Math.min(camera.right - camera.left, camera.top - camera.bottom) / (2.4 * radius);
        }
        camera.position.copy(center).addScaledVector(direction, dist);
        camera.near = Math.min(camera.near, Math.max(.001, radius / 1000));
        camera.far = Math.max(camera.far, dist + radius * 2);
        camera.lookAt(center); camera.updateProjectionMatrix();
        if (this.options.controls?.maxDistance !== undefined) this.options.controls.maxDistance = Math.max(this.options.controls.maxDistance, dist);
        this.options.controls?.target.copy(center);
        this.options.controls?.update();
    }
    tick(now: number) {
        if (this.has_disposed) return;
        this.frames.begin_frame(now);
        this.state.advance(now / 1000, false);
        if (this.is_dirty) {
            this.is_dirty = false;
            this.root_scene.traverse(object => {
                object.layers.set(3);
                const mesh = object as THREE.Mesh;
                if (!mesh.material) return;
                for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
                    // 計測結果はスタジオ演出用の霧から除外。遠方地図の可読性を維持。
                    if ('fog' in material && material.fog) { material.fog = false; material.needsUpdate = true; }
                    if (material.clippingPlanes !== this.clipping_planes) {
                        material.clippingPlanes = this.clipping_planes;
                        material.needsUpdate = true;
                    }
                }
            });
            if (this.clipping_planes.length) this.options.renderer.localClippingEnabled = true;
        }
        if (this.has_focus_request) {
            this.has_focus_request = false;
            this.focus_visible();
        }
    }
    dispose() {
        if (this.has_disposed) return;
        this.has_disposed = true;
        this.inspection_picker = null;
        this.resize_observer.disconnect();
        this.state.events.disconnect?.();
        this.root.unmount();
        this.frames.clear();
        this.options.scene.remove(this.root_scene);
    }
}
