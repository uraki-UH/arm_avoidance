import { createContext, type ComponentType, type ReactNode } from 'react';
import type * as THREE from 'three';
import type { useWebSocket } from './hooks/useWebSocket';
import type { TransformData } from './types';

// 単独Viewerと外部シミュレーターで共用する描画・UIの接続口。
export interface viewer_environment {
    mesh_base_url: string;
    portal_target?: HTMLElement;
    resolve_frame?: (frame: string) => THREE.Matrix4 | null;
}
export const ViewerEnvironment = createContext<viewer_environment>({ mesh_base_url: '' });

export interface scene_surface_props { children: ReactNode; clipping_planes: THREE.Plane[] }
export interface viewer_host {
    scene_surface: ComponentType<scene_surface_props>;
    layout: ComponentType<{ sidebar: ReactNode; children: ReactNode; isSidebarOpen: boolean }>;
    sidebar: ComponentType<{ children: ReactNode; isOpen: boolean; onToggle: () => void }>;
    endpoint: string;
    environment: viewer_environment;
    on_transforms: (items: TransformData[], is_static: boolean) => void;
    on_clear: () => void;
    on_state: (state: ReturnType<typeof useWebSocket>) => void;
}
