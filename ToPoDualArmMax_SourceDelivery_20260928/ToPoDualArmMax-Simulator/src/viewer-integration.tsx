import { createRoot } from 'react-dom/client';
import { createElement, useEffect, useMemo, useState, type ReactNode } from 'react';
import ViewerApp from '@viewer/App';
import type { viewer_host } from '@viewer/embedding';
import type { useWebSocket } from '@viewer/hooks/useWebSocket';
import ui_css from './viewer-ui.css?inline';
import { graph_scene, type scene_options } from './graph-scene';

function embedded_layout({ sidebar, children }: { sidebar: ReactNode; children: ReactNode }) { return <>{sidebar}{children}</>; }
function embedded_sidebar({ children }: { children: ReactNode }) { return <div className="viewer-sidebar">{children}</div>; }

export function mount_viewer(container: HTMLElement, options: scene_options) {
    const shadow = container.attachShadow({ mode: 'open' });
    const style = document.createElement('style');
    style.textContent = ui_css.replaceAll(':root', ':host');
    const panel = document.createElement('div');
    shadow.append(style, panel);
    const scene = new graph_scene(options);
    let api: ReturnType<typeof useWebSocket> | undefined;
    function FrameSelector({ fixed_frame, on_select_frame }: { fixed_frame: string; on_select_frame: (frame: string) => void }) {
        const [frames, set_frames] = useState<string[]>(() => scene.frames.available_frames);
        useEffect(() => {
            const timer = window.setInterval(() => {
                const available = scene.frames.available_frames;
                set_frames(previous => previous.join('\n') === available.join('\n') ? previous : available);
            }, 500);
            return () => window.clearInterval(timer);
        }, []);
        return <div role="group" aria-label="表示基準フレーム" className="flex max-h-32 flex-wrap gap-1 overflow-y-auto py-1">
            {frames.map(frame => <button key={frame} type="button" aria-pressed={fixed_frame.trim() === frame}
                className="btn-secondary px-2 py-1 text-xs break-all aria-pressed:ring-1 aria-pressed:ring-cyan-400"
                onClick={() => on_select_frame(frame)}>{frame}</button>)}
        </div>;
    }
    function viewer_panel() {
        const [endpoint, set_endpoint] = useState(() => new URLSearchParams(location.search).get('observe') || 'ws://127.0.0.1:9001/observe');
        const [fixed_frame, set_fixed_frame] = useState('world');
        const [error, set_error] = useState('');
        const host = useMemo<viewer_host>(() => {
            const environment = { mesh_base_url: endpoint.replace(/^ws/, 'http').replace(/\/observe$/, '/meshes/'),
                portal_target: panel, resolve_frame: (frame: string) => scene.frames.resolve(frame),
                set_inspection_picker: (picker: ((client_x: number, client_y: number) => boolean) | null) => scene.set_inspection_picker(picker) };
            return { endpoint, environment, scene_surface: scene.surface(environment), layout: embedded_layout, sidebar: embedded_sidebar,
                on_transforms: (items, is_static) => scene.frames.update(items, is_static),
                on_clear: () => scene.frames.clear(), on_state: state => { api = state; } };
        }, [endpoint]);
        return <section className="viewer-shell">
            <details className="viewer-connection">
                <summary><strong>ROS Scene Layers</strong><span>接続・座標設定</span></summary>
                <div className="viewer-connection-fields">
                    <label>表示サーバー<input className="input-field" aria-label="ROS結果の接続先" defaultValue={endpoint}
                        onBlur={event => {
                            try {
                                const url = new URL(event.target.value);
                                if (!['ws:', 'wss:'].includes(url.protocol) || url.pathname !== '/observe' || url.username || url.password || url.search || url.hash) throw Error('ws(s)://ホスト:ポート/observe を指定してください');
                                if (endpoint !== url.href) { api?.disconnect(); set_endpoint(url.href); }
                                set_error('');
                            } catch (error) { set_error(String(error)); }
                        }} /></label>
                    <details className="viewer-frame-settings">
                        <summary>シーン原点に対応するROSフレーム</summary>
                        <label>フレーム名<input className="input-field" value={fixed_frame}
                            onChange={event => { set_fixed_frame(event.target.value); scene.frames.fixed_frame = event.target.value.trim(); }} /></label>
                        <FrameSelector fixed_frame={fixed_frame} on_select_frame={frame => {
                            set_fixed_frame(frame); scene.frames.fixed_frame = frame; scene.request_focus();
                        }} />
                    </details>
                </div>
            </details>
            {error && <p role="alert" className="text-xs text-red-300">{error}</p>}
            <ViewerApp host={host} />
        </section>;
    }
    const root = createRoot(panel);
    root.render(createElement(viewer_panel));
    const disconnect = () => api?.disconnect();
    window.addEventListener('pagehide', disconnect);
    return { scene, get api() { return api; }, tick: (now: number) => scene.tick(now), dispose() {
        window.removeEventListener('pagehide', disconnect);
        disconnect(); root.unmount(); scene.dispose(); container.remove();
    } };
}
