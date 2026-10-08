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
    function FrameStatus({ on_select_frame }: { on_select_frame: (frame: string) => void }) {
        const [unresolved, set_unresolved] = useState<string[]>([]);
        useEffect(() => {
            const timer = window.setInterval(() => {
                const frames = [...scene.frames.unresolved_frames].sort();
                set_unresolved(previous => previous.join('\n') === frames.join('\n') ? previous : frames);
            }, 500);
            return () => window.clearInterval(timer);
        }, []);
        if (unresolved.length === 0) return null;
        return <div className="space-y-1 py-1">
            <p role="status" className="text-xs text-amber-300 break-words">表示待ち（TF未解決）: {unresolved.join(', ')}</p>
            <div className="flex flex-wrap gap-1">{unresolved.map(frame => <button key={frame} className="btn-secondary px-2 py-1 text-xs"
                onClick={() => on_select_frame(frame)}>{frame} を基準に表示</button>)}</div>
        </div>;
    }
    function viewer_panel() {
        const [endpoint, set_endpoint] = useState('ws://127.0.0.1:9001/observe');
        const [fixed_frame, set_fixed_frame] = useState('world');
        const [error, set_error] = useState('');
        const host = useMemo<viewer_host>(() => {
            const environment = { mesh_base_url: endpoint.replace(/^ws/, 'http').replace(/\/observe$/, '/meshes/'),
                portal_target: panel, resolve_frame: (frame: string) => scene.frames.resolve(frame) };
            return { endpoint, environment, scene_surface: scene.surface(environment), layout: embedded_layout, sidebar: embedded_sidebar,
                on_transforms: (items, is_static) => scene.frames.update(items, is_static),
                on_clear: () => scene.frames.clear(), on_state: state => { api = state; } };
        }, [endpoint]);
        return <section className="viewer-shell"><h3 className="text-sm font-bold">ROS Scene Layers</h3>
            <div className="viewer-connection">
                <label>表示サーバー<input className="input-field" aria-label="ROS結果の接続先" defaultValue={endpoint}
                    onBlur={event => {
                        try {
                            const url = new URL(event.target.value);
                            if (!['ws:', 'wss:'].includes(url.protocol) || url.pathname !== '/observe' || url.username || url.password || url.search || url.hash) throw Error('ws(s)://ホスト:ポート/observe を指定してください');
                            if (endpoint !== url.href) { api?.disconnect(); set_endpoint(url.href); }
                            set_error('');
                        } catch (error) { set_error(String(error)); }
                    }} /></label>
                <label>シーン原点に対応するROSフレーム<input className="input-field" value={fixed_frame}
                    onChange={event => { set_fixed_frame(event.target.value); scene.frames.fixed_frame = event.target.value.trim(); }} /></label>
                {error && <p role="alert">{error}</p>}
            </div>
            <FrameStatus on_select_frame={frame => {
                set_fixed_frame(frame); scene.frames.fixed_frame = frame; scene.request_focus();
            }} />
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
