import type { scene_options } from './graph-scene';
import './ros-results.css';

// ROSパネル初回表示時だけの読込。通常のSimulator起動にはViewerの描画UIを含めない。
export function create_ros_results(options: scene_options) {
    const container = document.createElement('section');
    container.id = 'ros-results';
    document.getElementById('ros-panel')!.append(container);
    let viewer: ReturnType<typeof import('./viewer-integration')['mount_viewer']> | undefined;
    let loading: Promise<void> | undefined;
    let has_disposed = false;
    const open = () => loading ??= import('./viewer-integration').then(module => {
        if (!has_disposed) { container.textContent = ''; viewer = module.mount_viewer(container, options); }
    }).catch(error => { container.textContent = `表示UIの読込失敗: ${String(error)}`; throw error; });
    container.textContent = 'ROS表示UIを準備中…';
    const observer = new IntersectionObserver(entries => {
        if (entries.some(entry => entry.isIntersecting)) { observer.disconnect(); void open().catch(() => {}); }
    });
    observer.observe(container);
    return { open, get api() { return viewer?.api; }, get scene() { return viewer?.scene; },
        tick(now: number) { viewer?.tick(now); }, dispose() { has_disposed = true; observer.disconnect(); viewer?.dispose(); container.remove(); } };
}
