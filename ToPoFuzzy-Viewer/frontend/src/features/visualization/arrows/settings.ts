import { useSyncExternalStore } from 'react';
import { arrow_style } from './geometry';

const cache = new Map<string, Partial<arrow_style>>();
const listeners = new Set<() => void>();
const prefix = 'topofuzzy.arrow.v1:';
const empty: Partial<arrow_style> = {};
function read(key: string): Partial<arrow_style> {
    if (!cache.has(key)) {
        try {
            const parsed = JSON.parse(localStorage.getItem(prefix + key) ?? '{}');
            cache.set(key, parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : {});
        } catch { cache.set(key, {}); }
    }
    return cache.get(key) ?? empty;
}
function subscribe(listener: () => void) {
    listeners.add(listener);
    return () => { listeners.delete(listener); };
}
if (typeof window !== 'undefined') window.addEventListener('storage', event => {
    if (event.key === null || event.key.startsWith(prefix)) {
        cache.clear();
        listeners.forEach(listener => listener());
    }
});
// 表示設定はブラウザのレイヤー設定として永続化。ROS・WSの更新データへの添付なし
export function useArrowSettings(key: string) {
    return useSyncExternalStore(subscribe, () => read(key), () => empty);
}
export function update_arrow_settings(key: string, value: Partial<arrow_style>) {
    cache.set(key, value);
    try { localStorage.setItem(prefix + key, JSON.stringify(value)); } catch { /* 保存不可時はセッション内だけで保持 */ }
    listeners.forEach(listener => listener());
}
