import { useCallback, useEffect, useRef, useState } from 'react';
import { Transform } from '../../types';
import { local_mesh_asset, load_local_mesh } from './local_mesh_loader';

export interface local_mesh_item {
    id: string;
    name: string;
    asset: local_mesh_asset;
    is_visible: boolean;
    enable_fill_light: boolean;
    unit_scale: number;
    transform: Transform;
}

// React Hooksの判定規約に基づくフック名
export function useLocalMeshes() {
    const [items, set_items] = useState<local_mesh_item[]>([]);
    const [is_loading, set_is_loading] = useState(false);
    const [error, set_error] = useState('');
    const [focus, set_focus] = useState<{ id: string; req: number } | null>(null);
    const assets = useRef(new Map<string, local_mesh_asset>());
    const is_mounted = useRef(true);
    const is_pending = useRef(false);
    useEffect(() => {
        is_mounted.current = true;
        const current_assets = assets.current;
        return () => {
            is_mounted.current = false;
            for (const asset of current_assets.values()) asset.dispose();
            current_assets.clear();
        };
    }, []);
    const add = useCallback(async (primary: File, files: File[], unit_scale: number) => {
        if (is_pending.current) return;
        is_pending.current = true;
        set_is_loading(true);
        set_error('');
        try {
            // 読込中表示の描画後に開始するファイル解析
            await new Promise<void>(resolve => window.setTimeout(resolve, 30));
            const asset = await load_local_mesh(primary, files);
            if (!is_mounted.current) { asset.dispose(); return; }
            const id = asset.object.uuid;
            assets.current.set(id, asset);
            set_items(previous => [...previous, { id, name: primary.name, asset, unit_scale, is_visible: true, enable_fill_light: true,
                transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } }]);
            set_focus(previous => ({ id, req: (previous?.req ?? 0) + 1 }));
        } catch (reason) {
            if (is_mounted.current) set_error(reason instanceof Error ? reason.message : String(reason));
        } finally {
            is_pending.current = false;
            if (is_mounted.current) set_is_loading(false);
        }
    }, []);
    const update = (id: string, changes: Partial<Pick<local_mesh_item, 'is_visible' | 'enable_fill_light' | 'unit_scale' | 'transform'>>) =>
        set_items(previous => previous.map(item => item.id === id ? { ...item, ...changes } : item));
    const remove = (id: string) => {
        assets.current.get(id)?.dispose();
        assets.current.delete(id);
        set_items(previous => previous.filter(item => item.id !== id));
    };
    const focus_item = (id: string) => set_focus(previous => ({ id, req: (previous?.req ?? 0) + 1 }));
    return { items, is_loading, error, focus, add, update, remove, focus_item };
}
