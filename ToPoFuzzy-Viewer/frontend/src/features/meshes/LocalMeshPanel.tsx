import { useState } from 'react';
import { GenericTransformPanel } from '../manipulation/GenericTransformPanel';
import { mesh_extension, mesh_extensions } from './local_mesh_loader';
import { useLocalMeshes } from './use_local_meshes';

export function LocalMeshPanel({ meshes }: { meshes: ReturnType<typeof useLocalMeshes> }) {
    const [files, set_files] = useState<File[]>([]);
    const [selected_path, set_selected_path] = useState('');
    const [unit_scale, set_unit_scale] = useState(1);
    const candidates = files.filter(file => mesh_extensions.includes(mesh_extension(file.name)));
    const file_path = (file: File) => file.webkitRelativePath || file.name;
    const primary = candidates.find(file => file_path(file) === selected_path);
    const select_files = (selection: FileList | null) => {
        const next = Array.from(selection ?? []);
        set_files(next);
        const first = next.find(file => mesh_extensions.includes(mesh_extension(file.name)));
        set_selected_path(first ? file_path(first) : '');
    };
    const unit_options = <><option value={1}>m</option><option value={0.01}>cm → m</option><option value={0.001}>mm → m</option></>;
    return <div className="space-y-3 text-xs">
        <p>メッシュを面付きで表示。ROSへの配信・点群化なし。原本の軸・原点を保持。</p>
        <div className="flex flex-wrap gap-2">
            <label className="btn-secondary cursor-pointer px-2 py-1">ファイル選択
                <input className="hidden" aria-label="メッシュファイル選択" type="file" multiple disabled={meshes.is_loading}
                    onChange={event => { select_files(event.target.files); event.target.value = ''; }} />
            </label>
            <label className="btn-secondary cursor-pointer px-2 py-1">フォルダ選択
                <input className="hidden" aria-label="メッシュフォルダ選択" type="file" multiple {...{ webkitdirectory: '' }} disabled={meshes.is_loading}
                    onChange={event => { select_files(event.target.files); event.target.value = ''; }} />
            </label>
        </div>
        <p className="text-[var(--text-secondary)]">OBJ / PLY / STL / GLB / glTF / FBX。MTL・画像・BINも同時選択。モデル256 MiB、合計512 MiBまで。</p>
        {files.length > 0 && <div className="space-y-2">
            <select aria-label="追加するメッシュ" className="input-base w-full" value={selected_path} onChange={event => set_selected_path(event.target.value)}>
                {!candidates.length && <option value="">対応モデルなし</option>}
                {candidates.map(file => <option key={file_path(file)} value={file_path(file)}>{file_path(file)}</option>)}
            </select>
            <div className="flex items-center gap-2">
                <label>原本の単位 <select aria-label="原本の単位" className="input-base" value={unit_scale} onChange={event => set_unit_scale(Number(event.target.value))}>{unit_options}</select></label>
                <button className="btn-primary px-3 py-1" disabled={!primary || meshes.is_loading}
                    onClick={() => { if (primary) void meshes.add(primary, files, unit_scale); }}>追加</button>
            </div>
        </div>}
        {meshes.is_loading && <p role="status">メッシュ読込中…</p>}
        {meshes.error && <p role="alert" className="break-words text-red-400">{meshes.error}</p>}
        {meshes.items.map(item => <details key={item.id} className="surface-muted p-2" open>
            <summary className="cursor-pointer break-all font-semibold">{item.name}</summary>
            <p className="my-2">{item.asset.num_vertices.toLocaleString()} 頂点 / {item.asset.num_triangles.toLocaleString()} 三角形</p>
            <div className="mb-2 flex flex-wrap items-center gap-2">
                <label><input type="checkbox" checked={item.is_visible} onChange={event => meshes.update(item.id, { is_visible: event.target.checked })} /> 表示</label>
                <label><input type="checkbox" checked={item.enable_fill_light} onChange={event => meshes.update(item.id, { enable_fill_light: event.target.checked })} /> 明るさ補助</label>
                <button className="btn-secondary px-2" disabled={!item.is_visible} onClick={() => meshes.focus_item(item.id)}>全体を見る</button>
                <button className="btn-secondary px-2" onClick={() => meshes.remove(item.id)}>削除</button>
                <select aria-label={`${item.name}の単位`} className="input-base" value={item.unit_scale} onChange={event => meshes.update(item.id, { unit_scale: Number(event.target.value) })}>{unit_options}</select>
            </div>
            {item.asset.warnings.map((warning, idx) => <p key={idx} className="break-words text-amber-300">{warning}</p>)}
            <GenericTransformPanel title="表示姿勢（world・位置m・回転deg）" transform={item.transform}
                onUpdate={changes => meshes.update(item.id, { transform: { ...item.transform, ...changes } })}
                onReset={() => meshes.update(item.id, { transform: { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] } })} />
        </details>)}
        <p className="text-[var(--text-secondary)]">このタブ内の表示のみ。ページ再読込で解除。FBX/glTFのアニメーション再生・圧縮拡張は未対応。</p>
    </div>;
}
