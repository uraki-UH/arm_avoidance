import { ReactNode, useEffect, useLayoutEffect, useMemo, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { Transform } from '../../types';
import { EllipsoidInstance, updateEllipsoidInstances } from './ellipsoid';
import { arrow_sample, arrow_style, arrow_dimensions, build_arrow_parts, resolve_arrow_style, update_arrow_settings, useArrowSettings } from './arrows';

// 描画部品と共用の再描画要求。フック変更時は上位モジュールへ更新を伝播
// eslint-disable-next-line react-refresh/only-export-components
export function useDemandUpdate(dependencies: readonly unknown[]) {
    const { invalidate } = useThree();
    useEffect(() => {
        invalidate();
        // 呼出元による表示依存値の指定
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [...dependencies, invalidate]);
}

// 共通の表示座標系。適用順はTFまたは基準姿勢、手動変換、子要素の姿勢
export function DisplayFrame({ tf, manual_transform, name, is_visible = true, children }: {
    tf?: { pos: number[]; quat: number[] } | null;
    manual_transform?: Transform | null;
    name?: string;
    is_visible?: boolean;
    children: ReactNode;
}) {
    useDemandUpdate([tf, manual_transform, is_visible]);
    return <group name={name} visible={is_visible}
        position={tf ? [tf.pos[0], tf.pos[1], tf.pos[2]] : [0, 0, 0]}
        quaternion={tf ? [tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]] : [0, 0, 0, 1]}>
        <group position={manual_transform?.position ?? [0, 0, 0]}
            rotation={manual_transform?.rotation ?? [0, 0, 0]} scale={manual_transform?.scale ?? [1, 1, 1]}>
            {children}
        </group>
    </group>;
}

// 共分散・可操作性の共通一括描画。共有geometry・materialの寿命は呼出元で管理
export function EllipsoidBatch<instance_type extends EllipsoidInstance>({ instances, geometry, material,
    sigma_multiplier, default_color, render_order, on_pick }: {
    instances: instance_type[];
    geometry: THREE.BufferGeometry;
    material: THREE.Material;
    sigma_multiplier?: number;
    default_color?: string;
    render_order: number;
    on_pick?: (instance: instance_type) => void;
}) {
    const mesh_ref = useRef<THREE.InstancedMesh>(null);
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, instances.length))));
    capacity_ref.current = capacity;
    const { invalidate } = useThree();
    useLayoutEffect(() => {
        if (!mesh_ref.current) return;
        updateEllipsoidInstances(mesh_ref.current, instances, { defaultColor: default_color, sigmaMultiplier: sigma_multiplier });
        mesh_ref.current.computeBoundingSphere();
        invalidate();
    }, [instances, default_color, sigma_multiplier, capacity, geometry, material, invalidate]);
    return <instancedMesh key={capacity} ref={mesh_ref} args={[geometry, material, capacity]}
        count={0} visible={instances.length > 0} frustumCulled={false} renderOrder={render_order} dispose={null}
        onClick={on_pick ? event => {
            event.stopPropagation();
            if (event.instanceId !== undefined && instances[event.instanceId]) on_pick(instances[event.instanceId]);
        } : undefined} />;
}

// 全矢印共通の円柱・円錐インスタンス描画。方向だけの入力には補助軸の生成なし
export function ArrowBatch({ samples, style }: { samples: arrow_sample[]; style?: Partial<arrow_style> }) {
    const shaft_ref = useRef<THREE.InstancedMesh>(null);
    const head_ref = useRef<THREE.InstancedMesh>(null);
    const { invalidate } = useThree();
    const resolved = useMemo(() => resolve_arrow_style(style), [style]);
    const parts = useMemo(() => samples.flatMap(sample => build_arrow_parts(sample, resolved)), [samples, resolved]);
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, parts.length))));
    capacity_ref.current = capacity;
    useLayoutEffect(() => {
        const shaft = shaft_ref.current;
        const head = head_ref.current;
        if (!shaft || !head) return;
        shaft.count = head.count = parts.length;
        parts.forEach((part, idx) => {
            shaft.setMatrixAt(idx, part.shaft);
            head.setMatrixAt(idx, part.head);
            shaft.setColorAt(idx, part.color);
            head.setColorAt(idx, part.color);
        });
        for (const mesh of [shaft, head]) {
            mesh.instanceMatrix.needsUpdate = true;
            if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        }
        invalidate();
    }, [parts, invalidate]);
    const is_overlay = resolved.depth_mode === 'overlay';
    return <group name="common-arrows">
        <instancedMesh key={`shaft-${capacity}`} ref={shaft_ref} args={[undefined, undefined, capacity]}
            frustumCulled={false} renderOrder={is_overlay ? 1000 : 0}>
            <cylinderGeometry args={[0.5, 0.5, 1, 8]} />
            <meshBasicMaterial color="white" opacity={resolved.opacity} transparent={resolved.opacity < 1}
                depthTest={!is_overlay} depthWrite={!is_overlay && resolved.opacity === 1} toneMapped={false} />
        </instancedMesh>
        <instancedMesh key={`head-${capacity}`} ref={head_ref} args={[undefined, undefined, capacity]}
            frustumCulled={false} renderOrder={is_overlay ? 1000 : 0}>
            <coneGeometry args={[0.5, 1, 8]} />
            <meshBasicMaterial color="white" opacity={resolved.opacity} transparent={resolved.opacity < 1}
                depthTest={!is_overlay} depthWrite={!is_overlay && resolved.opacity === 1} toneMapped={false} />
        </instancedMesh>
    </group>;
}

export function ArrowStyleControls({ style_key, can_have_orientation, base_style }: {
    style_key: string; can_have_orientation: boolean; base_style?: Partial<arrow_style>;
}) {
    const overrides = useArrowSettings(style_key);
    const style = resolve_arrow_style({ ...base_style, ...overrides });
    const update = (value: Partial<arrow_style>) => update_arrow_settings(style_key, { ...overrides, ...value });
    const dimensions = (value: arrow_dimensions, change: (value: Partial<arrow_dimensions>) => void) => <>
        {([['length', '全長'], ['shaft_diameter', '軸直径'], ['head_length', '矢先長'], ['head_diameter', '矢先直径']] as const).map(([key, label]) =>
            <label key={key} className="flex justify-between gap-2">{label} [m]
                <input className="w-20 bg-black/20" aria-label={label} type="number" min="0.0001" step="0.001"
                    value={value[key]} onChange={event => {
                        const next = event.currentTarget.valueAsNumber;
                        if (Number.isFinite(next) && next > 0) change({ [key]: next });
                    }} />
            </label>)}
        <label className="flex justify-between">色<input type="color" value={value.color}
            onChange={event => change({ color: event.target.value })} /></label>
    </>;
    return <details className="mt-1 text-xs" onClick={event => event.stopPropagation()}>
        <summary>矢印表示</summary>
        <div className="space-y-1 p-1">
            <label className="flex justify-between">基準位置<select value={style.anchor} className="bg-black/20"
                onChange={event => update({ anchor: event.target.value as arrow_style['anchor'] })}>
                <option value="tail">根元</option><option value="tip">矢先</option><option value="center">中点</option>
            </select></label>
            {can_have_orientation && <label className="flex justify-between">主軸<select value={style.primary_axis}
                className="bg-black/20" onChange={event => update({ primary_axis: event.target.value as arrow_style['primary_axis'] })}>
                <option value="x">X</option><option value="y">Y</option><option value="z">Z</option>
            </select></label>}
            {dimensions(style, value => update(value))}
            {style.head_length > style.length && <p>矢先長を全長以内に設定してください。</p>}
            <label className="flex justify-between">不透明度<input className="w-20" type="range" min="0" max="1" step="0.05"
                value={style.opacity} onChange={event => update({ opacity: event.currentTarget.valueAsNumber })} /></label>
            <label>前面表示<input type="checkbox" checked={style.depth_mode === 'overlay'}
                onChange={event => update({ depth_mode: event.target.checked ? 'overlay' : 'scene' })} /></label>
            <label className="block">状態による色<input type="checkbox" checked={style.enable_state_colors}
                onChange={event => update({ enable_state_colors: event.target.checked })} /></label>
            {style.enable_state_colors && (['未評価', '範囲内', '範囲外']).map((label, state) =>
                <label key={state} className="flex justify-between">{label}<input type="color" value={style.state_colors[state] ?? style.color}
                    onChange={event => update({ state_colors: { ...style.state_colors, [state]: event.target.value } })} /></label>)}
            <label className="block">補助2軸<input type="checkbox" disabled={!can_have_orientation}
                checked={can_have_orientation && style.enable_transverse_axes}
                onChange={event => update({ enable_transverse_axes: event.target.checked })} /></label>
            {!can_have_orientation && <p>方向のみの入力では補助軸はありません。</p>}
            {can_have_orientation && style.enable_transverse_axes && (['x', 'y', 'z'] as const)
                .filter(axis => axis !== style.primary_axis).map(axis => <details key={axis}>
                    <summary>補助 {axis.toUpperCase()} 軸</summary>
                    {dimensions(style.transverse_axes[axis], value => update({ transverse_axes: { ...style.transverse_axes, [axis]: { ...style.transverse_axes[axis], ...value } } }))}
                </details>)}
            <button type="button" onClick={() => update_arrow_settings(style_key, {})}>入力・既定設定に戻す</button>
        </div>
    </details>;
}
