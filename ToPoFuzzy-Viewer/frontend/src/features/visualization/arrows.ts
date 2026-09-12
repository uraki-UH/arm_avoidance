import { useSyncExternalStore } from 'react';
import * as THREE from 'three';
import { MarkerArrayData, MarkerMessage } from '../../types';

export type vector3 = [number, number, number];
export type quaternion4 = [number, number, number, number];
export type arrow_axis = 'x' | 'y' | 'z';
export interface arrow_dimensions {
    length: number;
    shaft_diameter: number;
    head_length: number;
    head_diameter: number;
    color: string;
}
export interface arrow_style extends arrow_dimensions {
    anchor: 'tail' | 'tip' | 'center';
    primary_axis: arrow_axis;
    opacity: number;
    depth_mode: 'scene' | 'overlay';
    is_visible: boolean;
    enable_transverse_axes: boolean;
    transverse_axes: Record<arrow_axis, arrow_dimensions>;
    enable_state_colors: boolean;
    state_colors: Record<number, string>;
}
export interface arrow_sample {
    position: vector3;
    orientation?: quaternion4;
    direction?: vector3;
    state?: number;
    length?: number;
}

const axis_style = (color: string): arrow_dimensions => ({
    color, length: 0.04, shaft_diameter: 0.004, head_length: 0.01, head_diameter: 0.008,
});
export const default_arrow_style: arrow_style = {
    length: 0.08, shaft_diameter: 0.008, head_length: 0.02, head_diameter: 0.016,
    color: '#00d1ff', opacity: 1, anchor: 'tail', primary_axis: 'z', depth_mode: 'scene',
    is_visible: true, enable_transverse_axes: false, enable_state_colors: true,
    transverse_axes: { x: axis_style('#ff4444'), y: axis_style('#44dd44'), z: axis_style('#4488ff') },
    state_colors: {},
};
export function resolve_arrow_style(style: Partial<arrow_style> = {}): arrow_style {
    return { ...default_arrow_style, ...style,
        anchor: ['tail', 'tip', 'center'].includes(style.anchor ?? '') ? style.anchor! : 'tail',
        primary_axis: ['x', 'y', 'z'].includes(style.primary_axis ?? '') ? style.primary_axis! : 'z',
        transverse_axes: {
            x: { ...default_arrow_style.transverse_axes.x, ...style.transverse_axes?.x },
            y: { ...default_arrow_style.transverse_axes.y, ...style.transverse_axes?.y },
            z: { ...default_arrow_style.transverse_axes.z, ...style.transverse_axes?.z },
        },
        state_colors: { ...default_arrow_style.state_colors, ...style.state_colors } };
}
export interface arrow_part {
    shaft: THREE.Matrix4;
    head: THREE.Matrix4;
    color: THREE.Color;
}
const axes: Record<arrow_axis, vector3> = { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };

// 単体・一括描画共通の端点と実寸計算。単位円柱・円錐は直径1、高さ1
export function build_arrow_parts(sample: arrow_sample, style: arrow_style): arrow_part[] {
    if (!style.is_visible || !sample.position.every(Number.isFinite) ||
        !Number.isFinite(style.opacity) || style.opacity <= 0 || style.opacity > 1) return [];
    const position = new THREE.Vector3(...sample.position);
    let orientation: THREE.Quaternion | undefined;
    let direction: THREE.Vector3;
    if (sample.orientation) {
        if (!sample.orientation.every(Number.isFinite)) return [];
        orientation = new THREE.Quaternion(...sample.orientation);
        if (orientation.lengthSq() <= 1e-16) return [];
        orientation.normalize();
        direction = new THREE.Vector3(...axes[style.primary_axis]).applyQuaternion(orientation);
    } else if (sample.direction?.every(Number.isFinite)) {
        direction = new THREE.Vector3(...sample.direction);
        if (direction.lengthSq() <= 1e-16) return [];
        direction.normalize();
    } else return [];

    const parts: arrow_part[] = [];
    const append = (dir: THREE.Vector3, dimensions: arrow_dimensions, anchor: arrow_style['anchor']) => {
        const { length, shaft_diameter, head_length, head_diameter } = dimensions;
        if (![length, shaft_diameter, head_length, head_diameter].every(v => Number.isFinite(v) && v > 0) || head_length > length) return;
        const tail = position.clone().addScaledVector(dir, anchor === 'tip' ? -length : anchor === 'center' ? -length / 2 : 0);
        const rotation = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        const shaft_length = length - head_length;
        parts.push({
            shaft: new THREE.Matrix4().compose(tail.clone().addScaledVector(dir, shaft_length / 2), rotation,
                new THREE.Vector3(shaft_diameter, shaft_length, shaft_diameter)),
            head: new THREE.Matrix4().compose(tail.clone().addScaledVector(dir, length - head_length / 2), rotation,
                new THREE.Vector3(head_diameter, head_length, head_diameter)),
            color: new THREE.Color(dimensions.color),
        });
    };
    const color = style.enable_state_colors && sample.state !== undefined
        ? style.state_colors[sample.state] ?? style.state_colors[0] ?? style.color : style.color;
    append(direction, { ...style, length: sample.length ?? style.length, color }, style.anchor);
    if (parts.length && orientation && style.enable_transverse_axes) {
        for (const axis of ['x', 'y', 'z'] as const) {
            if (axis !== style.primary_axis) append(new THREE.Vector3(...axes[axis]).applyQuaternion(orientation), style.transverse_axes[axis], 'tail');
        }
    }
    return parts;
}

// グラフ・クラスタ詳細の補助表示用共通値。GUIによる上書き対象外
export const normal_arrow_style = {
    length: 0.0375, color: '#4fa3a5', opacity: 0.65, shaft_diameter: 0.003,
    head_length: 0.009, head_diameter: 0.006, depth_mode: 'overlay',
} satisfies Partial<arrow_style>;

// 速度の大きさに応じた表示長。速度の単位はm/s、表示倍率は0.25秒相当
export function velocity_arrow_style(speed: number): Partial<arrow_style> {
    const length = Math.min(0.5, speed * 0.25);
    return { length, color: '#ffb347', shaft_diameter: 0.008,
        head_length: Math.min(0.02, length * 0.28) };
}

// 矢印・通常Marker共通のlinear RGB色と不透明度
export function marker_color(value: MarkerMessage['color']) {
    const rgba = Array.isArray(value) ? value : [value?.r ?? 1, value?.g ?? 1, value?.b ?? 1, value?.a ?? 1];
    return { color: new THREE.Color(rgba[0], rgba[1], rgba[2]), opacity: rgba[3] ?? 1 };
}

// 姿勢方式・始点終点方式の解釈を描画と設定UIで共用
function marker_arrow(marker: MarkerMessage, styles: MarkerArrayData['arrow_styles']) {
    if (marker.type !== 'arrow' || marker.action === 2 || marker.action === 3) return;
    if (marker.arrow_style_id !== undefined) {
        const shared = styles?.[marker.arrow_style_id];
        if (!shared) return;
        marker = { ...marker, ...shared };
    }
    const sample: arrow_sample = { position: marker.pos ?? [0, 0, 0], state: marker.state };
    let style: Partial<arrow_style> = { primary_axis: 'z', depth_mode: 'overlay', state_colors: marker.state_colors };
    if (marker.orientation) sample.orientation = marker.orientation;
    else {
        const points = marker.points ?? [];
        const head_length = marker.scale?.[2] ?? 0;
        const is_endpoints = points.length >= 2;
        const { color, opacity } = marker_color(marker.color);
        const rotation = new THREE.Quaternion(...(marker.quat ?? [0, 0, 0, 1]));
        if (!rotation.toArray().every(Number.isFinite) || rotation.lengthSq() <= 1e-16) return;
        rotation.normalize();
        let length = marker.scale?.[0] ?? 0.08;
        if (is_endpoints) {
            const start = new THREE.Vector3(...points[0]);
            const direction = new THREE.Vector3(...points[1]).sub(start);
            length = direction.length();
            sample.position = start.applyQuaternion(rotation).add(new THREE.Vector3(...sample.position)).toArray();
            sample.direction = direction.applyQuaternion(rotation).toArray();
        } else sample.orientation = rotation.toArray();
        style = { ...style, primary_axis: is_endpoints ? 'z' : 'x', length,
            color: '#' + color.getHexString(), opacity,
            shaft_diameter: marker.scale?.[is_endpoints ? 0 : 1] ?? 0.008,
            head_diameter: marker.scale?.[is_endpoints ? 1 : 2] ?? 0.016,
            head_length: is_endpoints && head_length > 0 ? head_length : length * 0.23 };
    }
    return { sample, style };
}

export function marker_arrow_options(data: MarkerArrayData) {
    let base_style: Partial<arrow_style> | undefined;
    let can_have_orientation = false;
    for (const marker of data.markers) {
        const arrow = marker_arrow(marker, data.arrow_styles);
        if (!arrow) continue;
        base_style ??= arrow.style;
        can_have_orientation ||= !!arrow.sample.orientation;
    }
    return { base_style, can_have_orientation };
}

// 全入力方式を同じ座標系・描画設定ごとに一括描画
export function marker_arrow_batches(data: MarkerArrayData, overrides: Partial<arrow_style>) {
    const batches = new Map<string, { marker: MarkerMessage; samples: arrow_sample[]; style: Partial<arrow_style> }>();
    for (const marker of data.markers) {
        const arrow = marker_arrow(marker, data.arrow_styles);
        if (!arrow) continue;
        const style = { ...arrow.style, ...overrides };
        const key = JSON.stringify([marker.frameId ?? '', style]);
        if (!batches.has(key)) batches.set(key, { marker, samples: [], style });
        batches.get(key)!.samples.push(arrow.sample);
    }
    return [...batches.entries()];
}

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
