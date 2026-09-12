import * as THREE from 'three';

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
    state_colors: { 0: '#f3da59', 1: '#7ceeb6', 2: '#c4c4c4' },
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
        ? style.state_colors[sample.state] ?? style.color : style.color;
    append(direction, { ...style, length: sample.length ?? style.length, color }, style.anchor);
    if (parts.length && orientation && style.enable_transverse_axes) {
        for (const axis of ['x', 'y', 'z'] as const) {
            if (axis !== style.primary_axis) append(new THREE.Vector3(...axes[axis]).applyQuaternion(orientation), style.transverse_axes[axis], 'tail');
        }
    }
    return parts;
}

// グラフ表示と設定UIで共用する用途別初期値
export function normal_arrow_style(scale = 0.075, color = '#4fa3a5'): Partial<arrow_style> {
    return { length: Math.min(0.35, scale), color, opacity: 0.65, shaft_diameter: 0.003,
        head_length: Math.min(0.02, scale * 0.24), head_diameter: 0.012, depth_mode: 'overlay' };
}
export function velocity_arrow_style(length = 0.25, color = '#ffb347'): Partial<arrow_style> {
    return { length: Math.min(0.5, length), color, shaft_diameter: 0.008,
        head_length: Math.min(0.02, length * 0.28) };
}
