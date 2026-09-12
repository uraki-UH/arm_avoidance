import * as THREE from 'three';
import { MarkerArrayData, MarkerMessage } from '../../../types';
import { arrow_sample, arrow_style } from './geometry';

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
        const rgba = Array.isArray(marker.color) ? marker.color :
            [marker.color?.r ?? 1, marker.color?.g ?? 1, marker.color?.b ?? 1, marker.color?.a ?? 1];
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
            color: '#' + new THREE.Color(rgba[0], rgba[1], rgba[2]).getHexString(), opacity: rgba[3] ?? 1,
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
