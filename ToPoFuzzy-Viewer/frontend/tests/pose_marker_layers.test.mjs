import assert from 'node:assert/strict';
import { test } from 'node:test';
import { build } from 'esbuild';

const compiled = await build({
    entryPoints: ['src/features/visualization/pose_marker_layers.ts'],
    bundle: true, platform: 'node', format: 'esm', write: false,
});
const { pose_marker_layers } = await import(
    `data:text/javascript;base64,${Buffer.from(compiled.outputFiles[0].text).toString('base64')}`);
const tag = '/grasp_pose_cands';
const evaluation_tag = `${tag}/reachability_markers`;
const marker = {
    id: 0, action: 0, type: 'arrow', frameId: 'table', header_stamp: [12, 34],
    points: [[0, 0, 1], [0, 0, 0.92]], color: [0, 1, 1, 1],
};
const raw = { source_type: 'pose_array', markers: [marker], count: 1 };
const evaluated = { ...marker, color: [0, 1, 0, 1] };

test('計画側なしでも候補を表示', () => {
    assert.equal(pose_marker_layers({ [tag]: raw }, {}, new Set())[tag], raw);
});

test('同一更新の到達性は色のみ統合し二重描画なし', () => {
    const result = pose_marker_layers({
        [tag]: raw, [evaluation_tag]: { markers: [evaluated] },
    }, {}, new Set());
    assert.deepEqual(Object.keys(result), [tag]);
    assert.deepEqual(result[tag].markers[0].color, evaluated.color);
    assert.deepEqual(raw.markers[0].color, [0, 1, 1, 1]);
});

test('古い更新・座標系・位置の評価色を流用しない', () => {
    for (const change of [{ header_stamp: [11, 34] }, { frameId: 'other' },
        { points: [[1, 0, 1], [1, 0, 0.92]] }]) {
        const result = pose_marker_layers({
            [tag]: raw, [evaluation_tag]: { markers: [{ ...evaluated, ...change }] },
        }, {}, new Set());
        assert.deepEqual(result[tag].markers[0].color, marker.color);
        assert.equal(result[evaluation_tag], undefined);
    }
});

test('空候補で旧評価の矢印を残さない', () => {
    const result = pose_marker_layers({
        [tag]: { ...raw, markers: [], count: 0 },
        [evaluation_tag]: { markers: [evaluated] },
    }, {}, new Set());
    assert.equal(result[tag].markers.length, 0);
    assert.equal(result[evaluation_tag], undefined);
});

test('非表示・無効化した候補は評価表示を抑制しない', () => {
    const data = { [tag]: raw, [evaluation_tag]: { markers: [evaluated] } };
    assert.ok(pose_marker_layers(data, { [tag]: { visible: false } }, new Set())[evaluation_tag]);
    assert.ok(pose_marker_layers(data, {}, new Set([tag]))[evaluation_tag]);
    assert.equal(pose_marker_layers(data, {}, new Set([evaluation_tag]))[tag], raw);
});
