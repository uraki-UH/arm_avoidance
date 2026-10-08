import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { frame_tree } from '../src/frame-tree';
import { validate_graph, deserializeTopologicalMap } from '@topo/visualization/graph_protocol';

function make_frames() {
    const base = new THREE.Group(); base.position.set(1, 2, 3);
    return { base, frames: new frame_tree(() => ({ links: { base_footprint: base } })) };
}
test('異常座標・切れたバイナリの描画前拒否', () => {
    const graph = { nodes: [{ x: NaN, y: 0, z: 0 }], edges: [], clusters: [] };
    assert.throws(() => validate_graph(graph));
    assert.throws(() => deserializeTopologicalMap(new ArrayBuffer(35)));
});
test('TF合成・不明フレーム・動的TF期限と循環の拒否', () => {
    const { frames } = make_frames();
    frames.update([{ frameId: 'world', childFrameId: 'sensor', pos: [4, 0, 0], quat: [0, 0, 0, 1] }], false);
    frames.begin_frame(performance.now());
    assert.equal(frames.resolve('sensor')!.elements[12], 4);
    assert.equal(frames.resolve('map'), null);
    frames.begin_frame(performance.now() + 3000);
    assert.equal(frames.resolve('sensor'), null);
    frames.update([{ frameId: 'sensor', childFrameId: 'world', pos: [0, 0, 0], quat: [0, 0, 0, 1] }], true);
    frames.fixed_frame = 'unconnected'; frames.begin_frame(performance.now());
    assert.equal(frames.resolve('sensor'), null);
});
test('フレーム内の変換再利用・手動配置追従・切断時のTF破棄', () => {
    const { frames, base } = make_frames(); frames.begin_frame(performance.now());
    const first = frames.resolve('base_footprint')!;
    assert.deepEqual(new THREE.Vector3().setFromMatrixPosition(first).toArray(), [1, 2, 3]);
    assert.equal(frames.resolve('base_footprint'), first);
    base.position.x = 5; frames.begin_frame(performance.now());
    assert.equal(frames.resolve('base_footprint')!.elements[12], 5);
    frames.update([{ frameId: 'world', childFrameId: 'sensor', pos: [1, 0, 0], quat: [0, 0, 0, 1] }], true);
    frames.clear(); assert.equal(frames.resolve('sensor'), null);
});

test('名前空間付きマップと外部センサーの実リンク追従・モデル切替時の分離', () => {
    const base = new THREE.Group();
    base.position.set(.3, -.2, .1); base.rotation.z = Math.PI / 2;
    const link = new THREE.Group(); link.position.set(0, .4, .2); base.add(link);
    const robot = { ros_frame_prefix: 'topo_dual_arm_max_long', links: { base_footprint: base, base_link: link } };
    const frames = new frame_tree(() => robot);
    // 旧ゲートウェイ経由の期限切れTFも、選択モデルの明示対応へ影響なし。
    frames.update([{ frameId: 'world', childFrameId: 'topo_dual_arm_max_long/base_link',
        pos: [99, 0, 0], quat: [0, 0, 0, 1] }], false);
    frames.update([{ frameId: 'topo_dual_arm_max_long/base_link', childFrameId: 'sensor',
        pos: [.1, 0, 0], quat: [0, 0, 0, 1] }], true);
    frames.begin_frame(performance.now() + 3000);
    const resolved = frames.resolve('topo_dual_arm_max_long/base_link')!;
    assert.deepEqual(resolved.elements, link.matrixWorld.elements);
    const sensor = new THREE.Vector3().setFromMatrixPosition(frames.resolve('sensor')!);
    assert.ok(sensor.distanceTo(new THREE.Vector3(-.1, -.1, .3)) < 1e-8);
    assert.equal(frames.resolve('other_robot/base_link'), null);
    assert.ok(frames.unresolved_frames.has('other_robot/base_link'));
    base.position.x += 1; frames.begin_frame(performance.now() + 3000);
    assert.ok(Math.abs(frames.resolve('topo_dual_arm_max_long/base_link')!.elements[12] - .9) < 1e-8);
    robot.ros_frame_prefix = 'topo_dual_arm_max'; frames.begin_frame(performance.now() + 3000);
    assert.equal(frames.resolve('topo_dual_arm_max_long/base_link'), null);
    assert.deepEqual(frames.resolve('topo_dual_arm_max/base_link')!.elements, link.matrixWorld.elements);
    frames.clear(); assert.equal(frames.unresolved_frames.size, 0);
});
