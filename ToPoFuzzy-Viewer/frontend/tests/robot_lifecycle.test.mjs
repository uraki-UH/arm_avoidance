import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { runInNewContext } from 'node:vm';

test('ロボット削除時の未描画姿勢破棄と他ロボット維持', () => {
    // 本番のイベントハンドラを抽出し、React状態更新のみ同期代替。
    const source = readFileSync(new URL('../src/hooks/useWebSocket.ts', import.meta.url), 'utf8');
    const handler = source.match(/'stream\.robot\.delete': (\(p\) => \{[\s\S]*?\n                        \}),/)[1];
    const real_robot = { urdf: 'real' };
    let robots = { ToPoDualArm: real_robot, sim_ToPoDualArm: { urdf: 'sim' } };
    const pending = new Map(Object.entries(robots));
    const on_delete = runInNewContext(`(${handler})`, {
        pendingRobotPoseUpdatesRef: { current: pending },
        setRobotData: (update) => { robots = update(robots); },
    });
    on_delete({ tag: 'sim_ToPoDualArm' });
    assert.deepEqual(Object.keys(robots), ['ToPoDualArm']);
    assert.equal(robots.ToPoDualArm, real_robot);
    assert.deepEqual([...pending.keys()], ['ToPoDualArm']);
    on_delete({});
    on_delete({ tag: 'sim_ToPoDualArm' });
    assert.deepEqual(Object.keys(robots), ['ToPoDualArm']);
});
