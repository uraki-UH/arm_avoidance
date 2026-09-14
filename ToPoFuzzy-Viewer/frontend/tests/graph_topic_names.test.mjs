import assert from 'node:assert/strict';
import { build } from 'esbuild';
import { test } from 'node:test';

test('新旧グラフトピック名の経路レイヤー判定', async () => {
    const result = await build({
        entryPoints: ['src/types/index.ts'], bundle: true, write: false,
        platform: 'node', format: 'esm', logLevel: 'silent',
    });
    const { isTrajectoryGraphTag } = await import(
        `data:text/javascript;base64,${Buffer.from(result.outputFiles[0].contents).toString('base64')}`);
    for (const name of ['Tmap', 'tmap', 'topological_map']) {
        for (const prefix of ['plan', 'cand']) {
            for (const suffix of ['', '_vis_L0', '/nodes']) {
                assert.equal(isTrajectoryGraphTag(`/ToPoDualArm/${prefix}_${name}${suffix}`), true);
            }
        }
        for (const tag of [`/${name}`, `/selected_${name}`, `/basket/${name}_static`]) {
            assert.equal(isTrajectoryGraphTag(tag), false);
        }
    }
    assert.equal(isTrajectoryGraphTag('/plan_tmapping'), false);
});
