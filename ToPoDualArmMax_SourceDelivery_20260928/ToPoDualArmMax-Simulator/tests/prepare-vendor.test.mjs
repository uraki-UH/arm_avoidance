import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp, mkdir, writeFile, readFile, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {prepare_vendor} from '../tools/prepare_vendor.mjs';

test('Three.jsの本体・内部依存の復元と、addonsとの版不一致の拒否', async () => {
  const root = await mkdtemp(path.join(tmpdir(), 'topo-vendor-test-'));
  try {
    await mkdir(path.join(root, 'node_modules/three/build'), {recursive: true});
    await mkdir(path.join(root, 'app/vendor/three'), {recursive: true});
    for (const folder of ['node_modules/three', 'app/vendor/three']) {
      await writeFile(path.join(root, folder, 'package.json'), JSON.stringify({version: '0.180.0'}));
    }
    for (const file of ['three.module.js', 'three.core.js']) {
      await writeFile(path.join(root, 'node_modules/three/build', file), `// ${file}\n`);
    }
    await prepare_vendor(root);
    for (const file of ['three.module.js', 'three.core.js']) {
      assert.equal(await readFile(path.join(root, 'app/vendor/three/build', file), 'utf8'), `// ${file}\n`);
    }
    await writeFile(path.join(root, 'node_modules/three/package.json'), JSON.stringify({version: '0.181.0'}));
    await assert.rejects(prepare_vendor(root), /版が不一致/);
  } finally {
    await rm(root, {recursive: true, force: true});
  }
});
