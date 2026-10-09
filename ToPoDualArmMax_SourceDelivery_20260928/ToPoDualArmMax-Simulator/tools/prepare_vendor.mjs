import {copyFile, mkdir, readFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

export async function prepare_vendor(root) {
  const source = path.join(root, 'node_modules/three');
  const destination = path.join(root, 'app/vendor/three');
  const installed = JSON.parse(await readFile(path.join(source, 'package.json'), 'utf8'));
  const bundled = JSON.parse(await readFile(path.join(destination, 'package.json'), 'utf8'));
  if (installed.version !== bundled.version) {
    throw new Error(`Three.jsの版が不一致です: npm=${installed.version}, 同梱addons=${bundled.version}`);
  }
  // import mapの参照先と、その内部依存。Gitのbuild除外後もnpmから復元可能な配置
  await mkdir(path.join(destination, 'build'), {recursive: true});
  for (const name of ['three.module.js', 'three.core.js']) {
    await copyFile(path.join(source, 'build', name), path.join(destination, 'build', name));
  }
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  await prepare_vendor(fileURLToPath(new URL('..', import.meta.url)));
}
