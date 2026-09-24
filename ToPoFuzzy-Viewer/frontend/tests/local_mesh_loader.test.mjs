import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';

test('直接メッシュの形式・参照解決・容量制限・解放', { timeout: 60000 }, async () => {
    const temporary = await mkdtemp(resolve('tests/.local-mesh-'));
    try {
        const output = resolve(temporary, 'loader.mjs');
        await build({ entryPoints: ['src/features/meshes/local_mesh_loader.ts'], outfile: output,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', logLevel: 'silent' });
        const { load_local_mesh, local_resource_bundle, max_mesh_file_bytes } = await import(pathToFileURL(output));
        const file = (name, body, path) => {
            const value = new File([body], name);
            if (path) Object.defineProperty(value, 'webkitRelativePath', { value: path });
            return value;
        };
        const obj = file('body.OBJ', 'mtllib missing.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n');
        const asset = await load_local_mesh(obj, [obj]);
        assert.equal(asset.num_triangles, 1);
        assert.equal(asset.warnings.length, 1);
        let num_disposed = 0;
        asset.object.children[0].geometry.addEventListener('dispose', () => num_disposed++);
        asset.dispose();
        assert.equal(num_disposed, 1);
        const no_faces = file('points.ply', 'ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n');
        await assert.rejects(load_local_mesh(no_faces, [no_faces]), /面のないPLY/);
        const stl = file('triangle.stl', 'solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t');
        const stl_asset = await load_local_mesh(stl, [stl]);
        assert.equal(stl_asset.num_triangles, 1);
        stl_asset.dispose();
        const large = { name: 'large.obj', size: max_mesh_file_bytes + 1, arrayBuffer() { assert.fail('容量検査前の読込'); } };
        await assert.rejects(load_local_mesh(large, [large]), /サイズ上限/);
        const root = file('scene.gltf', '', 'bundle/model/scene.gltf');
        const texture = file('diffuse.png', 'pixels', 'bundle/textures/diffuse.png');
        const bundle = local_resource_bundle([root, texture], root);
        assert.equal(bundle.find_file('../textures/diffuse.png'), texture);
        assert.equal(bundle.find_file('bundle/textures/diffuse.png'), texture);
        assert.throws(() => bundle.resolve('https://not-selected.invalid/private.bin'), /付属ファイル/);
        const url = bundle.resolve('../textures/diffuse.png');
        assert.equal(await (await fetch(url)).text(), 'pixels');
        bundle.dispose();
        await assert.rejects(fetch(url));
        const duplicate = file('diffuse.png', '', 'bundle/other/diffuse.png');
        assert.throws(() => local_resource_bundle([root, texture, duplicate], root).find_file('diffuse.png'), /曖昧/);
        assert.equal(local_resource_bundle([root, texture, duplicate], root).find_file('../textures/diffuse.png'), texture);
        const broken = file('broken.obj', 'v 0 0 0\nv 1 0 0\nf 1 2 99\n');
        await assert.rejects(load_local_mesh(broken, [broken]), /有限座標/);

        // 任意指定の取得原本による形式互換性の検証
        if (process.env.VEHICLE_MODELS) {
            for (const [path, expected] of [
                ['opengameart/fancy_motorcycle/bike.obj', 9000],
                ['artec3d/motorbike/ply/Motorbike_ply.ply', 1042148],
                ['poly_pizza/bicycle/bicycle.glb', 4780],
                ['mirageym/road_bike/RoadBike_SubDiv.fbx', 1000],
            ]) {
                const model = file(path.split('/').pop(), await readFile(resolve(process.env.VEHICLE_MODELS, path)));
                const loaded = await load_local_mesh(model, [model]);
                assert.ok(loaded.num_triangles >= expected, path);
                console.log('実モデル', path, loaded.num_vertices, '頂点', loaded.num_triangles, '三角形');
                loaded.dispose();
            }
        }
    } finally {
        await rm(temporary, { recursive: true, force: true });
    }
});
