import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { createElement } from 'react';
import { renderToStaticMarkup } from 'react-dom/server';
import { build } from 'esbuild';

test('候補独立ビューの短縮名・情報の1行化・元ソースの維持', async () => {
    const directory = await mkdtemp(resolve('tests/.cluster-detail-'));
    try {
        const outfile = resolve(directory, 'panel.mjs');
        // Canvasのみ省略した、実パネルのHTML出力検証
        await build({ entryPoints: ['src/features/visualization/ClusterDetailPanel.tsx'], outfile,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', jsx: 'automatic',
            plugins: [{ name: 'canvas', setup(builder) {
                builder.onResolve({ filter: /^@react-three\/fiber$/ }, () => ({ path: 'canvas', namespace: 'test' }));
                builder.onLoad({ filter: /.*/, namespace: 'test' }, () => ({ contents:
                    'export const Canvas = () => null; export const useThree = () => ({}); export const useFrame = () => {};' }));
            } }] });
        const { ClusterDetailPanel } = await import(pathToFileURL(outfile).href);
        const snapshot = { title: 'nonplane_components #7', source_id: '/nonplane_components',
            selection: { kind: 'marker', ns: 'nonplane_components', id: 7 },
            graph: { nodes: [{ id: 0 }, { id: 1 }], edges: [0, 1], frameId: 'world' },
            min_position: [0, 0, 0], max_position: [0.049, 0.065, 0.161] };
        const original = structuredClone(snapshot);
        const render = value => renderToStaticMarkup(createElement(ClusterDetailPanel, {
            snapshot: value, onClose() {}, on_refresh() {}, is_loading: false, error: null,
        }));
        const html = render(snapshot);
        assert.match(html, /<h3[^>]*>nonplane_7<\/h3>/);
        assert.match(html, /title="\/nonplane_components">\/nonplane_<\/p>/);
        assert.match(html, /<footer[^>]*whitespace-nowrap[^>]*><p>2 nodes \/ 1 edges \/ XYZ寸法: 0.049 \/ 0.065 \/ 0.161 m<\/p><\/footer>/);
        assert.doesNotMatch(html, /座標系:|元シーン・TFの変更なし/);
        assert.match(html, /<input type="checkbox"\/> XYZ軸/);
        assert.match(html, /<input type="checkbox"\/> Bbox/);
        assert.deepEqual(snapshot, original);
        const other = render({ ...snapshot, title: 'cluster #3', source_id: '/nonplane_components_extra' });
        assert.match(other, /<h3[^>]*>cluster #3<\/h3>/);
        assert.match(other, /title="\/nonplane_components_extra">\/nonplane_components_extra<\/p>/);
    } finally {
        await rm(directory, { recursive: true, force: true });
    }
});
