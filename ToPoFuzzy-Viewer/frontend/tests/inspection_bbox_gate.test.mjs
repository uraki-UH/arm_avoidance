import assert from 'node:assert/strict';
import { mkdtemp, readFile, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import { renderToStaticMarkup } from 'react-dom/server';
import ts from 'typescript';

// 実Appの接続条件と非同期コールバックの検証。DOM・WebGL・通信のみ対象外
const source = ts.createSourceFile('App.tsx', await readFile('src/App.tsx', 'utf8'),
    ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX);
function find(predicate, node = source) {
    if (predicate(node)) return node;
    return ts.forEachChild(node, child => find(predicate, child));
}
function evaluate(expression, scope) {
    const code = ts.transpileModule(`return (${expression});`, {
        compilerOptions: { target: ts.ScriptTarget.ES2022 },
    }).outputText;
    return new Function(...Object.keys(scope), code)(...Object.values(scope));
}
function attribute(component, name) {
    const element = find(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(source) === component);
    assert.ok(element, component);
    return element.attributes.properties.find(prop => prop.name?.getText(source) === name)
        .initializer.expression.getText(source);
}

test('未指定トピックのBbox GUI非表示と、明示OFFからの再有効化', async () => {
    const directory = await mkdtemp(resolve('tests/.bbox-controls-'));
    try {
        const outfile = resolve(directory, 'controls.mjs');
        await build({ stdin: { contents: `
            export { GngLayerControls } from './src/features/visualization/GngLayerControls';
            export { createDefaultGraphLayerSettings } from './src/features/visualization/graphLayerSettings';
        `, resolveDir: process.cwd() }, outfile, bundle: true, packages: 'external',
            platform: 'node', format: 'esm', jsx: 'automatic' });
        const { GngLayerControls, createDefaultGraphLayerSettings } = await import(pathToFileURL(outfile).href);
        const graph = { mode: 'dynamic', nodes: [], edges: [], clusters: [] };
        for (const tag of ['/topological_map', '/ToPoDualArm/Tmap_static', '/grasp_pose_cands/Tmap', '/nonplane_components', '/custom']) {
            let settings = createDefaultGraphLayerSettings(tag, graph);
            const render = () => GngLayerControls({ tag, graphData: graph, settings, onRemove() {},
                onUpdate: updates => { settings = { ...settings, ...updates }; } });
            assert.equal(renderToStaticMarkup(render()).includes('Bounding Box'),
                tag === '/grasp_pose_cands/Tmap');
            if (tag === '/nonplane_components') assert.equal(settings.enable_bounding_box, undefined);
            for (const enable_bounding_box of [false, true]) {
                settings.enable_bounding_box = enable_bounding_box;
                const tree = render();
                const controls = [];
                const visit = node => {
                    if (Array.isArray(node)) node.forEach(visit);
                    else if (node?.props) { controls.push(node); visit(node.props.children); }
                };
                visit(tree);
                const toggle = controls.find(node => node.props.label === 'Bounding Box');
                if (tag === '/topological_map') {
                    assert.equal(toggle, undefined);
                    assert.ok(!renderToStaticMarkup(render()).includes('車両照合の対象選択'));
                    continue;
                }
                assert.equal(toggle.props.isOn, enable_bounding_box);
                toggle.props.onToggle();
                assert.equal(settings.enable_bounding_box, !enable_bounding_box);
                assert.ok(renderToStaticMarkup(render()).includes('Bounding Box'));
            }
            delete settings.enable_bounding_box;
            assert.ok(!renderToStaticMarkup(render()).includes('Bounding Box'));
        }
    } finally {
        await rm(directory, { recursive: true, force: true });
    }
});

test('ノード・クラスタ・Markerの直接選択はBboxと編集状態に従属', () => {
    for (const tag of ['/test', '/topological_map']) for (const enable_bounding_box of [undefined, false, true]) {
        for (const isEditMode of [false, true]) for (const isDrawing of [false, true]) {
            const settings = { enable_bounding_box };
            const scope = { settings, layerSettings: { [tag]: settings }, tag,
                isEditMode, zoneMonitor: { isDrawing }, handle_inspect() {} };
            const expected = tag !== '/topological_map' && enable_bounding_box === true && !isEditMode && !isDrawing;
            assert.equal(evaluate(attribute('GraphRenderer', 'enableClusterSelection'), scope), expected);
            assert.equal(typeof evaluate(attribute('MarkerArrayRenderer', 'on_inspect'), scope) === 'function', expected);
        }
    }
});

test('Bbox OFF時のRPC抑止と、取得中のOFFによる遅延表示の防止', async () => {
    const declaration = find(node => ts.isVariableDeclaration(node) && node.name.getText(source) === 'handle_inspect');
    const expression = declaration.initializer.arguments[0].getText(source);
    for (const kind of ['node', 'cluster', 'marker']) {
        const settings = {};
        const selection = { kind, id: 7 };
        let complete;
        let fail;
        let num_requests = 0;
        let is_loading = false;
        const snapshots = [];
        const scope = {
            inspection_sources: { current: { layerSettings: { '/test': settings },
                graphData: { '/test': {} }, markerData: { '/test': {} } } },
            inspection_request: { current: 0 },
            set_is_inspecting: value => { is_loading = value; },
            set_inspection_error: error => { assert.equal(error, null); },
            setSelectedClusterSnapshot: snapshot => snapshots.push(snapshot),
            inspect_graph: () => { ++num_requests; return new Promise((resolve, reject) => { complete = resolve; fail = reject; }); },
        };
        const inspect = evaluate(expression, scope);
        await inspect('/test', selection);
        settings.enable_bounding_box = false;
        await inspect('/test', selection);
        assert.equal(num_requests, 0);
        assert.equal(is_loading, false);
        settings.enable_bounding_box = true;
        scope.inspection_sources.current.layerSettings['/topological_map'] = settings;
        await inspect('/topological_map', selection);
        assert.equal(num_requests, 0);
        const pending = inspect('/test', selection);
        assert.equal(num_requests, 1);
        assert.equal(is_loading, true);
        settings.enable_bounding_box = false;
        complete({ source_id: '/test', selection });
        await pending;
        assert.equal(snapshots.length, 0);
        assert.equal(is_loading, false);
        settings.enable_bounding_box = true;
        const enabled = inspect('/test', selection);
        complete({ source_id: '/test', selection });
        await enabled;
        assert.equal(snapshots.length, 1);
        assert.equal(is_loading, false);
        const failed = inspect('/test', selection);
        settings.enable_bounding_box = false;
        fail(new Error('遅延した取得失敗'));
        await failed;
        assert.equal(snapshots.length, 1);
        assert.equal(is_loading, false);
    }
});
