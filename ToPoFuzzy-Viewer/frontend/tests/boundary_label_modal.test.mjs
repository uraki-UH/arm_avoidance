import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { renderToStaticMarkup } from 'react-dom/server';
import { build } from 'esbuild';

const temporary_directory = await mkdtemp(resolve('tests/.boundary-modal-'));
const previous_document = globalThis.document;
try {
    const output = resolve(temporary_directory, 'test.mjs');
    // DOMへの挿入だけを置換した、実コンポーネントの構造・イベント検査。
    await build({ stdin: { contents: "export { GngLabelModal } from './src/features/visualization/GngLabelModal';",
        resolveDir: process.cwd() }, outfile: output, bundle: true, platform: 'node', format: 'esm', jsx: 'automatic',
        external: ['react', 'react/jsx-runtime', 'lucide-react'], plugins: [{ name: 'portal', setup(builder) {
            builder.onResolve({ filter: /^react-dom$/ }, () => ({ path: 'portal', namespace: 'test' }));
            builder.onLoad({ filter: /.*/, namespace: 'test' }, () => ({ contents: 'export const createPortal = (node) => node;' }));
        } }] });
    const { GngLabelModal } = await import(pathToFileURL(output).href);
    globalThis.document = { body: {} };
    let updates;
    const props = { open: true, visibleLabels: { 0: false, 1: false, 2: false, 3: false, 4: false, 5: false },
        onClose() {}, onUpdate(value) { updates = value; } };
    const tree = GngLabelModal(props);
    const elements = [];
    const visit = (value) => {
        if (Array.isArray(value)) value.forEach(visit);
        else if (value?.props) { elements.push(value); visit(value.props.children); }
    };
    visit(tree);
    const html = renderToStaticMarkup(tree);
    const group = elements.find((item) => item.props['aria-label'] === '重複ラベル');
    const heading = group.props.children[0];
    assert.ok(heading.props.className.includes('flex'));
    assert.deepEqual(heading.props.children.map((item) => item.props.children), ['重複ラベル', '複数選択可・上ほど色を優先']);
    assert.doesNotMatch(html, /Semantic labels/);
    assert.doesNotMatch(html, /優先度を上げる|優先度を下げる/);
    assert.equal((html.match(/を先頭へ移動"/g) ?? []).length, 6);
    assert.match(html, /data-priority-action="true"[^>]*class="[^"]*border-slate-500 bg-slate-700/);
    assert.doesNotMatch(html, /data-priority-action="true"[^>]*disabled:opacity/);
    assert.equal((html.match(/を長押しして移動"/g) ?? []).length, 6);
    assert.equal((html.match(/border-slate-600 bg-slate-800\/40 hover:border-slate-400/g) ?? []).length, 6);
    assert.equal((html.match(/data-drag-indicator/g) ?? []).length, 6);
    assert.equal((html.match(/lucide-hand-grab/g) ?? []).length, 6);
    assert.doesNotMatch(html, /lucide-grip-vertical/);
    assert.match(html, /data-drag-indicator="true" aria-hidden="true" class="pointer-events-none/);
    assert.equal(elements.filter((item) => item.props['aria-pressed'] !== undefined).length, 2);
    assert.equal(elements.filter((item) => item.type === 'details').length, 1);
    const summary = elements.find((item) => item.type === 'summary');
    assert.ok(summary.props.className.includes('min-h-[44px]'));
    assert.ok(summary.props.className.includes('w-full'));
    assert.ok(summary.props.className.includes('focus-visible:outline'));
    assert.doesNotMatch(html, /<details[^>]*\bopen[=>\s]/);
    assert.equal(elements.filter((item) => item.props.type === 'checkbox').length, 4);
    assert.equal(elements.filter((item) => item.props.type === 'color').length, 4);
    const button = elements.find((item) => item.props['aria-label'] === '境界候補の色分け');
    button.props.onClick();
    assert.equal(updates.node_label_visibility.boundary, false);
    assert.equal(updates.node_label_visibility.boundary_fov, true);
    assert.match(renderToStaticMarkup(GngLabelModal({ ...props, label_settings: updates })), /<fieldset disabled=""/);
    elements.find((item) => item.props['aria-label'] === '視野端の色').props.onChange({ target: { value: '#123456' } });
    assert.equal(updates.node_label_colors.boundary_fov, '#123456');
    elements.find((item) => item.props.type === 'checkbox').props.onChange({ target: { checked: false } });
    assert.equal(updates.node_label_visibility.boundary_fov, false);
    assert.equal(updates.node_label_visibility.boundary, true);
    assert.equal(GngLabelModal({ ...props, open: false }), null);
    console.log('boundary_label_modal=passed');
} finally {
    if (previous_document === undefined) delete globalThis.document;
    else globalThis.document = previous_document;
    await rm(temporary_directory, { recursive: true, force: true });
}
