import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { test } from 'node:test';
import ts from 'typescript';

test('HTMLは全点と把持部位をsemantic_pointsへ配信し、到達性ラベルを生成しない', async () => {
    const html = await readFile('../../ToPo-FUZZY_Manipulation_v1.html', 'utf8');
    const script = [...html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/g)]
        .map((match) => match[1]).find((text) => text.includes('function buildPointCloudMsgFromPoints('));
    assert.ok(script);
    const source = ts.createSourceFile('html.js', script, ts.ScriptTarget.Latest, true, ts.ScriptKind.JS);
    const declarations = new Map();
    function visit(node) {
        if ((ts.isFunctionDeclaration(node) || ts.isVariableDeclaration(node)) && node.name) {
            declarations.set(node.name.getText(source), node);
        }
        ts.forEachChild(node, visit);
    }
    visit(source);
    const names = ['getCurrentTopoPoints', 'getCurrentHandlePoints', 'getCurrentSemanticPoints',
        'semanticLabelValue', 'normalizePointObjects', 'buildPointCloudMsgFromPoints'];
    const functions = names.map((name) => declarations.get(name).getText(source)).join('\n');
    const constants = ['SEMANTIC_LABELS', 'DEFAULT_TOPO_POINTS_TOPIC', 'DEFAULT_TOPO_POINTS_FRAME']
        .map((name) => 'const ' + declarations.get(name).getText(source) + ';').join('\n');
    const points = ['body', 'handle', 'rim', 'cap', 'table', 'floor', 'wall', 'outlier']
        .map((label, idx) => ({ x: idx, y: 2, z: 3, label }));
    points.push({ x: 8, y: 2, z: 3, semantic_label: 1 });
    const context = { __topoState: { points } };
    const publishers = declarations.get('rosPublishersConfig').initializer.getText(source);
    const api = new Function('window', 'nowRosTime', `${constants}\n${functions}
        return {getCurrentHandlePoints, getCurrentSemanticPoints, publishers: ${publishers}};`
    )(context, () => ({ sec: 1, nanosec: 0 }));
    assert.equal(api.getCurrentSemanticPoints(), points);
    assert.equal(api.getCurrentHandlePoints().length, 1);
    assert.ok(!api.publishers.some((publisher) => publisher.defaultTopic === 'handle_points'));
    assert.doesNotMatch(html, /id="handle(?:PublishEnabled|TopicName|AutoHz|RosStatus)"/);
    const publisher = api.publishers.find((item) => item.defaultTopic === 'semantic_points');
    assert.ok(publisher);
    assert.match(html, /id="semanticPublishEnabled"[^>]*checked/);
    const message = publisher.messageBuilder(publisher.getPointsFn());
    const field = message.fields.find((item) => item.name === 'semantic_label');
    assert.equal(field.datatype, 2);
    assert.equal(message.width, points.length);
    const bytes = Uint8Array.from(message.data);
    const view = new DataView(bytes.buffer);
    const labels = points.map((point, idx) => {
        assert.equal(view.getFloat32(idx * message.point_step, true), point.x);
        return view.getUint8(idx * message.point_step + field.offset);
    });
    assert.deepEqual(labels, [0, 1, 0, 0, 0, 0, 0, 0, 1]);
});
