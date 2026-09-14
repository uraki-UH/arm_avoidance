import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import React from 'react';
import { act, createRoot, extend } from '@react-three/fiber';
import * as THREE from 'three';

test('候補ロボットの全件復帰GUI・単体選択・先頭N件・TFの維持', async () => {
    const temporary_directory = await mkdtemp(resolve('tests/.robot-limit-'));
    const previous_window = globalThis.window;
    globalThis.IS_REACT_ACT_ENVIRONMENT = true;
    extend(THREE);
    let root;
    try {
        const output = resolve(temporary_directory, 'renderer.mjs');
        // URDF解析とGPUのみ代替。実コンポーネントの関節更新・TF・描画対象を検証
        await build({ stdin: { contents: `
            export { RobotRenderer } from './src/features/visualization/RobotRenderer';
            export { RobotCandidateControls } from './src/features/visualization/RobotCandidateControls';
        `, resolveDir: process.cwd() }, outfile: output,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', jsx: 'automatic',
            logLevel: 'silent', plugins: [{ name: 'urdf', setup(builder) {
                builder.onResolve({ filter: /^urdf-loader$/ }, () => ({ path: 'urdf', namespace: 'test' }));
                builder.onLoad({ filter: /.*/, namespace: 'test' }, () => ({ contents: `
                    import { Group } from 'three';
                    export default class {
                        loadMeshCb() {}
                        parse() {
                            const robot = new Group();
                            robot.name = 'preview-body';
                            robot.joints = { test_joint: { setJointValue(value) { robot.position.x = value; } } };
                            return robot;
                        }
                    }`, resolveDir: process.cwd() }));
            } }] });
        const { RobotRenderer, RobotCandidateControls } = await import(pathToFileURL(output).href);
        let settings = { max_visible_candidates: 2 };
        const controls = (num_candidates = 3) => {
            const tree = RobotCandidateControls({ settings, num_candidates,
                on_update: updates => { settings = { ...settings, ...updates }; } });
            return { buttons: tree.props.children[0].props.children, slider: tree.props.children[1] };
        };
        const click = label => controls().buttons.find(button => button.props.children === label).props.onClick();
        click('1体選択');
        assert.equal(settings.selected_candidate_idx, 0);
        controls().slider.props.onChange(3);
        assert.equal(settings.selected_candidate_idx, 2);
        assert.equal(controls().slider.props.value, 3);
        assert.equal(controls(1).slider.props.value, 1);
        click('先頭N件');
        assert.equal(settings.selected_candidate_idx, null);
        assert.equal(settings.max_visible_candidates, 2);
        click('1体選択');
        click('全件に戻す');
        assert.deepEqual(settings, { max_visible_candidates: 0, selected_candidate_idx: null });
        assert.equal(controls().buttons[0].props['aria-pressed'], true);
        click('先頭N件');
        assert.equal(settings.max_visible_candidates, 1);
        assert.equal(controls(0).slider.props.disabled, true);
        assert.equal(controls(0).slider.props.formatValue(1), '候補なし');
        assert.equal(controls(0).buttons[2].props.disabled, true);
        assert.notEqual(controls(0).buttons[0].props.disabled, true);
        click('全件に戻す');
        const scene = new THREE.Scene();
        const canvas = { width: 100, height: 100, addEventListener() {}, removeEventListener() {} };
        root = createRoot(canvas);
        root.configure({ gl: { render() {}, setSize() {}, setPixelRatio() {}, domElement: canvas },
            scene, frameloop: 'never', size: { width: 100, height: 100, top: 0, left: 0 } });
        globalThis.window = { location: { hostname: 'localhost' },
            requestAnimationFrame: callback => setTimeout(callback, 0), cancelAnimationFrame: clearTimeout };
        const instances = Object.freeze([31, 7, 12].map(value => Object.freeze({
            jointNames: ['test_joint'], jointValues: [value], positions: [], orientations: [],
        })));
        const data = { urdf: 'fixture', frameId: 'base_link', ...instances[0], instances };
        const tf = { pos: [0.15, 0, -0.2], quat: [0, 0, 1, 0] };
        for (const [current_data, limit, expected, selected_candidate_idx] of [
            [data, undefined, [31, 7, 12]], [data, 2, [31, 7]], [data, 1, [31]],
            [data, 0, [31, 7, 12]], [data, 10, [31, 7, 12]], [data, 2.9, [31, 7]],
            [data, NaN, [31, 7, 12]], [data, -1, [31, 7, 12]],
            [{ ...data, instances: [] }, 2, []],
            [{ ...data, instances: [instances[2], instances[0], instances[1]] }, 2, [12, 31]],
            [{ ...data, instances: [instances[1]] }, 2, [7]],
            [data, 1, [7], 1], [data, 0, [12], 2], [data, 0, [31], 0],
            [data, 0, [12], 99], [data, 0, [31], -1], [data, 0, [7], 1.9],
            [data, 1, [31], NaN],
            [{ ...data, instances: [] }, 0, [], 2],
            [{ ...data, instances: [instances[1]] }, 0, [7], 2],
            [data, 0, [12], 2],
            [data, settings.max_visible_candidates, [31, 7, 12], settings.selected_candidate_idx],
            [data, 2, [31, 7]], [{ ...data, instances: undefined }, 1, [31]],
        ]) {
            await act(async () => { root.render(React.createElement(RobotRenderer, {
                tag: 'ToPoDualArm/candidate_goal_preview', data: current_data, tf,
                max_visible_candidates: limit, selected_candidate_idx,
            })); });
            const robots = [];
            scene.traverse(object => { if (object.name === 'preview-body') robots.push(object); });
            assert.deepEqual(robots.map(robot => robot.position.x), expected);
            for (const robot of robots) {
                const position = robot.getWorldPosition(new THREE.Vector3());
                assert.ok(Math.abs(position.x - (0.15 - robot.position.x)) < 1e-6);
                assert.equal(position.z, -0.2);
            }
        }
        assert.deepEqual(data.instances.map(instance => instance.jointValues[0]), [31, 7, 12]);
    } finally {
        if (root) await act(async () => { root.unmount(); });
        await new Promise(resolve => setTimeout(resolve, 600));
        if (previous_window === undefined) delete globalThis.window;
        else globalThis.window = previous_window;
        delete globalThis.IS_REACT_ACT_ENVIRONMENT;
        await rm(temporary_directory, { recursive: true, force: true });
    }
});
