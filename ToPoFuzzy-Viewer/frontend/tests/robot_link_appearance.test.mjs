import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import * as THREE from 'three';
import { URDFLink, URDFJoint } from 'urdf-loader/src/URDFClasses.js';

test('全候補の同一リンクへの色反映と、子リンク・共有マテリアルの分離', async () => {
    const temporary_directory = await mkdtemp(resolve('node_modules/.robot-color-test-'));
    try {
        const output_file = resolve(temporary_directory, 'appearance.mjs');
        await build({ entryPoints: ['src/features/visualization/robot_link_appearance.ts'], outfile: output_file,
            bundle: true, packages: 'external', platform: 'node', format: 'esm', logLevel: 'silent' });
        const { apply_robot_appearance } = await import(pathToFileURL(output_file).href);
        const shared = new THREE.MeshPhongMaterial({ color: '#abcdef' });
        const geometry = new THREE.BoxGeometry();
        const instances = Array.from({ length: 3 }, () => {
            const arm = new URDFLink();
            arm.name = 'arm';
            const finger = new URDFLink();
            finger.name = 'finger';
            const joint = new URDFJoint();
            arm.add(joint);
            joint.add(finger);
            const arm_mesh = new THREE.Mesh(geometry, shared);
            const finger_mesh = new THREE.Mesh(geometry, [shared, shared]);
            arm.add(arm_mesh);
            finger.add(finger_mesh);
            return { arm, finger, arm_mesh, finger_mesh };
        });
        for (const instance of instances) {
            apply_robot_appearance(instance.arm, { arm: '#ff0000' }, true, '#0000ff', 0.5, 0.2);
            assert.equal(instance.arm_mesh.material.color.getHexString(), 'ff0000');
            assert.equal(instance.finger_mesh.material[0].color.getHexString(), 'abcdef');
            assert.equal(instance.arm_mesh.material.depthWrite, false);
        }
        assert.equal(shared.color.getHexString(), 'abcdef');
        assert.notEqual(instances[0].arm_mesh.material, instances[1].arm_mesh.material);

        // 全候補のリンク別不透明度・発光強度とゼロ値の反映
        for (const instance of instances) {
            apply_robot_appearance(instance.arm, {}, true, '#0000ff', 0.5, 0.2, {
                arm: { opacity: 0, emissive_intensity: 0 },
                finger: { opacity: 1, emissive_intensity: 1.3 },
            });
            assert.equal(instance.arm_mesh.material.opacity, 0);
            assert.equal(instance.arm_mesh.material.emissive.getHex(), 0);
            assert.equal(instance.arm_mesh.material.depthWrite, false);
            for (const material of instance.finger_mesh.material) {
                assert.equal(material.opacity, 1);
                assert.equal(material.transparent, false);
                assert.equal(material.depthWrite, true);
                assert.ok(material.emissive.equals(material.color.clone().multiplyScalar(1.3)));
            }
        }

        // 後着メッシュにも、色変更後の設定を適用
        const late_mesh = new THREE.Mesh(geometry, shared);
        instances[0].finger.add(late_mesh);
        apply_robot_appearance(late_mesh, { finger: '#00ff00' }, true, '#0000ff', 1, 0.2,
            { finger: { opacity: 0.25, emissive_intensity: 0.7 } });
        assert.equal(late_mesh.material.color.getHexString(), '00ff00');
        assert.equal(late_mesh.material.opacity, 0.25);
        assert.ok(late_mesh.material.emissive.equals(late_mesh.material.color.clone().multiplyScalar(0.7)));

        for (const instance of instances) {
            const previous_material = instance.arm_mesh.material;
            apply_robot_appearance(instance.arm, { finger: '#00ff00' }, false, '#0000ff', 1, 0.2);
            assert.equal(instance.arm_mesh.material.color.getHexString(), '0000ff');
            assert.equal(instance.finger_mesh.material[1].color.getHexString(), '00ff00');
            assert.equal(instance.arm_mesh.material, previous_material);
            apply_robot_appearance(instance.arm, {}, true, '#0000ff', 1, 0.2);
            assert.equal(instance.arm_mesh.material.color.getHexString(), 'abcdef');
            assert.equal(instance.finger_mesh.material[0].color.getHexString(), 'abcdef');
            assert.equal(instance.arm_mesh.material.depthWrite, true);
            assert.equal(instance.arm_mesh.material.transparent, false);
            assert.equal(instance.finger_mesh.material[0].opacity, 1);
            assert.ok(instance.finger_mesh.material[0].emissive.equals(instance.finger_mesh.material[0].color.clone().multiplyScalar(0.2)));
            instance.arm.traverse((object) => {
                if (object.isMesh) {
                    const materials = Array.isArray(object.material) ? object.material : [object.material];
                    materials.forEach((material) => material.dispose());
                }
            });
        }
        geometry.dispose();
        shared.dispose();
    } finally {
        await rm(temporary_directory, { recursive: true, force: true });
    }
});
