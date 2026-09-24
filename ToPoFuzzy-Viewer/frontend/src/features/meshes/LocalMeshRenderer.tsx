import { useEffect, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { local_mesh_item } from './use_local_meshes';

export function LocalMeshRenderer({ item, focus_req }: { item: local_mesh_item; focus_req?: number }) {
    const root = useRef<THREE.Group>(null);
    const { camera, controls, invalidate } = useThree();
    useEffect(() => { invalidate(); }, [item, invalidate]);
    useEffect(() => {
        if (!item.enable_fill_light) return;
        // 当該メッシュだけの補助発光。既存ロボットの照明・原本の変更なし
        const materials = new Set<THREE.Material>();
        const restore: (() => void)[] = [];
        item.asset.object.traverse(child => {
            if (child instanceof THREE.Mesh) (Array.isArray(child.material) ? child.material : [child.material]).forEach(material => materials.add(material));
        });
        for (const material of materials) {
            if (!(material instanceof THREE.MeshLambertMaterial || material instanceof THREE.MeshPhongMaterial || material instanceof THREE.MeshStandardMaterial)) continue;
            if (material.emissive.getHex() !== 0 || material.emissiveMap) continue;
            const original = material.emissive.clone();
            material.emissive.copy(material.color).multiplyScalar(0.3);
            material.emissiveMap = material.map;
            material.needsUpdate = true;
            restore.push(() => { material.emissive.copy(original); material.emissiveMap = null; material.needsUpdate = true; });
        }
        invalidate();
        return () => { restore.forEach(reset => reset()); invalidate(); };
    }, [item.asset, item.enable_fill_light, invalidate]);
    useEffect(() => {
        if (!focus_req || !root.current || !item.is_visible || !(camera instanceof THREE.PerspectiveCamera)) return;
        root.current.updateWorldMatrix(true, true);
        const box = new THREE.Box3().setFromObject(root.current);
        if (box.isEmpty()) return;
        const sphere = box.getBoundingSphere(new THREE.Sphere());
        if (!Number.isFinite(sphere.radius) || !sphere.center.toArray().every(Number.isFinite)) return;
        const half_fov = Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * Math.min(1, camera.aspect));
        const dist = Math.max(sphere.radius, 0.01) / Math.sin(half_fov) * 1.2;
        const direction = camera.getWorldDirection(new THREE.Vector3()).negate();
        camera.position.copy(sphere.center).addScaledVector(direction, dist);
        camera.near = Math.min(0.1, dist / 100);
        camera.far = Math.max(1000, dist * 10);
        camera.updateProjectionMatrix();
        camera.lookAt(sphere.center);
        const orbit = controls as unknown as { target?: THREE.Vector3; update: () => void } | null;
        if (orbit?.target) { orbit.target.copy(sphere.center); orbit.update(); }
        invalidate();
    }, [focus_req, camera, controls, invalidate, item.is_visible]);
    return <group ref={root} visible={item.is_visible} position={item.transform.position}
        rotation={item.transform.rotation} scale={item.transform.scale} dispose={null}>
        <group scale={item.unit_scale}><primitive object={item.asset.object} dispose={null} /></group>
    </group>;
}
