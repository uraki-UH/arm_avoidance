import * as THREE from 'three';
import type { RobotSettings } from '../../types';

// 共有URDFマテリアルから分離したメッシュ固有の色とマテリアル
const mesh_materials = new WeakMap<THREE.Mesh, {
    materials: THREE.Material[];
    colors: Array<THREE.Color | undefined>;
}>();

type colored_material = THREE.Material & { color?: THREE.Color; emissive?: THREE.Color };

export function apply_robot_appearance(
    object: THREE.Object3D,
    link_colors: Record<string, string>,
    use_urdf_colors: boolean,
    color: string,
    opacity: number,
    emissive_intensity: number,
    link_appearance: RobotSettings['link_appearance'] = {},
) {
    object.traverse((child) => {
        if (!(child as THREE.Mesh).isMesh) return;
        const mesh = child as THREE.Mesh;
        let entry = mesh_materials.get(mesh);
        if (!entry) {
            const is_array = Array.isArray(mesh.material);
            const originals = is_array ? mesh.material as THREE.Material[] : [mesh.material as THREE.Material];
            const materials = originals.map((material) => material.clone());
            entry = { materials, colors: materials.map((material) => (material as colored_material).color?.clone()) };
            mesh_materials.set(mesh, entry);
            mesh.material = is_array ? materials : materials[0];
        }

        // 最も近い所属リンクだけを対象とした色選択（子関節への色の伝播防止）
        let link: THREE.Object3D | null = mesh;
        while (link && !(link as THREE.Object3D & { isURDFLink?: boolean }).isURDFLink) {
            link = link.parent;
        }
        const override = link ? link_colors[link.name] : undefined;
        const appearance = link ? link_appearance[link.name] : undefined;
        const link_opacity = appearance?.opacity ?? opacity;
        const link_emissive_intensity = appearance?.emissive_intensity ?? emissive_intensity;
        entry.materials.forEach((material, idx) => {
            const target = material as colored_material;
            if (target.color) {
                if (override) target.color.set(override);
                else if (!use_urdf_colors) target.color.set(color);
                else if (entry.colors[idx]) target.color.copy(entry.colors[idx]!);
            }
            target.opacity = link_opacity;
            target.transparent = link_opacity < 1;
            target.depthTest = true;
            target.depthWrite = link_opacity >= 1;
            if (target.emissive && target.color) {
                target.emissive.copy(target.color).multiplyScalar(Math.max(0, link_emissive_intensity));
            }
            target.needsUpdate = true;
        });
        mesh.castShadow = false;
        mesh.receiveShadow = false;
        mesh.renderOrder = 10;
    });
}
