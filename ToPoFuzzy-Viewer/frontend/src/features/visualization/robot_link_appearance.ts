import * as THREE from 'three';
import type { RobotSettings } from '../../types';

// 双腕Simulatorと共通の外観値。未知の材質名は元の材質を維持
const robot_finishes: Record<string, [string, number, number]> = {
    black: ['#393f3b', .45, .36], green: ['#00ff00', 0, .31],
    silver: ['#a2abad', .92, .3], gray: ['#69716e', .65, .36],
    glass: ['#26383a', .65, .15], blue: ['#235bdd', .15, .4], orange: ['#ed9b32', .1, .4],
};
function simulator_material(mesh: THREE.Mesh, original: THREE.Material): THREE.Material {
    let is_dual_arm = false, filename = '';
    for (let node: THREE.Object3D | null = mesh; node; node = node.parent) {
        const urdf = node as THREE.Object3D & { robotName?: string; urdfNode?: Element };
        if ([node.name, urdf.robotName].some(name => name === 'topo_dual_arm_max' || name === 'topo_dual_arm_max_long')) is_dual_arm = true;
        filename ||= urdf.urdfNode?.querySelector('mesh')?.getAttribute('filename') ?? '';
    }
    const name = filename.includes('gripper_base_green') ? 'green' : original.name;
    const finish = is_dual_arm ? robot_finishes[name] : undefined;
    if (!finish) return original.clone();
    const material = new THREE.MeshPhysicalMaterial({ name: original.name, color: finish[0], metalness: finish[1], roughness: finish[2],
        opacity: name === 'glass' ? .5 : original.opacity, transparent: name === 'glass' || original.transparent,
        side: original.side });
    material.userData.has_simulator_finish = true;
    if (name === 'green') { material.specularIntensity = .25; material.clearcoat = .22; material.clearcoatRoughness = .22; }
    return material;
}

// 共有URDFマテリアルから分離したメッシュ固有の色とマテリアル
const mesh_materials = new WeakMap<THREE.Mesh, {
    materials: THREE.Material[];
    opacity: number[];
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
            const materials = originals.map((material) => simulator_material(mesh, material));
            entry = { materials, opacity: materials.map(material => material.userData.has_simulator_finish ? material.opacity : 1), colors: materials.map((material) => (material as colored_material).color?.clone()) };
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
            target.opacity = link_opacity * entry.opacity[idx];
            target.transparent = target.opacity < 1;
            target.depthTest = true;
            target.depthWrite = target.opacity >= 1;
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
