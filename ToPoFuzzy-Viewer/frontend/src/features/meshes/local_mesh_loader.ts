import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js';

export const mesh_extensions = ['obj', 'ply', 'stl', 'glb', 'gltf', 'fbx'];
// ブラウザの文字列・展開後バッファ肥大化に対する読込前の安全上限
export const max_mesh_file_bytes = 256 * 1024 * 1024;
const max_bundle_bytes = 512 * 1024 * 1024;

export interface local_mesh_asset {
    object: THREE.Object3D;
    num_vertices: number;
    num_triangles: number;
    warnings: string[];
    dispose: () => void;
}

export function mesh_extension(name: string) {
    return name.split('.').pop()?.toLowerCase() ?? '';
}

function normalize_path(path: string) {
    const parts: string[] = [];
    for (const part of path.replace(/\\/g, '/').split('/')) {
        if (part === '..') parts.pop();
        else if (part && part !== '.') parts.push(part);
    }
    return parts.join('/');
}

// 選択したファイルだけに限定した付属ファイル解決。外部URLへの通信なし
export function local_resource_bundle(files: File[], primary: File) {
    const paths = new Map(files.map(file => [normalize_path(file.webkitRelativePath || file.name), file]));
    const primary_path = normalize_path(primary.webkitRelativePath || primary.name);
    const base_path = primary_path.includes('/') ? primary_path.slice(0, primary_path.lastIndexOf('/') + 1) : '';
    const urls = new Map<File, string>();
    const extra_urls = new Set<string>();
    const find_file = (request: string) => {
        const raw_path = decodeURIComponent(request.split(/[?#]/)[0]);
        const path = normalize_path(raw_path);
        const exact = paths.get(normalize_path(base_path + raw_path)) ?? paths.get(path);
        if (exact) return exact;
        const matches = files.filter(file => file.name === path.split('/').pop());
        if (matches.length === 1) return matches[0];
        throw new Error(`付属ファイルが${matches.length ? '同名で曖昧' : '不足'}: ${request}。フォルダごと選択してください。`);
    };
    return {
        base_path,
        find_file,
        resolve: (request: string) => {
            if (request.startsWith('data:')) return request;
            // FBXLoaderが埋込画像から生成したBlobの追跡
            if (request.startsWith('blob:')) { extra_urls.add(request); return request; }
            const file = find_file(request);
            if (!urls.has(file)) urls.set(file, URL.createObjectURL(file));
            return urls.get(file)!;
        },
        dispose: () => {
            for (const url of [...urls.values(), ...extra_urls]) URL.revokeObjectURL(url);
            urls.clear();
            extra_urls.clear();
        },
    };
}

export function dispose_mesh_object(object: THREE.Object3D) {
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    const textures = new Set<THREE.Texture>();
    object.traverse(child => {
        const mesh = child as THREE.Mesh;
        if (mesh.geometry) geometries.add(mesh.geometry);
        if (mesh.material) (Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach(material => materials.add(material));
    });
    for (const material of materials) {
        for (const value of Object.values(material)) if (value instanceof THREE.Texture) textures.add(value);
        material.dispose();
    }
    for (const texture of textures) {
        texture.dispose();
        if (typeof ImageBitmap !== 'undefined' && texture.image instanceof ImageBitmap) texture.image.close();
    }
    for (const geometry of geometries) geometry.dispose();
}

export async function load_local_mesh(primary: File, files: File[]): Promise<local_mesh_asset> {
    const extension = mesh_extension(primary.name);
    if (!mesh_extensions.includes(extension)) throw new Error(`未対応のメッシュ形式: ${extension}`);
    if (primary.size > max_mesh_file_bytes || files.reduce((sum, file) => sum + file.size, 0) > max_bundle_bytes) {
        throw new Error('サイズ上限: モデル256 MiB、選択ファイル合計512 MiB。原本を残したまま別ファイルを軽量化してください。');
    }
    const bundle = local_resource_bundle(files, primary);
    const manager = new THREE.LoadingManager();
    manager.setURLModifier(bundle.resolve);
    const warnings: string[] = [];
    const failed_resources: string[] = [];
    manager.onError = url => failed_resources.push(url);
    const resources_done = new Promise<void>(resolve => { manager.onLoad = resolve; });
    manager.itemStart('local-mesh-root');
    let object: THREE.Object3D | undefined;
    try {
        const buffer = await primary.arrayBuffer();
        if (extension === 'obj') {
            const text = new TextDecoder().decode(buffer);
            const loader = new OBJLoader(manager);
            const libraries = [...text.matchAll(/^mtllib\s+(.+)$/gm)].map(match => match[1].trim());
            if (libraries.length) {
                const creators: ReturnType<MTLLoader['parse']>[] = [];
                for (const name of libraries) {
                    try {
                        const file = bundle.find_file(name);
                        const path = normalize_path(file.webkitRelativePath || file.name);
                        const material_base = path.includes('/') ? path.slice(0, path.lastIndexOf('/') + 1) : '';
                        creators.push(new MTLLoader(manager).parse(await file.text(), material_base));
                    }
                    catch (error) { warnings.push(error instanceof Error ? error.message : String(error)); }
                }
                if (creators.length) {
                    const materials = new MTLLoader(manager).parse('', '');
                    const create_default = materials.create.bind(materials);
                    materials.create = name => {
                        const creator = [...creators].reverse().find(value => Object.prototype.hasOwnProperty.call(value.materialsInfo, name));
                        return creator ? creator.create(name) : create_default(name);
                    };
                    loader.setMaterials(materials);
                }
            }
            object = loader.parse(text);
        } else if (extension === 'stl' || extension === 'ply') {
            const geometry = extension === 'stl' ? new STLLoader(manager).parse(buffer) : new PLYLoader(manager).parse(buffer);
            if (extension === 'ply' && (!geometry.index || geometry.index.count === 0)) {
                geometry.dispose();
                throw new Error('面のないPLYです。点群用の読み込みを使用してください。');
            }
            if (!geometry.attributes.normal) geometry.computeVertexNormals();
            object = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({
                color: '#cbd5e1', vertexColors: Boolean(geometry.attributes.color), side: THREE.DoubleSide,
            }));
            if (extension === 'ply') warnings.push('PLYは面・頂点色を表示。独自の外部テクスチャ拡張は未対応。');
        } else if (extension === 'fbx') {
            object = new FBXLoader(manager).parse(buffer, bundle.base_path);
        } else {
            object = (await new GLTFLoader(manager).parseAsync(buffer, bundle.base_path)).scene;
        }
    } catch (error) {
        manager.itemEnd('local-mesh-root');
        await resources_done;
        if (object) dispose_mesh_object(object);
        bundle.dispose();
        throw error;
    }
    manager.itemEnd('local-mesh-root');
    await resources_done;
    let num_vertices = 0;
    let num_triangles = 0;
    let is_valid_geometry = true;
    object.traverse(child => {
        if (!(child instanceof THREE.Mesh)) return;
        child.geometry.computeBoundingBox();
        const box = child.geometry.boundingBox;
        if (!box || ![...box.min.toArray(), ...box.max.toArray()].every(Number.isFinite)) is_valid_geometry = false;
        num_vertices += child.geometry.attributes.position?.count ?? 0;
        num_triangles += (child.geometry.index?.count ?? child.geometry.attributes.position?.count ?? 0) / 3;
        // 選択・編集対象外の重いメッシュに対するポインタ照合の省略
        child.raycast = () => {};
    });
    const loaded_object = object;
    const dispose = () => { dispose_mesh_object(loaded_object); bundle.dispose(); };
    if (!is_valid_geometry || !num_triangles || !Number.isFinite(num_triangles) || failed_resources.length) {
        dispose();
        throw new Error(failed_resources.length ? '付属画像・バッファを読み込めませんでした。選択ファイルと形式を確認してください。' : '有効な有限座標と面を持つメッシュがありません。');
    }
    return { object, num_vertices, num_triangles, warnings, dispose };
}
