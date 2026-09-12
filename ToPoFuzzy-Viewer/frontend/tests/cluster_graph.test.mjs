import assert from 'node:assert/strict';
import { mkdtemp, rm } from 'node:fs/promises';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { test } from 'node:test';
import { build } from 'esbuild';
import * as THREE from 'three';

test('曲面所属色のノードID参照・edgeとの一致・未所属・通常色復帰', async () => {
    const directory = await mkdtemp(resolve('node_modules/.cluster-graph-test-'));
    const geometry = new THREE.SphereGeometry(1, 4, 3);
    const material = new THREE.MeshStandardMaterial();
    const mesh = new THREE.InstancedMesh(geometry, material, 4);
    try {
        const outfile = resolve(directory, 'helpers.mjs');
        await build({
            stdin: { contents: `
                export * from './src/features/visualization/utils/gngGraphics';
            `, resolveDir: resolve('.') },
            outfile, bundle: true, packages: 'external', platform: 'node', format: 'esm',
        });
        const { build_cluster_node_colors, updateNodeInstances, updateEdgeInstances } = await import(pathToFileURL(outfile));
        const graph = {
            nodes: [101, 205, 307, 409].map((id, i) => ({ id, x: i, y: 0, z: 0, label: 2, semanticLabel: 1 })),
            edges: [0, 1, 2, 3],
            clusters: [{ id: 21, nodeIds: [307] }, { id: 0, nodeIds: [101, 205] }],
        };
        const original = structuredClone(graph);
        const colors = build_cluster_node_colors(graph);
        assert.equal(colors.get(101), colors.get(205));
        assert.notEqual(colors.get(101), colors.get(307));
        assert.equal(colors.get(409), '#737373');
        assert.deepEqual(colors, build_cluster_node_colors({ ...graph, clusters: [...graph.clusters].reverse() }));
        assert.deepEqual(graph, original);
        const color = new THREE.Color();
        updateNodeInstances(mesh, graph.nodes, 0.004, {
            node_colors: new Map(graph.nodes.map((node) => [node, colors.get(node.id)])),
        });
        mesh.getColorAt(0, color);
        assert.equal(color.getHexString(), colors.get(101).slice(1));
        const matrix_attribute = mesh.instanceMatrix;
        for (let i = 0; i < 40; i++) updateEdgeInstances(mesh, graph.edges, graph.nodes, 0.002, colors);
        assert.equal(mesh.instanceMatrix, matrix_attribute);
        assert.equal(mesh.count, 2);
        mesh.getColorAt(0, color);
        assert.equal(color.getHexString(), colors.get(101).slice(1));
        updateEdgeInstances(mesh, graph.edges, graph.nodes, 0.002);
        mesh.getColorAt(0, color);
        assert.equal(color.getHexString(), 'ffffff');
        assert.equal(build_cluster_node_colors({ ...graph, clusters: [] }).get(101), '#737373');
    } finally {
        mesh.dispose(); geometry.dispose(); material.dispose();
        await rm(directory, { recursive: true, force: true });
    }
});
