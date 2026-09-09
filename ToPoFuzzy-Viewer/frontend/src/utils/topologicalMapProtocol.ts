import { GraphData } from '../types';

const MAGIC = 0x31474d54;
const VERSION = 1;
const HEADER_SIZE = 36;
const NODE_RECORD_SIZE = 84;
const CLUSTER_RECORD_SIZE = 80;
const MAX_PACKET_BYTES = 64 * 1024 * 1024;
const decoder = new TextDecoder();

export interface TopologicalMapPacket {
    tag: string;
    graph: GraphData;
}

function requireBytes(buffer: ArrayBuffer, offset: number, size: number): void {
    if (size < 0 || offset < 0 || offset + size > buffer.byteLength) {
        throw new Error('Topological map packet is truncated');
    }
}

export function isTopologicalMapPacket(buffer: ArrayBuffer): boolean {
    return buffer.byteLength >= 4 && new DataView(buffer).getUint32(0, true) === MAGIC;
}

export function deserializeTopologicalMap(buffer: ArrayBuffer): TopologicalMapPacket {
    if (buffer.byteLength < HEADER_SIZE || buffer.byteLength > MAX_PACKET_BYTES) {
        throw new Error('Invalid topological map packet size');
    }
    const view = new DataView(buffer);
    if (view.getUint32(0, true) !== MAGIC || view.getUint16(4, true) !== VERSION) {
        throw new Error('Unsupported topological map packet');
    }
    const tagSize = view.getUint32(8, true);
    const frameIdSize = view.getUint32(12, true);
    const timestamp = view.getUint32(16, true);
    const nodeNum = view.getUint32(20, true);
    const edgeNum = view.getUint32(24, true);
    const clusterNum = view.getUint32(28, true);
    const payloadSize = view.getUint32(32, true);
    if (payloadSize !== buffer.byteLength - HEADER_SIZE) {
        throw new Error('Invalid topological map payload size');
    }

    let offset = HEADER_SIZE;
    requireBytes(buffer, offset, tagSize + frameIdSize);
    const tag = decoder.decode(new Uint8Array(buffer, offset, tagSize));
    offset += tagSize;
    const frameId = decoder.decode(new Uint8Array(buffer, offset, frameIdSize));
    offset += frameIdSize;

    const nodes: GraphData['nodes'] = [];
    requireBytes(buffer, offset, nodeNum * NODE_RECORD_SIZE);
    for (let i = 0; i < nodeNum; i += 1) {
        const base = offset + i * NODE_RECORD_SIZE;
        nodes.push({
            id: view.getUint16(base, true),
            label: view.getUint8(base + 2),
            semanticLabel: view.getUint8(base + 3),
            isGoal: view.getUint8(base + 4) !== 0,
            is_boundary_candidate: view.getUint8(base + 5) !== 0,
            boundary_evidence: view.getUint8(base + 6),
            age: view.getUint32(base + 8, true),
            nonplaneComponentId: view.getUint32(base + 12, true),
            winnerPointCount: view.getUint32(base + 16, true),
            semanticReliability: view.getFloat32(base + 20, true),
            x: view.getFloat32(base + 24, true), y: view.getFloat32(base + 28, true), z: view.getFloat32(base + 32, true),
            nx: view.getFloat32(base + 36, true), ny: view.getFloat32(base + 40, true), nz: view.getFloat32(base + 44, true),
            winnerPointCovariance: [
                view.getFloat32(base + 48, true), view.getFloat32(base + 52, true), view.getFloat32(base + 56, true),
                view.getFloat32(base + 60, true), view.getFloat32(base + 64, true), view.getFloat32(base + 68, true),
                view.getFloat32(base + 72, true), view.getFloat32(base + 76, true), view.getFloat32(base + 80, true),
            ],
        });
    }
    offset += nodeNum * NODE_RECORD_SIZE;

    requireBytes(buffer, offset, edgeNum * 2);
    const edges: number[] = [];
    for (let i = 0; i < edgeNum; i += 1) edges.push(view.getUint16(offset + i * 2, true));
    offset += edgeNum * 2;

    const clusters: GraphData['clusters'] = [];
    for (let i = 0; i < clusterNum; i += 1) {
        requireBytes(buffer, offset, CLUSTER_RECORD_SIZE);
        const nodeIdNum = view.getUint32(offset + 76, true);
        requireBytes(buffer, offset + CLUSTER_RECORD_SIZE, nodeIdNum * 2);
        const nodeIds: number[] = [];
        for (let j = 0; j < nodeIdNum; j += 1) nodeIds.push(view.getUint16(offset + CLUSTER_RECORD_SIZE + j * 2, true));
        clusters.push({
            id: view.getUint32(offset, true), label: view.getUint8(offset + 4), semanticLabel: view.getUint8(offset + 5),
            semanticReliability: view.getFloat32(offset + 8, true),
            pos: [view.getFloat32(offset + 12, true), view.getFloat32(offset + 16, true), view.getFloat32(offset + 20, true)],
            scale: [view.getFloat32(offset + 24, true), view.getFloat32(offset + 28, true), view.getFloat32(offset + 32, true)],
            quat: [view.getFloat32(offset + 36, true), view.getFloat32(offset + 40, true), view.getFloat32(offset + 44, true), view.getFloat32(offset + 48, true)],
            match: view.getFloat32(offset + 52, true), reliability: view.getFloat32(offset + 56, true),
            velocity: [view.getFloat32(offset + 60, true), view.getFloat32(offset + 64, true), view.getFloat32(offset + 68, true)],
            nodeIds,
        });
        offset += CLUSTER_RECORD_SIZE + nodeIdNum * 2;
    }
    if (offset !== buffer.byteLength) throw new Error('Invalid topological map trailing data');
    return { tag, graph: { timestamp, tag, frameId, mode: tag.includes('static') ? 'static' : 'dynamic', nodes, edges, clusters } };
}
