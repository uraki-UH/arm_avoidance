import { PointCloudData } from '../../types';
import * as THREE from 'three';

/**
 * Export point cloud data to PLY format (ASCII or Binary)
 */

export interface PLYExportOptions {
    format: 'ascii' | 'binary';
    includeTransform: boolean;
}

export function exportToPLY(
    data: PointCloudData,
    options: PLYExportOptions = { format: 'ascii', includeTransform: false }
): Blob {
    const { format, includeTransform } = options;

    // Apply transform if requested
    let positions = data.points;
    const colors = data.colors;

    if (includeTransform && (data.position || data.rotation || data.scale)) {
        const matrix = new THREE.Matrix4();

        if (data.position) {
            matrix.makeTranslation(...data.position);
        }
        if (data.rotation) {
            const rotMatrix = new THREE.Matrix4();
            rotMatrix.makeRotationFromEuler(new THREE.Euler(...data.rotation));
            matrix.multiply(rotMatrix);
        }
        if (data.scale) {
            const scaleMatrix = new THREE.Matrix4();
            scaleMatrix.makeScale(...data.scale);
            matrix.multiply(scaleMatrix);
        }

        // Transform points
        const transformedPos = new Float32Array(positions.length);
        for (let i = 0; i < positions.length; i += 3) {
            const v = new THREE.Vector3(positions[i], positions[i + 1], positions[i + 2]);
            v.applyMatrix4(matrix);
            transformedPos[i] = v.x;
            transformedPos[i + 1] = v.y;
            transformedPos[i + 2] = v.z;
        }
        positions = transformedPos;
    }

    if (format === 'ascii') {
        return exportPLYAscii(positions, colors, data.count);
    } else {
        return exportPLYBinary(positions, colors, data.count);
    }
}

function exportPLYAscii(positions: Float32Array, colors: Float32Array | undefined, count: number): Blob {
    let plyContent = '';

    // Header
    plyContent += 'ply\n';
    plyContent += 'format ascii 1.0\n';
    plyContent += `element vertex ${count}\n`;
    plyContent += 'property float x\n';
    plyContent += 'property float y\n';
    plyContent += 'property float z\n';

    if (colors) {
        plyContent += 'property uchar red\n';
        plyContent += 'property uchar green\n';
        plyContent += 'property uchar blue\n';
    }

    plyContent += 'end_header\n';

    // Data
    for (let i = 0; i < count; i++) {
        const x = positions[i * 3];
        const y = positions[i * 3 + 1];
        const z = positions[i * 3 + 2];

        if (colors) {
            const r = Math.floor(colors[i * 3] * 255);
            const g = Math.floor(colors[i * 3 + 1] * 255);
            const b = Math.floor(colors[i * 3 + 2] * 255);
            plyContent += `${x} ${y} ${z} ${r} ${g} ${b}\n`;
        } else {
            plyContent += `${x} ${y} ${z}\n`;
        }
    }

    return new Blob([plyContent], { type: 'text/plain' });
}

function exportPLYBinary(positions: Float32Array, colors: Float32Array | undefined, count: number): Blob {
    // Header (ASCII part)
    let header = '';
    header += 'ply\n';
    header += 'format binary_little_endian 1.0\n';
    header += `element vertex ${count}\n`;
    header += 'property float x\n';
    header += 'property float y\n';
    header += 'property float z\n';

    if (colors) {
        header += 'property uchar red\n';
        header += 'property uchar green\n';
        header += 'property uchar blue\n';
    }

    header += 'end_header\n';

    // Binary data
    const pointSize = colors ? 15 : 12; // 3 floats (12 bytes) or 3 floats + 3 bytes RGB (15 bytes)
    const buffer = new ArrayBuffer(pointSize * count);
    const view = new DataView(buffer);

    for (let i = 0; i < count; i++) {
        const offset = i * pointSize;

        view.setFloat32(offset, positions[i * 3], true); // x (little endian)
        view.setFloat32(offset + 4, positions[i * 3 + 1], true); // y
        view.setFloat32(offset + 8, positions[i * 3 + 2], true); // z

        if (colors) {
            const r = Math.floor(colors[i * 3] * 255);
            const g = Math.floor(colors[i * 3 + 1] * 255);
            const b = Math.floor(colors[i * 3 + 2] * 255);
            view.setUint8(offset + 12, r);
            view.setUint8(offset + 13, g);
            view.setUint8(offset + 14, b);
        }
    }

    // Combine header and binary data
    const headerBlob = new Blob([header], { type: 'text/plain' });
    const dataBlob = new Blob([buffer], { type: 'application/octet-stream' });

    return new Blob([headerBlob, dataBlob], { type: 'application/octet-stream' });
}
