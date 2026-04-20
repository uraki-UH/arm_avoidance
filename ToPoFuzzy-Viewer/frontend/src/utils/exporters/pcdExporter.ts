import { PointCloudData } from '../../types';
import * as THREE from 'three';

/**
 * Export point cloud data to PCD format (ASCII or Binary)
 */

export interface PCDExportOptions {
    format: 'ascii' | 'binary';
    includeTransform: boolean;
}

export function exportToPCD(
    data: PointCloudData,
    options: PCDExportOptions = { format: 'ascii', includeTransform: false }
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
        return exportPCDAscii(positions, colors, data.count);
    } else {
        return exportPCDBinary(positions, colors, data.count);
    }
}

function exportPCDAscii(positions: Float32Array, colors: Float32Array | undefined, count: number): Blob {
    let pcdContent = '';

    // Header
    pcdContent += '# .PCD v0.7 - Point Cloud Data file format\n';
    pcdContent += 'VERSION 0.7\n';

    if (colors) {
        pcdContent += 'FIELDS x y z rgb\n';
        pcdContent += 'SIZE 4 4 4 4\n';
        pcdContent += 'TYPE F F F U\n';
        pcdContent += 'COUNT 1 1 1 1\n';
    } else {
        pcdContent += 'FIELDS x y z\n';
        pcdContent += 'SIZE 4 4 4\n';
        pcdContent += 'TYPE F F F\n';
        pcdContent += 'COUNT 1 1 1\n';
    }

    pcdContent += `WIDTH ${count}\n`;
    pcdContent += 'HEIGHT 1\n';
    pcdContent += 'VIEWPOINT 0 0 0 1 0 0 0\n';
    pcdContent += `POINTS ${count}\n`;
    pcdContent += 'DATA ascii\n';

    // Data
    for (let i = 0; i < count; i++) {
        const x = positions[i * 3];
        const y = positions[i * 3 + 1];
        const z = positions[i * 3 + 2];

        if (colors) {
            const r = Math.floor(colors[i * 3] * 255);
            const g = Math.floor(colors[i * 3 + 1] * 255);
            const b = Math.floor(colors[i * 3 + 2] * 255);
            const rgb = (r << 16) | (g << 8) | b;
            pcdContent += `${x} ${y} ${z} ${rgb}\n`;
        } else {
            pcdContent += `${x} ${y} ${z}\n`;
        }
    }

    return new Blob([pcdContent], { type: 'text/plain' });
}

function exportPCDBinary(positions: Float32Array, colors: Float32Array | undefined, count: number): Blob {
    // Header (ASCII part)
    let header = '';
    header += '# .PCD v0.7 - Point Cloud Data file format\n';
    header += 'VERSION 0.7\n';

    if (colors) {
        header += 'FIELDS x y z rgb\n';
        header += 'SIZE 4 4 4 4\n';
        header += 'TYPE F F F U\n';
        header += 'COUNT 1 1 1 1\n';
    } else {
        header += 'FIELDS x y z\n';
        header += 'SIZE 4 4 4\n';
        header += 'TYPE F F F\n';
        header += 'COUNT 1 1 1\n';
    }

    header += `WIDTH ${count}\n`;
    header += 'HEIGHT 1\n';
    header += 'VIEWPOINT 0 0 0 1 0 0 0\n';
    header += `POINTS ${count}\n`;
    header += 'DATA binary\n';

    // Binary data
    const pointSize = colors ? 16 : 12; // 3 floats (12 bytes) or 3 floats + 1 uint (16 bytes)
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
            const rgb = (r << 16) | (g << 8) | b;
            view.setUint32(offset + 12, rgb, true);
        }
    }

    // Combine header and binary data
    const headerBlob = new Blob([header], { type: 'text/plain' });
    const dataBlob = new Blob([buffer], { type: 'application/octet-stream' });

    return new Blob([headerBlob, dataBlob], { type: 'application/octet-stream' });
}
