import { PointCloudData } from '../../types';
import * as THREE from 'three';

/**
 * Export point cloud data to LAS format (version 1.2)
 * Note: This is a simplified implementation focusing on basic point data
 */

export interface LASExportOptions {
    includeTransform: boolean;
}

export function exportToLAS(
    data: PointCloudData,
    options: LASExportOptions = { includeTransform: false }
): Blob {
    const { includeTransform } = options;

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

    return exportLAS12(positions, colors, data.count);
}

function exportLAS12(positions: Float32Array, colors: Float32Array | undefined, count: number): Blob {
    // Calculate bounds
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

    for (let i = 0; i < count; i++) {
        const x = positions[i * 3];
        const y = positions[i * 3 + 1];
        const z = positions[i * 3 + 2];

        minX = Math.min(minX, x);
        minY = Math.min(minY, y);
        minZ = Math.min(minZ, z);
        maxX = Math.max(maxX, x);
        maxY = Math.max(maxY, y);
        maxZ = Math.max(maxZ, z);
    }

    // LAS 1.2 Public Header Block (227 bytes)
    const headerSize = 227;
    const pointDataRecordFormat = colors ? 2 : 0; // Format 0: XYZ, Format 2: XYZ + RGB
    const pointDataRecordLength = colors ? 26 : 20;
    const offsetToPointData = headerSize;

    const totalSize = headerSize + (pointDataRecordLength * count);
    const buffer = new ArrayBuffer(totalSize);
    const view = new DataView(buffer);

    let offset = 0;

    // File Signature ("LASF")
    view.setUint8(offset++, 76); // 'L'
    view.setUint8(offset++, 65); // 'A'
    view.setUint8(offset++, 83); // 'S'
    view.setUint8(offset++, 70); // 'F'

    // File Source ID
    view.setUint16(offset, 0, true); offset += 2;

    // Global Encoding
    view.setUint16(offset, 0, true); offset += 2;

    // Project ID (GUID) - 16 bytes of zeros
    offset += 16;

    // Version Major / Minor
    view.setUint8(offset++, 1); // Major
    view.setUint8(offset++, 2); // Minor

    // System Identifier (32 bytes) - "PointCloudViewer"
    const sysId = 'PointCloudViewer';
    for (let i = 0; i < 32; i++) {
        view.setUint8(offset++, i < sysId.length ? sysId.charCodeAt(i) : 0);
    }

    // Generating Software (32 bytes)
    const genSoft = 'ToPoFuzzy WebViewer';
    for (let i = 0; i < 32; i++) {
        view.setUint8(offset++, i < genSoft.length ? genSoft.charCodeAt(i) : 0);
    }

    // File Creation Day/Year
    const now = new Date();
    view.setUint16(offset, now.getDate(), true); offset += 2;
    view.setUint16(offset, now.getFullYear(), true); offset += 2;

    // Header Size
    view.setUint16(offset, headerSize, true); offset += 2;

    // Offset to point data
    view.setUint32(offset, offsetToPointData, true); offset += 4;

    // Number of Variable Length Records
    view.setUint32(offset, 0, true); offset += 4;

    // Point Data Record Format
    view.setUint8(offset++, pointDataRecordFormat);

    // Point Data Record Length
    view.setUint16(offset, pointDataRecordLength, true); offset += 2;

    // Legacy Number of Point Records
    view.setUint32(offset, count, true); offset += 4;

    // Legacy Number of Points by Return (5 * 4 bytes)
    view.setUint32(offset, count, true); offset += 4; // Return 1
    offset += 16; // Returns 2-5 (zeros)

    // X/Y/Z Scale Factors
    const scale = 0.001; // 1mm precision
    view.setFloat64(offset, scale, true); offset += 8;
    view.setFloat64(offset, scale, true); offset += 8;
    view.setFloat64(offset, scale, true); offset += 8;

    // X/Y/Z Offsets
    view.setFloat64(offset, minX, true); offset += 8;
    view.setFloat64(offset, minY, true); offset += 8;
    view.setFloat64(offset, minZ, true); offset += 8;


    // Max X/Y/Z (MUST come BEFORE Min per LAS 1.2 spec!)
    view.setFloat64(offset, maxX, true); offset += 8;
    view.setFloat64(offset, maxY, true); offset += 8;
    view.setFloat64(offset, maxZ, true); offset += 8;

    // Min X/Y/Z
    view.setFloat64(offset, minX, true); offset += 8;
    view.setFloat64(offset, minY, true); offset += 8;
    view.setFloat64(offset, minZ, true); offset += 8;

    // Point Data Records
    for (let i = 0; i < count; i++) {
        const x = positions[i * 3];
        const y = positions[i * 3 + 1];
        const z = positions[i * 3 + 2];

        // Scale and offset to integer
        const xInt = Math.round((x - minX) / scale);
        const yInt = Math.round((y - minY) / scale);
        const zInt = Math.round((z - minZ) / scale);

        view.setInt32(offset, xInt, true); offset += 4;
        view.setInt32(offset, yInt, true); offset += 4;
        view.setInt32(offset, zInt, true); offset += 4;

        // Intensity
        view.setUint16(offset, 0, true); offset += 2;

        // Return Number, Number of Returns, etc. (bit fields)
        view.setUint8(offset++, 0);

        // Classification
        view.setUint8(offset++, 0);

        // Scan Angle Rank
        view.setInt8(offset++, 0);

        // User Data
        view.setUint8(offset++, 0);

        // Point Source ID
        view.setUint16(offset, 0, true); offset += 2;

        // RGB (if format 2)
        if (colors) {
            const r = Math.floor(colors[i * 3] * 65535);
            const g = Math.floor(colors[i * 3 + 1] * 65535);
            const b = Math.floor(colors[i * 3 + 2] * 65535);
            view.setUint16(offset, r, true); offset += 2;
            view.setUint16(offset, g, true); offset += 2;
            view.setUint16(offset, b, true); offset += 2;
        }
    }

    return new Blob([buffer], { type: 'application/octet-stream' });
}
