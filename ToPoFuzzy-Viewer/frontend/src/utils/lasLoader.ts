import { PointCloudData } from '../types';
import { generateUUID } from './uuid';

const MAX_SAFE_SIZE = 500 * 1024 * 1024; // 500MB safe limit
const MAX_POINTS_LIMIT = 10_000_000; // 10M points max

export async function loadLAS(file: File): Promise<PointCloudData> {
    // Check file size
    if (file.size > MAX_SAFE_SIZE) {
        const sizeMB = (file.size / 1024 / 1024).toFixed(1);
        console.warn(`Large LAS file detected: ${sizeMB}MB. Loading may take time...`);
    }

    // Read header first (only first 227 bytes)
    const headerBlob = file.slice(0, 227);
    const headerBuffer = await headerBlob.arrayBuffer();
    const headerView = new DataView(headerBuffer);

    // Verify signature
    const signature = String.fromCharCode(
        headerView.getUint8(0),
        headerView.getUint8(1),
        headerView.getUint8(2),
        headerView.getUint8(3)
    );
    if (signature !== 'LASF') {
        throw new Error('Invalid LAS file');
    }

    const pointDataOffset = headerView.getUint32(96, true);
    const pointRecordLength = headerView.getUint16(105, true);
    const pointCount = headerView.getUint32(107, true);

    const scaleX = headerView.getFloat64(131, true);
    const scaleY = headerView.getFloat64(139, true);
    const scaleZ = headerView.getFloat64(147, true);
    const offsetX = headerView.getFloat64(155, true);
    const offsetY = headerView.getFloat64(163, true);
    const offsetZ = headerView.getFloat64(171, true);

    // Check if we need to downsample
    let actualPointCount = pointCount;
    let stride = 1; // Read every Nth point

    if (pointCount > MAX_POINTS_LIMIT) {
        stride = Math.ceil(pointCount / MAX_POINTS_LIMIT);
        actualPointCount = Math.floor(pointCount / stride);
        console.warn(
            `Point cloud too large (${pointCount.toLocaleString()} points). ` +
            `Downsampling to ${actualPointCount.toLocaleString()} points (1/${stride} sampling).`
        );
    }

    const points = new Float32Array(actualPointCount * 3);
    const intensities = new Float32Array(actualPointCount);

    // Read point data in chunks to avoid memory issues
    const CHUNK_SIZE = 1024 * 1024 * 10; // 10MB chunks
    const POINTS_PER_CHUNK = Math.floor(CHUNK_SIZE / pointRecordLength);

    let pointsRead = 0;

    for (let chunkStart = 0; chunkStart < pointCount; chunkStart += POINTS_PER_CHUNK) {
        const chunkEnd = Math.min(chunkStart + POINTS_PER_CHUNK, pointCount);
        const chunkByteStart = pointDataOffset + chunkStart * pointRecordLength;
        const chunkByteEnd = pointDataOffset + chunkEnd * pointRecordLength;

        // Read chunk
        const chunkBlob = file.slice(chunkByteStart, chunkByteEnd);
        const chunkBuffer = await chunkBlob.arrayBuffer();
        const chunkView = new DataView(chunkBuffer);

        // Process points in this chunk
        for (let i = 0; i < (chunkEnd - chunkStart); i++) {
            const globalIndex = chunkStart + i;

            // Skip if downsampling
            if (globalIndex % stride !== 0) continue;

            const offset = i * pointRecordLength;

            const x = chunkView.getInt32(offset, true) * scaleX + offsetX;
            const y = chunkView.getInt32(offset + 4, true) * scaleY + offsetY;
            const z = chunkView.getInt32(offset + 8, true) * scaleZ + offsetZ;
            const intensity = chunkView.getUint16(offset + 12, true);

            points[pointsRead * 3] = x;
            points[pointsRead * 3 + 1] = y;
            points[pointsRead * 3 + 2] = z;
            intensities[pointsRead] = intensity / 65535;

            pointsRead++;
        }

        // Log progress for large files
        if (pointCount > 1000000 && chunkStart % (POINTS_PER_CHUNK * 10) === 0) {
            const progress = ((chunkEnd / pointCount) * 100).toFixed(1);
            console.log(`Loading LAS: ${progress}%`);
        }
    }

    console.log(`Loaded ${pointsRead.toLocaleString()} points from ${file.name}`);

    return {
        id: generateUUID(),
        name: file.name + (stride > 1 ? ` (1/${stride} sampled)` : ''),
        points,
        intensities,
        count: pointsRead
    };
}
