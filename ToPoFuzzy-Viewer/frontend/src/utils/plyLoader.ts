import { PointCloudData } from '../types';
import { generateUUID } from './uuid';

export async function loadPLY(file: File): Promise<PointCloudData> {
    const buffer = await file.arrayBuffer();
    const dataView = new DataView(buffer);
    const textDecoder = new TextDecoder('utf-8');

    // Parse Header
    let headerEnd = 0;
    let headerText = '';
    // Read header (it's always ASCII)
    while (headerEnd < buffer.byteLength) {
        const char = textDecoder.decode(buffer.slice(headerEnd, headerEnd + 1));
        headerText += char;
        headerEnd++;
        if (headerText.endsWith('end_header\n') || headerText.endsWith('end_header\r\n')) {
            break;
        }
    }

    const lines = headerText.split(/\r?\n/);
    let format = '';
    let vertexCount = 0;
    const properties: { name: string, type: string }[] = [];

    // Simple state machine for header parsing
    let inVertexElement = false;

    for (const line of lines) {
        const parts = line.trim().split(/\s+/);
        if (parts.length === 0) continue;

        if (parts[0] === 'format') {
            format = parts[1];
        } else if (parts[0] === 'element') {
            if (parts[1] === 'vertex') {
                vertexCount = parseInt(parts[2]);
                inVertexElement = true;
            } else {
                inVertexElement = false;
            }
        } else if (parts[0] === 'property' && inVertexElement) {
            properties.push({
                type: parts[1],
                name: parts[2]
            });
        }
    }

    if (!format || vertexCount === 0) {
        throw new Error('Invalid PLY file: Missing format or vertex count');
    }

    const points = new Float32Array(vertexCount * 3);
    const colors = properties.some(p => ['red', 'r', 'diffuse_red'].includes(p.name)) ? new Float32Array(vertexCount * 3) : undefined;
    const intensities = properties.some(p => ['intensity', 'scalar_intensity', 'i'].includes(p.name)) ? new Float32Array(vertexCount) : undefined;

    // Helper to find property index
    const getPropIndex = (names: string[]) => properties.findIndex(p => names.includes(p.name));

    const xIdx = getPropIndex(['x']);
    const yIdx = getPropIndex(['y']);
    const zIdx = getPropIndex(['z']);
    const rIdx = getPropIndex(['red', 'r', 'diffuse_red']);
    const gIdx = getPropIndex(['green', 'g', 'diffuse_green']);
    const bIdx = getPropIndex(['blue', 'b', 'diffuse_blue']);
    const iIdx = getPropIndex(['intensity', 'scalar_intensity', 'i']);

    if (xIdx === -1 || yIdx === -1 || zIdx === -1) {
        throw new Error('Invalid PLY file: Missing x, y, or z coordinates');
    }

    if (format === 'ascii') {
        // Continue reading text from headerEnd
        const bodyText = textDecoder.decode(buffer.slice(headerEnd));
        const bodyLines = bodyText.trim().split(/\s+/); // Split by any whitespace

        let valueIndex = 0;


        // We use valueIndex to step through all values in the file
        // Each point has `properties.length` valus
        const propsPerPoint = properties.length;

        for (let i = 0; i < vertexCount; i++) {
            if (valueIndex + propsPerPoint > bodyLines.length) break;

            points[i * 3] = parseFloat(bodyLines[valueIndex + xIdx]);
            points[i * 3 + 1] = parseFloat(bodyLines[valueIndex + yIdx]);
            points[i * 3 + 2] = parseFloat(bodyLines[valueIndex + zIdx]);

            if (colors && rIdx !== -1 && gIdx !== -1 && bIdx !== -1) {
                // PLY colors are usually 0-255 uchar
                colors[i * 3] = parseFloat(bodyLines[valueIndex + rIdx]) / 255.0;
                colors[i * 3 + 1] = parseFloat(bodyLines[valueIndex + gIdx]) / 255.0;
                colors[i * 3 + 2] = parseFloat(bodyLines[valueIndex + bIdx]) / 255.0;
            }

            if (intensities && iIdx !== -1) {
                intensities[i] = parseFloat(bodyLines[valueIndex + iIdx]);
            }

            valueIndex += propsPerPoint;
        }

    } else if (format === 'binary_little_endian') {
        let offset = headerEnd;

        // Helper to read property value
        const readValue = (type: string, off: number): { value: number, size: number } => {
            switch (type) {
                case 'char': case 'int8': return { value: dataView.getInt8(off), size: 1 };
                case 'uchar': case 'uint8': return { value: dataView.getUint8(off), size: 1 };
                case 'short': case 'int16': return { value: dataView.getInt16(off, true), size: 2 };
                case 'ushort': case 'uint16': return { value: dataView.getUint16(off, true), size: 2 };
                case 'int': case 'int32': return { value: dataView.getInt32(off, true), size: 4 };
                case 'uint': case 'uint32': return { value: dataView.getUint32(off, true), size: 4 };
                case 'float': case 'float32': return { value: dataView.getFloat32(off, true), size: 4 };
                case 'double': case 'float64': return { value: dataView.getFloat64(off, true), size: 8 };
                default: throw new Error(`Unsupported PLY property type: ${type}`);
            }
        };

        for (let i = 0; i < vertexCount; i++) {
            let tempX = 0, tempY = 0, tempZ = 0;
            let tempR = 0, tempG = 0, tempB = 0;
            let tempI = 0;

            for (let p = 0; p < properties.length; p++) {
                const prop = properties[p];
                const res = readValue(prop.type, offset);
                const val = res.value;
                offset += res.size;

                if (p === xIdx) tempX = val;
                if (p === yIdx) tempY = val;
                if (p === zIdx) tempZ = val;
                if (p === rIdx) tempR = val;
                if (p === gIdx) tempG = val;
                if (p === bIdx) tempB = val;
                if (p === iIdx) tempI = val;
            }

            points[i * 3] = tempX;
            points[i * 3 + 1] = tempY;
            points[i * 3 + 2] = tempZ;

            if (colors) {
                colors[i * 3] = tempR / 255.0;
                colors[i * 3 + 1] = tempG / 255.0;
                colors[i * 3 + 2] = tempB / 255.0;
            }

            if (intensities) {
                intensities[i] = tempI;
            }
        }
    } else {
        throw new Error(`Unsupported PLY format: ${format}`);
    }

    return {
        id: generateUUID(),
        name: file.name,
        visible: true,
        opacity: 1,
        count: vertexCount,
        points: points,
        colors: colors,
        intensities: intensities
    };
}
