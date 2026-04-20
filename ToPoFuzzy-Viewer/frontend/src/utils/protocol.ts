export const MASK_XYZ = 0x01;
export const MASK_RGB = 0x02;
export const MASK_INTENSITY = 0x04;
export const MASK_NORMALS = 0x08;

export const PROTOCOL_VERSION = 1;
export const MAGIC = 0x50434458; // "PCDX"

export interface ProtocolHeader {
    magic: number;
    version: number;
    pointCount: number;
    dataMask: number;
    payloadSize: number;
}

export interface DeserializedPointCloud {
    pointCount: number;
    positions: Float32Array;
    colors?: Uint8Array;
    intensities?: Float32Array;
    normals?: Float32Array;
}

export function deserializePointCloud(buffer: ArrayBuffer): DeserializedPointCloud {
    const view = new DataView(buffer);
    let offset = 0;

    // Read header
    const header: ProtocolHeader = {
        magic: view.getUint32(offset, true),
        version: view.getUint32(offset + 4, true),
        pointCount: view.getUint32(offset + 8, true),
        dataMask: view.getUint8(offset + 12),
        payloadSize: view.getUint32(offset + 16, true)
    };

    // Validate magic
    if (header.magic !== MAGIC) {
        throw new Error(`Invalid magic number: 0x${header.magic.toString(16)}`);
    }

    // Validate version
    if (header.version !== PROTOCOL_VERSION) {
        throw new Error(`Unsupported protocol version: ${header.version}`);
    }

    offset = 20; // Header size

    // Read positions (always present)
    const positionsByteLength = header.pointCount * 3 * 4; // 3 floats per point
    const positions = new Float32Array(buffer, offset, header.pointCount * 3);
    offset += positionsByteLength;

    const result: DeserializedPointCloud = {
        pointCount: header.pointCount,
        positions: new Float32Array(positions) // Create copy
    };

    // Read colors if present
    if (header.dataMask & MASK_RGB) {
        const colorsByteLength = header.pointCount * 3; // 3 uint8 per point
        const colors = new Uint8Array(buffer, offset, header.pointCount * 3);
        result.colors = new Uint8Array(colors); // Create copy
        offset += colorsByteLength;
    }

    // Read intensities if present
    if (header.dataMask & MASK_INTENSITY) {
        const intensitiesByteLength = header.pointCount * 4; // 1 float per point
        const intensities = new Float32Array(buffer, offset, header.pointCount);
        result.intensities = new Float32Array(intensities); // Create copy
        offset += intensitiesByteLength;
    }

    // Read normals if present
    if (header.dataMask & MASK_NORMALS) {
        const normalsByteLength = header.pointCount * 3 * 4; // 3 floats per point
        const normals = new Float32Array(buffer, offset, header.pointCount * 3);
        result.normals = new Float32Array(normals); // Create copy
        offset += normalsByteLength;
    }

    return result;
}
