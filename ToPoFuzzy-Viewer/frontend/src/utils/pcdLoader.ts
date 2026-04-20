import { PointCloudData } from '../types';
import { generateUUID } from './uuid';

export async function loadPCD(file: File): Promise<PointCloudData> {
    const text = await file.text();
    const lines = text.split('\n');

    let pointCount = 0;
    let hasColor = false;
    let hasIntensity = false;
    let dataStart = 0;
    let isBinary = false;

    // Parse header
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();

        if (line.startsWith('POINTS')) {
            pointCount = parseInt(line.split(' ')[1]);
        } else if (line.startsWith('FIELDS')) {
            const fields = line.split(' ').slice(1);
            hasColor = fields.includes('rgb') || fields.includes('rgba');
            hasIntensity = fields.includes('intensity');
        } else if (line.startsWith('DATA')) {
            isBinary = line.includes('binary');
            dataStart = i + 1;
            break;
        }
    }

    const points = new Float32Array(pointCount * 3);
    const colors = hasColor ? new Float32Array(pointCount * 3) : undefined;
    const intensities = hasIntensity ? new Float32Array(pointCount) : undefined;

    if (!isBinary) {
        // ASCII format
        let pointIndex = 0;
        for (let i = dataStart; i < lines.length && pointIndex < pointCount; i++) {
            const line = lines[i].trim();
            if (!line) continue;

            const values = line.split(/\s+/).map(parseFloat);
            if (values.length >= 3) {
                points[pointIndex * 3] = values[0];
                points[pointIndex * 3 + 1] = values[1];
                points[pointIndex * 3 + 2] = values[2];

                if (hasColor && colors && values.length >= 4) {
                    // Assume RGB packed as uint32
                    const rgb = values[3];
                    colors[pointIndex * 3] = ((rgb >> 16) & 0xFF) / 255;
                    colors[pointIndex * 3 + 1] = ((rgb >> 8) & 0xFF) / 255;
                    colors[pointIndex * 3 + 2] = (rgb & 0xFF) / 255;
                }

                if (hasIntensity && intensities) {
                    intensities[pointIndex] = values[hasColor ? 4 : 3] || 0;
                }

                pointIndex++;
            }
        }
    }

    return {
        id: generateUUID(),
        name: file.name,
        points,
        colors,
        intensities,
        count: pointCount
    };
}
