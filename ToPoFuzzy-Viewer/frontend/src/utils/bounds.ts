import { PointCloudData } from '../types';

export interface Bounds {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
    maxDist: number;
}

/**
 * Calculate the bounding box of multiple point clouds
 * @param clouds Array of point cloud data
 * @returns Bounds object containing min/max values for each axis and max distance from origin
 */
export function calculateBounds(clouds: PointCloudData[]): Bounds {
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    let maxDist = 0;

    clouds.forEach(cloud => {
        for (let i = 0; i < cloud.points.length; i += 3) {
            const x = cloud.points[i];
            const y = cloud.points[i + 1];
            const z = cloud.points[i + 2];

            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
            minZ = Math.min(minZ, z);
            maxZ = Math.max(maxZ, z);

            const dist = Math.sqrt(x ** 2 + y ** 2 + z ** 2);
            maxDist = Math.max(maxDist, dist);
        }
    });

    return { minX, maxX, minY, maxY, minZ, maxZ, maxDist };
}
