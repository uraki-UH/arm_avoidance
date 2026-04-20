import { useState, useCallback } from 'react';
import { GraphData, GraphCluster, LAYER_LABELS } from '../../types';

export interface ZonePoint {
    x: number;
    y: number;
    z: number;
}

type Rect2 = {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
};

type Bounds3 = Rect2 & {
    minZ: number;
    maxZ: number;
};

const EPS = 1e-9;

const isPointInRect = (x: number, y: number, rect: Rect2) => {
    return x >= rect.minX - EPS && x <= rect.maxX + EPS && y >= rect.minY - EPS && y <= rect.maxY + EPS;
};

const orientation = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) => {
    const val = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (Math.abs(val) < EPS) return 0;
    return val > 0 ? 1 : 2;
};

const onSegment = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) => {
    return bx >= Math.min(ax, cx) - EPS && bx <= Math.max(ax, cx) + EPS
        && by >= Math.min(ay, cy) - EPS && by <= Math.max(ay, cy) + EPS;
};

const segmentsIntersect = (a: ZonePoint, b: ZonePoint, c: ZonePoint, d: ZonePoint) => {
    const o1 = orientation(a.x, a.y, b.x, b.y, c.x, c.y);
    const o2 = orientation(a.x, a.y, b.x, b.y, d.x, d.y);
    const o3 = orientation(c.x, c.y, d.x, d.y, a.x, a.y);
    const o4 = orientation(c.x, c.y, d.x, d.y, b.x, b.y);

    if (o1 !== o2 && o3 !== o4) return true;

    if (o1 === 0 && onSegment(a.x, a.y, c.x, c.y, b.x, b.y)) return true;
    if (o2 === 0 && onSegment(a.x, a.y, d.x, d.y, b.x, b.y)) return true;
    if (o3 === 0 && onSegment(c.x, c.y, a.x, a.y, d.x, d.y)) return true;
    if (o4 === 0 && onSegment(c.x, c.y, b.x, b.y, d.x, d.y)) return true;

    return false;
};

const rotateVectorByQuat = (vx: number, vy: number, vz: number, qx: number, qy: number, qz: number, qw: number) => {
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);

    const rx = vx + qw * tx + (qy * tz - qz * ty);
    const ry = vy + qw * ty + (qz * tx - qx * tz);
    const rz = vz + qw * tz + (qx * ty - qy * tx);

    return [rx, ry, rz] as const;
};

const getClusterBounds = (cluster: GraphCluster): Bounds3 => {
    const [px, py, pz] = cluster.pos;
    const [sx, sy, sz] = cluster.scale;
    const hx = Math.abs(Number.isFinite(sx) ? sx : 0) * 0.5;
    const hy = Math.abs(Number.isFinite(sy) ? sy : 0) * 0.5;
    const hz = Math.abs(Number.isFinite(sz) ? sz : 0) * 0.5;

    let qx = cluster.quat[0] ?? 0;
    let qy = cluster.quat[1] ?? 0;
    let qz = cluster.quat[2] ?? 0;
    let qw = cluster.quat[3] ?? 1;
    const qNorm = Math.hypot(qx, qy, qz, qw);
    if (qNorm > EPS) {
        qx /= qNorm;
        qy /= qNorm;
        qz /= qNorm;
        qw /= qNorm;
    } else {
        qx = 0;
        qy = 0;
        qz = 0;
        qw = 1;
    }

    let minX = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;

    const signs = [-1, 1];
    for (const sxSign of signs) {
        for (const sySign of signs) {
            for (const szSign of signs) {
                const vx = hx * sxSign;
                const vy = hy * sySign;
                const vz = hz * szSign;
                const [rx, ry, rz] = rotateVectorByQuat(vx, vy, vz, qx, qy, qz, qw);
                const wx = px + rx;
                const wy = py + ry;
                const wz = pz + rz;

                if (wx < minX) minX = wx;
                if (wx > maxX) maxX = wx;
                if (wy < minY) minY = wy;
                if (wy > maxY) maxY = wy;
                if (wz < minZ) minZ = wz;
                if (wz > maxZ) maxZ = wz;
            }
        }
    }

    return { minX, maxX, minY, maxY, minZ, maxZ };
};

export function useZoneMonitor() {
    const [points, setPoints] = useState<ZonePoint[]>([]);
    const [isDrawing, setIsDrawing] = useState(false);
    const [zRange, setZRange] = useState<[number, number]>([-2.0, 5.0]); // Default range

    // Add a point to the current zone polygon
    const addPoint = useCallback((point: ZonePoint) => {
        setPoints(prev => [...prev, point]);
    }, []);

    // Clear the current zone
    const clearZone = useCallback(() => {
        setPoints([]);
        setIsDrawing(false);
    }, []);

    const startDrawing = useCallback(() => {
        setPoints([]);
        setIsDrawing(true);
    }, []);

    const finishDrawing = useCallback(() => {
        setIsDrawing(false);
    }, []);

    // Check if a point (x, y) is inside the polygon defined by points
    const isPointInPolygon = useCallback((x: number, y: number, polyPoints: ZonePoint[]) => {
        let inside = false;
        for (let i = 0, j = polyPoints.length - 1; i < polyPoints.length; j = i++) {
            const xi = polyPoints[i].x, yi = polyPoints[i].y;
            const xj = polyPoints[j].x, yj = polyPoints[j].y;

            const intersect = ((yi > y) !== (yj > y))
                && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }, []);

    const polygonIntersectsRect = useCallback((polyPoints: ZonePoint[], rect: Rect2) => {
        if (polyPoints.length < 3) return false;

        const rectCorners: ZonePoint[] = [
            { x: rect.minX, y: rect.minY, z: 0 },
            { x: rect.maxX, y: rect.minY, z: 0 },
            { x: rect.maxX, y: rect.maxY, z: 0 },
            { x: rect.minX, y: rect.maxY, z: 0 }
        ];

        if (rectCorners.some(corner => isPointInPolygon(corner.x, corner.y, polyPoints))) {
            return true;
        }

        if (polyPoints.some(point => isPointInRect(point.x, point.y, rect))) {
            return true;
        }

        for (let i = 0; i < polyPoints.length; i++) {
            const a = polyPoints[i];
            const b = polyPoints[(i + 1) % polyPoints.length];
            for (let j = 0; j < rectCorners.length; j++) {
                const c = rectCorners[j];
                const d = rectCorners[(j + 1) % rectCorners.length];
                if (segmentsIntersect(a, b, c, d)) {
                    return true;
                }
            }
        }

        return false;
    }, [isPointInPolygon]);

    // Calculate counts for each label within the zone
    const getZoneCounts = useCallback((graphData: GraphData | null) => {
        if (!graphData || points.length < 3) {
            return new Map<string, number>();
        }

        const counts = new Map<string, number>();
        // Initialize counts
        LAYER_LABELS.forEach(label => counts.set(label, 0));

        graphData.clusters.forEach(cluster => {
            const bounds = getClusterBounds(cluster);

            // Z-range overlap check (any part of the cluster volume)
            if (bounds.maxZ < zRange[0] || bounds.minZ > zRange[1]) return;

            if (polygonIntersectsRect(points, bounds)) {
                const labelName = LAYER_LABELS[cluster.label % LAYER_LABELS.length];
                counts.set(labelName, (counts.get(labelName) || 0) + 1);
            }
        });

        return counts;
    }, [points, polygonIntersectsRect, zRange]);

    return {
        points,
        isDrawing,
        zRange,
        setZRange,
        addPoint,
        clearZone,
        startDrawing,
        finishDrawing,
        getZoneCounts
    };
}
