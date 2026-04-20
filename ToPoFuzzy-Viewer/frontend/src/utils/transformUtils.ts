import * as THREE from 'three';

/**
 * Build a transform matrix from position, rotation (Euler), and scale.
 */
export function buildTransformMatrix(
    position?: [number, number, number],
    rotation?: [number, number, number],
    scale?: [number, number, number]
): THREE.Matrix4 {
    const matrix = new THREE.Matrix4();

    const pos = position || [0, 0, 0];
    const rot = rotation || [0, 0, 0];
    const scl = scale || [1, 1, 1];

    const euler = new THREE.Euler(rot[0], rot[1], rot[2], 'XYZ');
    const quaternion = new THREE.Quaternion().setFromEuler(euler);
    const posVec = new THREE.Vector3(pos[0], pos[1], pos[2]);
    const scaleVec = new THREE.Vector3(scl[0], scl[1], scl[2]);

    matrix.compose(posVec, quaternion, scaleVec);
    return matrix;
}

/**
 * Apply a transform matrix to all points in a Float32Array.
 * Returns a new Float32Array with transformed positions.
 */
export function applyTransformToPoints(
    points: Float32Array,
    matrix: THREE.Matrix4
): Float32Array {
    const pointCount = points.length / 3;
    const result = new Float32Array(points.length);
    const tempVec = new THREE.Vector3();

    for (let i = 0; i < pointCount; i++) {
        tempVec.set(
            points[i * 3],
            points[i * 3 + 1],
            points[i * 3 + 2]
        );
        tempVec.applyMatrix4(matrix);

        result[i * 3] = tempVec.x;
        result[i * 3 + 1] = tempVec.y;
        result[i * 3 + 2] = tempVec.z;
    }

    return result;
}

/**
 * Check if a transform is identity (no transformation applied).
 */
export function isIdentityTransform(
    position?: [number, number, number],
    rotation?: [number, number, number],
    scale?: [number, number, number]
): boolean {
    const pos = position || [0, 0, 0];
    const rot = rotation || [0, 0, 0];
    const scl = scale || [1, 1, 1];

    const epsilon = 0.0001;
    return (
        Math.abs(pos[0]) < epsilon && Math.abs(pos[1]) < epsilon && Math.abs(pos[2]) < epsilon &&
        Math.abs(rot[0]) < epsilon && Math.abs(rot[1]) < epsilon && Math.abs(rot[2]) < epsilon &&
        Math.abs(scl[0] - 1) < epsilon && Math.abs(scl[1] - 1) < epsilon && Math.abs(scl[2] - 1) < epsilon
    );
}
