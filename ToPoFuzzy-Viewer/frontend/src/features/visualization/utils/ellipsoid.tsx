import { useMemo } from 'react';
import * as THREE from 'three';

export interface EllipsoidInstance {
    center: [number, number, number];
    covariance?: ArrayLike<number> | null;
    scale?: [number, number, number];
    quaternion?: [number, number, number, number];
    color?: string;
    visible?: boolean;
}

export interface EllipsoidTransform {
    position: [number, number, number];
    quaternion: [number, number, number, number];
    scale: [number, number, number];
}

export interface EllipsoidInstanceOptions {
    defaultColor?: string;
    sigmaMultiplier?: number;
    minAxisLength?: number;
    maxAxisLength?: number;
}

const tempMatrix = new THREE.Matrix4();
const tempColor = new THREE.Color();
const tempQuaternion = new THREE.Quaternion();
const tempPosition = new THREE.Vector3();
const tempScale = new THREE.Vector3();
const tempRotationMatrix = new THREE.Matrix4();
const identityQuaternion = [0, 0, 0, 1] as [number, number, number, number];

function cloneSymmetricMatrix(covariance: ArrayLike<number>): number[][] {
    return [
        [covariance[0] ?? 0, covariance[1] ?? 0, covariance[2] ?? 0],
        [covariance[3] ?? 0, covariance[4] ?? 0, covariance[5] ?? 0],
        [covariance[6] ?? 0, covariance[7] ?? 0, covariance[8] ?? 0],
    ];
}

function toSymmetricMatrix3(covariance: ArrayLike<number>): number[][] {
    const m = cloneSymmetricMatrix(covariance);
    // Force symmetry so small numeric asymmetries do not destabilize the solver.
    m[1][0] = m[0][1] = 0.5 * (m[0][1] + m[1][0]);
    m[2][0] = m[0][2] = 0.5 * (m[0][2] + m[2][0]);
    m[2][1] = m[1][2] = 0.5 * (m[1][2] + m[2][1]);
    return m;
}

function jacobiEigenDecomposition3x3(matrix: number[][]) {
    const a = matrix.map(row => row.slice());
    const v = [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ];

    for (let iter = 0; iter < 24; iter++) {
        let p = 0;
        let q = 1;
        let max = Math.abs(a[0][1]);
        const candidates: Array<[number, number, number]> = [
            [0, 2, Math.abs(a[0][2])],
            [1, 2, Math.abs(a[1][2])],
        ];
        for (const [i, j, value] of candidates) {
            if (value > max) {
                max = value;
                p = i;
                q = j;
            }
        }
        if (max < 1e-12) break;

        const app = a[p][p];
        const aqq = a[q][q];
        const apq = a[p][q];
        if (Math.abs(apq) < 1e-12) continue;

        const tau = (aqq - app) / (2 * apq);
        const t = Math.sign(tau) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;
        const tauPrime = s / (1 + c);

        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = 0;
        a[q][p] = 0;

        for (let k = 0; k < 3; k++) {
            if (k === p || k === q) continue;
            const aik = a[k][p];
            const akq = a[k][q];
            a[k][p] = aik - s * (akq + tauPrime * aik);
            a[p][k] = a[k][p];
            a[k][q] = akq + s * (aik - tauPrime * akq);
            a[q][k] = a[k][q];
        }

        for (let k = 0; k < 3; k++) {
            const vip = v[k][p];
            const viq = v[k][q];
            v[k][p] = vip - s * (viq + tauPrime * vip);
            v[k][q] = viq + s * (vip - tauPrime * viq);
        }
    }

    const eigenpairs = [
        { value: a[0][0], vector: [v[0][0], v[1][0], v[2][0]] as [number, number, number] },
        { value: a[1][1], vector: [v[0][1], v[1][1], v[2][1]] as [number, number, number] },
        { value: a[2][2], vector: [v[0][2], v[1][2], v[2][2]] as [number, number, number] },
    ].sort((lhs, rhs) => rhs.value - lhs.value);

    return eigenpairs;
}

function normalize3(v: [number, number, number]): [number, number, number] {
    const len = Math.hypot(v[0], v[1], v[2]);
    if (!Number.isFinite(len) || len <= 1e-12) return [1, 0, 0];
    return [v[0] / len, v[1] / len, v[2] / len];
}

function cross3(a: [number, number, number], b: [number, number, number]): [number, number, number] {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
}

function dot3(a: [number, number, number], b: [number, number, number]): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function ensureRightHandedBasis(
    x: [number, number, number],
    y: [number, number, number],
    z: [number, number, number]
): [[number, number, number], [number, number, number], [number, number, number]] {
    const nx = normalize3(x);
    const ny = normalize3(y);
    let nz = normalize3(z);
    const cross = cross3(nx, ny);
    if (dot3(cross, nz) < 0) {
        nz = [-nz[0], -nz[1], -nz[2]];
    }
    return [nx, ny, nz];
}

export function covarianceToEllipsoidTransform(
    center: [number, number, number],
    covariance: ArrayLike<number>,
    options: EllipsoidInstanceOptions = {}
): EllipsoidTransform | null {
    if (!covariance || covariance.length < 9) {
        return null;
    }

    const sigmaMultiplier = options.sigmaMultiplier ?? 2.0;
    const minAxisLength = options.minAxisLength ?? 0.003;
    const maxAxisLength = options.maxAxisLength ?? 0.6;

    const eigenpairs = jacobiEigenDecomposition3x3(toSymmetricMatrix3(covariance));
    if (eigenpairs.length < 3) {
        return null;
    }

    const axes = eigenpairs.map((pair) => {
        const eigenvalue = Number.isFinite(pair.value) ? Math.max(pair.value, 0) : 0;
        const axisLength = Math.min(maxAxisLength, Math.max(minAxisLength, Math.sqrt(eigenvalue) * sigmaMultiplier));
        return {
            length: axisLength,
            vector: normalize3(pair.vector),
        };
    });

    const [xAxis, yAxis, zAxis] = ensureRightHandedBasis(
        axes[0].vector,
        axes[1].vector,
        axes[2].vector
    );

    tempRotationMatrix.makeBasis(
        new THREE.Vector3(xAxis[0], xAxis[1], xAxis[2]),
        new THREE.Vector3(yAxis[0], yAxis[1], yAxis[2]),
        new THREE.Vector3(zAxis[0], zAxis[1], zAxis[2])
    );
    tempQuaternion.setFromRotationMatrix(tempRotationMatrix);

    return {
        position: center,
        quaternion: [tempQuaternion.x, tempQuaternion.y, tempQuaternion.z, tempQuaternion.w],
        scale: [axes[0].length, axes[1].length, axes[2].length],
    };
}

export function getEllipsoidTransform(
    instance: EllipsoidInstance,
    options: EllipsoidInstanceOptions = {}
): EllipsoidTransform | null {
    const center = instance.center;
    if (instance.covariance && instance.covariance.length >= 9) {
        return covarianceToEllipsoidTransform(center, instance.covariance, options);
    }

    const scale = instance.scale || [1, 1, 1];
    const quaternion = instance.quaternion || identityQuaternion;
    return {
        position: center,
        quaternion,
        scale,
    };
}

export function updateEllipsoidInstances(
    mesh: THREE.InstancedMesh,
    instances: EllipsoidInstance[],
    options: EllipsoidInstanceOptions = {}
) {
    if (!mesh) return;
    const defaultColor = options.defaultColor ?? '#00ffff';
    mesh.count = instances.length;

    instances.forEach((instance, index) => {
        if (instance.visible === false) {
            tempMatrix.identity().scale(tempScale.set(0, 0, 0));
            mesh.setMatrixAt(index, tempMatrix);
            return;
        }

        const transform = getEllipsoidTransform(instance, options);
        if (!transform) {
            tempMatrix.identity().scale(tempScale.set(0, 0, 0));
            mesh.setMatrixAt(index, tempMatrix);
            return;
        }

        tempPosition.set(transform.position[0], transform.position[1], transform.position[2]);
        tempQuaternion.set(transform.quaternion[0], transform.quaternion[1], transform.quaternion[2], transform.quaternion[3]);
        tempScale.set(transform.scale[0], transform.scale[1], transform.scale[2]);
        tempMatrix.compose(tempPosition, tempQuaternion, tempScale);
        mesh.setMatrixAt(index, tempMatrix);

        if (instance.color || defaultColor) {
            tempColor.set(instance.color || defaultColor);
            mesh.setColorAt(index, tempColor);
        }
    });

    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) {
        mesh.instanceColor.needsUpdate = true;
    }
}

export function useEllipsoidGeometry() {
    return useMemo(() => new THREE.SphereGeometry(1, 16, 12), []);
}
