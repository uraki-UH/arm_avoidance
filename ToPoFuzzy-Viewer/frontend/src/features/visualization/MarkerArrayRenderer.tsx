import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { MarkerArrayData, MarkerMessage } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';

interface MarkerArrayRendererProps {
    tag: string;
    data: MarkerArrayData;
    visible?: boolean;
    tf?: { pos: number[]; quat: number[] } | null;
}

const isDeleteAll = (marker: MarkerMessage) => marker.action === 'deleteall';

const toMaterialParams = (marker: MarkerMessage) => {
    const hasExplicitColor = Boolean(marker.color) && (
        (marker.color?.r ?? 0) !== 0 ||
        (marker.color?.g ?? 0) !== 0 ||
        (marker.color?.b ?? 0) !== 0 ||
        (marker.color?.a ?? 0) > 0
    );
    const alpha = hasExplicitColor ? (marker.color?.a ?? 1) : 1;
    return {
        color: hasExplicitColor
            ? new THREE.Color(marker.color?.r ?? 1, marker.color?.g ?? 1, marker.color?.b ?? 1)
            : new THREE.Color(1, 1, 1),
        opacity: alpha,
        transparent: alpha < 1,
    };
};

function useMarkerFrame(tf: { pos: number[]; quat: number[] } | null) {
    const groupRef = useRef<THREE.Group>(null);
    const { invalidate } = useThree();

    useEffect(() => {
        if (!groupRef.current) return;
        if (tf) {
            groupRef.current.position.set(tf.pos[0], tf.pos[1], tf.pos[2]);
            groupRef.current.quaternion.set(tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]);
        } else {
            groupRef.current.position.set(0, 0, 0);
            groupRef.current.quaternion.set(0, 0, 0, 1);
        }
        invalidate();
    }, [tf, invalidate]);

    return groupRef;
}

function LineMarker({
    marker,
    strip,
}: {
    marker: MarkerMessage;
    strip: boolean;
}) {
    const params = useMemo(() => toMaterialParams(marker), [marker]);
    const rotation = useMemo(() => {
        const quaternion = new THREE.Quaternion(
            marker.pose.orientation[0],
            marker.pose.orientation[1],
            marker.pose.orientation[2],
            marker.pose.orientation[3]
        );
        const euler = new THREE.Euler().setFromQuaternion(quaternion);
        return [euler.x, euler.y, euler.z] as [number, number, number];
    }, [marker.pose.orientation]);
    const material = useMemo(() => new THREE.LineBasicMaterial({
        color: params.color,
        transparent: params.transparent,
        opacity: params.opacity,
        depthTest: true,
        depthWrite: false,
    }), [params]);
    const geometry = useMemo(() => {
        const positions = new Float32Array(marker.points.flat());
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geom.computeBoundingSphere();
        return geom;
    }, [marker.points]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    return strip ? (
        <line
            geometry={geometry}
            material={material}
            position={marker.pose.position}
            rotation={rotation}
            scale={marker.scale}
            renderOrder={30}
        />
    ) : (
        <lineSegments
            geometry={geometry}
            material={material}
            position={marker.pose.position}
            rotation={rotation}
            scale={marker.scale}
            renderOrder={30}
        />
    );
}

function PointsMarker({ marker }: { marker: MarkerMessage }) {
    const params = useMemo(() => toMaterialParams(marker), [marker]);
    const rotation = useMemo(() => {
        const quaternion = new THREE.Quaternion(
            marker.pose.orientation[0],
            marker.pose.orientation[1],
            marker.pose.orientation[2],
            marker.pose.orientation[3]
        );
        const euler = new THREE.Euler().setFromQuaternion(quaternion);
        return [euler.x, euler.y, euler.z] as [number, number, number];
    }, [marker.pose.orientation]);
    const material = useMemo(() => new THREE.PointsMaterial({
        color: params.color,
        transparent: params.transparent,
        opacity: params.opacity,
        size: Math.max(0.001, marker.scale[0] || 0.01),
        sizeAttenuation: true,
        depthTest: true,
        depthWrite: false,
    }), [params, marker.scale]);
    const geometry = useMemo(() => {
        const positions = new Float32Array(marker.points.flat());
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geom.computeBoundingSphere();
        return geom;
    }, [marker.points]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    return (
        <points
            geometry={geometry}
            material={material}
            position={marker.pose.position}
            rotation={rotation}
            scale={marker.scale}
            renderOrder={30}
        />
    );
}

function MarkerPrimitive({ marker }: { marker: MarkerMessage }) {
    const params = useMemo(() => toMaterialParams(marker), [marker]);
    const rotation = useMemo(() => {
        const quaternion = new THREE.Quaternion(
            marker.pose.orientation[0],
            marker.pose.orientation[1],
            marker.pose.orientation[2],
            marker.pose.orientation[3]
        );
        const euler = new THREE.Euler().setFromQuaternion(quaternion);
        return [euler.x, euler.y, euler.z] as [number, number, number];
    }, [marker.pose.orientation]);
    const geometry = useMemo(() => {
        if (marker.type === 'sphere') {
            return new THREE.SphereGeometry(0.5, 16, 12);
        }
        if (marker.type === 'cylinder') {
            return new THREE.CylinderGeometry(0.5, 0.5, 1, 16);
        }
        return new THREE.BoxGeometry(1, 1, 1);
    }, [marker.type]);
    const material = useMemo(() => new THREE.MeshBasicMaterial({
        color: params.color,
        transparent: params.transparent,
        opacity: params.opacity,
        wireframe: true,
        depthTest: true,
        depthWrite: false,
    }), [params]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    return (
        <mesh
            geometry={geometry}
            position={marker.pose.position}
            rotation={rotation}
            scale={[
                Math.max(0.0001, marker.scale[0] || 1),
                Math.max(0.0001, marker.scale[1] || 1),
                Math.max(0.0001, marker.scale[2] || 1),
            ]}
            material={material}
            renderOrder={30}
        />
    );
}

function ListMarker({ marker }: { marker: MarkerMessage }) {
    const params = useMemo(() => toMaterialParams(marker), [marker]);
    const geometry = useMemo(() => (
        marker.type === 'sphere_list'
            ? new THREE.SphereGeometry(0.5, 12, 8)
            : new THREE.BoxGeometry(1, 1, 1)
    ), [marker.type]);
    const material = useMemo(() => new THREE.MeshBasicMaterial({
        color: params.color,
        transparent: params.transparent,
        opacity: params.opacity,
        wireframe: true,
        depthTest: true,
        depthWrite: false,
    }), [params]);
    const instRef = useRef<THREE.InstancedMesh>(null);

    useEffect(() => {
        if (!instRef.current) return;
        const dummy = new THREE.Object3D();
        const sx = Math.max(0.0001, marker.scale[0] || 1);
        const sy = Math.max(0.0001, marker.scale[1] || 1);
        const sz = Math.max(0.0001, marker.scale[2] || 1);
        marker.points.forEach((point, index) => {
            dummy.position.set(point[0], point[1], point[2]);
            dummy.scale.set(sx, sy, sz);
            dummy.updateMatrix();
            instRef.current?.setMatrixAt(index, dummy.matrix);
        });
        instRef.current.count = marker.points.length;
        instRef.current.instanceMatrix.needsUpdate = true;
    }, [marker.points, marker.scale]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    return (
        <instancedMesh
            ref={instRef}
            args={[geometry, material, Math.max(1, marker.points.length)]}
            count={marker.points.length}
            renderOrder={30}
        />
    );
}

function renderMarker(marker: MarkerMessage) {
    if (isDeleteAll(marker)) return null;

    switch (marker.type) {
    case 'cube':
    case 'sphere':
    case 'cylinder':
        return <MarkerPrimitive key={`${marker.ns}:${marker.id}`} marker={marker} />;
    case 'line_strip':
        return <LineMarker key={`${marker.ns}:${marker.id}`} marker={marker} strip={true} />;
    case 'line_list':
        return <LineMarker key={`${marker.ns}:${marker.id}`} marker={marker} strip={false} />;
    case 'points':
        return <PointsMarker key={`${marker.ns}:${marker.id}`} marker={marker} />;
    case 'cube_list':
    case 'sphere_list':
        return <ListMarker key={`${marker.ns}:${marker.id}`} marker={marker} />;
    default:
        return null;
    }
}

export function MarkerArrayRenderer({
    tag,
    data,
    visible = true,
    tf = null,
}: MarkerArrayRendererProps) {
    const groupRef = useMarkerFrame(tf);

    useDemandUpdate([tag, data, visible, tf]);

    if (!visible || data.visible === false || data.markers.length === 0) return null;

    return (
        <group ref={groupRef} name={`${tag}-markers`}>
            {data.markers.map(renderMarker)}
        </group>
    );
}
