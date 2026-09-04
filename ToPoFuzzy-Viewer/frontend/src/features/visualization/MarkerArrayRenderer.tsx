import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { MarkerArrayData, MarkerMessage, Transform } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { DirectionalArrow } from './utils/DirectionalArrow';

interface MarkerArrayRendererProps {
    tag: string;
    data: MarkerArrayData;
    visible?: boolean;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    manualTransform?: Transform;
}

const MARKER_RENDER_ORDER = 1000;

const is_delete_action = (marker: MarkerMessage) => marker.action === 2 || marker.action === 3;

const getPose = (marker: any) => {
    const pos = marker.pos || [0, 0, 0];
    const quat = marker.quat || [0, 0, 0, 1];
    
    // Fallback for object format if needed
    const px = pos.x ?? pos[0] ?? 0;
    const py = pos.y ?? pos[1] ?? 0;
    const pz = pos.z ?? pos[2] ?? 0;
    
    const qx = quat.x ?? quat[0] ?? 0;
    const qy = quat.y ?? quat[1] ?? 0;
    const qz = quat.z ?? quat[2] ?? 0;
    const qw = quat.w ?? quat[3] ?? 1;

    const position = [px, py, pz] as [number, number, number];
    const quaternion = new THREE.Quaternion(qx, qy, qz, qw);
    const euler = new THREE.Euler().setFromQuaternion(quaternion);
    const rotation = [euler.x, euler.y, euler.z] as [number, number, number];
    
    return { position, rotation };
};

const getColor = (color: any) => {
    if (Array.isArray(color)) {
        return {
            color: new THREE.Color(color[0], color[1], color[2]),
            opacity: color[3] ?? 1,
            transparent: (color[3] ?? 1) < 1,
        };
    }
    return {
        color: new THREE.Color(color?.r ?? 1, color?.g ?? 1, color?.b ?? 1),
        opacity: color?.a ?? 1,
        transparent: (color?.a ?? 1) < 1,
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

function MarkerFrame({
    marker,
    transforms,
    manualTransform,
}: {
    marker: MarkerMessage;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    manualTransform: Transform;
}) {
    const frameId = marker.frameId || 'world';
    const tf = frameId === 'world' ? null : (transforms[frameId] ?? null);
    const groupRef = useMarkerFrame(tf);

    // TF未受信時はMarker座標をViewer固定座標として扱うフォールバック

    return (
        <group ref={groupRef}>
            <group
                position={manualTransform.position}
                rotation={manualTransform.rotation}
                scale={manualTransform.scale}
            >
                {renderMarker(marker)}
            </group>
        </group>
    );
}

function ListMarker({ marker }: { marker: MarkerMessage }) {
    const { color, opacity } = useMemo(() => getColor(marker.color), [marker.color]);
    const pts = useMemo(() => marker.points || [], [marker.points]);
    const pointsLen = pts.length;
    const { position, rotation } = useMemo(
        () => getPose({ pos: marker.pos, quat: marker.quat }),
        [marker.pos, marker.quat]
    );
    
    const isCube = marker.type === 'cube_list';
    
    const lineGeometry = useMemo(() => {
        if (!isCube || pointsLen === 0) return null;
        const sx = Math.max(0.0001, marker.scale[0] || 0.02);
        const sy = Math.max(0.0001, marker.scale[1] || 0.02);
        const sz = Math.max(0.0001, marker.scale[2] || 0.02);
        const hx = sx / 2, hy = sy / 2, hz = sz / 2;
        
        const positions = new Float32Array(pts.length * 24 * 3);
        let idx = 0;
        for (let i = 0; i < pts.length; i++) {
            const pt = pts[i];
            const px = pt[0], py = pt[1], pz = pt[2];
            // 12 edges without diagonals
            const c = [
                px-hx, py-hy, pz-hz,  px+hx, py-hy, pz-hz,  px+hx, py+hy, pz-hz,  px-hx, py+hy, pz-hz,
                px-hx, py-hy, pz+hz,  px+hx, py-hy, pz+hz,  px+hx, py+hy, pz+hz,  px-hx, py+hy, pz+hz
            ];
            const edges = [0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 3,7];
            for (let e = 0; e < 24; e++) {
                const vi = edges[e] * 3;
                positions[idx++] = c[vi];
                positions[idx++] = c[vi+1];
                positions[idx++] = c[vi+2];
            }
        }
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        return geom;
    }, [isCube, pts, marker.scale, pointsLen]);

    const lineMaterial = useMemo(() => new THREE.LineBasicMaterial({
        color, transparent: true, opacity, depthTest: false, depthWrite: false,
    }), [color, opacity]);

    const meshGeometry = useMemo(() => isCube ? null : new THREE.SphereGeometry(0.5, 12, 8), [isCube]);
    const meshMaterial = useMemo(() => isCube ? null : new THREE.MeshLambertMaterial({
        color, transparent: true, opacity, depthTest: false, depthWrite: false,
    }), [color, opacity, isCube]);

    const instRef = useRef<THREE.InstancedMesh>(null);
    useEffect(() => {
        if (isCube || !instRef.current || pointsLen === 0) return;
        const dummy = new THREE.Object3D();
        const sx = marker.scale[0] || 1, sy = marker.scale[1] || 1, sz = marker.scale[2] || 1;
        pts.forEach((pt, index) => {
            dummy.position.set(pt[0], pt[1], pt[2]);
            dummy.scale.set(sx, sy, sz);
            dummy.updateMatrix();
            instRef.current?.setMatrixAt(index, dummy.matrix);
        });
        instRef.current.count = pointsLen;
        instRef.current.instanceMatrix.needsUpdate = true;
    }, [isCube, pts, marker.scale, pointsLen]);

    useEffect(() => () => {
        lineMaterial.dispose();
        lineGeometry?.dispose();
        meshMaterial?.dispose();
        meshGeometry?.dispose();
    }, [lineMaterial, lineGeometry, meshMaterial, meshGeometry]);

    if (isCube) {
        if (!lineGeometry) return null;
        return (
            <lineSegments
                geometry={lineGeometry}
                material={lineMaterial}
                position={position}
                rotation={rotation}
                renderOrder={MARKER_RENDER_ORDER}
            />
        );
    }

    return (
        <instancedMesh
            key={pointsLen}
            ref={instRef}
            args={[meshGeometry!, meshMaterial!, Math.max(1, pointsLen)]}
            count={pointsLen}
            position={position}
            rotation={rotation}
            renderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function MarkerPrimitive({ marker }: { marker: MarkerMessage }) {
    const { color, opacity } = useMemo(() => getColor(marker.color), [marker.color]);
    const { position, rotation } = useMemo(
        () => getPose({ pos: marker.pos, quat: marker.quat }),
        [marker.pos, marker.quat]
    );
    const isCube = marker.type === 'cube';

    const geometry = useMemo(() => {
        if (marker.type === 'sphere') return new THREE.SphereGeometry(0.5, 16, 12);
        if (marker.type === 'cylinder') return new THREE.CylinderGeometry(0.5, 0.5, 1, 16);
        return isCube ? new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)) : new THREE.BoxGeometry(1, 1, 1);
    }, [marker.type, isCube]);

    const material = useMemo(() => {
        if (isCube) return new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false });
        return new THREE.MeshLambertMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false });
    }, [isCube, color, opacity]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    const scale: [number, number, number] = [
        Math.max(0.0001, marker.scale[0] || 1),
        Math.max(0.0001, marker.scale[1] || 1),
        Math.max(0.0001, marker.scale[2] || 1),
    ];

    if (isCube) {
        return (
            <lineSegments
                geometry={geometry}
                material={material as THREE.LineBasicMaterial}
                position={position}
                rotation={rotation}
                scale={scale}
                renderOrder={MARKER_RENDER_ORDER}
            />
        );
    }

    return (
        <mesh
            geometry={geometry}
            material={material}
            position={position}
            rotation={rotation}
            scale={scale}
            renderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function LineMarker({ marker, strip }: { marker: MarkerMessage; strip: boolean }) {
    const { color, opacity } = useMemo(() => getColor(marker.color), [marker.color]);
    const { position, rotation } = useMemo(
        () => getPose({ pos: marker.pos, quat: marker.quat }),
        [marker.pos, marker.quat]
    );
    
    const material = useMemo(() => new THREE.LineBasicMaterial({
        color, transparent: true, opacity, depthTest: false, depthWrite: false,
    }), [color, opacity]);

    const geometry = useMemo(() => {
        // pts: [number, number, number][] -> Float32Array: [x, y, z, x, y, z, ...]
        const pts = marker.points || [];
        const positions = new Float32Array(pts.length * 3);
        for (let i = 0; i < pts.length; i++) {
            positions[i * 3 + 0] = pts[i][0];
            positions[i * 3 + 1] = pts[i][1];
            positions[i * 3 + 2] = pts[i][2];
        }
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        return geom;
    }, [marker.points]);

    const lineObject = useMemo(() => {
        const obj = strip ? new THREE.Line(geometry, material) : new THREE.LineSegments(geometry, material);
        obj.computeLineDistances();
        return obj;
    }, [strip, geometry, material]);

    useEffect(() => () => {
        material.dispose();
        geometry.dispose();
    }, [material, geometry]);

    return (
        <primitive
            object={lineObject}
            position={position}
            rotation={rotation}
            renderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function ArrowMarker({ marker }: { marker: MarkerMessage }) {
    const { color } = useMemo(() => getColor(marker.color), [marker.color]);
    const { position, direction, lengthScale, maxLength, shaftWidth, head_length, head_width } = useMemo(() => {
        const pos = marker.pos || [0, 0, 0];
        const quat = marker.quat || [0, 0, 0, 1];
        const px = pos[0] ?? 0;
        const py = pos[1] ?? 0;
        const pz = pos[2] ?? 0;

        const qx = quat[0] ?? 0;
        const qy = quat[1] ?? 0;
        const qz = quat[2] ?? 0;
        const qw = quat[3] ?? 1;

        const origin: [number, number, number] = [px, py, pz];
        let dir = new THREE.Vector3(1, 0, 0);

        if ((marker.points || []).length >= 2) {
            const p0 = marker.points[0];
            const p1 = marker.points[1];
            dir = new THREE.Vector3(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
            origin[0] = p0[0];
            origin[1] = p0[1];
            origin[2] = p0[2];
            const length = dir.length();
            const shaft_diameter = marker.scale?.[0] ?? 0;
            const head_diameter = marker.scale?.[1] ?? 0;
            const marker_head_length = marker.scale?.[2] ?? 0;
            return {
                position: origin,
                direction: [dir.x, dir.y, dir.z] as [number, number, number],
                lengthScale: 1.0,
                maxLength: Math.max(0.2, length * 1.5),
                shaftWidth: shaft_diameter > 0 && shaft_diameter < length * 0.5
                    ? shaft_diameter
                    : Math.max(0.003, length * 0.06),
                head_length: marker_head_length > 0
                    ? Math.min(length, marker_head_length)
                    : length * 0.24,
                head_width: head_diameter > 0 ? head_diameter : length * 0.12,
            };
        } else {
            const quaternion = new THREE.Quaternion(qx, qy, qz, qw);
            dir.applyQuaternion(quaternion);
        }

        const scale = Math.max(0.0001, marker.scale?.[0] || 0.15);
        return {
            position: origin,
            direction: [dir.x, dir.y, dir.z] as [number, number, number],
            lengthScale: scale,
            maxLength: Math.max(0.2, scale * 1.5),
            shaftWidth: Math.max(0.005, scale * 0.06),
            head_length: scale * 0.28,
            head_width: scale * 0.18,
        };
    }, [marker.pos, marker.quat, marker.points, marker.scale]);

    return (
        <DirectionalArrow
            origin={position}
            direction={direction}
            lengthScale={lengthScale}
            color={color.getStyle()}
            maxLength={maxLength}
            shaftWidth={shaftWidth}
            head_length={head_length}
            head_width={head_width}
            overlayRenderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function renderMarker(marker: MarkerMessage) {
    if (is_delete_action(marker)) return null;

    switch (marker.type) {
    case 'arrow':
        return <ArrowMarker key={`${marker.ns}:${marker.id}`} marker={marker} />;
    case 'cube':
    case 'sphere':
    case 'cylinder':
        return <MarkerPrimitive key={`${marker.ns}:${marker.id}`} marker={marker} />;
    case 'line_strip':
        return <LineMarker key={`${marker.ns}:${marker.id}`} marker={marker} strip={true} />;
    case 'line_list':
        return <LineMarker key={`${marker.ns}:${marker.id}`} marker={marker} strip={false} />;
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
    transforms,
    manualTransform,
}: MarkerArrayRendererProps) {
    const transform: Transform = manualTransform || {
        position: [0, 0, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
    };

    useDemandUpdate([tag, data, visible, transforms, manualTransform]);

    if (!visible || data.visible === false || data.markers.length === 0) return null;

    return (
        <group name={`${tag}-markers`}>
            {data.markers.map((marker) => (
                <MarkerFrame
                    key={`${marker.ns}:${marker.id}`}
                    marker={marker}
                    transforms={transforms}
                    manualTransform={transform}
                />
            ))}
        </group>
    );
}
