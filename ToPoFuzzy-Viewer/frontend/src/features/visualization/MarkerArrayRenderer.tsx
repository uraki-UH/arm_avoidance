import { ReactNode, useEffect, useLayoutEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { MarkerArrayData, MarkerMessage, Transform } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { useArrowSettings } from './arrows/settings';
import { ArrowBatch } from './arrows/ArrowBatch';
import { marker_arrow_batches, marker_color } from './arrows/marker_input';

interface MarkerArrayRendererProps {
    tag: string;
    data: MarkerArrayData;
    visible?: boolean;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    manualTransform?: Transform;
}

const MARKER_RENDER_ORDER = 1000;

const is_delete_action = (marker: MarkerMessage) => marker.action === 2 || marker.action === 3;

function MarkerFrame({
    marker,
    transforms,
    manualTransform,
    allow_untransformed,
    children,
}: {
    marker: MarkerMessage;
    transforms: Record<string, { pos: number[]; quat: number[] }>;
    manualTransform: Transform;
    allow_untransformed: boolean;
    children: ReactNode;
}) {
    const frameId = marker.frameId || 'world';
    const tf = frameId === 'world' ? null : (transforms[frameId] ?? null);

    // 候補PoseはTF不明時に非表示。通常Markerの既存フォールバックは維持
    if (!allow_untransformed && (!marker.frameId || (frameId !== 'world' && !tf))) return null;

    return (
        <group position={tf ? [tf.pos[0], tf.pos[1], tf.pos[2]] : [0, 0, 0]}
            quaternion={tf ? [tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]] : [0, 0, 0, 1]}>
            <group
                position={manualTransform.position}
                rotation={manualTransform.rotation}
                scale={manualTransform.scale}
            >
                {/* 矢印の姿勢は入力変換済み。通常Markerの姿勢は共通フレームで適用 */}
                {marker.type === 'arrow' ? children :
                    <group position={marker.pos ?? [0, 0, 0]} quaternion={marker.quat ?? [0, 0, 0, 1]}>{children}</group>}
            </group>
        </group>
    );
}

function ListMarker({ marker }: { marker: MarkerMessage }) {
    const { invalidate } = useThree();
    const { color, opacity } = useMemo(() => marker_color(marker.color), [marker.color]);
    const pts = useMemo(() => marker.points || [], [marker.points]);
    const pointsLen = pts.length;
    
    const isCube = marker.type === 'cube_list';
    // 個数の揺れによるInstancedMesh再生成の抑止。容量不足時のみ倍増。
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, pointsLen))));
    capacity_ref.current = capacity;
    
    const lineGeometry = useMemo(() => {
        if (!isCube || pointsLen === 0) return null;
        const sx = Math.max(0.0001, marker.scale?.[0] || 0.02);
        const sy = Math.max(0.0001, marker.scale?.[1] || 0.02);
        const sz = Math.max(0.0001, marker.scale?.[2] || 0.02);
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
        transparent: true, depthTest: false, depthWrite: false,
    }), []);

    const meshGeometry = useMemo(() => isCube ? null : new THREE.SphereGeometry(0.5, 12, 8), [isCube]);
    const meshMaterial = useMemo(() => isCube ? null : new THREE.MeshLambertMaterial({
        transparent: true, depthTest: false, depthWrite: false,
    }), [isCube]);

    useLayoutEffect(() => {
        for (const material of [lineMaterial, meshMaterial]) {
            if (!material) continue;
            material.color.copy(color);
            material.opacity = opacity;
        }
        invalidate();
    }, [lineMaterial, meshMaterial, color, opacity, invalidate]);

    const instRef = useRef<THREE.InstancedMesh>(null);
    useLayoutEffect(() => {
        if (isCube || !instRef.current) return;
        const dummy = new THREE.Object3D();
        // SPHERE_LISTの直径は先頭の有効なscale値で統一
        const diameter = Math.max(
            0.0001,
            marker.scale?.[0] || marker.scale?.[1] || marker.scale?.[2] || 0.02,
        );
        pts.forEach((pt, index) => {
            dummy.position.set(pt[0], pt[1], pt[2]);
            dummy.scale.set(diameter, diameter, diameter);
            dummy.updateMatrix();
            instRef.current?.setMatrixAt(index, dummy.matrix);
        });
        instRef.current.count = pointsLen;
        instRef.current.instanceMatrix.needsUpdate = true;
        instRef.current.computeBoundingSphere();
        invalidate();
        // 個数・材質変更によるメッシュ再生成時も描画前に行列を初期化
    }, [isCube, pts, marker.scale, pointsLen, meshGeometry, meshMaterial, capacity, invalidate]);

    // 各リソース自身の交換・アンマウント時のみ解放。生存中geometryの巻き込み破棄防止。
    useEffect(() => () => lineMaterial.dispose(), [lineMaterial]);
    useEffect(() => () => lineGeometry?.dispose(), [lineGeometry]);
    useEffect(() => () => meshMaterial?.dispose(), [meshMaterial]);
    useEffect(() => () => meshGeometry?.dispose(), [meshGeometry]);

    if (isCube) {
        if (!lineGeometry) return null;
        return (
            <lineSegments
                geometry={lineGeometry}
                material={lineMaterial}
                renderOrder={MARKER_RENDER_ORDER}
            />
        );
    }

    return (
        <instancedMesh
            key={capacity}
            ref={instRef}
            args={[meshGeometry!, meshMaterial!, capacity]}
            // 行列初期化前の単位サイズ球の描画防止
            count={0}
            renderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function MarkerPrimitive({ marker }: { marker: MarkerMessage }) {
    const { color, opacity } = useMemo(() => marker_color(marker.color), [marker.color]);
    const isCube = marker.type === 'cube';

    const geometry = useMemo(() => {
        if (marker.type === 'sphere') return new THREE.SphereGeometry(0.5, 16, 12);
        if (marker.type === 'cylinder') return new THREE.CylinderGeometry(0.5, 0.5, 1, 16);
        return new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
    }, [marker.type]);

    const material = useMemo(() => {
        if (isCube) return new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false });
        return new THREE.MeshLambertMaterial({ color, transparent: true, opacity, depthTest: false, depthWrite: false });
    }, [isCube, color, opacity]);

    useEffect(() => () => material.dispose(), [material]);
    useEffect(() => () => geometry.dispose(), [geometry]);

    const scale: [number, number, number] = [
        Math.max(0.0001, marker.scale?.[0] || 1),
        Math.max(0.0001, marker.scale?.[1] || 1),
        Math.max(0.0001, marker.scale?.[2] || 1),
    ];

    const object = useMemo(() => isCube ? new THREE.LineSegments(geometry, material as THREE.LineBasicMaterial)
        : new THREE.Mesh(geometry, material), [isCube, geometry, material]);
    return <primitive object={object} scale={scale} renderOrder={MARKER_RENDER_ORDER} />;
}

function LineMarker({ marker, strip }: { marker: MarkerMessage; strip: boolean }) {
    const { invalidate } = useThree();
    const { color, opacity } = useMemo(() => marker_color(marker.color), [marker.color]);
    
    const material = useMemo(() => new THREE.LineBasicMaterial({
        transparent: true, depthTest: false, depthWrite: false,
    }), []);

    const points = marker.points;
    const point_num = points?.length ?? 0;
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, point_num))));
    capacity_ref.current = capacity;

    const geometry = useMemo(() => {
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3).setUsage(THREE.DynamicDrawUsage));
        geom.setDrawRange(0, 0);
        return geom;
    }, [capacity]);

    useLayoutEffect(() => {
        const positions = geometry.getAttribute('position') as THREE.BufferAttribute;
        points?.forEach((point, idx) => positions.setXYZ(idx, point[0], point[1], point[2]));
        positions.clearUpdateRanges();
        if (point_num > 0) positions.addUpdateRange(0, point_num * 3);
        positions.needsUpdate = true;
        geometry.setDrawRange(0, point_num);
        geometry.computeBoundingSphere();
        material.color.copy(color);
        material.opacity = opacity;
        invalidate();
    }, [geometry, points, point_num, material, color, opacity, invalidate]);

    const lineObject = useMemo(() => strip ? new THREE.Line(geometry, material) : new THREE.LineSegments(geometry, material),
        [strip, geometry, material]);

    useEffect(() => () => material.dispose(), [material]);
    useEffect(() => () => geometry.dispose(), [geometry]);

    return (
        <primitive
            object={lineObject}
            renderOrder={MARKER_RENDER_ORDER}
        />
    );
}

function renderMarker(marker: MarkerMessage) {
    if (is_delete_action(marker)) return null;

    switch (marker.type) {
    case 'cube':
    case 'sphere':
    case 'cylinder':
        return <MarkerPrimitive marker={marker} />;
    case 'line_strip':
    case 'line_list':
        return <LineMarker marker={marker} strip={marker.type === 'line_strip'} />;
    case 'cube_list':
    case 'sphere_list':
        return <ListMarker marker={marker} />;
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
    const effective_style = useArrowSettings(tag);
    const arrow_batches = useMemo(() => marker_arrow_batches(data, effective_style), [data, effective_style]);
    const transform: Transform = manualTransform || {
        position: [0, 0, 0],
        rotation: [0, 0, 0],
        scale: [1, 1, 1],
    };

    useDemandUpdate([tag, data, visible, transforms, manualTransform, effective_style]);

    if (!visible || data.visible === false || data.markers.length === 0) return null;

    return (
        <group name={`${tag}-markers`}>
            {arrow_batches.map(([key, batch]) => <MarkerFrame key={key} marker={batch.marker}
                transforms={transforms} manualTransform={transform} allow_untransformed={data.source_type !== 'pose_array'}>
                <ArrowBatch samples={batch.samples} style={batch.style} />
            </MarkerFrame>)}
            {data.markers.filter(marker => marker.type !== 'arrow').map(marker => <MarkerFrame
                key={`${marker.ns}:${marker.id}`} marker={marker} transforms={transforms}
                manualTransform={transform} allow_untransformed={data.source_type !== 'pose_array'}>
                {renderMarker(marker)}
            </MarkerFrame>)}
        </group>
    );
}
