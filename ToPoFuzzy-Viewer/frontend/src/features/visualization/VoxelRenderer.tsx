import { useMemo, useRef, useEffect } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { VoxelSettings } from '../../types';

interface VoxelLayout {
    voxelSize: number;
    xShift: number;
    yShift: number;
    zShift: number;
    offset: number;
}

interface VoxelMessage {
    type: 'stream.voxel';
    tag: string;
    data: string[]; // BigInt IDs as strings
    layout: VoxelLayout;
}

export const VoxelRenderer = ({ message, settings, tf }: { message: VoxelMessage, settings: VoxelSettings, tf?: { pos: number[]; quat: number[] } | null }) => {
    const meshRef = useRef<THREE.InstancedMesh>(null);
    const groupRef = useRef<THREE.Group>(null);
    const { invalidate } = useThree();
    const { data, layout } = message;
    const voxelSize = layout.voxelSize;

    // TFの適用
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

    // ボクセルの復元計算
    const positions = useMemo(() => {
        const xShift = BigInt(layout.xShift);
        const yShift = BigInt(layout.yShift);
        const zShift = BigInt(layout.zShift);
        const offset = BigInt(layout.offset);

        // y_shiftが各軸のビット幅の間隔として使われている前提の簡易マスク
        // 本来は (1n << bitWidth) - 1n だが、現在の21bit packingに合わせて構成
        const mask = (1n << yShift) - 1n;

        return data.map(idStr => {
            const id = BigInt(idStr);
            const x = Number((id >> xShift)) - Number(offset);
            const y = Number((id >> yShift) & mask) - Number(offset);
            const z = Number((id >> zShift) & mask) - Number(offset);
            return [x * voxelSize, y * voxelSize, z * voxelSize];
        });
    }, [data, layout, voxelSize]);

    // マニュアルでの描画更新指示
    useDemandUpdate([positions]);

    // InstancedMeshの更新
    useEffect(() => {
        if (!meshRef.current) return;

        const dummy = new THREE.Object3D();
        positions.forEach((pos, i) => {
            dummy.position.set(pos[0], pos[1], pos[2]);
            dummy.updateMatrix();
            meshRef.current?.setMatrixAt(i, dummy.matrix);
        });

        meshRef.current.count = positions.length;
        meshRef.current.instanceMatrix.needsUpdate = true;
    }, [positions]);

    const displaySize = voxelSize;

    return (
        <group ref={groupRef}>
            <instancedMesh
                ref={meshRef}
                args={[undefined, undefined, positions.length]}
                frustumCulled={false}
            >
                <boxGeometry args={[displaySize, displaySize, displaySize]} />
                <meshStandardMaterial
                    color={settings?.color || "#00ff88"}
                    transparent={true}
                    opacity={settings?.opacity ?? 0.6}
                    metalness={0.2}
                    roughness={0.1}
                />
            </instancedMesh>
        </group>
    );
};
