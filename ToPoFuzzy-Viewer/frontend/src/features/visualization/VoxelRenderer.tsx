import { useMemo, useRef, useEffect } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';
import { VoxelSettings, Transform } from '../../types';

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
    frameId?: string;
}

export const VoxelRenderer = ({ message, settings, tf, manualTransform }: { message: VoxelMessage, settings: VoxelSettings, tf?: { pos: number[]; quat: number[] } | null, manualTransform?: Transform }) => {
    const meshRef = useRef<THREE.InstancedMesh>(null);
    const groupRef = useRef<THREE.Group>(null);
    const { invalidate } = useThree();
    const { data, layout } = message;
    const voxelSize = Math.round(layout.voxelSize * 1000) / 1000;

    // TFおよび手動トランスフォームの適用
    useEffect(() => {
        if (!groupRef.current) return;
        
        // ベース位置をTFまたは原点にリセット
        if (tf) {
            groupRef.current.position.set(tf.pos[0], tf.pos[1], tf.pos[2]);
            groupRef.current.quaternion.set(tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]);
        } else {
            groupRef.current.position.set(0, 0, 0);
            groupRef.current.quaternion.set(0, 0, 0, 1);
        }

        // 手動オフセットの適用（既存の共通仕様）
        if (manualTransform) {
            if (manualTransform.position) {
                groupRef.current.position.x += manualTransform.position[0];
                groupRef.current.position.y += manualTransform.position[1];
                groupRef.current.position.z += manualTransform.position[2];
            }
            if (manualTransform.rotation) {
                const euler = new THREE.Euler(
                    manualTransform.rotation[0] * Math.PI / 180,
                    manualTransform.rotation[1] * Math.PI / 180,
                    manualTransform.rotation[2] * Math.PI / 180
                );
                groupRef.current.quaternion.multiply(new THREE.Quaternion().setFromEuler(euler));
            }
        }

        invalidate();
    }, [tf, manualTransform, invalidate]);

    // ボクセルの復元計算
    const positions = useMemo(() => {
        const xShift = BigInt(layout.xShift);
        const yShift = BigInt(layout.yShift);
        const zShift = BigInt(layout.zShift);
        const offset = BigInt(layout.offset);
        const mask = (1n << yShift) - 1n;

        return data.map(idStr => {
            const id = BigInt(idStr);
            const x = Number((id >> xShift)) - Number(offset);
            const y = Number((id >> yShift) & mask) - Number(offset);
            const z = Number((id >> zShift) & mask) - Number(offset);
            // Voxel IDs represent grid cells, so render at the cell center.
            return [(x + 0.5) * voxelSize, (y + 0.5) * voxelSize, (z + 0.5) * voxelSize];
        });
    }, [data, layout, voxelSize]);

    useDemandUpdate([positions]);

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
            <instancedMesh ref={meshRef} args={[undefined, undefined, positions.length]} frustumCulled={false}>
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
