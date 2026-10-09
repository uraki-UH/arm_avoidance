import { useMemo, useRef, useLayoutEffect } from 'react';
import * as THREE from 'three';
import { DisplayFrame, useDemandUpdate } from './SharedRenderers';
import { LAYER_COLORS, VoxelSettings, Transform } from '../../types';

interface VoxelLayout {
    voxelSize: number;
    originX?: number;
    originY?: number;
    originZ?: number;
    xShift: number;
    yShift: number;
    zShift: number;
    offset: number;
}

interface VoxelMessage {
    type: 'stream.voxel';
    tag: string;
    data: string[]; // BigInt IDの文字列表現
    labels?: number[];
    layout: VoxelLayout;
    frameId?: string;
}

export const VoxelRenderer = ({ message, settings, tf, manualTransform }: { message: VoxelMessage, settings: VoxelSettings, tf?: { pos: number[]; quat: number[] } | null, manualTransform?: Transform }) => {
    const meshRef = useRef<THREE.InstancedMesh>(null);
    const { data, labels, layout } = message;
    const voxelSize = Math.round(layout.voxelSize * 1000) / 1000;
    const emissiveIntensity = settings?.emissiveIntensity ?? 0.2;
    const hasLabels = Array.isArray(labels) && labels.length === data.length;
    const useLabelColors = settings?.colorMode === 'label' && hasLabels;
    const uniformColor = settings?.color || '#ffff00';

    const instanceColors = useMemo(() => {
        if (!useLabelColors || !labels) return [];
        return labels.map((label) => {
            const rawLabel = Number.isFinite(label) ? Math.trunc(label) : 0;
            const index = ((rawLabel % LAYER_COLORS.length) + LAYER_COLORS.length) % LAYER_COLORS.length;
            return new THREE.Color(LAYER_COLORS[index] || uniformColor);
        });
    }, [labels, uniformColor, useLabelColors]);

    const positions = useMemo(() => {
        const xShift = BigInt(layout.xShift);
        const yShift = BigInt(layout.yShift);
        const zShift = BigInt(layout.zShift);
        const offset = BigInt(layout.offset);
        const mask = (1n << yShift) - 1n;
        const originX = layout.originX ?? 0;
        const originY = layout.originY ?? 0;
        const originZ = layout.originZ ?? 0;

        return data.map(idStr => {
            const id = BigInt(idStr);
            const x = Number((id >> xShift)) - Number(offset);
            const y = Number((id >> yShift) & mask) - Number(offset);
            const z = Number((id >> zShift) & mask) - Number(offset);
            // ボクセルIDが表すセル中心への配置。
            return [
                originX + (x + 0.5) * voxelSize,
                originY + (y + 0.5) * voxelSize,
                originZ + (z + 0.5) * voxelSize,
            ];
        });
    }, [data, layout, voxelSize]);

    useDemandUpdate([positions, instanceColors, settings?.color, settings?.opacity, settings?.wireframe, emissiveIntensity]);

    // 個数の増減だけで描画資源を破棄しないための容量保持。
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, positions.length))));
    capacity_ref.current = capacity;
    const white = useMemo(() => new THREE.Color('white'), []);
    useLayoutEffect(() => {
        if (!meshRef.current) return;
        const dummy = new THREE.Object3D();
        positions.forEach((pos, i) => {
            dummy.position.set(pos[0], pos[1], pos[2]);
            dummy.updateMatrix();
            meshRef.current?.setMatrixAt(i, dummy.matrix);
            meshRef.current?.setColorAt(i, useLabelColors ? instanceColors[i] : white);
        });
        meshRef.current.count = positions.length;
        meshRef.current.instanceMatrix.needsUpdate = true;
        if (meshRef.current.instanceColor) {
            meshRef.current.instanceColor.needsUpdate = true;
        }
    }, [instanceColors, positions, useLabelColors, white, capacity]);

    const displaySize = voxelSize;

    return (
        <DisplayFrame name={message.tag} frame_id={message.frameId} tf={tf} manual_transform={manualTransform}>
            <instancedMesh key={capacity} ref={meshRef} args={[undefined, undefined, capacity]} count={0} frustumCulled={false}>
                <boxGeometry args={[displaySize, displaySize, displaySize]} />
                <meshStandardMaterial
                    color={useLabelColors ? '#ffffff' : uniformColor}
                    emissive={useLabelColors ? '#000000' : uniformColor}
                    emissiveIntensity={emissiveIntensity}
                    vertexColors={useLabelColors}
                    transparent={true}
                    opacity={settings?.opacity ?? 0.6}
                    wireframe={settings?.wireframe ?? true}
                    metalness={0.2}
                    roughness={0.1}
                />
            </instancedMesh>
        </DisplayFrame>
    );
};
