import { useLayoutEffect, useMemo, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { arrow_sample, arrow_style, build_arrow_parts, resolve_arrow_style } from './geometry';

// 全矢印共通の円柱・円錐インスタンス描画。方向だけの入力には補助軸の生成なし
export function ArrowBatch({ samples, style }: { samples: arrow_sample[]; style?: Partial<arrow_style> }) {
    const shaft_ref = useRef<THREE.InstancedMesh>(null);
    const head_ref = useRef<THREE.InstancedMesh>(null);
    const { invalidate } = useThree();
    const resolved = useMemo(() => resolve_arrow_style(style), [style]);
    const parts = useMemo(() => samples.flatMap(sample => build_arrow_parts(sample, resolved)), [samples, resolved]);
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, parts.length))));
    capacity_ref.current = capacity;
    useLayoutEffect(() => {
        const shaft = shaft_ref.current;
        const head = head_ref.current;
        if (!shaft || !head) return;
        shaft.count = head.count = parts.length;
        parts.forEach((part, idx) => {
            shaft.setMatrixAt(idx, part.shaft);
            head.setMatrixAt(idx, part.head);
            shaft.setColorAt(idx, part.color);
            head.setColorAt(idx, part.color);
        });
        for (const mesh of [shaft, head]) {
            mesh.instanceMatrix.needsUpdate = true;
            if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        }
        invalidate();
    }, [parts, invalidate]);
    const is_overlay = resolved.depth_mode === 'overlay';
    return <group name="common-arrows">
        <instancedMesh key={`shaft-${capacity}`} ref={shaft_ref} args={[undefined, undefined, capacity]}
            frustumCulled={false} renderOrder={is_overlay ? 1000 : 0}>
            <cylinderGeometry args={[0.5, 0.5, 1, 8]} />
            <meshBasicMaterial color="white" opacity={resolved.opacity} transparent={resolved.opacity < 1}
                depthTest={!is_overlay} depthWrite={!is_overlay && resolved.opacity === 1} toneMapped={false} />
        </instancedMesh>
        <instancedMesh key={`head-${capacity}`} ref={head_ref} args={[undefined, undefined, capacity]}
            frustumCulled={false} renderOrder={is_overlay ? 1000 : 0}>
            <coneGeometry args={[0.5, 1, 8]} />
            <meshBasicMaterial color="white" opacity={resolved.opacity} transparent={resolved.opacity < 1}
                depthTest={!is_overlay} depthWrite={!is_overlay && resolved.opacity === 1} toneMapped={false} />
        </instancedMesh>
    </group>;
}
