import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { TransformControls } from '@react-three/drei';
import { useThree } from '@react-three/fiber';

interface EditAabbToolProps {
    enabled: boolean;
    center: [number, number, number];
    size: [number, number, number];
    mode: 'translate' | 'scale';
    onChange: (center: [number, number, number], size: [number, number, number]) => void;
}

export function EditAabbTool({
    enabled,
    center,
    size,
    mode,
    onChange,
}: EditAabbToolProps) {
    const groupRef = useRef<THREE.Group>(null);
    const { camera, gl } = useThree();

    useEffect(() => {
        if (!groupRef.current) return;
        groupRef.current.position.set(center[0], center[1], center[2]);
        groupRef.current.scale.set(
            Math.max(0.001, size[0]),
            Math.max(0.001, size[1]),
            Math.max(0.001, size[2])
        );
    }, [center, size]);

    const handleObjectChange = () => {
        if (!groupRef.current) return;
        const nextCenter: [number, number, number] = [
            groupRef.current.position.x,
            groupRef.current.position.y,
            groupRef.current.position.z,
        ];
        const nextSize: [number, number, number] = [
            Math.max(0.001, Math.abs(groupRef.current.scale.x)),
            Math.max(0.001, Math.abs(groupRef.current.scale.y)),
            Math.max(0.001, Math.abs(groupRef.current.scale.z)),
        ];
        onChange(nextCenter, nextSize);
    };

    if (!enabled) return null;

    return (
        <>
            <group ref={groupRef}>
                <mesh>
                    <boxGeometry args={[1, 1, 1]} />
                    <meshBasicMaterial color="#f59e0b" wireframe transparent opacity={0.9} />
                </mesh>
                <mesh>
                    <boxGeometry args={[1, 1, 1]} />
                    <meshBasicMaterial color="#fbbf24" transparent opacity={0.06} depthWrite={false} />
                </mesh>
            </group>

            {groupRef.current && (
                <TransformControls
                    object={groupRef.current}
                    mode={mode}
                    onObjectChange={handleObjectChange}
                    camera={camera}
                    domElement={gl.domElement}
                    size={0.8}
                />
            )}
        </>
    );
}
