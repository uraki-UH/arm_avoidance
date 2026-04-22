import { useMemo } from 'react';
import * as THREE from 'three';
import { RobotArmData } from '../../types';

interface RobotArmRendererProps {
    data: RobotArmData | null;
    visible?: boolean;
    segmentRadius?: number;
    jointRadius?: number;
}

const SEGMENT_COLORS = ['#38bdf8', '#22c55e', '#f59e0b', '#ec4899', '#a78bfa', '#f97316'];

export function RobotArmRenderer({
    data,
    visible = true,
    segmentRadius = 0.03,
    jointRadius = 0.045,
}: RobotArmRendererProps) {
    const materialProps = useMemo(() => ({
        segment: { roughness: 0.5, metalness: 0.05 } as const,
        joint: { roughness: 0.4, metalness: 0.08 } as const,
        tip: { roughness: 0.35, metalness: 0.1 } as const,
    }), []);

    if (!data || !visible || !data.positions || data.positions.length < 2) {
        return null;
    }

    const tempUp = new THREE.Vector3(0, 1, 0);

    return (
        <group renderOrder={2}>
            {data.positions.slice(0, -1).map((startArr, index) => {
                const endArr = data.positions[index + 1];
                const start = new THREE.Vector3(startArr[0], startArr[1], startArr[2]);
                const end = new THREE.Vector3(endArr[0], endArr[1], endArr[2]);
                const delta = new THREE.Vector3().subVectors(end, start);
                const length = delta.length();
                if (length < 1e-6) {
                    return null;
                }

                const midpoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
                const quaternion = new THREE.Quaternion().setFromUnitVectors(
                    tempUp,
                    delta.clone().normalize()
                );
                const color = SEGMENT_COLORS[index % SEGMENT_COLORS.length];

                return (
                    <mesh
                        key={`segment-${index}`}
                        position={midpoint}
                        quaternion={quaternion}
                        scale={[segmentRadius, length, segmentRadius]}
                    >
                        <cylinderGeometry args={[1, 1, 1, 12]} />
                        <meshStandardMaterial
                            color={color}
                            roughness={materialProps.segment.roughness}
                            metalness={materialProps.segment.metalness}
                        />
                    </mesh>
                );
            })}

            {data.positions.map((posArr, index) => {
                const pos = new THREE.Vector3(posArr[0], posArr[1], posArr[2]);
                const radius = index === data.positions.length - 1 ? jointRadius * 1.35 : jointRadius;

                return (
                    <mesh
                        key={`joint-${index}`}
                        position={pos}
                        scale={[radius, radius, radius]}
                        renderOrder={3}
                    >
                        <sphereGeometry args={[1, 14, 10]} />
                        <meshStandardMaterial
                            color={index === data.positions.length - 1 ? '#f97316' : '#e2e8f0'}
                            roughness={index === data.positions.length - 1 ? materialProps.tip.roughness : materialProps.joint.roughness}
                            metalness={index === data.positions.length - 1 ? materialProps.tip.metalness : materialProps.joint.metalness}
                        />
                    </mesh>
                );
            })}
        </group>
    );
}
