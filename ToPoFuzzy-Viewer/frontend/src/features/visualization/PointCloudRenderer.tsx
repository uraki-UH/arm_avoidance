import { useRef, useMemo, useEffect } from 'react';
import * as THREE from 'three';
import { PointCloudData, HeatmapSettings } from '../../types';
import { heatmapVertexShader, heatmapFragmentShader } from '../../utils/heatmapShaders';
import { useFrame, useThree } from '@react-three/fiber';
import { TransformControls } from '@react-three/drei';

interface PointCloudRendererProps {
    data: PointCloudData;
    heatmapSettings?: HeatmapSettings;
    selected?: boolean;
    transformMode?: 'translate' | 'rotate' | 'scale';
    onTransformChange?: (position: [number, number, number], rotation: [number, number, number], scale: [number, number, number]) => void;
}

export function PointCloudRenderer({
    data,
    heatmapSettings,
    selected = false,
    transformMode = 'translate',
    onTransformChange,
}: PointCloudRendererProps) {
    const pointsRef = useRef<THREE.Points>(null);
    const groupRef = useRef<THREE.Group>(null);
    const transformControlsRef = useRef<any>(null);
    const { camera, gl } = useThree();

    const geometry = useMemo(() => {
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.BufferAttribute(data.points, 3));

        if (data.colors) {
            geom.setAttribute('color', new THREE.BufferAttribute(data.colors, 3));
        } else {
            const colors = new Float32Array(data.count * 3).fill(1);
            geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        }

        // Add intensity attribute for heatmap shader
        if (data.intensities) {
            geom.setAttribute('intensity', new THREE.BufferAttribute(data.intensities, 1));
        } else {
            // Default to 0 if no intensity data
            const intensities = new Float32Array(data.count).fill(0);
            geom.setAttribute('intensity', new THREE.BufferAttribute(intensities, 1));
        }

        geom.computeBoundingSphere();
        return geom;
    }, [data]);

    useEffect(() => {
        return () => {
            geometry.dispose();
        };
    }, [geometry]);

    const material = useMemo(() => {
        const pointSize = heatmapSettings?.pointSize || 0.02;
        const opacity = data.opacity ?? 1;
        const isTransparent = opacity < 1;

        // Check if we should use ShaderMaterial (Height, Distance, Intensity)
        // RGB and Simple modes use standard PointsMaterial
        const isShaderMode = heatmapSettings && ['height', 'distance', 'intensity'].includes(heatmapSettings.mode);

        if (isShaderMode && heatmapSettings) {
            return new THREE.ShaderMaterial({
                vertexShader: heatmapVertexShader,
                fragmentShader: heatmapFragmentShader,
                uniforms: {
                    uMode: {
                        value:
                            heatmapSettings.mode === 'height' ? 1 :
                                heatmapSettings.mode === 'distance' ? 2 :
                                    heatmapSettings.mode === 'intensity' ? 3 : 0
                    },
                    uMin: { value: heatmapSettings.min },
                    uMax: { value: heatmapSettings.max },
                    uColorScheme: {
                        value:
                            heatmapSettings.colorScheme === 'viridis' ? 0 :
                                heatmapSettings.colorScheme === 'plasma' ? 1 :
                                    heatmapSettings.colorScheme === 'magma' ? 2 :
                                        heatmapSettings.colorScheme === 'inferno' ? 3 :
                                            heatmapSettings.colorScheme === 'jet' ? 4 : 5
                    },
                    uCameraPosition: { value: new THREE.Vector3() },
                    uPointSize: { value: pointSize },
                    uOpacity: { value: opacity }
                },
                vertexColors: true,
                opacity,
                transparent: isTransparent,
                depthWrite: !isTransparent,
                depthTest: true,
            });
        } else {
            // Standard material for RGB (original) and Simple modes
            const isSimple = heatmapSettings?.mode === 'simple';
            const simpleColor = heatmapSettings?.simpleColor || '#ffffff';

            return new THREE.PointsMaterial({
                size: pointSize,
                vertexColors: !isSimple, // Use vertex colors only if NOT simple mode
                color: isSimple ? new THREE.Color(simpleColor) : new THREE.Color('#c8ff4a'),
                sizeAttenuation: true, // Consistent with shader implementation
                opacity,
                transparent: isTransparent,
                depthWrite: !isTransparent,
                depthTest: true,
            });
        }
    }, [heatmapSettings, data.opacity]);

    useEffect(() => {
        return () => {
            material.dispose();
        };
    }, [material]);

    useFrame(({ camera }) => {
        if (material instanceof THREE.ShaderMaterial && heatmapSettings?.mode === 'distance') {
            material.uniforms.uCameraPosition.value.copy(camera.position);
        }
    });

    useEffect(() => {
        if (groupRef.current) {
            if (data.position) groupRef.current.position.set(...data.position);
            if (data.rotation) groupRef.current.rotation.set(...data.rotation);
            if (data.scale) groupRef.current.scale.set(...data.scale);
        }
    }, [data.position, data.rotation, data.scale]);

    const handleTransformChange = () => {
        if (groupRef.current && onTransformChange) {
            const pos = groupRef.current.position;
            const rot = groupRef.current.rotation;
            const scale = groupRef.current.scale;

            onTransformChange(
                [pos.x, pos.y, pos.z],
                [rot.x, rot.y, rot.z],
                [scale.x, scale.y, scale.z]
            );
        }
    };

    if (data.visible === false) return null;

    return (
        <>
            <group ref={groupRef}>
                <points ref={pointsRef} geometry={geometry} material={material} />
            </group>

            {selected && groupRef.current && (
                <TransformControls
                    ref={transformControlsRef}
                    object={groupRef.current}
                    mode={transformMode}
                    onObjectChange={handleTransformChange}
                    camera={camera}
                    domElement={gl.domElement}
                    size={0.75}
                />
            )}
        </>
    );
}
