import { useRef, useMemo, useEffect } from 'react';
import * as THREE from 'three';
import { PointCloudData, HeatmapSettings } from '../../types';
import { heatmapVertexShader, heatmapFragmentShader } from '../../utils/heatmapShaders';
import { useFrame, useThree } from '@react-three/fiber';
import { TransformControls } from '@react-three/drei';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';

interface PointCloudRendererProps {
    data: PointCloudData;
    heatmapSettings?: HeatmapSettings;
    selected?: boolean;
    transformMode?: 'translate' | 'rotate' | 'scale';
    onTransformChange?: (position: [number, number, number], rotation: [number, number, number], scale: [number, number, number]) => void;
}

function configureSrgbVertexColors(material: THREE.PointsMaterial) {
    material.toneMapped = false;
    material.onBeforeCompile = (shader) => {
        shader.vertexShader = shader.vertexShader.replace(
            '#include <color_vertex>',
            `#include <color_vertex>
#ifdef USE_COLOR
    vec3 srgbVertexColor = vColor;
    vec3 linearLow = srgbVertexColor / 12.92;
    vec3 linearHigh = pow((srgbVertexColor + 0.055) / 1.055, vec3(2.4));
    vColor = mix(linearLow, linearHigh, step(vec3(0.04045), srgbVertexColor));
#endif`
        );
    };
    material.customProgramCacheKey = () => 'pointcloud-srgb-vertex-colors-v1';
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

    // Trigger re-render in demand mode
    useDemandUpdate([data, heatmapSettings, selected, transformMode]);

    // Geometry is created once and its GPU buffers are reused across streaming
    // updates (updated in place / grown as needed) instead of being disposed
    // and recreated on every incoming frame. Repeatedly allocating/freeing
    // large GPU buffers at ~10Hz puts heavy pressure on the GPU driver and was
    // a contributor to WebGL context loss under sustained streaming.
    const geometry = useMemo(() => new THREE.BufferGeometry(), []);
    const capacityRef = useRef(0);
    const hasColorRef = useRef<boolean | null>(null);
    const hasIntensityRef = useRef<boolean | null>(null);

    useEffect(() => {
        return () => { geometry.dispose(); };
    }, [geometry]);

    useEffect(() => {
        // Hidden layers still receive streamed updates from the parent; skip
        // the GPU upload entirely while not visible instead of churning
        // buffers for something that isn't drawn. The geometry will catch up
        // on the next update once the layer is shown again.
        if (data.visible === false) return;

        const needsColor = !!data.colors;
        const needsIntensity = !!data.intensities;
        const structureChanged = needsColor !== hasColorRef.current || needsIntensity !== hasIntensityRef.current;

        if (data.count > capacityRef.current || structureChanged) {
            // Grow with headroom so we don't reallocate on every small increase.
            const capacity = Math.max(data.count, Math.ceil(capacityRef.current * 1.5));

            const positions = new Float32Array(capacity * 3);
            positions.set(data.points);
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));

            const colors = new Float32Array(capacity * 3).fill(1);
            if (data.colors) colors.set(data.colors);
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

            const intensities = new Float32Array(capacity);
            if (data.intensities) intensities.set(data.intensities);
            geometry.setAttribute('intensity', new THREE.BufferAttribute(intensities, 1));

            capacityRef.current = capacity;
            hasColorRef.current = needsColor;
            hasIntensityRef.current = needsIntensity;
        } else {
            const positionAttr = geometry.getAttribute('position') as THREE.BufferAttribute;
            (positionAttr.array as Float32Array).set(data.points);
            positionAttr.needsUpdate = true;

            const colorAttr = geometry.getAttribute('color') as THREE.BufferAttribute;
            if (data.colors) {
                (colorAttr.array as Float32Array).set(data.colors);
            } else {
                (colorAttr.array as Float32Array).fill(1, 0, data.count * 3);
            }
            colorAttr.needsUpdate = true;

            const intensityAttr = geometry.getAttribute('intensity') as THREE.BufferAttribute;
            if (data.intensities) {
                (intensityAttr.array as Float32Array).set(data.intensities);
            } else {
                (intensityAttr.array as Float32Array).fill(0, 0, data.count);
            }
            intensityAttr.needsUpdate = true;
        }

        geometry.setDrawRange(0, data.count);
        geometry.computeBoundingSphere();
    }, [data, geometry]);

    const material = useMemo(() => {
        const pointSize = heatmapSettings?.pointSize || 0.02;
        const opacity = data.opacity ?? 1;
        const isTransparent = opacity < 1;
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
            const isSimple = heatmapSettings?.mode === 'simple';
            const simpleColor = heatmapSettings?.simpleColor || '#ffffff';

            const pointsMaterial = new THREE.PointsMaterial({
                size: pointSize,
                vertexColors: !isSimple,
                color: isSimple ? new THREE.Color(simpleColor) : new THREE.Color('#ffffff'),
                sizeAttenuation: true,
                opacity,
                transparent: isTransparent,
                depthWrite: !isTransparent,
                depthTest: true,
            });
            if (!isSimple) {
                configureSrgbVertexColors(pointsMaterial);
            }
            return pointsMaterial;
        }
    }, [heatmapSettings, data.opacity]);

    useEffect(() => {
        return () => { material.dispose(); };
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
            onTransformChange([pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z], [scale.x, scale.y, scale.z]);
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
