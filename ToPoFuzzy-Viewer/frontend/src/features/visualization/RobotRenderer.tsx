import { memo, useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import URDFLoader from 'urdf-loader';
import { RobotData, RobotPoseInstance, Transform } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';

interface RobotRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
    color?: string;
    useUrdfColors?: boolean;
    emissiveIntensity?: number;
    opacity?: number;
    jointValuesOverride?: number[];
    tf?: { pos: number[]; quat: number[] } | null;
    manualTransform?: Transform;
}

function RobotInstanceRenderer({
    tag,
    data,
    visible = true,
    color = 'blue',
    useUrdfColors = true,
    emissiveIntensity = 0.2,
    opacity = 0.8,
    jointValuesOverride = [],
    tf = null,
    manualTransform,
}: RobotRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const lastLoadSignatureRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);
    const { invalidate } = useThree();

    const viewerPort = 9001;
    const effectiveOpacity = data.opacity ?? opacity;

    // Trigger re-render in demand mode
    useDemandUpdate([robot, data, visible, color, useUrdfColors, emissiveIntensity, effectiveOpacity, tf, jointValuesOverride]);

    // --- Memoize Robot Material ---
    const robotMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: new THREE.Color(color),
        emissive: new THREE.Color(color).multiplyScalar(Math.max(0, emissiveIntensity)),
        roughness: 0.7,
        metalness: 0.1,
        transparent: effectiveOpacity < 1,
        opacity: effectiveOpacity,
    }), [color, emissiveIntensity, effectiveOpacity]);

    const applyMaterialTweaks = useCallback((material: THREE.Material | THREE.Material[]) => {
        const applyOne = (m: THREE.Material) => {
            const anyMaterial = m as THREE.Material & {
                transparent?: boolean;
                opacity?: number;
                emissive?: THREE.Color;
                color?: THREE.Color;
            };

            anyMaterial.transparent = effectiveOpacity < 1;
            anyMaterial.opacity = effectiveOpacity;

            if (anyMaterial.emissive && anyMaterial.color) {
                anyMaterial.emissive.copy(anyMaterial.color).multiplyScalar(Math.max(0, emissiveIntensity));
            }

            anyMaterial.needsUpdate = true;
        };

        if (Array.isArray(material)) {
            material.forEach(applyOne);
        } else {
            applyOne(material);
        }
    }, [emissiveIntensity, effectiveOpacity]);

    const applyRobotMaterial = useCallback((obj: THREE.Object3D) => {
        if (!obj) return;
        obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                if (Array.isArray(mesh.material)) {
                    mesh.material = mesh.material.map(() => robotMaterial);
                } else if (mesh.material !== robotMaterial) {
                    mesh.material = robotMaterial;
                }
                applyMaterialTweaks(mesh.material);
                mesh.castShadow = false;
                mesh.receiveShadow = false;
                mesh.renderOrder = 10;
            }
        });
    }, [applyMaterialTweaks, robotMaterial]);

    const applyUrdfAppearanceTweaks = useCallback((obj: THREE.Object3D) => {
        if (!obj) return;
        obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                applyMaterialTweaks(mesh.material);
                mesh.castShadow = false;
                mesh.receiveShadow = false;
                mesh.renderOrder = 10;
            }
        });
    }, [applyMaterialTweaks]);

    useEffect(() => {
        if (!robot) return;
        const retryDelays = [0, 60, 160, 360, 760, 1500, 2500];
        const timers = retryDelays.map((delay) => window.setTimeout(() => {
            if (!useUrdfColors) {
                applyRobotMaterial(robot);
            } else {
                applyUrdfAppearanceTweaks(robot);
            }
            invalidate();
        }, delay));
        return () => {
            timers.forEach((timer) => window.clearTimeout(timer));
        };
    }, [robot, applyRobotMaterial, applyUrdfAppearanceTweaks, useUrdfColors, invalidate]);

    useEffect(() => {
        if (!robot) return;
        if (useUrdfColors) {
            applyUrdfAppearanceTweaks(robot);
        } else {
            applyRobotMaterial(robot);
        }
        invalidate();
    }, [robot, useUrdfColors, effectiveOpacity, emissiveIntensity, applyRobotMaterial, applyUrdfAppearanceTweaks, invalidate]);

    // --- Load URDF ---
    useEffect(() => {
        if (!data?.urdf) return;
        const loadSignature = `${data.urdf}::${useUrdfColors ? 'urdf' : 'robot'}`;
        if (loadSignature === lastLoadSignatureRef.current) return;
        lastLoadSignatureRef.current = loadSignature;

        const urdfLoader = new URDFLoader();
        urdfLoader.packages = (pkg) => `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;
        const defaultLoadMeshCb = urdfLoader.loadMeshCb.bind(urdfLoader);
        urdfLoader.loadMeshCb = (path, manager, onComplete) => {
            defaultLoadMeshCb(path, manager, (obj, err) => {
                if (obj && !useUrdfColors) {
                    applyRobotMaterial(obj);
                }
                onComplete(obj, err);
            });
        };

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            if (!useUrdfColors) {
                applyRobotMaterial(robotObj);
            }
            applyUrdfAppearanceTweaks(robotObj);
            setRobot(robotObj);
        } catch (err) {
            console.error("Failed to parse URDF:", err);
            lastLoadSignatureRef.current = null;
        }
    }, [applyRobotMaterial, applyUrdfAppearanceTweaks, data?.urdf, tag, useUrdfColors]);

    // --- Update Joints ---
    useEffect(() => {
        if (!robot || !data?.jointNames || !data?.jointValues) return;
        const effectiveJointValues = jointValuesOverride.length > 0
            ? jointValuesOverride
            : data.jointValues;
        const signature = effectiveJointValues.map((v) => v.toFixed(4)).join(',');
        if (signature === lastJointSignatureRef.current) return;
        lastJointSignatureRef.current = signature;

        data.jointNames.forEach((name, i) => {
            if (robot.joints[name]) {
                const nextValue = effectiveJointValues[i] ?? data.jointValues[i];
                robot.joints[name].setJointValue(nextValue);
            }
        });
    }, [robot, data?.jointNames, data?.jointValues, jointValuesOverride]);

    const effectiveTransform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };

    // --- TF-based Positioning ---
    useEffect(() => {
        if (!groupRef.current) return;
        if (tf) {
            groupRef.current.position.set(tf.pos[0], tf.pos[1], tf.pos[2]);
            groupRef.current.quaternion.set(tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]);
        } else {
            groupRef.current.position.set(
                data.basePosition?.[0] || 0,
                data.basePosition?.[1] || 0,
                data.basePosition?.[2] || 0
            );
            const orient = data.baseOrientation || [0, 0, 0, 1];
            groupRef.current.quaternion.set(orient[0], orient[1], orient[2], orient[3]);
        }
    }, [tf, data.basePosition, data.baseOrientation]);

    if (!visible || !robot) return null;

    return (
        <group ref={groupRef} name={tag} visible={visible}>
            <group 
                position={effectiveTransform.position} 
                rotation={effectiveTransform.rotation} 
                scale={effectiveTransform.scale}
            >
                {robot && <primitive key={tag} object={robot} />}
            </group>
        </group>
    );
}

function RobotRenderer({
    tag,
    data,
    visible = true,
    color = 'blue',
    useUrdfColors = true,
    emissiveIntensity = 0.2,
    opacity = 0.8,
    jointValuesOverride = [],
    tf = null,
    manualTransform,
}: RobotRendererProps) {
    const outerGroupRef = useRef<THREE.Group>(null);
    const hasInstances = Array.isArray(data.instances) && data.instances.length > 0;
    const effectiveTransform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };

    useDemandUpdate([data, visible, color, useUrdfColors, emissiveIntensity, opacity, tf, jointValuesOverride, manualTransform]);

    useEffect(() => {
        if (!hasInstances || !outerGroupRef.current) return;
        if (tf) {
            outerGroupRef.current.position.set(tf.pos[0], tf.pos[1], tf.pos[2]);
            outerGroupRef.current.quaternion.set(tf.quat[0], tf.quat[1], tf.quat[2], tf.quat[3]);
        } else {
            outerGroupRef.current.position.set(0, 0, 0);
            outerGroupRef.current.quaternion.set(0, 0, 0, 1);
        }
    }, [hasInstances, tf]);

    if (hasInstances) {
        const instances = data.instances as RobotPoseInstance[];
        return (
            <group ref={outerGroupRef} name={tag} visible={visible}>
                <group
                    position={effectiveTransform.position}
                    rotation={effectiveTransform.rotation}
                    scale={effectiveTransform.scale}
                >
                    {instances.map((instance, index) => {
                        const instanceData: RobotData = {
                            ...data,
                            ...instance,
                            instances: undefined,
                            opacity: instance.opacity ?? data.opacity ?? opacity,
                        };
                        return (
                            <RobotInstanceRenderer
                                key={`${tag}-${index}`}
                                tag={`${tag}-${index}`}
                                data={instanceData}
                                visible={visible}
                                color={color}
                                useUrdfColors={useUrdfColors}
                                emissiveIntensity={emissiveIntensity}
                                opacity={instance.opacity ?? data.opacity ?? opacity}
                                jointValuesOverride={jointValuesOverride}
                                tf={null}
                                manualTransform={undefined}
                            />
                        );
                    })}
                </group>
            </group>
        );
    }

    return (
        <RobotInstanceRenderer
            tag={tag}
            data={data}
            visible={visible}
            color={color}
            useUrdfColors={useUrdfColors}
            emissiveIntensity={emissiveIntensity}
            opacity={opacity}
            jointValuesOverride={jointValuesOverride}
            tf={tf}
            manualTransform={manualTransform}
        />
    );
}

export const RobotRendererMemo = memo(RobotRenderer);
export { RobotRendererMemo as RobotRenderer };
export default RobotRendererMemo;
