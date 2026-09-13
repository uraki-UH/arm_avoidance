import { memo, useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import { createPortal, useThree } from '@react-three/fiber';
import URDFLoader from 'urdf-loader';
import { apply_robot_appearance } from './robot_link_appearance';

const empty_link_colors: Record<string, string> = {};
import { RobotData, RobotPoseInstance, RobotSettings, Transform } from '../../types';
import { DisplayFrame, useDemandUpdate } from './SharedRenderers';

interface RobotRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
    color?: string;
    useUrdfColors?: boolean;
    link_colors?: Record<string, string>;
    link_appearance?: RobotSettings['link_appearance'];
    emissiveIntensity?: number;
    opacity?: number;
    jointValuesOverride?: number[];
    tf?: { pos: number[]; quat: number[] } | null;
    manualTransform?: Transform;
    showManipulabilityEllipsoid?: boolean;
    manipEllipsoidType?: 'translational' | 'rotational' | 'both';
    manipLinkName?: string;
    onManipClick?: (linkName: string) => void;
}

type MeshLoadFunction = URDFLoader['loadMeshCb'];

const meshObjectCache = new Map<string, Promise<THREE.Object3D>>();

function cloneMeshObject(source: THREE.Object3D): THREE.Object3D {
    const clone = source.clone(true);
    clone.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
            const mesh = child as THREE.Mesh;
            mesh.material = Array.isArray(mesh.material)
                ? mesh.material.map((material) => material.clone())
                : mesh.material.clone();
        }
    });
    return clone;
}

function loadCachedMesh(
    path: string,
    manager: THREE.LoadingManager,
    loadMesh: MeshLoadFunction,
    onComplete: Parameters<MeshLoadFunction>[2],
): void {
    let pending = meshObjectCache.get(path);
    if (!pending) {
        pending = new Promise<THREE.Object3D>((resolve, reject) => {
            loadMesh(path, manager, (obj, err) => {
                if (err) {
                    reject(err);
                } else {
                    resolve(obj);
                }
            });
        });
        meshObjectCache.set(path, pending);
    }

    pending.then(
        (source) => onComplete(cloneMeshObject(source)),
        (err: Error) => {
            if (meshObjectCache.get(path) === pending) {
                meshObjectCache.delete(path);
            }
            onComplete(new THREE.Object3D(), err);
        },
    );
}

function RobotInstanceRenderer({
    tag,
    data,
    visible = true,
    color = 'blue',
    useUrdfColors = true,
    link_colors = empty_link_colors,
    link_appearance,
    emissiveIntensity = 0.2,
    opacity = 1,
    jointValuesOverride = [],
    tf = null,
    manualTransform,
    showManipulabilityEllipsoid = false,
    manipEllipsoidType = 'translational',
    manipLinkName = '',
    onManipClick,
}: RobotRendererProps) {
    const manipDisplayScale = 0.25;
    const [robot, setRobot] = useState<any>(null);
    const lastLoadSignatureRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);
    const invalidateFrameRef = useRef<number | null>(null);
    const mountedRef = useRef(true);
    const { invalidate } = useThree();

    const viewerPort = 9001;
    const effectiveOpacity = data.opacity ?? opacity;
    const manipGeometry = useMemo(() => new THREE.SphereGeometry(1, 16, 12), []);
    const translationalMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: new THREE.Color('#7fd9ff'),
        emissive: new THREE.Color('#7fd9ff'),
        emissiveIntensity: 0.35,
        transparent: true,
        opacity: Math.max(0.15, Math.min(0.55, effectiveOpacity * 0.35)),
        roughness: 0.35,
        metalness: 0.0,
        toneMapped: false,
    }), [effectiveOpacity]);

    const rotationalMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: new THREE.Color('#ff7f50'),
        emissive: new THREE.Color('#ff7f50'),
        emissiveIntensity: 0.35,
        transparent: true,
        opacity: Math.max(0.15, Math.min(0.55, effectiveOpacity * 0.35)),
        roughness: 0.35,
        metalness: 0.0,
        toneMapped: false,
    }), [effectiveOpacity]);

    const selectedManipLinkName = useMemo(() => {
        return manipLinkName || data.linkNames?.[data.linkNames.length - 1] || data.linkManipulabilities?.[data.linkManipulabilities.length - 1]?.linkName || '';
    }, [data.linkManipulabilities, data.linkNames, manipLinkName]);

    const selectedManipFrame = useMemo(() => {
        if (!robot || !selectedManipLinkName) return null;
        return robot.links?.[selectedManipLinkName] || robot.getFrame?.(selectedManipLinkName) || null;
    }, [robot, selectedManipLinkName]);

    const selectedManipInfo = useMemo(() => {
        if (!showManipulabilityEllipsoid) return null;
        const fromLinks = selectedManipLinkName
            ? data.linkManipulabilities?.find((entry) => entry.linkName === selectedManipLinkName)
            : undefined;
        if (manipLinkName && !fromLinks) return [];

        const transValid = fromLinks?.manipValid ?? data.manipValid;
        const rotValid = fromLinks?.rotationalManipValid ?? data.rotationalManipValid;

        const trans = transValid ? {
            valid: true,
            center: fromLinks?.manipCenter || data.manipCenter || [0, 0, 0],
            scale: fromLinks?.manipScale || data.manipScale,
            orientation: fromLinks?.manipOrientation || data.manipOrientation || [0, 0, 0, 1],
            material: translationalMaterial,
            linkAnchored: Boolean(fromLinks),
            key: `trans-${selectedManipLinkName}-${(fromLinks?.manipScale || data.manipScale)?.join(',')}`,
        } : null;

        const rot = rotValid ? {
            valid: true,
            center: fromLinks?.rotationalManipCenter || data.rotationalManipCenter || fromLinks?.manipCenter || data.manipCenter || [0, 0, 0],
            scale: fromLinks?.rotationalManipScale || data.rotationalManipScale,
            orientation: fromLinks?.rotationalManipOrientation || data.rotationalManipOrientation || [0, 0, 0, 1],
            material: rotationalMaterial,
            linkAnchored: Boolean(fromLinks),
            key: `rot-${selectedManipLinkName}-${(fromLinks?.rotationalManipScale || data.rotationalManipScale)?.join(',')}`,
        } : null;

        const type = manipEllipsoidType ?? 'translational';
        const list: Array<{
            key: string;
            center: [number, number, number];
            scale: [number, number, number];
            orientation: [number, number, number, number];
            material: THREE.Material;
            linkAnchored: boolean;
        }> = [];

        if ((type === 'translational' || type === 'both') && trans && trans.scale) {
            list.push(trans as any);
        }
        if ((type === 'rotational' || type === 'both') && rot && rot.scale) {
            list.push(rot as any);
        }
        return list;
    }, [
        data.linkManipulabilities,
        data.manipCenter,
        data.manipOrientation,
        data.manipScale,
        data.manipValid,
        data.rotationalManipOrientation,
        data.rotationalManipCenter,
        data.rotationalManipScale,
        data.rotationalManipValid,
        manipLinkName,
        manipEllipsoidType,
        selectedManipLinkName,
        showManipulabilityEllipsoid,
        translationalMaterial,
        rotationalMaterial,
    ]);

    // 描画要求モードでの色・姿勢変更の反映
    useDemandUpdate([robot, data, visible, color, useUrdfColors, link_colors, link_appearance, emissiveIntensity, effectiveOpacity, tf, jointValuesOverride, showManipulabilityEllipsoid, manipLinkName]);

    const apply_appearance = useCallback((object: THREE.Object3D) => {
        apply_robot_appearance(object, link_colors, useUrdfColors, color, effectiveOpacity, emissiveIntensity, link_appearance);
    }, [link_colors, link_appearance, useUrdfColors, color, effectiveOpacity, emissiveIntensity]);

    const applyCurrentAppearanceRef = useRef(apply_appearance);
    applyCurrentAppearanceRef.current = apply_appearance;

    const scheduleInvalidate = useCallback(() => {
        if (!mountedRef.current || invalidateFrameRef.current !== null) return;
        invalidateFrameRef.current = window.requestAnimationFrame(() => {
            invalidateFrameRef.current = null;
            invalidate();
        });
    }, [invalidate]);

    useEffect(() => {
        mountedRef.current = true;
        return () => {
            mountedRef.current = false;
            if (invalidateFrameRef.current !== null) {
                window.cancelAnimationFrame(invalidateFrameRef.current);
                invalidateFrameRef.current = null;
            }
        };
    }, []);

    useEffect(() => {
        if (!robot) return;
        apply_appearance(robot);
        scheduleInvalidate();
    }, [robot, apply_appearance, scheduleInvalidate]);

    // --- Load URDF ---
    useEffect(() => {
        if (!data?.urdf) return;
        const loadSignature = data.urdf;
        if (loadSignature === lastLoadSignatureRef.current) return;
        lastLoadSignatureRef.current = loadSignature;

        const urdfLoader = new URDFLoader();
        urdfLoader.packages = (pkg) => `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;
        const defaultLoadMeshCb = urdfLoader.loadMeshCb.bind(urdfLoader);
        urdfLoader.loadMeshCb = (path, manager, onComplete) => {
            loadCachedMesh(path, manager, defaultLoadMeshCb, (obj, err) => {
                onComplete(obj, err);
                if (!err && obj) {
                    applyCurrentAppearanceRef.current(obj);
                    scheduleInvalidate();
                }
            });
        };

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            applyCurrentAppearanceRef.current(robotObj);
            lastJointSignatureRef.current = null;
            setRobot(robotObj);
        } catch (err) {
            console.error("Failed to parse URDF:", err);
            lastLoadSignatureRef.current = null;
        }
    }, [data?.urdf, scheduleInvalidate, tag]);

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

    if (!visible || !robot) return null;

    return (
        <DisplayFrame name={tag} tf={tf ?? { pos: data.basePosition ?? [0, 0, 0],
            quat: data.baseOrientation ?? [0, 0, 0, 1] }} manual_transform={manualTransform}>
            {robot && <primitive key={tag} object={robot} />}
            {showManipulabilityEllipsoid && selectedManipInfo && selectedManipInfo.map((info) => {
                const scaleVec: [number, number, number] = [
                    info.scale[0] * manipDisplayScale,
                    info.scale[1] * manipDisplayScale,
                    info.scale[2] * manipDisplayScale,
                ];
                const mesh = (
                    <mesh
                        key={info.key}
                        geometry={manipGeometry}
                        material={info.material}
                        position={info.linkAnchored ? [0, 0, 0] : info.center}
                        quaternion={new THREE.Quaternion(
                            info.orientation?.[0] ?? 0,
                            info.orientation?.[1] ?? 0,
                            info.orientation?.[2] ?? 0,
                            info.orientation?.[3] ?? 1
                        )}
                        scale={scaleVec}
                        frustumCulled={false}
                        onClick={(e) => {
                            e.stopPropagation();
                            onManipClick?.(selectedManipLinkName);
                        }}
                    />
                );
                return info.linkAnchored && selectedManipFrame
                    ? createPortal(mesh, selectedManipFrame)
                    : mesh;
            })}
        </DisplayFrame>
    );
}

function RobotRenderer({
    tag,
    data,
    visible = true,
    color = 'blue',
    useUrdfColors = true,
    link_colors = empty_link_colors,
    link_appearance,
    emissiveIntensity = 0.2,
    opacity = 1,
    jointValuesOverride = [],
    tf = null,
    manualTransform,
    showManipulabilityEllipsoid = false,
    manipEllipsoidType = 'translational',
    manipLinkName = '',
    onManipClick,
}: RobotRendererProps) {
    const enable_urdf_colors = /(^|[/_-])candidate(?:[/_-]|$)/i.test(tag) || useUrdfColors;
    const hasInstances = Array.isArray(data.instances) && data.instances.length > 0;

    useDemandUpdate([data, visible, color, useUrdfColors, link_colors, link_appearance, emissiveIntensity, opacity, tf, jointValuesOverride, manualTransform, showManipulabilityEllipsoid, manipEllipsoidType]);

    if (hasInstances) {
        const instances = data.instances as RobotPoseInstance[];
        return (
            <DisplayFrame name={tag} tf={tf} manual_transform={manualTransform} is_visible={visible}>
                {instances.map((instance, index) => {
                    const instanceData: RobotData = {
                        ...data,
                        ...instance,
                        instances: undefined,
                        basePosition: [0, 0, 0],
                        baseOrientation: [0, 0, 0, 1],
                        opacity: instance.opacity ?? data.opacity ?? opacity,
                    };
                    return (
                        <RobotInstanceRenderer
                            key={`${tag}-${index}`}
                            tag={`${tag}-${index}`}
                            data={instanceData}
                            visible={visible}
                            color={color}
                            useUrdfColors={enable_urdf_colors}
                            link_colors={link_colors} link_appearance={link_appearance}
                            emissiveIntensity={emissiveIntensity}
                            opacity={instance.opacity ?? data.opacity ?? opacity}
                            jointValuesOverride={jointValuesOverride}
                            tf={null}
                            manualTransform={undefined}
                            showManipulabilityEllipsoid={showManipulabilityEllipsoid}
                            manipEllipsoidType={manipEllipsoidType}
                            manipLinkName={manipLinkName}
                            onManipClick={onManipClick}
                        />
                    );
                })}
            </DisplayFrame>
        );
    }

    return (
        <RobotInstanceRenderer
            tag={tag}
            data={data}
            visible={visible}
            color={color}
            useUrdfColors={enable_urdf_colors}
            link_colors={link_colors} link_appearance={link_appearance}
            emissiveIntensity={emissiveIntensity}
            opacity={opacity}
            jointValuesOverride={jointValuesOverride}
            tf={tf}
            manualTransform={manualTransform}
            showManipulabilityEllipsoid={showManipulabilityEllipsoid}
            manipEllipsoidType={manipEllipsoidType}
            manipLinkName={manipLinkName}
            onManipClick={onManipClick}
        />
    );
}

export const RobotRendererMemo = memo(RobotRenderer);
export { RobotRendererMemo as RobotRenderer };
export default RobotRendererMemo;
