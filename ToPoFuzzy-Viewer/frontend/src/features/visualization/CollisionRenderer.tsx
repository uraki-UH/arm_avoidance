import { useContext } from 'react';
import { ViewerEnvironment } from '../../embedding';
import { memo, useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { RobotData, Transform } from '../../types';
import { DisplayFrame, useDemandUpdate } from './SharedRenderers';

interface CollisionRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
    color?: string;
    opacity?: number;
    tf?: { pos: number[]; quat: number[] } | null;
    manualTransform?: Transform;
}

function CollisionRenderer({
    tag,
    data,
    visible = true,
    color = '#ff9f1c',
    opacity = 0.28,
    tf = null,
    manualTransform,
}: CollisionRendererProps) {
    const [robot, setRobot] = useState<any>(null);
    const lastUrdfRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);

    const { mesh_base_url } = useContext(ViewerEnvironment);
    const mesh_url = mesh_base_url || `http://${window.location.hostname}:9001/meshes/`;

    useDemandUpdate([robot, data, visible, color, opacity, tf, manualTransform]);

    const effectiveTransform = manualTransform || { position: [0, 0, 0], rotation: [0, 0, 0], scale: [1, 1, 1] };

    const collisionMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: new THREE.Color(color),
        transparent: opacity < 1,
        opacity,
        wireframe: true,
        depthTest: false,
        depthWrite: false,
    }), [color, opacity]);

    const applyCollisionMaterial = useCallback((obj: THREE.Object3D) => {
        if (!obj) return;
        obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                if (Array.isArray(mesh.material)) {
                    mesh.material = mesh.material.map(() => collisionMaterial);
                } else {
                    mesh.material = collisionMaterial;
                }
                if (Array.isArray(mesh.material)) {
                    mesh.material.forEach((material) => {
                        material.needsUpdate = true;
                    });
                } else {
                    mesh.material.needsUpdate = true;
                }
                mesh.visible = true;
                mesh.castShadow = false;
                mesh.receiveShadow = false;
                mesh.renderOrder = 20;
            }
        });
    }, [collisionMaterial]);

    useEffect(() => {
        if (!robot) return;
        const retryDelays = [0, 80, 220, 520, 1200, 2400];
        const timers = retryDelays.map((delay) => window.setTimeout(() => applyCollisionMaterial(robot), delay));
        return () => {
            timers.forEach((timer) => window.clearTimeout(timer));
        };
    }, [robot, applyCollisionMaterial]);

    useEffect(() => {
        if (!robot) return;
        const interval = window.setInterval(() => applyCollisionMaterial(robot), 600);
        const timeout = window.setTimeout(() => window.clearInterval(interval), 5000);
        return () => {
            window.clearInterval(interval);
            window.clearTimeout(timeout);
        };
    }, [robot, applyCollisionMaterial]);

    useEffect(() => {
        if (!data?.urdf || mesh_url + data.urdf === lastUrdfRef.current) return;
        lastUrdfRef.current = mesh_url + data.urdf;

        const urdfLoader = new URDFLoader();
        urdfLoader.packages = (pkg) => `${mesh_url}${encodeURIComponent(pkg)}`;
        urdfLoader.parseVisual = false;
        urdfLoader.parseCollision = true;
        const defaultLoadMeshCb = urdfLoader.loadMeshCb.bind(urdfLoader);
        urdfLoader.loadMeshCb = (path, manager, onComplete) => {
            defaultLoadMeshCb(path, manager, (obj, err) => {
                if (obj) {
                    applyCollisionMaterial(obj);
                }
                onComplete(obj, err);
            });
        };

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            applyCollisionMaterial(robotObj);
            setRobot(robotObj);
        } catch (err) {
            console.error('Failed to parse URDF collision model:', err);
            lastUrdfRef.current = null;
        }
    }, [data?.urdf, mesh_url, applyCollisionMaterial, tag]);

    useEffect(() => {
        if (!robot || !data?.jointNames || !data?.jointValues) return;
        const signature = data.jointValues.map((v) => v.toFixed(4)).join(',');
        if (signature === lastJointSignatureRef.current) return;
        lastJointSignatureRef.current = signature;

        data.jointNames.forEach((name, i) => {
            if (robot.joints[name]) {
                robot.joints[name].setJointValue(data.jointValues[i]);
            }
        });
    }, [robot, data?.jointNames, data?.jointValues]);

    if (!visible || !robot) return null;

    return (
        <DisplayFrame frame_id={data.frameId} name={`${tag}-collision`} tf={tf} is_visible={visible}
            base_pose={!tf ? { pos: data.basePosition ?? [0, 0, 0], quat: data.baseOrientation ?? [0, 0, 0, 1] } : undefined}>
            <group
                position={effectiveTransform.position}
                rotation={effectiveTransform.rotation}
                scale={effectiveTransform.scale}
            >
                {robot && <primitive key={`${tag}-collision`} object={robot} />}
            </group>
        </DisplayFrame>
    );
}

export const CollisionRendererMemo = memo(CollisionRenderer);
export { CollisionRendererMemo as CollisionRenderer };
export default CollisionRendererMemo;
