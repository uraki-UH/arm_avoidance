import { useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { RobotData } from '../../types';
import { useDemandUpdate } from '../../hooks/useDemandUpdate';

interface CollisionRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
    color?: string;
    tf?: { pos: number[]; quat: number[] } | null;
}

export function CollisionRenderer({
    tag,
    data,
    visible = true,
    color = '#ff9f1c',
    tf = null,
}: CollisionRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const lastUrdfRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);

    const viewerPort = 9001;

    useDemandUpdate([robot, data, visible, color, tf]);

    const collisionMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: new THREE.Color(color),
        transparent: true,
        opacity: 0.28,
        wireframe: true,
        depthTest: false,
        depthWrite: false,
    }), [color]);

    const applyCollisionMaterial = useCallback((obj: THREE.Object3D) => {
        if (!obj) return;
        obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                mesh.material = collisionMaterial;
                mesh.visible = true;
                mesh.castShadow = false;
                mesh.receiveShadow = false;
                mesh.renderOrder = 20;
            }
        });
    }, [collisionMaterial]);

    useEffect(() => {
        if (!robot) return;
        applyCollisionMaterial(robot);
        const interval = setInterval(() => applyCollisionMaterial(robot), 150);
        const timeout = setTimeout(() => clearInterval(interval), 2500);
        return () => {
            clearInterval(interval);
            clearTimeout(timeout);
        };
    }, [robot, applyCollisionMaterial]);

    useEffect(() => {
        if (!data?.urdf || data.urdf === lastUrdfRef.current) return;
        lastUrdfRef.current = data.urdf;

        const urdfLoader = new URDFLoader();
        urdfLoader.packages = (pkg) => `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;
        urdfLoader.parseVisual = false;
        urdfLoader.parseCollision = true;

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            applyCollisionMaterial(robotObj);
            setRobot(robotObj);
        } catch (err) {
            console.error('Failed to parse URDF collision model:', err);
            lastUrdfRef.current = null;
        }
    }, [data?.urdf, applyCollisionMaterial, tag]);

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
        <group ref={groupRef} name={`${tag}-collision`} visible={visible}>
            {robot && <primitive key={`${tag}-collision`} object={robot} />}
        </group>
    );
}
