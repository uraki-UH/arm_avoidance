import { useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { RobotData } from '../../types';

interface RobotRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
    color?: string;
    tf?: { pos: number[]; quat: number[] } | null;
}

export function RobotRenderer({
    tag,
    data,
    visible = true,
    color = 'skyblue',
    tf = null,
}: RobotRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const lastUrdfRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);

    const viewerPort = 9001;

    // --- Memoize Robot Material ---
    const robotMaterial = useMemo(() => new THREE.MeshStandardMaterial({
        color: 'skyblue',
        roughness: 0.5,
        metalness: 0.5,
        transparent: true,
        opacity: 0.8,
    }), []);

    // --- Memoize Helper Function ---
    const applyRobotMaterial = useCallback((obj: THREE.Object3D) => {
        if (!obj) return;
        obj.traverse((child) => {
            if ((child as THREE.Mesh).isMesh) {
                const mesh = child as THREE.Mesh;
                // Only set if not already set to our material to avoid redundant updates
                if (mesh.material !== robotMaterial) {
                    mesh.material = robotMaterial;
                    mesh.castShadow = false;
                    mesh.receiveShadow = false;
                    mesh.renderOrder = 10;
                }
            }
        });
    }, [robotMaterial]);

    useEffect(() => {
        if (!robot) return;

        applyRobotMaterial(robot);

        // Use a persistent interval for a short duration to win the "race" against the loader
        const interval = setInterval(() => applyRobotMaterial(robot), 100);
        const timeout = setTimeout(() => clearInterval(interval), 3000); // Give up after 3s

        return () => {
            clearInterval(interval);
            clearTimeout(timeout);
        };
    }, [robot, applyRobotMaterial]);

    // --- Load URDF (Only when content changes) ---
    useEffect(() => {
        if (!data?.urdf || data.urdf === lastUrdfRef.current) return;

        lastUrdfRef.current = data.urdf;

        const urdfLoader = new URDFLoader();
        urdfLoader.packages = (pkg) => `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            applyRobotMaterial(robotObj);
            setRobot(robotObj);
        } catch (err) {
            console.error("Failed to parse URDF:", err);
            lastUrdfRef.current = null;
        }
    }, [data?.urdf, viewerPort, applyRobotMaterial, tag]);

    // --- Update Joints ---
    useEffect(() => {
        if (!robot || !data?.jointNames || !data?.jointValues) return;

        // Create a signature to avoid redundant joint updates
        const signature = data.jointValues.map((v) => v.toFixed(4)).join(',');
        if (signature === lastJointSignatureRef.current) return;
        lastJointSignatureRef.current = signature;

        data.jointNames.forEach((name, i) => {
            if (robot.joints[name]) {
                robot.joints[name].setJointValue(data.jointValues[i]);
            }
        });
    }, [robot, data?.jointNames, data?.jointValues]);

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
            {robot && <primitive key={tag} object={robot} />}
        </group>
    );
}
