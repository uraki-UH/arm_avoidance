import { useMemo, useRef, useEffect, useState, useCallback } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { RobotData } from '../../types';

interface RobotRendererProps {
    tag: string;
    data: RobotData;
    visible?: boolean;
}

export function RobotRenderer({ tag, data, visible = true }: RobotRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const lastUrdfRef = useRef<string | null>(null);
    const lastJointSignatureRef = useRef<string | null>(null);

    const { scene } = useThree();
    const viewerPort = 9001; // Actual mesh server port in gateway node

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
        console.log(`[RobotRenderer] Parsing new URDF for: ${tag}`);

        const urdfLoader = new URDFLoader();
        // Redirect package:// to our mesh server
        urdfLoader.packages = (pkg) => `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            applyRobotMaterial(robotObj);
            setRobot(robotObj);
        } catch (err) {
            console.error("Failed to parse URDF:", err);
            lastUrdfRef.current = null; // Allow retry
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

    if (!visible || !robot) return null;

    return (
        <primitive
            key={tag} // Ensure fresh component if tag changes
            ref={groupRef}
            object={robot}
            position={data?.basePosition || [0, 0, 0]}
            quaternion={data?.baseOrientation || [0, 0, 0, 1]}
            renderOrder={10}
        />
    );
}
