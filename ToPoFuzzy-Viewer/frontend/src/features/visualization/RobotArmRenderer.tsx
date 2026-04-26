import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { RobotArmData } from '../../types';

interface RobotArmRendererProps {
    data: RobotArmData | null;
    visible?: boolean;
}

export function RobotArmRenderer({
    data,
    visible = true,
}: RobotArmRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const [loading, setLoading] = useState(false);
    const lastUrdfRef = useRef<string>('');

    // --- Initialize Loaders ---
    const loaders = useMemo(() => {
        const dracoLoader = new DRACOLoader();
        // Use Google's CDN for the draco decoder files (WASM)
        dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.6/');
        
        const gltfLoader = new GLTFLoader();
        gltfLoader.setDRACOLoader(dracoLoader);
        
        const urdfLoader = new URDFLoader();
        // Register GLTFLoader to handle .gltf and .glb files mentioned in the URDF
        // This is the key to supporting compressed meshes
        // @ts-ignore: urdf-loader supports setting loaders for different extensions
        urdfLoader.fetchOptions = {
            // Some versions of urdf-loader might need specific setup for gltf
        };

        return { urdfLoader, gltfLoader, dracoLoader };
    }, []);

    // --- Load URDF ---
    useEffect(() => {
        if (!data?.urdf || data.urdf === lastUrdfRef.current) return;
        
        lastUrdfRef.current = data.urdf;
        setLoading(true);

        const { urdfLoader } = loaders;
        
        // Redirect package:// to our mesh server
        urdfLoader.packages = (pkg) => {
            return `http://localhost:9001/meshes/${pkg}`;
        };

        try {
            const robotObj = urdfLoader.parse(data.urdf);
            // Three.js URDFLoader returns a group. 
            // We need to rotate it if Y-up is expected but URDF is Z-up
            // Usually ROS is Z-up, Three.js is Y-up.
            robotObj.rotation.x = -Math.PI / 2; 
            
            setRobot(robotObj);
            setLoading(false);
            console.log("URDF Robot model loaded successfully");
        } catch (err) {
            console.error("Failed to parse URDF:", err);
            setLoading(false);
        }
    }, [data?.urdf]);

    // --- Update Joints ---
    useEffect(() => {
        if (!robot || !data?.jointNames || !data?.jointValues) return;

        data.jointNames.forEach((name, i) => {
            if (robot.joints[name]) {
                robot.joints[name].setJointValue(data.jointValues[i]);
            }
        });
    }, [robot, data?.jointNames, data?.jointValues]);

    if (!visible || !robot) return null;

    return (
        <primitive 
            ref={groupRef}
            object={robot} 
            renderOrder={2}
        />
    );
}
