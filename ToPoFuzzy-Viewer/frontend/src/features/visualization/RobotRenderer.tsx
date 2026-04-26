import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js';
import { RobotData } from '../../types';

interface RobotRendererProps {
    data: RobotData | null;
    visible?: boolean;
}

export function RobotRenderer({
    data,
    visible = true,
}: RobotRendererProps) {
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const [loading, setLoading] = useState(false);
    const lastUrdfRef = useRef<string>('');

    // --- Initialize Loaders ---
    const loaders = useMemo(() => {
        const dracoLoader = new DRACOLoader();
        dracoLoader.setDecoderPath('https://www.gstatic.com/draco/versioned/decoders/1.5.6/');
        
        const gltfLoader = new GLTFLoader();
        gltfLoader.setDRACOLoader(dracoLoader);
        
        const urdfLoader = new URDFLoader();
        // @ts-ignore
        urdfLoader.fetchOptions = {};

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
            // Rotate to match Three.js coordinate system (Y-up)
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
