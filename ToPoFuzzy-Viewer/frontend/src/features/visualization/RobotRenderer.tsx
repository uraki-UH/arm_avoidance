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
    const viewerPort = import.meta.env.VITE_VIEWER_WS_PORT ?? '9001';
    const groupRef = useRef<THREE.Group>(null);
    const [robot, setRobot] = useState<any>(null);
    const lastUrdfRef = useRef<string>('');
    const robotMaterial = useMemo(() => new THREE.MeshBasicMaterial({
        color: '#00ff66',
        transparent: false,
        depthWrite: true,
        depthTest: true,
        side: THREE.FrontSide,
    }), []);

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

    const applyRobotMaterial = useMemo(() => {
        return (root: THREE.Object3D) => {
            root.traverse((child: any) => {
                if (!child?.isMesh) {
                    return;
                }

                const material = robotMaterial.clone();
                child.material = material;
                child.castShadow = false;
                child.receiveShadow = false;
                child.renderOrder = 10;
                child.material.needsUpdate = true;
            });
        };
    }, [robotMaterial]);

    // --- Load URDF ---
    useEffect(() => {
        if (!data?.urdf || data.urdf === lastUrdfRef.current) return;
        
        lastUrdfRef.current = data.urdf;

        const { urdfLoader } = loaders;
        
        // Redirect package:// to our mesh server
        urdfLoader.packages = (pkg) => {
            return `http://${window.location.hostname}:${viewerPort}/meshes/${pkg}`;
        };
        urdfLoader.loadMeshCb = (path, manager, done) => {
            urdfLoader.defaultMeshLoader(path, manager, (obj, err) => {
                if (obj) {
                    if ((obj as THREE.Mesh).isMesh) {
                        const mesh = obj as THREE.Mesh;
                        mesh.material = robotMaterial.clone();
                        mesh.castShadow = false;
                        mesh.receiveShadow = false;
                        mesh.renderOrder = 10;
                        mesh.material.needsUpdate = true;

                        const wrapper = new THREE.Group();
                        wrapper.add(mesh);
                        done(wrapper, err);
                        return;
                    }

                    applyRobotMaterial(obj);
                }
                done(obj, err);
            });
        };

        try {
            const robotObj = urdfLoader.parse(data.urdf);

            // Make the robot easier to see in the viewer by overriding imported materials.
            applyRobotMaterial(robotObj);
            
            setRobot(robotObj);
            console.log("URDF Robot model loaded successfully");
        } catch (err) {
            console.error("Failed to parse URDF:", err);
        }
    }, [data?.urdf, loaders, robotMaterial, viewerPort, applyRobotMaterial]);

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
            renderOrder={10}
        />
    );
}
