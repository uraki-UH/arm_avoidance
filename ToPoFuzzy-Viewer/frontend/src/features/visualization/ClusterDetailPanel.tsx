import { useMemo, useState, useEffect, useRef, memo } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, Html } from '@react-three/drei';
import * as THREE from 'three';
import { GraphCluster, GraphNode, LAYER_COLORS, LAYER_LABELS } from '../../types';

export interface ClusterSnapshot {
    cluster: GraphCluster;
    nodes: GraphNode[];
    edges: { source: GraphNode, target: GraphNode }[];
}

interface ClusterDetailPanelProps {
    snapshot: ClusterSnapshot;
    onClose: () => void;
}

function ClusterDetailPanelInner({ snapshot, onClose }: ClusterDetailPanelProps) {
    const { cluster, nodes: clusterNodes, edges: clusterEdges } = snapshot;
    const [showNodes, setShowNodes] = useState(true);
    const [showEdges, setShowEdges] = useState(true);
    const [showNormals, setShowNormals] = useState(true);

    // Draggable state
    // Initial position: somewhat top-right but in pixels. 
    // Assuming window width ~1920, top-right might be left: 1500, top: 20
    // But safely starting at left: 50%, top: 10% or using viewport units converted to pixels is hard without ref.
    // Let's settle for a safe fixed default like left: 1000, top: 50
    const [position, setPosition] = useState({ x: window.innerWidth - 780, y: window.innerHeight - 420 });
    const [isDragging, setIsDragging] = useState(false);
    const dragStartRef = useRef<{ x: number, y: number } | null>(null);
    const initialPosRef = useRef<{ x: number, y: number } | null>(null);

    useEffect(() => {
        const handleMouseMove = (e: MouseEvent) => {
            if (isDragging && dragStartRef.current && initialPosRef.current) {
                const dx = e.clientX - dragStartRef.current.x;
                const dy = e.clientY - dragStartRef.current.y;
                setPosition({
                    x: initialPosRef.current.x + dx,
                    y: initialPosRef.current.y + dy
                });
            }
        };

        const handleMouseUp = () => {
            setIsDragging(false);
        };

        if (isDragging) {
            window.addEventListener('mousemove', handleMouseMove);
            window.addEventListener('mouseup', handleMouseUp);
        }

        return () => {
            window.removeEventListener('mousemove', handleMouseMove);
            window.removeEventListener('mouseup', handleMouseUp);
        };
    }, [isDragging]);

    const handleMouseDown = (e: React.MouseEvent) => {
        setIsDragging(true);
        dragStartRef.current = { x: e.clientX, y: e.clientY };
        initialPosRef.current = { x: position.x, y: position.y };
    };

    // Calculate cluster center for camera targeting
    const center = useMemo(() => {
        if (!cluster) return [0, 0, 0];
        return cluster.pos;
    }, [cluster]);

    const orbitTarget = useMemo(
        () => new THREE.Vector3(center[0], center[1], center[2]),
        [center[0], center[1], center[2]]
    );

    const nodeGeometry = useMemo(() => new THREE.SphereGeometry(0.025, 8, 8), []);
    const nodeMaterials = useMemo(
        () => LAYER_COLORS.map(color => new THREE.MeshBasicMaterial({ color })),
        []
    );

    const edgeGeometry = useMemo(() => new THREE.CylinderGeometry(1, 1, 1, 4), []);
    const edgeMaterial = useMemo(
        () => new THREE.MeshBasicMaterial({ color: '#006400', transparent: true, opacity: 1.0 }),
        []
    );

    const edgeTransforms = useMemo(() => {
        const up = new THREE.Vector3(0, 1, 0);
        return clusterEdges.map(edge => {
            const start = new THREE.Vector3(edge.source.x, edge.source.y, edge.source.z);
            const end = new THREE.Vector3(edge.target.x, edge.target.y, edge.target.z);
            const mid = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
            const len = start.distanceTo(end);
            const dir = new THREE.Vector3().subVectors(end, start).normalize();
            const quat = new THREE.Quaternion().setFromUnitVectors(up, dir);

            return {
                position: [mid.x, mid.y, mid.z] as [number, number, number],
                quaternion: [quat.x, quat.y, quat.z, quat.w] as [number, number, number, number],
                length: len
            };
        });
    }, [clusterEdges]);

    const normalHelpers = useMemo(() => {
        if (!showNormals || !showNodes) return [];
        return clusterNodes.map(node => {
            const origin = new THREE.Vector3(node.x, node.y, node.z);
            const dir = new THREE.Vector3(node.nx, node.ny, node.nz).normalize();
            return new THREE.ArrowHelper(dir, origin, 0.2, 0x00FFFF);
        });
    }, [clusterNodes, showNormals, showNodes]);

    useEffect(() => {
        return () => {
            normalHelpers.forEach(helper => {
                helper.line.geometry.dispose();
                (helper.line.material as THREE.Material).dispose();
                helper.cone.geometry.dispose();
                (helper.cone.material as THREE.Material).dispose();
            });
        };
    }, [normalHelpers]);



    // Dimensions
    const dimensions = useMemo(() => {
        if (!cluster) return null;
        // Calculate bounding box from nodes + raw points if available
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

        const pointsToMeasure: { x: number, y: number, z: number }[] = [...clusterNodes];

        if (pointsToMeasure.length === 0) return null;

        pointsToMeasure.forEach(p => {
            minX = Math.min(minX, p.x);
            minY = Math.min(minY, p.y);
            minZ = Math.min(minZ, p.z);
            maxX = Math.max(maxX, p.x);
            maxY = Math.max(maxY, p.y);
            maxZ = Math.max(maxZ, p.z);
        });

        // Add node radius padding (~0.025)
        const padding = 0.025;
        return {
            x: (maxX - minX + padding * 2).toFixed(3),
            y: (maxY - minY + padding * 2).toFixed(3),
            z: (maxZ - minZ + padding * 2).toFixed(3),
            width: maxX - minX + padding * 2,
            height: maxY - minY + padding * 2,
            depth: maxZ - minZ + padding * 2,
            center: [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2]
        };
    }, [clusterNodes, cluster]);

    if (!cluster) return null;

    return (
        <div
            className="surface-panel absolute z-50 flex h-96 w-96 min-h-0 flex-col overflow-hidden"
            style={{
                left: position.x,
                top: position.y,
                cursor: isDragging ? 'grabbing' : 'auto',
                resize: 'both',
                minWidth: 320,
                minHeight: 320,
                maxWidth: '90vw',
                maxHeight: '90vh'
            }}
        >
            {/* Header */}
            <div
                className="flex shrink-0 cursor-grab items-center justify-between border-b border-white/10 bg-black/25 p-2 active:cursor-grabbing"
                onMouseDown={handleMouseDown}
            >
                <h3 className="text-sm font-bold text-[var(--text-primary)]">
                    Cluster #{cluster.id} ({LAYER_LABELS[cluster.label]}) Details (Offline)
                </h3>
                <button
                    onClick={onClose}
                    className="btn-secondary inline-flex h-7 w-7 items-center justify-center p-0 text-[var(--text-secondary)] hover:text-[var(--text-primary)]"
                >
                    ✕
                </button>
            </div>

            {/* Controls */}
            <div className="flex shrink-0 gap-3 border-b border-white/10 bg-black/20 p-2 text-xs text-[var(--text-primary)]">
                <label className="inline-flex items-center gap-1.5">
                    <input type="checkbox" checked={showNodes} onChange={e => setShowNodes(e.target.checked)} />
                    Nodes
                </label>
                <label className="inline-flex items-center gap-1.5">
                    <input type="checkbox" checked={showEdges} onChange={e => setShowEdges(e.target.checked)} />
                    Edges
                </label>
                <label className="inline-flex items-center gap-1.5">
                    <input type="checkbox" checked={showNormals} onChange={e => setShowNormals(e.target.checked)} />
                    Normals
                </label>
            </div>

            {/* Canvas */}
            <div className="flex-1 min-h-0 relative">
                <Canvas
                    frameloop="demand"
                    dpr={1}
                    gl={{ powerPreference: 'low-power', antialias: false }}
                    camera={{ position: [center[0] + 5, center[1] + 5, center[2] + 5], up: [0, 0, 1], fov: 50 }}
                >
                    <ambientLight intensity={0.5} />
                    <pointLight position={[10, 10, 10]} intensity={0.8} />
                    <DemandOrbitControls target={orbitTarget} />

                    <group>
                        {/* Nodes */}
                        {showNodes && clusterNodes.map((node, i) => (
                            <mesh
                                key={i}
                                position={[node.x, node.y, node.z]}
                                geometry={nodeGeometry}
                                material={nodeMaterials[node.label % nodeMaterials.length]}
                            />
                        ))}

                        {/* Edges */}
                        {showEdges && edgeTransforms.map((edge, i) => (
                            <mesh
                                key={i}
                                position={edge.position}
                                quaternion={edge.quaternion}
                                scale={[0.01, edge.length, 0.01]}
                                geometry={edgeGeometry}
                                material={edgeMaterial}
                            />
                        ))}

                        {/* Normals */}
                        {showNormals && showNodes && normalHelpers.map((helper, i) => (
                            <primitive key={`norm-${i}`} object={helper} />
                        ))}


                        {/* Bounding Box & Dimensions */}
                        {dimensions && (
                            <group>
                                <mesh position={new THREE.Vector3(...dimensions.center as [number, number, number])}>
                                    <boxGeometry args={[dimensions.width, dimensions.height, dimensions.depth]} />
                                    <meshBasicMaterial color="white" wireframe transparent opacity={0.3} />
                                </mesh>
                                <Html position={[dimensions.center[0], dimensions.center[1] + dimensions.height / 2 + 0.1, dimensions.center[2]]} center>
                                    <div className="whitespace-nowrap rounded bg-black/65 px-1 text-xs text-white">
                                        W:{dimensions.x} H:{dimensions.z} D:{dimensions.y}
                                    </div>
                                </Html>
                            </group>
                        )}
                    </group>

                    <gridHelper args={[10, 20]} position={[center[0], center[1], center[2] - 1]} rotation={[Math.PI / 2, 0, 0]} />
                    <axesHelper args={[1]} position={[center[0], center[1], center[2]]} />
                </Canvas>
            </div>

            {/* Stats Footer */}
            <div className="grid shrink-0 grid-cols-2 gap-x-4 border-t border-white/10 bg-black/25 p-2 text-xs text-[var(--text-secondary)]">
                <span>Reliability: {cluster.reliability.toFixed(3)}</span>
                <span>Nodes: {clusterNodes.length}</span>
                <span>Pos: [{cluster.pos.map(v => v.toFixed(2)).join(',')}]</span>

            </div>
        </div>
    );
}

function DemandOrbitControls({ target }: { target: THREE.Vector3 }) {
    const { invalidate } = useThree();
    return <OrbitControls target={target} onChange={() => invalidate()} />;
}

export const ClusterDetailPanel = memo(
    ClusterDetailPanelInner,
    (prev, next) => prev.snapshot === next.snapshot
);

export default ClusterDetailPanel;
