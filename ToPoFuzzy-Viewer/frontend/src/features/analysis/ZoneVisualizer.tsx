
import { useState, useRef } from 'react';
import { ThreeEvent, useFrame } from '@react-three/fiber';
import { Line, Html } from '@react-three/drei';
import * as THREE from 'three';
import { ZonePoint } from './useZoneMonitor';

interface ZoneVisualizerProps {
    points: ZonePoint[];
    isDrawing: boolean;
    isWarning?: boolean;
    onAddPoint: (point: ZonePoint) => void;
    zRange?: [number, number];
}

export function ZoneVisualizer({ points, isDrawing, isWarning = false, onAddPoint, zRange }: ZoneVisualizerProps) {
    const [cursorPos, setCursorPos] = useState<[number, number, number] | null>(null);
    const materialRef = useRef<THREE.MeshBasicMaterial>(null);

    useFrame(() => {
        if (materialRef.current) {
            if (isWarning) {
                // Static red warning
                materialRef.current.opacity = 0.5;
                materialRef.current.color.set('#ff0000');
            } else {
                materialRef.current.opacity = 0.2;
                materialRef.current.color.set('#00ff00');
            }
        }
    });

    const handlePointerMove = (e: ThreeEvent<PointerEvent>) => {
        if (!isDrawing) return;
        setCursorPos([e.point.x, e.point.y, e.point.z]);
    };

    const handleClick = (e: ThreeEvent<MouseEvent>) => {
        if (!isDrawing) return;
        // Prevent event from bubbling if we clicked something else (though this is a background plane)
        e.stopPropagation();

        onAddPoint({ x: e.point.x, y: e.point.y, z: e.point.z });
    };

    // Prepare line points for rendering
    const linePoints = points.map(p => [p.x, p.y, p.z] as [number, number, number]);
    if (isDrawing && cursorPos) {
        linePoints.push(cursorPos);
    } else if (!isDrawing && points.length > 0) {
        // Close the loop if finished
        linePoints.push([points[0].x, points[0].y, points[0].z]);
    }

    // Prepare polygon shape for filling
    let shapeGeometry = null;
    if (points.length >= 3 && !isDrawing) {
        const shape = new THREE.Shape();
        shape.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i++) {
            shape.lineTo(points[i].x, points[i].y);
        }
        shapeGeometry = new THREE.ShapeGeometry(shape);
    }

    return (
        <group>
            {/* Invisible plane to catch clicks for drawing */}
            {isDrawing && (
                <mesh
                    visible={false} // Raycasting still hits invisible meshes by default in r3f if not configured otherwise? No, usually needs visible=true for raycasting unless Raycaster.layers match.
                    // Actually, let's make it barely visible grid or just a plane.
                    // For now, let's check r3f default.
                    // It's safer to use transparency.
                    onPointerMove={handlePointerMove}
                    onClick={handleClick}
                    rotation={[0, 0, 0]} // Default is XY plane if using PlaneGeometry? No, PlaneGeometry is XY.
                // App uses Z-up. We want the drawing on ground (Z=0).
                >
                    <planeGeometry args={[1000, 1000]} />
                    <meshBasicMaterial color="white" side={THREE.DoubleSide} />
                </mesh>
            )}

            {/* Render Zone Lines (Base) */}
            {linePoints.length > 0 && (
                <Line
                    points={linePoints}
                    color={isDrawing ? "#ff0000" : (isWarning ? "#ff0000" : "#00ff00")}
                    lineWidth={isWarning ? 3 : 2}
                    position={[0, 0, isDrawing ? 0 : (zRange ? zRange[0] : 0)]} // Lift base line to min Z only when done drawing
                />
            )}

            {/* Render Zone Lines (Top) */}
            {linePoints.length > 0 && !isDrawing && zRange && (
                <Line
                    points={linePoints}
                    color={isWarning ? "#ff0000" : "#00ff00"}
                    lineWidth={1}
                    position={[0, 0, zRange[1]]} // Top line at max Z
                    transparent
                    opacity={0.5}
                />
            )}

            {/* Render 3D Volume */}
            {shapeGeometry && !isDrawing && zRange && (
                <mesh position={[0, 0, zRange[0]]}>
                    <extrudeGeometry
                        args={[
                            new THREE.Shape(points.map(p => new THREE.Vector2(p.x, p.y))),
                            {
                                depth: zRange[1] - zRange[0],
                                bevelEnabled: false
                            }
                        ]}
                    />
                    <meshBasicMaterial
                        ref={materialRef}
                        transparent
                        opacity={0.1}
                        side={THREE.DoubleSide}
                        depthWrite={false} // Prevent z-fighting with stuff inside
                    />
                </mesh>
            )}

            {/* Render Zone Fill (2D Fallback or Floor if no zRange) */}
            {shapeGeometry && !isDrawing && !zRange && (
                <mesh geometry={shapeGeometry}>
                    <meshBasicMaterial
                        ref={materialRef}
                        transparent
                        side={THREE.DoubleSide}
                    />
                </mesh>
            )}

            {/* Render vertices */}
            {(isDrawing || points.length < 3) && points.map((p, i) => (
                <mesh key={i} position={[p.x, p.y, p.z]}>
                    <sphereGeometry args={[0.1, 16, 16]} />
                    <meshBasicMaterial color={isDrawing ? "red" : "lime"} />
                </mesh>
            ))}

            {isDrawing && (
                <Html position={[0, 0, 0]} style={{ pointerEvents: 'none' }}>
                    <div style={{
                        color: 'white',
                        background: 'rgba(0,0,0,0.7)',
                        padding: '4px',
                        borderRadius: '4px',
                        transform: 'translate(-50%, -100%)',
                        whiteSpace: 'nowrap'
                    }}>
                        Click to add point
                    </div>
                </Html>
            )}
        </group>
    );
}
