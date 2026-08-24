import { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface DirectionalArrowProps {
    origin: [number, number, number];
    direction: [number, number, number];
    lengthScale?: number;
    maxLength?: number;
    color?: string;
    visible?: boolean;
    headLengthRatio?: number;
    headWidthRatio?: number;
    head_length?: number;
    head_width?: number;
    shaftWidth?: number;
}

export function DirectionalArrow({
    origin,
    direction,
    lengthScale = 1.0,
    maxLength = 0.4,
    color = '#00ffff',
    visible = true,
    headLengthRatio = 0.28,
    headWidthRatio = 0.18,
    head_length,
    head_width,
    shaftWidth = 0.01,
}: DirectionalArrowProps) {
    const arrowRef = useRef<THREE.ArrowHelper | null>(null);

    if (!arrowRef.current) {
        arrowRef.current = new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(origin[0], origin[1], origin[2]),
            0.001,
            color
        );
    }

    useEffect(() => {
        const arrow = arrowRef.current;
        if (!arrow) return;
        const dir = new THREE.Vector3(direction[0], direction[1], direction[2]);
        const mag = dir.length();
        if (!visible || !Number.isFinite(mag) || mag <= 1e-8) {
            arrow.visible = false;
            return;
        }

        const length = Math.min(maxLength, mag * lengthScale);
        dir.normalize();
        arrow.position.set(origin[0], origin[1], origin[2]);
        arrow.setDirection(dir);
        const cone_length = Math.min(
            length,
            Math.max(0.0001, head_length ?? length * headLengthRatio),
        );
        const cone_width = Math.max(0.0001, head_width ?? length * headWidthRatio);
        arrow.setLength(length, cone_length, cone_width);
        arrow.setColor(new THREE.Color(color));
        arrow.visible = true;
    }, [origin, direction, lengthScale, maxLength, color, visible, headLengthRatio, headWidthRatio, head_length, head_width, shaftWidth]);

    useEffect(() => {
        return () => {
            const arrow = arrowRef.current;
            if (!arrow) return;
            arrow.line?.geometry?.dispose?.();
            arrow.cone?.geometry?.dispose?.();
            (arrow.line?.material as THREE.Material | undefined)?.dispose?.();
            const coneMaterial = arrow.cone?.material;
            if (coneMaterial && !Array.isArray(coneMaterial)) {
                coneMaterial.dispose();
            }
        };
    }, []);

    return <primitive object={arrowRef.current} />;
}
