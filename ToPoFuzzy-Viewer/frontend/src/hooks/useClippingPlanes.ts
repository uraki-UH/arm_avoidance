import { useState } from 'react';
import * as THREE from 'three';

export type ClippingAxis = 'x' | 'y' | 'z' | 'none';

export interface ClippingPlane {
    id: string;
    axis: ClippingAxis;
    position: number;
    inverted: boolean;
    enabled: boolean;
}

export function useClippingPlanes() {
    const [planes, setPlanes] = useState<ClippingPlane[]>([]);

    const addPlane = (axis: ClippingAxis) => {
        const id = `plane-${Date.now()}`;
        const newPlane: ClippingPlane = {
            id,
            axis,
            position: 0,
            inverted: false,
            enabled: true,
        };
        setPlanes(prev => [...prev, newPlane]);
        return id;
    };

    const updatePlane = (id: string, updates: Partial<ClippingPlane>) => {
        setPlanes(prev => prev.map(p => p.id === id ? { ...p, ...updates } : p));
    };

    const removePlane = (id: string) => {
        setPlanes(prev => prev.filter(p => p.id !== id));
    };

    const removeAll = () => {
        setPlanes([]);
    };

    // Convert to Three.js Plane objects
    const getThreePlanes = (): THREE.Plane[] => {
        return planes
            .filter(p => p.enabled && p.axis !== 'none')
            .map(p => {
                const normal = new THREE.Vector3(
                    p.axis === 'x' ? 1 : 0,
                    p.axis === 'y' ? 1 : 0,
                    p.axis === 'z' ? 1 : 0
                );

                if (p.inverted) {
                    normal.multiplyScalar(-1);
                }

                return new THREE.Plane(normal, -p.position);
            });
    };

    return {
        planes,
        addPlane,
        updatePlane,
        removePlane,
        removeAll,
        getThreePlanes,
    };
}
