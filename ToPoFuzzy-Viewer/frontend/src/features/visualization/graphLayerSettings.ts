import { useEffect, useState } from 'react';
import {
    DYNAMIC_GNG_DEFAULTS,
    GraphData,
    GraphNode,
    LayerSettings,
    STATIC_GNG_DEFAULTS,
    TRAJECTORY_GNG_DEFAULTS,
    isTrajectoryGraphTag,
} from '../../types';

export function nodeHasManipulabilityData(node?: GraphNode | null): boolean {
    return Boolean(
        node &&
        (
            node.manipValid !== undefined ||
            node.manipValue !== undefined ||
            node.manipConditionNumber !== undefined ||
            node.manipScale !== undefined ||
            node.manipOrientation !== undefined
        )
    );
}

export function graphHasManipulabilityData(graph?: GraphData | null): boolean {
    return Boolean(graph?.nodes.some((node) => nodeHasManipulabilityData(node)));
}

export function createDefaultGraphLayerSettings(tag: string, graph: GraphData): LayerSettings {
    const isStatic = graph.mode === 'static';
    const isTrajectory = isTrajectoryGraphTag(tag);
    const visualDefaults = isStatic ? STATIC_GNG_DEFAULTS : DYNAMIC_GNG_DEFAULTS;

    return {
        visible: true,
        showNodes: true,
        showEdges: !isStatic,
        showClusters: false,
        visibleSemanticLabels: {
            handle: true,
        },
        visibleLabels: {
            0: true,
            1: true,
            2: true,
            3: true,
            4: true,
            5: true,
        },
        showNormals: false,
        showVelocity: false,
        showCovarianceEllipsoids: false,
        showManipulabilityEllipsoids: false,
        manipEllipsoidMode: 'all',
        manipEllipsoidType: 'translational',
        // Preserve the existing initialized-layer opacity behavior during this refactor.
        nodeOpacity: STATIC_GNG_DEFAULTS.nodeOpacity,
        edgeOpacity: STATIC_GNG_DEFAULTS.edgeOpacity,
        graphTransform: {
            position: [0, 0, 0],
            rotation: [0, 0, 0],
            scale: [1, 1, 1],
        },
        nodeColor: isTrajectory ? TRAJECTORY_GNG_DEFAULTS.nodeColor : visualDefaults.nodeColor,
        edgeColor: isTrajectory ? TRAJECTORY_GNG_DEFAULTS.edgeColor : visualDefaults.edgeColor,
        normalColor: '#4fa3a5',
        velocityColor: '#ffb347',
        covarianceEllipsoidColor: '#aefeff',
        normalScale: 0.075,
        velocityScale: 0.25,
        covarianceEllipsoidScale: 2.0,
        emissiveIntensity: visualDefaults.nodeEmissiveIntensity,
    };
}

export function useGraphLayerSettings(graphData: Record<string, GraphData>) {
    const [layerSettings, setLayerSettings] = useState<Record<string, LayerSettings>>({});

    useEffect(() => {
        setLayerSettings((currentSettings) => {
            const nextSettings = { ...currentSettings };
            let changed = false;

            Object.entries(graphData).forEach(([tag, graph]) => {
                const isStatic = graph.mode === 'static';
                const isTrajectory = isTrajectoryGraphTag(tag);
                const hasManipulabilityData = graphHasManipulabilityData(graph);
                const previousNodeColor = isStatic
                    ? STATIC_GNG_DEFAULTS.nodeColor
                    : DYNAMIC_GNG_DEFAULTS.nodeColor;
                const previousEdgeColor = isStatic
                    ? STATIC_GNG_DEFAULTS.edgeColor
                    : DYNAMIC_GNG_DEFAULTS.edgeColor;
                const migrateNodeColor = isTrajectory && (
                    !nextSettings[tag]?.nodeColor || nextSettings[tag].nodeColor === previousNodeColor
                );
                const migrateEdgeColor = isTrajectory && (
                    !nextSettings[tag]?.edgeColor || nextSettings[tag].edgeColor === previousEdgeColor
                );

                if (!nextSettings[tag]) {
                    nextSettings[tag] = createDefaultGraphLayerSettings(tag, graph);
                    changed = true;
                } else if (migrateNodeColor || migrateEdgeColor) {
                    nextSettings[tag] = {
                        ...nextSettings[tag],
                        nodeColor: migrateNodeColor
                            ? TRAJECTORY_GNG_DEFAULTS.nodeColor
                            : nextSettings[tag].nodeColor,
                        edgeColor: migrateEdgeColor
                            ? TRAJECTORY_GNG_DEFAULTS.edgeColor
                            : nextSettings[tag].edgeColor,
                    };
                    changed = true;
                } else if (!nextSettings[tag].visibleLabels) {
                    nextSettings[tag] = {
                        ...nextSettings[tag],
                        visibleLabels: {
                            0: true,
                            1: true,
                            2: true,
                            3: true,
                            4: true,
                            5: true,
                        },
                    };
                    changed = true;
                } else if (!nextSettings[tag].visibleSemanticLabels) {
                    nextSettings[tag] = {
                        ...nextSettings[tag],
                        visibleSemanticLabels: {
                            handle: true,
                        },
                    };
                    changed = true;
                } else if (!hasManipulabilityData && nextSettings[tag].showManipulabilityEllipsoids) {
                    nextSettings[tag] = {
                        ...nextSettings[tag],
                        showManipulabilityEllipsoids: false,
                    };
                    changed = true;
                }
            });

            return changed ? nextSettings : currentSettings;
        });
    }, [graphData]);

    const updateLayerSettings = (tag: string, updates: Partial<LayerSettings>) => {
        setLayerSettings((currentSettings) => ({
            ...currentSettings,
            [tag]: { ...currentSettings[tag], ...updates },
        }));
    };

    const removeLayerSettings = (tag: string) => {
        setLayerSettings((currentSettings) => {
            const nextSettings = { ...currentSettings };
            delete nextSettings[tag];
            return nextSettings;
        });
    };

    return {
        layerSettings,
        updateLayerSettings,
        removeLayerSettings,
    };
}
