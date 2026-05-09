/**
 * ROS-related utility functions for the frontend.
 */

/**
 * Formats a ROS topic path or tag into a human-readable display title.
 * e.g., "/camera/depth/points" -> "points"
 *       "default" -> "GNG Topology" (if type is graph)
 */
export const getDisplayTitle = (id: string, type: string = 'topic'): string => {
    if (!id || id === 'default') {
        if (type === 'graph') return 'GNG Topology';
        if (type === 'robot') return 'Robot Model';
        if (type === 'marker') return 'Markers';
        return 'Unnamed Layer';
    }
    
    // Remove leading/trailing slashes and split
    const cleanId = id.startsWith('/') ? id.slice(1) : id;
    const parts = cleanId.split('/');
    const last = parts[parts.length - 1];
    
    return last || id;
};

/**
 * Returns a short, uppercase status label for a layer.
 */
export const getStatusLabel = (type: string, mode?: string): string => {
    if (type === 'graph') return mode === 'static' ? 'Static' : 'Dynamic';
    if (type === 'cloud') return 'Cloud';
    return type.toUpperCase();
};
