import * as THREE from 'three';

// --- Point Cloud Types ---

export interface PointCloudData {
    id: string;
    name: string;
    points: Float32Array; // [x, y, z, x, y, z, ...]
    colors?: Float32Array; // [r, g, b, r, g, b, ...] normalized 0-1
    intensities?: Float32Array; // [i, i, i, ...]
    count: number;
    visible?: boolean; // Layer visibility
    opacity?: number; // Layer opacity 0-1
    position?: [number, number, number]; // Translation
    rotation?: [number, number, number]; // Euler angles in radians
    scale?: [number, number, number]; // Scale factors
    matrix?: THREE.Matrix4; // Optional pre-calculated matrix
}

export interface MarkerColor {
    r: number;
    g: number;
    b: number;
    a: number;
}

export interface MarkerPose {
    position: [number, number, number];
    orientation: [number, number, number, number];
}

export interface MarkerMessage {
    ns: string;
    id: number;
    type: 'arrow' | 'cube' | 'sphere' | 'cylinder' | 'line_strip' | 'line_list' | 'cube_list' | 'sphere_list' | 'points' | 'text' | 'mesh_resource' | 'triangle_list' | 'unknown';
    action: number;
    frameId?: string;
    pos?: [number, number, number];
    quat?: [number, number, number, number];
    pose?: MarkerPose; // Keep for compatibility but prioritize pos/quat
    scale: [number, number, number];
    color: [number, number, number, number] | MarkerColor;
    points: [number, number, number][];
    colors?: ([number, number, number, number] | MarkerColor)[];
    text?: string;
    meshResource?: string;
    meshUseEmbeddedMaterials?: boolean;
    frameLocked?: boolean;
}

export interface MarkerArrayData {
    id: string;
    name: string;
    tag: string;
    frameId?: string;
    markers: MarkerMessage[];
    count: number;
    visible?: boolean;
}

export interface BoundingBox {
    id: string;
    position: [number, number, number];
    dimensions: [number, number, number]; // width, height, depth
    label: string; // "Person", "Car" etc.
    confidence: number;
    min?: { x: number; y: number; z: number }; // Legacy support if needed
    max?: { x: number; y: number; z: number }; // Legacy support if needed
}

export type HeatmapMode = 'rgb' | 'simple' | 'height' | 'distance' | 'intensity';

export interface HeatmapSettings {
    mode: HeatmapMode;
    min: number;
    max: number;
    colorScheme: 'viridis' | 'jet' | 'grayscale' | 'plasma' | 'magma' | 'inferno';
    pointSize: number;
    simpleColor: string;
}

// --- Graph / GNG Types ---

export interface GraphNode {
    id?: number;
    x: number;
    y: number;
    z: number;
    nx: number;
    ny: number;
    nz: number;
    label: number;
    semanticLabel?: number;
    semanticReliability?: number;
    age: number;
    winnerPointCount?: number;
    winnerPointMean?: [number, number, number];
    winnerPointCovariance?: [number, number, number, number, number, number, number, number, number];
}

export interface GraphCluster {
    id: number;
    label: number;
    semanticLabel?: number;
    semanticReliability?: number;
    pos: [number, number, number];
    scale: [number, number, number];
    quat: [number, number, number, number];
    match: number;
    reliability: number;
    velocity: [number, number, number];
    nodeIds: number[];  // IDs of nodes belonging to this cluster
}

export type GraphMode = 'static' | 'dynamic';

export interface GraphData {
    timestamp: number;
    nodes: GraphNode[];
    edges: number[]; // Flat array of indices [src, tgt, src, tgt...]
    clusters: GraphCluster[];
    clusterLabels?: number[];
    frameId?: string;
    tag?: string;
    mode?: GraphMode;
}

export interface Transform {
    position: [number, number, number];
    rotation: [number, number, number];
    scale: [number, number, number];
}

export interface LayerSettings {
    visible: boolean;
    showNodes: boolean;
    showEdges: boolean;
    showClusters: boolean;
    visibleSemanticLabels?: {
        handle: boolean;
    };
    visibleLabels?: {
        0: boolean;
        1: boolean;
        2: boolean;
        3: boolean;
        4: boolean;
        5: boolean;
    };
    showNormals?: boolean;
    showVelocity?: boolean;
    showCovarianceEllipsoids?: boolean;
    opacity: number;
    nodeColor?: string;
    edgeColor?: string;
    normalColor?: string;
    velocityColor?: string;
    covarianceEllipsoidColor?: string;
    emissiveIntensity?: number;
    nodeScale?: number;
    edgeWidth?: number;
    normalScale?: number;
    velocityScale?: number;
    covarianceEllipsoidScale?: number;
    graphTransform?: Transform;
}
export interface RobotSettings {
    visible: boolean;
    color: string;
    useUrdfColors?: boolean;
    showVisual: boolean;
    showCollision: boolean;
    collisionColor: string;
    emissiveIntensity?: number;
    opacity?: number;
    jointControlMode?: 'live' | 'manual';
    jointValues?: number[];
    transform?: Transform;
}

export interface RobotData {
    timestamp: number;
    frameId: string;
    tag?: string;
    urdf?: string;
    jointNames: string[];
    jointValues: number[];
    positions: [number, number, number][];
    orientations: [number, number, number, number][];
    basePosition?: [number, number, number];
    baseOrientation?: [number, number, number, number];
}

export interface TransformData {
    frameId: string;
    childFrameId: string;
    pos: [number, number, number];
    quat: [number, number, number, number];
}

export interface VoxelLayout {
    voxelSize: number;
    originX?: number;
    originY?: number;
    originZ?: number;
    xShift: number;
    yShift: number;
    zShift: number;
    offset: number;
}

export interface VoxelData {
    id: string;
    tag: string;
    data: string[];
    layout: VoxelLayout;
    frameId?: string;
    visible?: boolean;
}

export type EntityType = 'robot' | 'marker' | 'voxel' | 'graph';

export interface VoxelSettings {
    visible: boolean;
    color: string;
    wireframe: boolean;
    opacity: number;
    emissiveIntensity?: number;
    transform?: Transform;
}

// --- WebSocket / RPC Types ---

export interface DataSource {
    id: string;
    name: string;
    type: 'pointcloud' | 'topological_map' | 'marker' | 'voxel';
    active: boolean;
}

export interface RosbagInfo {
    path: string;
    name: string;
    relativePath: string;
}

export interface PlaybackStatus {
    isPlaying: boolean;
    currentBag: string;
}

export interface PointCloudFileInfo {
    path: string;
    name: string;
    relativePath: string;
    format: string;
    fileSize: number;
}

export interface GngStatus {
    isRunning: boolean;
    pid?: number;
    inputTopic?: string;
}

export interface GngParams {
    inputTopic: string;
    configFile?: string;
    maxNodes?: number;
    learningNum?: number;
    voxelGridUnit?: number;
    [key: string]: unknown;
}

export interface GngConfigInfo {
    name: string;
    path: string;
}

export interface ContinuousPublishStatus {
    isPublishing: boolean;
    topic: string;
    rateHz: number;
    pointCount: number;
}

export interface EditRegion {
    regionId: string;
    frameId: string;
    min: [number, number, number];
    max: [number, number, number];
}

export interface EditSessionInfo {
    sessionId: string;
    sourceTopic: string;
    targetFrame: string;
    sourceFrameId: string;
    pointCount: number;
    regions: EditRegion[];
}

export type EditJobEvent =
    | { type: 'job.progress'; jobId: string; sessionId: string; progress: number; stage: string }
    | { type: 'job.completed'; jobId: string; sessionId: string; publishedTopic: string; pointCount: number; durationMs: number; message?: string }
    | { type: 'job.failed'; jobId: string; sessionId: string; error: { code: string; message: string } };

export type ParamValue = number | string | boolean;

export interface ParameterInfo {
    name: string;
    description: string;
    type: 'float' | 'int' | 'bool' | 'string';
    min: number;
    max: number;
    step: number;
    value: ParamValue;
}

export interface NodeParameters {
    nodeName: string;
    parameters: ParameterInfo[];
}

export interface SetParameterResult {
    success: boolean;
    paramName: string;
    value: ParamValue;
}

// --- Constants ---

export const LAYER_COLORS = [
    '#7c8c66', // 0: DEFAULT (muted gray-green)
    '#1f8f3a', // 1: SAFE_TERRAIN (deep green)
    '#FF0000', // 2: COLLISION (Red)
    '#FFFF00', // 3: DANGER (Yellow)
    '#2aa84a', // 4: UNUSED / legacy (green fallback)
    '#53c86a'  // 5: OTHER / fallback (soft green)
];

export const LAYER_LABELS = [
    "DEFAULT", "SAFE_TERRAIN", "WALL", "UNKNOWN_OBJECT", "HUMAN", "CAR"
];

export const SEMANTIC_LABELS = [
   "HANDLE"
];

export const SEMANTIC_COLORS = [
    '#00d1ff',
];

export const STATIC_GNG_DEFAULTS = {
    nodeColor: '#1f8f3a',
    edgeColor: '#08d408',
    opacity: 0.25,
    nodeEmissiveIntensity: 1.00,
    edgeEmissiveIntensity: 1.00,
} as const;

export const DYNAMIC_GNG_DEFAULTS = {
    nodeColor: '#1f8f3a',
    edgeColor: '#08d408',
    nodeEmissiveIntensity: 1.00,
    edgeEmissiveIntensity: 1.00,
} as const;

export type ClippingAxis = 'x' | 'y' | 'z' | 'none';

export interface ClippingPlane {
    id: string;
    axis: ClippingAxis;
    position: number;
    min: number;
    max: number;
    inverted: boolean;
    enabled: boolean;
}

export interface ClippingBounds {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
    minZ: number;
    maxZ: number;
}
