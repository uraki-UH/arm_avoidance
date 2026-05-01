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
    x: number;
    y: number;
    z: number;
    nx: number;
    ny: number;
    nz: number;
    label: number;
    age: number;
}

export interface GraphCluster {
    id: number;
    label: number;
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

export interface GraphTransform {
    position: [number, number, number];
    rotation: [number, number, number];
    scale: [number, number, number];
}

export interface LayerSettings {
    visible: boolean;
    showNodes: boolean;
    showEdges: boolean;
    showClusters: boolean;
    opacity: number;
    nodeColor?: string;
    edgeColor?: string;
    graphTransform?: GraphTransform;
}
export interface RobotSettings {
    visible: boolean;
    color: string;
    showVisual: boolean;
    showCollision: boolean;
    collisionColor: string;
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

// --- WebSocket / RPC Types ---

export interface DataSource {
    id: string;
    name: string;
    type: 'pointcloud' | 'topological_map';
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
    '#c8ff4a', // 1: SAFE_TERRAIN (yellow-green)
    '#FF0000', // 2: COLLISION (Red)
    '#FFFF00', // 3: DANGER (Yellow)
    '#b9ff3f', // 4: UNUSED / legacy (yellow-green fallback)
    '#d4ff6a'  // 5: OTHER / fallback (light yellow-green)
];

export const LAYER_LABELS = [
    "default", "safe", "collision", "danger", "unused", "other"
];
