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
    nonplaneComponentId?: number;
    is_boundary_candidate?: boolean;
    boundary_evidence?: number;
    winnerPointCount?: number;
    // 集約元の姿勢数。合計0または未指定の場合は従来のラベル表示
    num_safe_states?: number;
    num_danger_states?: number;
    num_collision_states?: number;
    winnerPointCovariance?: [number, number, number, number, number, number, number, number, number];
    isGoal?: boolean;
    manipValid?: boolean;
    manipValue?: number;
    manipConditionNumber?: number;
    manipScale?: [number, number, number];
    manipOrientation?: [number, number, number, number];
    rotationalManipValid?: boolean;
    rotationalManipValue?: number;
    rotationalManipConditionNumber?: number;
    rotationalManipScale?: [number, number, number];
    rotationalManipOrientation?: [number, number, number, number];
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
    nodeIds: number[];  // Viewer受信時の正規化後の所属ノードID
    hasVelocityObservation?: boolean;
    velCovXx?: number;
    velCovXy?: number;
    velCovYy?: number;
}

export type GraphMode = 'static' | 'dynamic';

export interface GraphData {
    timestamp: number;
    nodes: GraphNode[];
    edges: number[]; // 接続端点の配列添字 [始点, 終点, ...]
    clusters: GraphCluster[];
    clusterLabels?: number[];
    frameId?: string;
    tag?: string;
    mode?: GraphMode;
}


export const LAYER_COLORS = [
    '#7c8c66', // 0: 通常（灰緑）
    '#1f8f3a', // 1: 安全（緑）
    '#FF0000', // 2: 衝突（赤）
    '#FFFF00', // 3: 危険（黄）
    '#d946ef', // 4: HUMAN（赤紫）
    '#8b5cf6'  // 5: CAR（青紫）
];

export const SEMANTIC_COLORS = [
    '#00d1ff',
    '#708090',
    '#00d1ff',
    '#2a7898',
];
