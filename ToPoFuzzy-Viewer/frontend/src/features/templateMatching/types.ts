export type TemplateMatchParameterValue = number | boolean;
export type TemplateMatchNodeKind = 'matcher' | 'validator';

export interface TemplateMatchTargets {
    matcherNode: string;
    validatorNode: string;
}

export interface TemplateMatchConfig extends TemplateMatchTargets {
    matcher: Record<string, TemplateMatchParameterValue>;
    validator: Record<string, TemplateMatchParameterValue>;
}

export interface TemplateMatchConfigResult extends TemplateMatchConfig {
    success: boolean;
    reason?: string;
}

export interface TemplateMatchParameterDefinition {
    key: string;
    label: string;
    node: TemplateMatchNodeKind;
    section: 'Orientation' | 'Node' | 'Plane' | 'Graph & Scale' | 'Contradiction' | 'Validation';
    type: 'boolean' | 'number';
    defaultValue: TemplateMatchParameterValue;
    min?: number;
    max?: number;
    step?: number;
}

export const DEFAULT_TEMPLATE_MATCH_TARGETS: TemplateMatchTargets = {
    matcherNode: '/object_template_matcher_node',
    validatorNode: '/object_template_match_validator_node',
};

export const TEMPLATE_MATCH_PARAMETER_DEFINITIONS: TemplateMatchParameterDefinition[] = [
    { key: 'enable_yaw_search', label: 'Yaw search', node: 'matcher', section: 'Orientation', type: 'boolean', defaultValue: true },
    { key: 'yaw_min_deg', label: 'Yaw minimum', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: -180, min: -180, max: 180, step: 1 },
    { key: 'yaw_max_deg', label: 'Yaw maximum', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: 180, min: -180, max: 180, step: 1 },
    { key: 'yaw_step_deg', label: 'Yaw step', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: 10, min: 1, max: 90, step: 1 },
    { key: 'enable_roll_pitch_search', label: 'Roll / pitch search', node: 'matcher', section: 'Orientation', type: 'boolean', defaultValue: false },
    { key: 'max_orientation_hypothesis_num', label: 'Hypothesis limit', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: 1000, min: 1, max: 5000, step: 1 },

    { key: 'max_normal_angle_full_deg', label: 'Normal full', node: 'matcher', section: 'Node', type: 'number', defaultValue: 12, min: 0, max: 45, step: 0.5 },
    { key: 'max_normal_angle_partial_deg', label: 'Normal reject', node: 'matcher', section: 'Node', type: 'number', defaultValue: 35, min: 1, max: 90, step: 0.5 },
    { key: 'enable_rho_evaluation', label: 'Rho evaluation', node: 'matcher', section: 'Node', type: 'boolean', defaultValue: true },
    { key: 'max_rho_dev_full_ratio', label: 'Rho full', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.15, min: 0, max: 1, step: 0.01 },
    { key: 'max_rho_dev_partial_ratio', label: 'Rho reject', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.45, min: 0.01, max: 2, step: 0.01 },
    { key: 'max_degree_dev_full', label: 'Degree full', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0, min: 0, max: 5, step: 0.1 },
    { key: 'max_degree_dev_partial', label: 'Degree reject', node: 'matcher', section: 'Node', type: 'number', defaultValue: 2, min: 0.1, max: 10, step: 0.1 },
    { key: 'normal_weight', label: 'Normal weight', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.55, min: 0, max: 1, step: 0.01 },
    { key: 'rho_weight', label: 'Rho weight', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'degree_weight', label: 'Degree weight', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.25, min: 0, max: 1, step: 0.01 },
    { key: 'min_node_score', label: 'Node score minimum', node: 'matcher', section: 'Node', type: 'number', defaultValue: 0.55, min: 0, max: 1, step: 0.01 },

    { key: 'enable_plane_cluster_evaluation', label: 'Plane evaluation', node: 'matcher', section: 'Plane', type: 'boolean', defaultValue: true },
    { key: 'max_plane_normal_angle_deg', label: 'Plane normal reject', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 35, min: 1, max: 90, step: 0.5 },
    { key: 'min_plane_extent_allow_ratio', label: 'Extent minimum', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 0.35, min: 0.01, max: 1, step: 0.01 },
    { key: 'min_plane_extent_full_match_ratio', label: 'Extent full minimum', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 0.70, min: 0.01, max: 1.5, step: 0.01 },
    { key: 'max_plane_extent_full_match_ratio', label: 'Extent full maximum', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 1.10, min: 0.1, max: 2, step: 0.01 },
    { key: 'max_plane_extent_overflow_ratio', label: 'Extent reject', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 1.30, min: 0.1, max: 3, step: 0.01 },
    { key: 'plane_weight', label: 'Plane weight', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 0.65, min: 0, max: 1, step: 0.01 },
    { key: 'plane_support_score_scale', label: 'Support scale', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 1.80, min: 0.05, max: 5, step: 0.05 },
    { key: 'min_plane_support_score', label: 'Strong support', node: 'matcher', section: 'Plane', type: 'number', defaultValue: 0.60, min: 0, max: 1, step: 0.01 },
    { key: 'enable_oversized_plane_filter', label: 'Oversized filter', node: 'matcher', section: 'Plane', type: 'boolean', defaultValue: true },

    { key: 'edge_weight', label: 'Edge weight', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 0.35, min: 0, max: 1, step: 0.01 },
    { key: 'enable_scale_evaluation', label: 'Scale evaluation', node: 'matcher', section: 'Graph & Scale', type: 'boolean', defaultValue: true },
    { key: 'min_scale_allow_ratio', label: 'Scale minimum', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 0.70, min: 0.1, max: 1, step: 0.01 },
    { key: 'min_scale_full_match_ratio', label: 'Scale full minimum', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 0.95, min: 0.1, max: 1.5, step: 0.01 },
    { key: 'max_scale_full_match_ratio', label: 'Scale full maximum', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 1.05, min: 0.5, max: 2, step: 0.01 },
    { key: 'max_scale_allow_ratio', label: 'Scale maximum', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 1.30, min: 1, max: 3, step: 0.01 },
    { key: 'scale_weight', label: 'Scale weight', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'min_scale_edge_num', label: 'Scale edge minimum', node: 'matcher', section: 'Graph & Scale', type: 'number', defaultValue: 3, min: 1, max: 30, step: 1 },

    { key: 'enable_contradiction_evaluation', label: 'Contradiction evaluation', node: 'matcher', section: 'Contradiction', type: 'boolean', defaultValue: true },
    { key: 'contradiction_weight', label: 'Contradiction weight', node: 'matcher', section: 'Contradiction', type: 'number', defaultValue: 0.40, min: 0, max: 1, step: 0.01 },
    { key: 'max_contradiction_point_ratio', label: 'Contradiction limit', node: 'matcher', section: 'Contradiction', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },

    { key: 'activate_score_th', label: 'Activate score', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.65, min: 0, max: 1, step: 0.01 },
    { key: 'deactivate_score_th', label: 'Deactivate score', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.40, min: 0, max: 1, step: 0.01 },
    { key: 'min_matched_node_ratio', label: 'Matched node minimum', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'min_matched_edge_ratio', label: 'Matched edge minimum', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.15, min: 0, max: 1, step: 0.01 },
    { key: 'enable_plane_cluster_evidence', label: 'Plane evidence', node: 'validator', section: 'Validation', type: 'boolean', defaultValue: true },
    { key: 'min_plane_support_score', label: 'Plane support minimum', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.60, min: 0, max: 1, step: 0.01 },
    { key: 'max_missing_node_ratio', label: 'Missing node maximum', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.80, min: 0, max: 1, step: 0.01 },
    { key: 'max_contradiction_point_ratio', label: 'Contradiction maximum', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'min_confirmed_frame_num', label: 'Confirmation frames', node: 'validator', section: 'Validation', type: 'number', defaultValue: 5, min: 1, max: 100, step: 1 },
    { key: 'min_confirm_duration_sec', label: 'Confirmation duration', node: 'validator', section: 'Validation', type: 'number', defaultValue: 0.5, min: 0, max: 10, step: 0.1 },
    { key: 'max_lost_duration_sec', label: 'Lost duration', node: 'validator', section: 'Validation', type: 'number', defaultValue: 1.0, min: 0, max: 10, step: 0.1 },
];

export function makeDefaultTemplateMatchConfig(): TemplateMatchConfig {
    const config: TemplateMatchConfig = {
        ...DEFAULT_TEMPLATE_MATCH_TARGETS,
        matcher: {},
        validator: {},
    };
    for (const definition of TEMPLATE_MATCH_PARAMETER_DEFINITIONS) {
        config[definition.node][definition.key] = definition.defaultValue;
    }
    return config;
}
