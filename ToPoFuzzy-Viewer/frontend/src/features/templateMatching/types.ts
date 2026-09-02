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
    section: 'Orientation' | 'Evidence' | 'Rejection' | 'Decision';
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
    { key: 'roll_tolerance_deg', label: 'Roll tolerance (+/- deg)', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: 8, min: 0, max: 8, step: 0.5 },
    { key: 'pitch_tolerance_deg', label: 'Pitch tolerance (+/- deg)', node: 'matcher', section: 'Orientation', type: 'number', defaultValue: 8, min: 0, max: 8, step: 0.5 },
    { key: 'shape_tolerance', label: 'Shape tolerance', node: 'matcher', section: 'Evidence', type: 'number', defaultValue: 0.35, min: 0, max: 1, step: 0.01 },
    { key: 'min_visible_ratio', label: 'Minimum visible ratio', node: 'validator', section: 'Evidence', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'scale_tolerance', label: 'Scale tolerance (+/- ratio)', node: 'matcher', section: 'Rejection', type: 'number', defaultValue: 0.30, min: 0.05, max: 0.95, step: 0.01 },
    { key: 'contradiction_limit', label: 'Contradiction limit', node: 'matcher', section: 'Rejection', type: 'number', defaultValue: 0.20, min: 0, max: 1, step: 0.01 },
    { key: 'recognition_threshold', label: 'Recognition threshold', node: 'validator', section: 'Decision', type: 'number', defaultValue: 0.55, min: 0, max: 1, step: 0.01 },
    { key: 'confirmation_time_sec', label: 'Confirmation time', node: 'validator', section: 'Decision', type: 'number', defaultValue: 0.5, min: 0, max: 10, step: 0.1 },
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
