import type { DataSource, RobotData } from '../types';

export const robot_source_id = (tag: string): string => `robot:${tag}`;

export function merge_robot_state(existing: RobotData | undefined, update: RobotData): RobotData {
    return existing ? {
        ...existing, ...update,
        urdf: existing.urdf ?? update.urdf,
        jointNames: update.jointNames?.length ? update.jointNames : existing.jointNames,
        jointValues: update.jointValues?.length ? update.jointValues : existing.jointValues,
    } : update;
}

/** 入力一覧と画面ごとの選択状態。ロボットはモデルと最新姿勢を合わせた論理入力。 */
export class stream_source_registry {
    // サーバーの共有配信状態を初期選択へ反映する許可。埋込み画面では無効
    allow_server_selection = true;
    topics = new Map<string, DataSource>();
    robots = new Map<string, RobotData>();
    choices = new Map<string, boolean>();
    unavailable_sources = new Set<string>();

    is_enabled(source_id: string): boolean {
        if (this.unavailable_sources.has(source_id)) return false;
        if (source_id.startsWith('robot:') && !this.robots.has(source_id.slice('robot:'.length))) return false;
        return this.choices.get(source_id) ?? (this.allow_server_selection
            ? this.topics.get(source_id)?.active ?? !source_id.startsWith('robot:')
            : false);
    }

    list(): DataSource[] {
        const items = [...this.topics.values()];
        for (const [tag, robot] of this.robots) {
            items.push({ id: robot_source_id(tag), name: robot.displayName || tag, type: 'robot', active: false });
        }
        return items.map(item => ({ ...item, active: this.is_enabled(item.id) }));
    }

    clear(): void {
        this.unavailable_sources.clear();
        this.topics.clear();
        this.robots.clear();
    }
}
