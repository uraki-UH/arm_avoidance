import { ControlSlider } from '../../components/ui/SharedControls';
import { RobotSettings } from '../../types';
import { robot_candidate_idx } from './robot_candidate_display';

export function RobotCandidateControls({ settings, num_candidates, on_update }: {
    settings: Pick<RobotSettings, 'max_visible_candidates' | 'selected_candidate_idx'>;
    num_candidates: number;
    on_update: (updates: Partial<RobotSettings>) => void;
}) {
    const selected_idx = robot_candidate_idx(settings.selected_candidate_idx, num_candidates);
    const is_single = selected_idx !== null;
    const max_visible = settings.max_visible_candidates ?? 0;
    return <div className="space-y-2">
        <div className="grid grid-cols-3 gap-1">
            <button className={`entity-btn justify-center text-[10px] ${!is_single && max_visible === 0 ? 'active-indigo' : ''}`}
                aria-pressed={!is_single && max_visible === 0}
                onClick={() => on_update({ selected_candidate_idx: null, max_visible_candidates: 0 })}>全件に戻す</button>
            <button className={`entity-btn justify-center text-[10px] ${!is_single && max_visible > 0 ? 'active-indigo' : ''}`}
                aria-pressed={!is_single && max_visible > 0} disabled={num_candidates === 0}
                onClick={() => on_update({ selected_candidate_idx: null, max_visible_candidates: max_visible || 1 })}>先頭N件</button>
            <button className={`entity-btn justify-center text-[10px] ${is_single ? 'active-indigo' : ''}`}
                aria-pressed={is_single} disabled={num_candidates === 0}
                onClick={() => on_update({ selected_candidate_idx: selected_idx ?? 0 })}>1体選択</button>
        </div>
        <ControlSlider label={is_single ? '表示する候補（配信順）' : '先頭N件表示（配信順・0: 全件）'}
            value={is_single ? selected_idx + 1 : max_visible} min={is_single ? 1 : 0}
            max={Math.max(1, num_candidates, is_single ? 0 : max_visible)} step={1}
            disabled={num_candidates === 0}
            formatValue={value => num_candidates === 0 ? '候補なし' : is_single ? `${value} / ${num_candidates}` : value === 0 ? '全件' : `${value}件`}
            onChange={value => on_update(is_single ? { selected_candidate_idx: value - 1 } : { max_visible_candidates: value })} />
    </div>;
}
