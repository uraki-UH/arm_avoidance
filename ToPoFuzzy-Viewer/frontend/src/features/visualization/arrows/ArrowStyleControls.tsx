import { arrow_dimensions, arrow_style, resolve_arrow_style } from './geometry';
import { update_arrow_settings, useArrowSettings } from './settings';

export function ArrowStyleControls({ style_key, can_have_orientation, title = '矢印表示', base_style }: {
    style_key: string; can_have_orientation: boolean; title?: string; base_style?: Partial<arrow_style>;
}) {
    const overrides = useArrowSettings(style_key);
    const style = resolve_arrow_style({ ...base_style, ...overrides });
    const update = (value: Partial<arrow_style>) => update_arrow_settings(style_key, { ...overrides, ...value });
    const dimensions = (value: arrow_dimensions, change: (value: Partial<arrow_dimensions>) => void) => <>
        {([['length', '全長'], ['shaft_diameter', '軸直径'], ['head_length', '矢先長'], ['head_diameter', '矢先直径']] as const).map(([key, label]) =>
            <label key={key} className="flex justify-between gap-2">{label} [m]
                <input className="w-20 bg-black/20" aria-label={label} type="number" min="0.0001" step="0.001"
                    value={value[key]} onChange={event => {
                        const next = event.currentTarget.valueAsNumber;
                        if (Number.isFinite(next) && next > 0) change({ [key]: next });
                    }} />
            </label>)}
        <label className="flex justify-between">色<input type="color" value={value.color}
            onChange={event => change({ color: event.target.value })} /></label>
    </>;
    return <details className="mt-1 text-xs" onClick={event => event.stopPropagation()}>
        <summary>{title}</summary>
        <div className="space-y-1 p-1">
            <label className="flex justify-between">基準位置<select value={style.anchor} className="bg-black/20"
                onChange={event => update({ anchor: event.target.value as arrow_style['anchor'] })}>
                <option value="tail">根元</option><option value="tip">矢先</option><option value="center">中点</option>
            </select></label>
            {can_have_orientation && <label className="flex justify-between">主軸<select value={style.primary_axis}
                className="bg-black/20" onChange={event => update({ primary_axis: event.target.value as arrow_style['primary_axis'] })}>
                <option value="x">X</option><option value="y">Y</option><option value="z">Z</option>
            </select></label>}
            {dimensions(style, value => update(value))}
            {style.head_length > style.length && <p>矢先長を全長以内に設定してください。</p>}
            <label className="flex justify-between">不透明度<input className="w-20" type="range" min="0" max="1" step="0.05"
                value={style.opacity} onChange={event => update({ opacity: event.currentTarget.valueAsNumber })} /></label>
            <label>前面表示<input type="checkbox" checked={style.depth_mode === 'overlay'}
                onChange={event => update({ depth_mode: event.target.checked ? 'overlay' : 'scene' })} /></label>
            <label className="block">状態による色<input type="checkbox" checked={style.enable_state_colors}
                onChange={event => update({ enable_state_colors: event.target.checked })} /></label>
            {style.enable_state_colors && (['未評価', '範囲内', '範囲外']).map((label, state) =>
                <label key={state} className="flex justify-between">{label}<input type="color" value={style.state_colors[state] ?? style.color}
                    onChange={event => update({ state_colors: { ...style.state_colors, [state]: event.target.value } })} /></label>)}
            <label className="block">補助2軸<input type="checkbox" disabled={!can_have_orientation}
                checked={can_have_orientation && style.enable_transverse_axes}
                onChange={event => update({ enable_transverse_axes: event.target.checked })} /></label>
            {!can_have_orientation && <p>方向のみの入力では補助軸はありません。</p>}
            {can_have_orientation && style.enable_transverse_axes && (['x', 'y', 'z'] as const)
                .filter(axis => axis !== style.primary_axis).map(axis => <details key={axis}>
                    <summary>補助 {axis.toUpperCase()} 軸</summary>
                    {dimensions(style.transverse_axes[axis], value => update({ transverse_axes: { ...style.transverse_axes, [axis]: { ...style.transverse_axes[axis], ...value } } }))}
                </details>)}
            <button type="button" onClick={() => update_arrow_settings(style_key, {})}>入力・既定設定に戻す</button>
        </div>
    </details>;
}
