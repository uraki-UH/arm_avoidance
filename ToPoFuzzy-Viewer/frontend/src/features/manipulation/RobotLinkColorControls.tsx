import { useMemo } from 'react';
import { Color } from 'three';
import { RobotData, RobotSettings } from '../../types';
import { ControlSlider } from '../../components/ui/SharedControls';

export function RobotLinkColorControls({ robot_data, settings, on_update }: {
    robot_data?: RobotData | null;
    settings: RobotSettings;
    on_update: (updates: Record<string, unknown>) => void;
}) {
    const links = useMemo(() => {
        if (!robot_data?.urdf) return [];
        const document = new DOMParser().parseFromString(robot_data.urdf, 'application/xml');
        if (document.querySelector('parsererror')) return [];
        const robot = document.documentElement;
        return Array.from(robot.querySelectorAll(':scope > link')).filter((link) => link.querySelector('visual')).map((link) => {
            const name = link.getAttribute('name') || '';
            const material = link.querySelector('visual > material');
            const material_name = material?.getAttribute('name');
            const shared_material = Array.from(robot.querySelectorAll(':scope > material')).find((item) => item.getAttribute('name') === material_name);
            const rgba = (material?.querySelector('color') || shared_material?.querySelector('color'))?.getAttribute('rgba')?.trim().split(/\s+/).map(Number);
            const color = rgba && rgba.length >= 3 && rgba.every(Number.isFinite)
                ? `#${new Color().setRGB(rgba[0], rgba[1], rgba[2]).getHexString()}` : '#87ceeb';
            return { name, color };
        });
    }, [robot_data?.urdf]);

    if (!links.length) return null;
    const link_colors = settings.link_colors ?? {};
    const link_appearance = settings.link_appearance ?? {};
    const update_appearance = (name: string, updates: { opacity?: number; emissive_intensity?: number }) => {
        on_update({ link_appearance: { ...link_appearance, [name]: { ...link_appearance[name], ...updates } } });
    };
    return (
        <div className="space-y-2 rounded-md border border-white/5 bg-black/15 p-2">
            <div className="flex items-center justify-between gap-2">
                <span className="text-xs text-gray-200">リンク別の表示設定{robot_data?.instances?.length ? '（全候補に共通）' : ''}</span>
                <button type="button" className="entity-btn text-[10px]" disabled={!Object.keys(link_colors).length && !Object.keys(link_appearance).length}
                    onClick={() => on_update({ link_colors: {}, link_appearance: {} })}>すべて戻す</button>
            </div>
            {links.map((link) => (
                <div key={link.name} className="space-y-2 border-t border-white/5 pt-2">
                    <div className="flex items-center gap-2">
                    <label htmlFor={`robot-link-color-${link.name}`} className="min-w-0 flex-1 break-all text-xs text-gray-200">
                        {link.name}
                    </label>
                    <input id={`robot-link-color-${link.name}`} type="color" className="h-7 w-10 shrink-0 cursor-pointer bg-transparent"
                        value={link_colors[link.name] ?? ((settings.useUrdfColors ?? true) ? link.color : `#${new Color(settings.color).getHexString()}`)}
                        onChange={(event) => on_update({ link_colors: { ...link_colors, [link.name]: event.target.value } })} />
                    <button type="button" aria-label={`${link.name}の表示設定を戻す`} className="entity-btn text-[10px]" disabled={!link_colors[link.name] && !link_appearance[link.name]}
                        onClick={() => {
                            const next_colors = { ...link_colors };
                            const next_appearance = { ...link_appearance };
                            delete next_colors[link.name];
                            delete next_appearance[link.name];
                            on_update({ link_colors: next_colors, link_appearance: next_appearance });
                        }}>戻す</button>
                    </div>
                    <ControlSlider label="Opacity" value={link_appearance[link.name]?.opacity ?? robot_data?.opacity ?? settings.opacity ?? 1}
                        min={0} max={1} step={0.01} onChange={(value) => update_appearance(link.name, { opacity: value })}
                        formatValue={(value) => `${Math.round(value * 100)}%`} />
                    <ControlSlider label="Emissive" value={link_appearance[link.name]?.emissive_intensity ?? settings.emissiveIntensity ?? 0.2}
                        min={0} max={1.5} step={0.01} onChange={(value) => update_appearance(link.name, { emissive_intensity: value })}
                        formatValue={(value) => `${value.toFixed(2)}x`} />
                </div>
            ))}
        </div>
    );
}
