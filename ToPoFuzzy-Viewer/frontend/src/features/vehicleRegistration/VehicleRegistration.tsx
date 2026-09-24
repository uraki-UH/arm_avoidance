import { useMemo } from 'react';
import type { vehicle_candidate } from './types';
import type { useVehicleRegistration } from './use_vehicle_registration';


type registration_state = ReturnType<typeof useVehicleRegistration>;
const percent = (value: number) => `${(value * 100).toFixed(1)}%`;

export function VehicleRegistrationControls({ state, num_nodes }: { state: registration_state; num_nodes: number }) {
    const { result, candidate } = state;
    return <div className="max-h-80 shrink-0 overflow-y-auto border-b border-white/10 p-2 text-xs" aria-label="車両モデル照合">
        <div className="flex flex-wrap items-center gap-2">
            <button className="btn-primary px-3 py-1" disabled={state.is_running || num_nodes < 12} onClick={() => void state.run()}>
                {state.is_running ? '4モデルを照合中…' : '車モデルを比較'}</button>
            <label>一致距離 m <input aria-label="車両一致距離" className="w-16 bg-black/20" type="number"
                min={0.05} max={1} step={0.05} value={state.dist_th} disabled={state.is_running}
                onChange={event => state.set_dist_th(Number(event.target.value))} /></label>
            <label>支持距離 m <input aria-label="車両支持距離" className="w-16 bg-black/20" type="number"
                min={0.05} max={1} step={0.05} value={state.support_dist_th} disabled={state.is_running}
                onChange={event => state.set_support_dist_th(Number(event.target.value))} /></label>
        </div>
        {num_nodes < 12 && <p>12ノード以上のクラスタを選択してください。</p>}
        {state.error && <p role="alert" className="text-red-300">{state.error}</p>}
        {!result && <p className="mt-1 opacity-70">選択時のGNG形状を固定して比較。軽自動車相当・セダン・バン・箱型トラック。</p>}
        {result && <>
            <p role="status" className="mt-2 font-bold">{result.message}</p>
            {candidate && <div className="mt-1 space-y-1">
                <p>未対応率（欠損候補）: <strong>{percent(candidate.unmatched_ratio)}</strong> / 一致点RMS: {candidate.inlier_rms_m === null ? '—' : `${candidate.inlier_rms_m.toFixed(3)} m`}</p>
                <p>モデル寸法: {candidate.dimensions_m.map(value => value.toFixed(2)).join(' × ')} m / {result.elapsed_ms.toFixed(0)} ms</p>
                <p><span style={{ color: '#5de3a6' }}>● 観測支持あり</span> <span style={{ color: '#f6b854' }}>● 未対応のモデル面</span> <span style={{ color: '#ff526e' }}>● 不一致ノード</span></p>
                <details><summary className="cursor-pointer opacity-70">指標・モデルの前提（適合度は認識確率ではありません）</summary>
                    <p>{candidate.model_note}</p>
                    <p>{result.limitations} 距離: 一致 {result.dist_th.toFixed(2)} m / 支持 {result.support_dist_th.toFixed(2)} m。</p>
                </details>
            </div>}
            <table className="mt-1 w-full text-left" style={{ fontSize: 11 }} aria-label="車両候補の比較">
                <thead><tr><th style={{ padding: '2px 4px' }}>候補 / 適合度</th><th style={{ padding: '2px 4px' }}>観測一致</th><th style={{ padding: '2px 4px' }}>モデル支持</th></tr></thead>
                <tbody>{result.candidates.map((item, idx) => <tr key={item.model_id} className={idx === state.selected_idx ? 'bg-purple-500/20' : ''}>
                    <td style={{ padding: '2px 4px' }}><button className="text-left" style={{ padding: 0 }} aria-pressed={idx === state.selected_idx} onClick={() => state.set_selected_idx(idx)}>
                        {item.label} / {item.compatibility.toFixed(2)}</button></td>
                    <td style={{ padding: '2px 4px' }}>{percent(item.match_ratio)}</td><td style={{ padding: '2px 4px' }}>{percent(item.support_ratio)}</td>
                </tr>)}</tbody>
            </table>
        </>}
    </div>;
}

function SurfacePoints({ positions, color, opacity, size }: { positions: number[]; color: string; opacity: number; size: number }) {
    const buffer = useMemo(() => new Float32Array(positions), [positions]);
    return <points frustumCulled={false}>
        <bufferGeometry><bufferAttribute attach="attributes-position" args={[buffer, 3]} /></bufferGeometry>
        <pointsMaterial color={color} size={size} transparent opacity={opacity} depthWrite={false} toneMapped={false} />
    </points>;
}

export function VehicleRegistrationLayer({ candidate }: { candidate?: vehicle_candidate }) {
    if (!candidate) return null;
    return <group>
        <SurfacePoints positions={candidate.matched_positions} color="#5de3a6" opacity={0.95} size={0.055} />
        <SurfacePoints positions={candidate.unmatched_positions} color="#f6b854" opacity={0.38} size={0.04} />
        <SurfacePoints positions={candidate.outlier_positions} color="#ff526e" opacity={1} size={0.12} />
    </group>;
}
