import { useEffect, useRef, useState } from 'react';
import type { graph_snapshot } from '../../types';
import type { register_vehicle, vehicle_registration_result } from './types';

export function useVehicleRegistration(snapshot: graph_snapshot, register?: register_vehicle) {
    const [stored_result, set_result] = useState<{ snapshot: graph_snapshot; value: vehicle_registration_result } | null>(null);
    const [selected_idx, set_selected_idx] = useState(0);
    const [dist_th, set_dist_th] = useState(0.25);
    const [support_dist_th, set_support_dist_th] = useState(0.35);
    const [is_running, set_is_running] = useState(false);
    const [error, set_error] = useState<string | null>(null);
    const req_serial = useRef(0);
    useEffect(() => {
        set_is_running(false);
        set_error(null);
        set_result(null);
        set_selected_idx(0);
        const serial = req_serial;
        return () => { serial.current++; };
    }, [snapshot]);
    const run = async () => {
        if (!register) return;
        const req_id = ++req_serial.current;
        set_is_running(true);
        set_error(null);
        try {
            const value = await register(snapshot, dist_th, support_dist_th);
            if (req_serial.current === req_id) {
                set_result({ snapshot, value });
                set_selected_idx(0);
            }
        } catch (error) {
            if (req_serial.current === req_id) set_error(error instanceof Error ? error.message : String(error));
        } finally {
            if (req_serial.current === req_id) set_is_running(false);
        }
    };
    const result = stored_result?.snapshot === snapshot ? stored_result.value : null;
    const candidate = result?.candidates[selected_idx];
    return { result, candidate, selected_idx, set_selected_idx, dist_th, set_dist_th,
        support_dist_th, set_support_dist_th, is_running, error, run };
}
