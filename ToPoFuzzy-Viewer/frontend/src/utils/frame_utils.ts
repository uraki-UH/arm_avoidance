export function is_fixed_frame(frame_id: string | undefined): boolean {
    return frame_id === 'world' || frame_id === 'map';
}
