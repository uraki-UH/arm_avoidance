import { useEffect, useRef } from 'react';

export function useLongPress(callback: () => void, ms = 80) {
    const timerRef = useRef<number | null>(null);

    const stop = () => {
        if (timerRef.current !== null) {
            window.clearInterval(timerRef.current);
            timerRef.current = null;
        }
    };

    const start = () => {
        stop();
        callback();
        timerRef.current = window.setInterval(callback, ms);
    };

    useEffect(() => stop, []);

    return {
        onPointerDown: start,
        onPointerUp: stop,
        onPointerLeave: stop,
    };
}
