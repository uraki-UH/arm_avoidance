import React, { useState, useEffect, useRef } from 'react';

interface DualRangeSliderProps {
    min: number;
    max: number;
    step?: number;
    value: [number, number];
    onChange: (value: [number, number]) => void;
    className?: string;
}

export const DualRangeSlider: React.FC<DualRangeSliderProps> = ({
    min,
    max,
    step = 1,
    value,
    onChange,
    className = ''
}) => {
    const [localValue, setLocalValue] = useState(value);
    const containerRef = useRef<HTMLDivElement>(null);
    const isDraggingRef = useRef<'min' | 'max' | null>(null);

    useEffect(() => {
        setLocalValue(value);
    }, [value]);

    const getPercentage = (val: number) => ((val - min) / (max - min)) * 100;

    const handlePointerDown = (thumb: 'min' | 'max') => (e: React.PointerEvent) => {
        e.preventDefault();
        isDraggingRef.current = thumb;
        document.addEventListener('pointermove', handlePointerMove);
        document.addEventListener('pointerup', handlePointerUp);
        // Capture the pointer to receive events even if it moves outside the element
        (e.target as HTMLElement).setPointerCapture(e.pointerId);
    };

    const handlePointerMove = (e: PointerEvent) => {
        if (!isDraggingRef.current || !containerRef.current) return;

        const rect = containerRef.current.getBoundingClientRect();
        const percentage = Math.min(Math.max((e.clientX - rect.left) / rect.width, 0), 1);
        let newValue = min + percentage * (max - min);

        // Snap to step
        newValue = Math.round(newValue / step) * step;

        setLocalValue(prev => {
            const next = [...prev] as [number, number];
            if (isDraggingRef.current === 'min') {
                next[0] = Math.min(newValue, prev[1] - step);
            } else {
                next[1] = Math.max(newValue, prev[0] + step);
            }
            onChange(next);
            return next;
        });
    };

    const handlePointerUp = () => {
        isDraggingRef.current = null;
        document.removeEventListener('pointermove', handlePointerMove);
        document.removeEventListener('pointerup', handlePointerUp);
    };

    const minPos = getPercentage(localValue[0]);
    const maxPos = getPercentage(localValue[1]);

    return (
        <div className={`relative w-full h-6 flex items-center select-none ${className}`} ref={containerRef}>
            {/* Track Background */}
            <div className="absolute w-full h-1 bg-white/20 rounded-full overflow-hidden">
                {/* Active Range */}
                <div
                    className="absolute h-full bg-[var(--accent-color)]"
                    style={{ left: `${minPos}%`, width: `${maxPos - minPos}%` }}
                />
            </div>

            {/* Min Thumb */}
            <div
                className="absolute w-4 h-4 bg-white rounded-full shadow-md cursor-grab active:cursor-grabbing hover:scale-110 transition-transform"
                style={{ left: `${minPos}%`, transform: 'translateX(-50%)' }}
                onPointerDown={handlePointerDown('min')}
            />

            {/* Max Thumb */}
            <div
                className="absolute w-4 h-4 bg-white rounded-full shadow-md cursor-grab active:cursor-grabbing hover:scale-110 transition-transform"
                style={{ left: `${maxPos}%`, transform: 'translateX(-50%)' }}
                onPointerDown={handlePointerDown('max')}
            />
        </div>
    );
};
