import { useEffect } from 'react';
import { useThree } from '@react-three/fiber';

/**
 * Hook to trigger a re-render in 'demand' frameloop when specific dependencies change.
 */
export function useDemandUpdate(dependencies: readonly unknown[]) {
    const { invalidate } = useThree();

    useEffect(() => {
        invalidate();
        // Renderers intentionally provide the visual dependency list for this shared hook.
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [...dependencies, invalidate]);
}
