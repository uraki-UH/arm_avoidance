import { useEffect } from 'react';
import { useThree } from '@react-three/fiber';

/**
 * Hook to trigger a re-render in 'demand' frameloop when specific dependencies change.
 */
export function useDemandUpdate(dependencies: any[]) {
    const { invalidate } = useThree();

    useEffect(() => {
        invalidate();
    }, [...dependencies, invalidate]);
}
