import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App.tsx'

// Release lingering WebGL contexts from previous HMR cycles BEFORE React mounts.
// Must run synchronously here (not in useEffect) to avoid destroying the new Canvas context.
// Browsers cap simultaneous WebGL contexts (~16 in Chrome); dev reloads leak one per cycle.
{
    const canvases = document.querySelectorAll('canvas');
    canvases.forEach((canvas) => {
        const gl = (canvas as HTMLCanvasElement).getContext('webgl2') ||
                   (canvas as HTMLCanvasElement).getContext('webgl');
        if (gl) {
            const ext = gl.getExtension('WEBGL_lose_context');
            if (ext) { ext.loseContext(); }
        }
    });
}

// StrictMode removed: causes WebGL context exhaustion with @react-three/fiber Canvas
// (double-mount in dev mode creates contexts that aren't properly disposed)
createRoot(document.getElementById('root')!).render(
    <App />
)
