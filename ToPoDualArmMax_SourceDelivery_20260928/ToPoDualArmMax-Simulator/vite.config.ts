import tailwindcss from '@tailwindcss/vite';
import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';

// Three.jsは既存シーンと同じimport mapのインスタンスを共用。
export default defineConfig({
  plugins: [{ name: 'viewer-css', enforce: 'pre', transform(source, id) {
    if (id.includes('/ToPoFuzzy-Viewer/frontend/src/index.css')) return source.replace(/@tailwind (base|components|utilities);/g, '');
  } }, tailwindcss()],
  resolve: { dedupe: ['react', 'react-dom', 'three', '@react-three/fiber', '@react-three/drei', 'lucide-react', 'urdf-loader'], alias: { '@viewer': fileURLToPath(new URL('../../ToPoFuzzy-Viewer/frontend/src', import.meta.url)), '@topo/visualization': fileURLToPath(new URL('../../libs/ros_visualization_web', import.meta.url)) } },
  build: {
    outDir: 'app/generated', emptyOutDir: true,
    lib: { entry: 'src/ros-results.tsx', formats: ['es'], fileName: () => 'ros-results.js', cssFileName: 'ros-results' },
    rollupOptions: { external: ['three'] },
  },
  define: { 'process.env.NODE_ENV': JSON.stringify('production') },
});
