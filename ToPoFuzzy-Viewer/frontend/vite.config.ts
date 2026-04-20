import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    host: true,  // 同一ネットワーク内からのアクセスを許可
    port: 5173,
  },
})
