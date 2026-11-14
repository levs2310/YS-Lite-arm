import { defineConfig } from 'vite'

// 精简版：只做静态预览 + 模型目录自动扫描
// - public/URDF 下的 .urdf / .STL 会被 Vite 当静态资源直接访问
// - 不再有 sim_settings / WebSocket 之类的业务插件
export default defineConfig({
  server: {
    host: '127.0.0.1',
    port: 5173,
    open: true,
  },
  build: {
    outDir: 'dist',
    chunkSizeWarningLimit: 2000,
  },
})
