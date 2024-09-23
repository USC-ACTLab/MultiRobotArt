import react from '@vitejs/plugin-react';
import path from 'path';
import { defineConfig } from 'vite';

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  assetsInclude: ['**/*.glb'],
  base: "/MultiRobotArt/",
  resolve: {
    alias: {
      '@MRAControl': path.resolve(__dirname, './src'),
    },

  },
  build: {
    minify: false,
    terserOptions: {
      compress: false,
      mangle: false,
    },
  }
});
