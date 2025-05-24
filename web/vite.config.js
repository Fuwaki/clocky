import { defineConfig } from 'vite';
import wasm from "vite-plugin-wasm";
import topLevelAwait from "vite-plugin-top-level-await";

export default defineConfig({
  plugins: [
    wasm(),
    topLevelAwait()
  ],
  server: {
    fs: {
      allow: [
        // Add external path to whitelist
        '/run/media/fuwaki/Workspace/Projects/clocky',
      ],
    },
  },
});