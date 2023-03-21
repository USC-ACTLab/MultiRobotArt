import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import path from "path";

// https://vitejs.dev/config/
export default defineConfig({
    plugins: [react()],
    assetsInclude: ["**/*.glb"],
    resolve: {
        alias: {
            "@MRAControl": path.resolve(__dirname, "./src"),
        },
    },
});
