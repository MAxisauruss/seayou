import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Where the drone's Pi 5 lives.
//
// The dev server PROXIES to it, so the browser only ever talks to this
// server. That matters for anything other than the dev machine: a phone
// or a teammate's laptop then does not have to resolve "drone.local"
// itself, and Android is notoriously unreliable at .local (mDNS) names -
// the single most likely thing to break a demo on a phone.
//
// This only affects `npm run dev`. The BUILT app is served by the Pi
// itself, so it talks to the drone over plain same-origin paths and
// never touches this proxy (see linkMode in services/droneLink.ts).
//
// Override without editing this file:
//   DRONE_HOST=192.168.0.50 npm run dev
//
// Read off globalThis rather than as a bare `process`, because
// @types/node is not installed in this project and a bare reference
// would not typecheck.
const env = (globalThis as { process?: { env?: Record<string, string | undefined> } })
  .process?.env;
const DRONE_HOST = env?.DRONE_HOST || "drone.local";

export default defineConfig({
  plugins: [react()],

  // base: '/my-website/' was here, left over from a GitHub Pages template.
  // Nothing deploys to that path - the ground station serves this build at
  // the root (groundstation/server.py registers "/" plus one static route
  // per folder in dist/), and so does the public Vercel mirror. With the
  // prefix, index.html asks for /my-website/assets/... and every asset
  // 404s: a blank page with no error on screen. If a GitHub Pages deploy
  // is ever added, give it `--base=/my-website/` at build time instead of
  // putting this back here and breaking the other two.

  server: {
    // Listen on every interface, not just localhost, so a phone or another
    // laptop on the same WiFi can open the dashboard. Note this does expose
    // the dev server to the local network while it is running.
    host: true,
    proxy: {
      "/drone": {
        target: `http://${DRONE_HOST}:8080`,
        changeOrigin: true,
        // The telemetry + control link is a WebSocket. Without this the
        // upgrade request gets proxied as an ordinary GET and the link
        // never opens at all.
        ws: true,
        rewrite: (p) => p.replace(/^\/drone/, ""),
        // Both long-lived streams have to survive here: /ws stays open for
        // the whole flight, and /stream.mjpg is an endless
        // multipart/x-mixed-replace response. Any timeout would cut them
        // mid-flight, so both are disabled deliberately.
        timeout: 0,
        proxyTimeout: 0,
      },
    },
  },
});
