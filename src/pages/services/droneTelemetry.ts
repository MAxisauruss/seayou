/**
 * SeaYou live telemetry - the deployed dashboard's read-only link to the
 * aircraft.
 *
 * ---------------------------------------------------------------------
 * THIS FILE CANNOT COMMAND THE DRONE, AND THAT IS STRUCTURAL.
 * ---------------------------------------------------------------------
 * There is no `send` anywhere in it. Not a disabled button, not a flag
 * that could be flipped by a stray call - the socket is opened, listened
 * to, and never written to. Flying happens from the ground station on the
 * laptop, on the same network as the drone, where the pilot can look up
 * and see the aircraft.
 *
 * Why a relay, and why the drone does not connect to this site directly:
 *
 *   1. Vercel has no WebSocket server. Its functions are request/response
 *      only, so there is no endpoint for the Pi to dial into and none for
 *      this page to listen on. No setting adds one.
 *   2. A https: page is forbidden from opening ws:// or http:// to a LAN
 *      address. Mixed content - blocked SILENTLY, so it presents as "the
 *      drone is offline" rather than as a rule.
 *
 * So this page talks to a relay: one https address forwarding to
 * `groundstation/server.py` on the laptop (a `cloudflared tunnel` in
 * front of it needs no account). The drone itself never touches the
 * relay - it stays on the laptop hotspot on a local socket and the laptop
 * makes the single hop out, so none of this can add latency to the
 * flying path.
 *
 * The relay is given at RUNTIME, not built in, because a quick tunnel
 * gets a new hostname every time it starts:
 *
 *   https://seayou-indol.vercel.app/?relay=https://xyz.trycloudflare.com&token=<viewer token>
 *
 * It is remembered afterwards, so the plain link keeps working. Clear it
 * with ?relay=off.
 *
 * WITH NO RELAY CONFIGURED THIS MODULE DOES NOTHING AT ALL - it opens no
 * socket and `hasRelay` is false, so the pages keep showing the original
 * static demo. Deploying this cannot change how the site looks to someone
 * who just opens the plain link.
 */
import { useSyncExternalStore } from "react";

// ---------------------------------------------------------------------
// Which relay, if any
// ---------------------------------------------------------------------

interface Relay {
  url: string;
  /** Read-only token the ground station issues (--viewer-token). */
  token: string;
}

/** Accept "host", "https://host" or "https://host/" alike. */
function normalise(raw: string): string {
  const trimmed = raw.trim().replace(/\/+$/, "");
  if (!trimmed) return "";
  return /^https?:\/\//i.test(trimmed) ? trimmed : `https://${trimmed}`;
}

function resolveRelay(): Relay | null {
  try {
    const q = new URLSearchParams(window.location.search);
    const asked = q.get("relay");
    if (asked !== null) {
      if (!asked || asked === "off") {
        window.localStorage.removeItem("relayUrl");
        window.localStorage.removeItem("relayToken");
        return null;
      }
      const found: Relay = {
        url: normalise(asked),
        token: (q.get("token") ?? q.get("viewer") ?? "").trim(),
      };
      window.localStorage.setItem("relayUrl", found.url);
      window.localStorage.setItem("relayToken", found.token);
      return found;
    }
    const url = window.localStorage.getItem("relayUrl");
    if (!url) return null;
    return { url, token: window.localStorage.getItem("relayToken") || "" };
  } catch {
    // Private mode, or no window - just stay on the static demo.
    return null;
  }
}

const relay = resolveRelay();

/** True when this page has been pointed at a live ground station. */
export const hasRelay = relay !== null;

/** The relay in use, for the UI to show. Null on the plain link. */
export const RELAY_URL = relay?.url ?? null;

const viewerQuery = relay?.token
  ? `?viewer=${encodeURIComponent(relay.token)}`
  : "";

/**
 * The drone's camera, re-served by the ground station as
 * multipart/x-mixed-replace. Safe in an <img src>; an image stream is not
 * subject to CORS because no script reads it.
 */
export const droneVideoStreamUrl = relay
  ? `${relay.url}/stream.mjpg${viewerQuery}`
  : "";

// ---------------------------------------------------------------------
// Telemetry shapes, exactly as the ground station relays them
// ---------------------------------------------------------------------

export interface DroneAttitude {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface DroneGps {
  fix: string | null;
  has_fix: boolean;
  sat_count: number;
  /** Raw NMEA as the module emits it. */
  lat_raw: string | null;
  lon_raw: string | null;
  /** Decimal degrees, or null when there is no fix. NEVER treat null as 0. */
  lat: number | null;
  lon: number | null;
}

export interface DroneBaro {
  temp_c: number;
  pressure_hpa: number;
  height_m: number;
}

/** Waypoint state, computed onboard the Pi and relayed in telemetry. */
export interface MissionStatus {
  state: "idle" | "running" | "holding" | "arrived" | "aborted";
  reason: string;
  target: { lat: number; lon: number } | null;
  target_alt_m: number;
  distance_m: number | null;
  bearing_deg: number | null;
  elapsed_s: number;
  interrupted_by_detection?: boolean;
}

/** Onboard drowning detector, also computed on the Pi. */
export interface DetectionStatus {
  enabled: boolean;
  ready: boolean;
  error: string | null;
  confirmed: boolean;
  frames: number;
  fps: number | null;
}

export interface DroneTelemetry {
  attitude: DroneAttitude;
  gps: DroneGps;
  baro: DroneBaro | null;
  mission?: MissionStatus;
  detection?: DetectionStatus;
  ts: number;
}

export type LinkState = "idle" | "connecting" | "connected" | "disconnected";

export interface TelemetrySnapshot {
  /** State of this page's socket to the relay. */
  linkState: LinkState;
  /** Whether the AIRCRAFT is connected to the ground station. Different
   *  thing entirely: the relay can be up while the drone is switched off. */
  droneConnected: boolean;
  telemetry: DroneTelemetry | null;
  /** Seconds since the last packet, for spotting a frozen feed. */
  packetAgeSeconds: number | null;
}

type Listener = () => void;

// ---------------------------------------------------------------------
// The store
// ---------------------------------------------------------------------

/** Backoff between reconnect attempts, milliseconds. */
const RECONNECT_MS = 2000;

class TelemetryLink {
  private ws: WebSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private ageTimer: ReturnType<typeof setInterval> | null = null;
  private listeners = new Set<Listener>();
  /** Refcounted, because React StrictMode mounts every effect twice in
   *  development and would otherwise tear down the socket it just made. */
  private refs = 0;

  private snapshot: TelemetrySnapshot = {
    linkState: hasRelay ? "connecting" : "idle",
    droneConnected: false,
    telemetry: null,
    packetAgeSeconds: null,
  };

  subscribe = (fn: Listener): (() => void) => {
    this.listeners.add(fn);
    this.refs += 1;
    if (this.refs === 1) this.start();
    return () => {
      this.listeners.delete(fn);
      this.refs -= 1;
      if (this.refs === 0) this.stop();
    };
  };

  getSnapshot = (): TelemetrySnapshot => this.snapshot;

  private set(next: Partial<TelemetrySnapshot>) {
    this.snapshot = { ...this.snapshot, ...next };
    this.listeners.forEach((fn) => fn());
  }

  private start() {
    if (!relay) return; // No relay: stay idle, open nothing.
    this.connect();
    this.ageTimer = setInterval(() => {
      const t = this.snapshot.telemetry;
      this.set({ packetAgeSeconds: t ? Date.now() / 1000 - t.ts : null });
    }, 1000);
  }

  private stop() {
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    if (this.ageTimer) clearInterval(this.ageTimer);
    this.reconnectTimer = this.ageTimer = null;
    const ws = this.ws;
    this.ws = null;
    if (ws) {
      // Drop the handlers first, so closing does not schedule a reconnect
      // for a link nobody is watching any more.
      ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
      ws.close();
    }
  }

  private connect = () => {
    if (!relay || this.refs === 0) return;
    this.set({ linkState: "connecting" });

    // https -> wss, http -> ws. The ground station speaks plain WebSocket;
    // it is the relay in front of it that supplies the TLS.
    const url = `${relay.url.replace(/^http/i, "ws")}/ws${viewerQuery}`;
    const ws = new WebSocket(url);
    this.ws = ws;

    ws.onopen = () => {
      if (this.ws !== ws) return;
      this.set({ linkState: "connected" });
    };

    ws.onmessage = (event) => {
      if (this.ws !== ws) return;
      try {
        const msg = JSON.parse(event.data);
        if (msg.type === "telemetry") {
          this.set({ telemetry: msg.data, packetAgeSeconds: 0 });
        } else if (msg.type === "drone_status") {
          this.set({ droneConnected: !!msg.data?.connected });
        }
      } catch {
        // One malformed frame is not a reason to tear down the link.
      }
    };

    const scheduleReconnect = () => {
      if (this.ws !== ws) return;
      // The aircraft is unaffected by any of this - nothing here commands
      // it. Losing the mirror loses the view, nothing else.
      this.set({ linkState: "disconnected", droneConnected: false });
      this.reconnectTimer = setTimeout(this.connect, RECONNECT_MS);
    };
    ws.onclose = scheduleReconnect;
    ws.onerror = () => ws.close();
  };
}

const link = new TelemetryLink();

/** Live telemetry, reconnecting on its own. Read-only, always. */
export function useDroneTelemetry(): TelemetrySnapshot {
  return useSyncExternalStore(link.subscribe, link.getSnapshot);
}

// ---------------------------------------------------------------------
// Formatting helpers, so every page shows a missing value the same way
// ---------------------------------------------------------------------

/** Decimal degrees -> "34.10830° S". Null stays "NO FIX", never 0. */
export function formatLat(lat: number | null | undefined): string {
  if (lat === null || lat === undefined) return "NO FIX";
  return `${Math.abs(lat).toFixed(5)}° ${lat >= 0 ? "N" : "S"}`;
}

export function formatLon(lon: number | null | undefined): string {
  if (lon === null || lon === undefined) return "NO FIX";
  return `${Math.abs(lon).toFixed(5)}° ${lon >= 0 ? "E" : "W"}`;
}

export function formatHeight(baro: DroneBaro | null | undefined): string {
  if (!baro || typeof baro.height_m !== "number") return "—";
  return `${baro.height_m.toFixed(1)}M`;
}
