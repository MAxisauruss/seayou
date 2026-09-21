// Live connection to the drone's own Pi 5 dashboard backend
// (Raspberry Pi 5\dashboard\server.py).
//
// This file used to own its own WebSocket and was read-only. It is now a
// thin view onto `droneLink`, which owns the single socket this app has
// to the drone and also carries flight control. The reason for the move
// is in droneLink.ts's header: the Pi keeps ONE shared control slot
// across all clients, so the app must have exactly one link and one
// sender.
//
// The hook below still returns exactly what it always did, so anything
// that only wants telemetry can keep using it unchanged and stays
// read-only - a component has to reach for droneLink.takeControl()
// before a single control packet is sent.
//
// The drone's address is no longer hardcoded here; it resolves to
// drone.local with a ?drone=<host> URL override. See droneLink.ts.
import { useSyncExternalStore } from "react";
import { droneLink } from "./droneLink";

export {
  DRONE_HOST,
  DRONE_PORT,
  LEVEL_RESULT_TEXT,
  droneVideoStreamUrl,
  setDroneHost,
  droneLink,
  GP_BUTTON_NAMES,
  TRIM_LIMIT,
  ARM_HOLD_MS,
  DEADZONE,
  BINDINGS,
  linkMode,
} from "./droneLink";

export type {
  MissionStatus,
  DetectionStatus,
  DroneAttitude,
  DroneLevel,
  DroneGps,
  DroneBaro,
  DroneTelemetry,
  DroneLinkState,
  DroneSnapshot,
  GamepadInfo,
  LinkMode,
} from "./droneLink";

import type { DroneTelemetry, DroneLinkState, DroneBaro } from "./droneLink";

interface DroneTelemetryState {
  linkState: DroneLinkState;
  telemetry: DroneTelemetry | null;
  packetAgeSeconds: number | null;
  /** Is the aircraft itself on the link? See droneLink's DroneSnapshot. */
  droneConnected: boolean;
}

/**
 * Always true in THIS build, and that is the whole point of it.
 *
 * The public mirror ships the same pages against a read-only relay
 * client, where the equivalent flag is false until someone passes
 * ?relay=... and the pages fall back to their static demo content. This
 * build is the ground-station dashboard: it always has a drone to talk
 * to, so the pages always render live values.
 *
 * It exists so the two builds can share page code verbatim. Do not
 * "simplify" it away - that is what forks the pages.
 */
export const hasRelay = true;

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

/**
 * Read-only telemetry, reconnecting automatically on drop - the drone's
 * link can bounce during flight-controller reflashes and power cycles,
 * and this dashboard should recover on its own rather than needing a
 * page reload.
 */
export function useDroneTelemetry(): DroneTelemetryState {
  const snap = useSyncExternalStore(droneLink.subscribe, droneLink.getSnapshot);
  return {
    linkState: snap.linkState,
    telemetry: snap.telemetry,
    packetAgeSeconds: snap.packetAgeSeconds,
    droneConnected: snap.droneConnected,
  };
}

/**
 * Everything the telemetry hook gives you, plus arm state, throttle,
 * trim and the connected controller. Only components that actually fly
 * the drone should use this one.
 */
export function useDroneLink() {
  return useSyncExternalStore(droneLink.subscribe, droneLink.getSnapshot);
}
