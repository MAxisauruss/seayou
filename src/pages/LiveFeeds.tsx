import { useEffect, useState } from "react";
import "./LiveFeeds.css";
import { Page } from "../types";
import FlightControl from "../components/FlightControl";
import MissionControl from "../components/MissionControl";
import {
  hasRelay,
  droneVideoStreamUrl,
  useDroneTelemetry,
  formatLat,
  formatLon,
  formatHeight,
} from "../services/droneTelemetry";

interface LiveFeedsProps {
  onNavigate: (page: Page) => void;
  activePage: Page;
}

type CameraId = "drone" | "zone-01" | "zone-02" | "zone-03" | "zone-04";

const cameras: Array<{
  id: CameraId;
  label: string;
  alt: string;
  src?: string;
  detection?: { label: string; className: string };
}> = [
  {
    id: "drone",
    label: "DRONE-04 // AERIAL",
    alt: "A high-altitude aerial view of a turquoise ocean meeting a white sandy coastline",
    src: "https://lh3.googleusercontent.com/aida-public/AB6AXuAq7wPmlIeKjkHsZHTMfNMWxDHQTYOB6jVEoXdv09T8rBFPyxtchas3gNTHFIs8ZHMN3571dYaSsCdoTW2gVO9cYJN2NH2J3O7nMK53gRncCnpE8mY9wCJcUvAiEwA4IEWsd6yVORc-lkG-aEXMBhkv_qB-o4NNJvwa4FLQQO1hwd1swA9NV4NWQxfGIE0Ns0CJjvRZP7UkZmWft1s5H6JO5ELzkfkx2oQfOe_xCr0aQ8Mv5UbfgXscOOv7eEI8jSqTuZkzKRNzThw",
  },
  {
    id: "zone-01",
    label: "ZONE 01 - NORTH",
    alt: "A CCTV camera angle over a crowded public beach with blue umbrellas, with an AI detection box highlighting swimmers",
    src: "https://lh3.googleusercontent.com/aida-public/AB6AXuCWJ1xBj4ts1AjauqBXohFMrysiCuEoHpOY4AgvN3QNduCI4qu9o5Opx5QfKa2pp8kJNm9xORovzVPWNr3HLqyDu4ktwZ-IUZuvrB_OjBewiIb2I70VUwhFhp9RRO1Jx1fqq3eWokdghSNAXSzodg5o_kBgFHlK0MHBQKAIRjI1hx2ePWbcTM94T33TK15IRlN-Lx7WX9D6ordBcx0-2egOCe0kZhL34N_EgpJJELTUQHJtEDDTvGVByMpy0pk4XRS3MlkiFPBVZB4",
    detection: { label: "HUMAN 0.98", className: "top-[30%] left-[45%] w-16 h-24" },
  },
  {
    id: "zone-02",
    label: "ZONE 02 - PIER",
    alt: "A wide-angle security feed of a wooden beach pier extending into water, with an AI detection box highlighting a vessel",
    src: "https://lh3.googleusercontent.com/aida-public/AB6AXuCb7GgnBDnvxNJnEgMNOzC5-UdWUWfGkzKQTMiZEW7ofEG10rGSUfQBkvVULtL3x9bglkNI6I-XySqCJ7VluaWxFLtltM9RSI2gSZRw2_urbD8SlUEd_QiFpDov_5D5dv7YUHnyX7iO_xLeAqznuutJ7FVFf-ys2TysgISu0R2ZyEM6rnu_YIMiMjJHakskMJv9vbw3dClFpeH9rhHceWWkiD1Vtlt2bs5TbnH_fJ7TxpSA8vSTM2WLQeNOLJhRYzoODYa7nEuPND0",
    detection: { label: "VESSEL 0.92", className: "top-[50%] left-[20%] w-20 h-12" },
  },
  {
    id: "zone-03",
    label: "ZONE 03 - SHORE",
    alt: "A low-angle surveillance view from a shoreline post showing waves crashing on a sun-drenched beach",
    src: "https://lh3.googleusercontent.com/aida-public/AB6AXuBRpYbVekLs-0da3gtnK8E9NJPbIxBan8xow61s-Mwjy7shEeF2rWUDeQ3rsO6Uid-U4NKr-aBTgM2CKEODwDm8t7Q6RJNoneHnSMGhTlYeIRhyhVuWfcZtS_a5RBDZ-m-3v56dwbTObofWBoBdsRTiGfnB9WAjdAinOigYJKtXGfQlxC0tJc9oZ_OIIe9dJxUVPze32qb_LtfvUwVCTQIUbLiGpKwborA8-NgDou1RuJ7utBSA8w_Y4bCsS633YniduoKSOYc47Uo",
  },
  { id: "zone-04", label: "ZONE 04 - OFFLINE", alt: "Zone 04 camera signal lost" },
];

export default function LiveFeeds({ onNavigate, activePage }: LiveFeedsProps) {
  // Read-only. See services/droneTelemetry.ts - that module has no send
  // path at all, so this page cannot command the aircraft even by mistake.
  //
  // Every live value below falls back to the original demo content when no
  // relay is configured, which is the case for anyone who just opens the
  // public link. Deploying this does not change what they see.
  const { linkState, droneConnected, telemetry, packetAgeSeconds } =
    useDroneTelemetry();
  // The ground station answers /stream.mjpg with 503 when the aircraft has
  // sent it no frames - no camera fitted, or the simulator. Without this
  // the panel renders a broken-image icon and alt text, which reads as "the
  // site is broken" rather than "this drone has no camera".
  const [videoFailed, setVideoFailed] = useState(false);
  const [selectedCamera, setSelectedCamera] = useState<CameraId>("drone");
  const gps = telemetry?.gps;
  const mission = telemetry?.mission;
  const detection = telemetry?.detection;
  const live = hasRelay && linkState === "connected" && droneConnected;
  // A feed that has stopped updating looks identical to a still one, so
  // the age is worth showing rather than hiding behind "connected".
  const stale = live && packetAgeSeconds !== null && packetAgeSeconds > 3;
  const activeCamera = cameras.find((camera) => camera.id === selectedCamera) ?? cameras[0];
  const gridCameras = cameras.filter((camera) => camera.id !== selectedCamera);

  // Give the stream another chance whenever the link comes back, rather
  // than staying stuck on the fallback until someone reloads the page.
  useEffect(() => {
    if (live) setVideoFailed(false);
  }, [live]);

  return (
    <div className="live-feeds-page h-screen flex flex-col">
      {/* Top Logo */}
      <header className="fixed top-0 left-5 w-full z-50 flex items-center">
        <div className="flex items-center gap-4">

          <h1 className="font-display-lg text-display-lg-mobile md:text-display-lg text-primary tracking-tight">SeaYou</h1>
        </div>
        
      </header>

      {/* Main Container */}
      <div className="flex flex-1 pt-16 h-full overflow-hidden">
        {/* Side Navigation Bar */}
        <nav className="hidden md:flex flex-col h-full py-6 gap-4 fixed left-0 top-0 w-64 bg-surface-container-low/80 backdrop-blur-2xl border-r border-white/10 shadow-2xl z-40 pt-20">
          <div className="px-6 mb-4">
            <div className="flex items-center gap-3 mb-2">
              <div className="w-10 h-10 rounded-full bg-secondary-container flex items-center justify-center">
                <span className="material-symbols-outlined text-white" data-icon="person">
                  person
                </span>
              </div>
              <div>
                <h3 className="font-headline-md text-[14px] text-primary">Sector Alpha</h3>
                <p className="font-label-caps text-label-caps text-on-surface-variant">Active Ops</p>
              </div>
            </div>
          </div>
          <div className="flex flex-col gap-1">
            <button
              className={`mx-2 flex items-center gap-3 px-4 py-3 rounded-[30px] font-bold text-white shadow-[0_30px_10px_-20px_rgba(0,0,0,0.2)] transition-all duration-300 ${
                activePage === "rescue-response"
                  ? "bg-[length:300%] hover:bg-[length:320%] bg-left hover:bg-right"
                  : "bg-transparent hover:bg-white/5"
              }`}
              style={
                activePage === "rescue-response"
                  ? {
                      background: "linear-gradient(15deg, #de6f3d, #f09f33, #de6f3d)",
                      textShadow: "2px 2px 3px rgb(255, 132, 0)",
                    }
                  : undefined
              }
              onClick={() => onNavigate?.("rescue-response")}
            >
              <span className="material-symbols-outlined" data-icon="dashboard" style={{ color: "#fbfbfb", transition: "0.3s ease" }}>
                dashboard
              </span>
              <span className="font-label-caps text-label-caps" style={{ color: "#ffffff", transition: "0.3s ease" }}>
                Dashboard
              </span>
            </button>

            <button
              className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-[#FF6B35]/5 transition-all ${
                activePage === "asset-map" ? "bg-[#FF6B35]/10 rounded-lg" : ""
              }`}
              onClick={() => onNavigate?.("asset-map")}
            >
              <span className="material-symbols-outlined" data-icon="explore">
                explore
              </span>
              <span className="font-label-caps text-label-caps">Asset Map</span>
            </button>
            <button
              className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all dashboard-btn ${
                activePage === "live-feeds" ? "bg-[#FF6B35]/10 rounded-lg" : ""
              }`}
              onClick={() => onNavigate?.("live-feeds")}
            >
              <span className="material-symbols-outlined" data-icon="videocam">
                videocam
              </span>
              <span className="font-label-caps text-label-caps">Live Feeds</span>
            </button>
            <button
              className={`text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all ${
                activePage === "rescue-logs" ? "bg-[#FF6B35]/10 rounded-lg" : ""
              }`}
              onClick={() => onNavigate?.("rescue-logs")}
            >
              <span
                className="material-symbols-outlined"
                data-icon="history"
                data-weight="fill"
                style={{ fontVariationSettings: "'FILL' 1" }}
              >
                history
              </span>
              <span className="font-label-caps text-label-caps">Rescue Logs</span>
            </button>
            <button
              className="text-on-surface-variant hover:text-on-surface flex items-center gap-3 px-4 py-3 mx-2 hover:bg-white/5 transition-all"
              onClick={() => onNavigate?.("rescue-logs")}
            >
              <span className="material-symbols-outlined" data-icon="bar_chart">
                bar_chart
              </span>
              <span className="font-label-caps text-label-caps">Analytics</span>
            </button>
          </div>
          <div className="mt-auto px-4 pb-4">
            <button className="w-full py-4 bg-secondary-container text-white font-bold rounded-lg emergency-glow border border-secondary pulse-emergency flex items-center justify-center gap-2 active:scale-95 transition-transform">
              <span className="material-symbols-outlined" data-icon="warning">
                warning
              </span>
              SOS EMERGENCY
            </button>
            <div className="mt-6 flex flex-col gap-2">
              <a className="flex items-center gap-3 px-4 py-2 text-on-surface-variant text-[12px] hover:text-on-surface" href="#">
                <span className="material-symbols-outlined text-[18px]" data-icon="help">
                  help
                </span>
                Support
              </a>
              <a className="flex items-center gap-3 px-4 py-2 text-on-surface-variant text-[12px] hover:text-on-surface" href="#">
                <span className="material-symbols-outlined text-[18px]" data-icon="dns">
                  dns
                </span>
                System Status
              </a>
            </div>
          </div>
        </nav>

        {/* Surveillance Canvas */}
        <main className="flex-1 flex flex-col overflow-hidden bg-background md:ml-64">
          {/* Surveillance Split View */}
          <div className="flex-1 grid grid-cols-1 lg:grid-cols-[20rem_1fr] xl:grid-cols-[20rem_1fr_1fr] gap-px bg-white/10 overflow-hidden">
            {/* Control column - the only part of this page that can fly the
                aircraft. Kept off the video itself so nothing of ours sits
                on top of the feed. */}
            <div className="bg-surface-container overflow-y-auto p-3 flex flex-col gap-3 order-first">
              {/* STICKY. This block carries STOP, and an emergency stop that
                  can scroll off the screen is not an emergency stop. */}
              <div className="sticky top-0 z-10 -mx-3 -mt-3 px-3 pt-3 pb-1 bg-surface-container">
                <FlightControl />
              </div>
              <MissionControl />
            </div>

            {/* Left Panel: 4K Drone Feed */}
            <div className="relative bg-black overflow-hidden group">
              {activeCamera.id === "drone" && hasRelay ? (
                /* The drone's own camera, re-served by the ground station as
                   multipart/x-mixed-replace. An <img> is the whole client -
                   the browser holds the connection open and repaints every
                   frame, no library involved. */
                videoFailed ? (
                  <div className="w-full h-full flex flex-col items-center justify-center gap-2 bg-black">
                    <span
                      className="material-symbols-outlined text-4xl text-on-surface-variant"
                      data-icon="videocam_off"
                    >
                      videocam_off
                    </span>
                    <span className="font-label-caps text-label-caps text-on-surface-variant">
                      NO CAMERA // TELEMETRY LINK OK
                    </span>
                  </div>
                ) : (
                  <img
                    alt="Live camera feed from the drone"
                    className="w-full h-full object-cover"
                    src={droneVideoStreamUrl}
                    onError={() => setVideoFailed(true)}
                  />
                )
              ) : activeCamera.src ? (
                <img
                  alt={activeCamera.alt}
                  className="w-full h-full object-cover opacity-80"
                  src={activeCamera.src}
                />
              ) : (
                <div className="w-full h-full bg-[#050B14] flex flex-col items-center justify-center gap-4">
                  <span className="material-symbols-outlined text-4xl text-white/20 animate-pulse" data-icon="videocam_off">
                    videocam_off
                  </span>
                  <span className="font-label-caps text-label-caps text-on-surface-variant">SIGNAL LOST // ZONE 04</span>
                </div>
              )}
              {activeCamera.id !== "drone" && (
                <div className="absolute top-6 left-6 bg-black/60 px-3 py-1 rounded text-xs font-telemetry-sm text-white">
                  {activeCamera.label}
                </div>
              )}
              {/* Drone Telemetry Overlay */}
              <div className="absolute inset-0 pointer-events-none p-6 flex flex-col justify-between">
                <div className="flex justify-between items-start">
                  <div className="glass-panel p-4 rounded-lg flex flex-col gap-2">
                    <span className="font-label-caps text-label-caps text-tertiary">DRONE-04 // SECTOR ALPHA</span>
                    <div className="flex items-center gap-4">
                      <div className="flex flex-col">
                        <span className="font-label-caps text-[10px] text-on-surface-variant">
                          {hasRelay ? "HEIGHT" : "ALTITUDE"}
                        </span>
                        <span className="font-telemetry-lg text-telemetry-lg text-white">
                          {hasRelay ? formatHeight(telemetry?.baro) : "42.5M"}
                        </span>
                      </div>
                      <div className="w-px h-8 bg-white/10"></div>
                      {/* SATS, not BATTERY: the aircraft reports no battery
                          state, and a made-up percentage next to real
                          numbers is worse than no number. Satellites are
                          real, and they are the thing that decides whether
                          autonomous flight can run at all. */}
                      <div className="flex flex-col">
                        <span className="font-label-caps text-[10px] text-on-surface-variant">
                          {hasRelay ? "SATS" : "BATTERY"}
                        </span>
                        <span
                          className={`font-telemetry-lg text-telemetry-lg ${
                            hasRelay && !gps?.has_fix ? "text-[#FF6B35]" : "text-[#00E676]"
                          }`}
                        >
                          {hasRelay ? (live ? (gps?.sat_count ?? 0) : "—") : "84%"}
                        </span>
                      </div>
                    </div>
                  </div>
                  <div className="flex flex-col gap-2 items-end">
                    <div className="bg-secondary-container/80 backdrop-blur-md px-3 py-1 rounded border border-secondary/20 flex items-center gap-2">
                      <span
                        className={`w-2 h-2 rounded-full ${
                          !hasRelay || live ? "bg-red-500 animate-pulse" : "bg-white/30"
                        }`}
                      ></span>
                      <span className="font-telemetry-sm text-telemetry-sm text-white">
                        {!hasRelay
                          ? "LIVE REC 4K"
                          : live
                            ? stale
                              ? `STALE ${packetAgeSeconds?.toFixed(0)}s`
                              : "LIVE TELEMETRY"
                            : linkState === "connected"
                              ? "DRONE OFFLINE"
                              : linkState.toUpperCase()}
                      </span>
                    </div>
                    {/* Said plainly and permanently, so nobody watching this
                        page expects to be able to do anything with it. */}
                    {hasRelay && (
                      <div className="glass-panel px-3 py-1 rounded flex items-center gap-2">
                        <span
                          className="material-symbols-outlined text-sm text-[#FF6B35]"
                          data-icon="visibility"
                        >
                          visibility
                        </span>
                        <span className="font-telemetry-sm text-telemetry-sm text-white">
                          READ-ONLY MIRROR
                        </span>
                      </div>
                    )}
                    <div className="glass-panel px-3 py-1 rounded flex items-center gap-2">
                      <span className="material-symbols-outlined text-sm text-tertiary" data-icon="thermostat">
                        thermostat
                      </span>
                      <span className="font-telemetry-sm text-telemetry-sm text-white">THERMAL: OFF</span>
                    </div>
                  </div>
                </div>
                {/* Center HUD Elements */}
                <div className="absolute inset-0 flex items-center justify-center opacity-30">
                  <div className="w-64 h-64 border border-white/20 rounded-full flex items-center justify-center">
                    <div className="w-1 h-16 bg-white/40 absolute top-0"></div>
                    <div className="w-1 h-16 bg-white/40 absolute bottom-0"></div>
                    <div className="w-16 h-1 bg-white/40 absolute left-0"></div>
                    <div className="w-16 h-1 bg-white/40 absolute right-0"></div>
                  </div>
                </div>
                {/* Bottom Telemetry */}
                <div className="flex justify-between items-end">
                  <div className="flex flex-col gap-2">
                    {/* Autonomous flight, computed ONBOARD the Pi and relayed
                        in telemetry. Shown, never commanded: starting or
                        aborting a waypoint is commanding the aircraft, so it
                        stays on the ground station. The server refuses it
                        here too - a viewer token gets 403 from /mission. */}
                    {hasRelay && mission && mission.state !== "idle" && (
                      <div className="glass-panel p-3 rounded flex gap-4">
                        <div className="flex flex-col">
                          <span className="font-label-caps text-[9px] text-on-surface-variant">
                            AUTONOMOUS
                          </span>
                          <span
                            className={`font-telemetry-sm text-telemetry-sm ${
                              mission.state === "aborted"
                                ? "text-[#FF3B30]"
                                : mission.state === "running"
                                  ? "text-[#FF6B35]"
                                  : "text-[#00E676]"
                            }`}
                          >
                            {mission.state.toUpperCase()}
                          </span>
                        </div>
                        <div className="flex flex-col">
                          <span className="font-label-caps text-[9px] text-on-surface-variant">
                            TO RUN
                          </span>
                          <span className="font-telemetry-sm text-telemetry-sm text-white">
                            {mission.distance_m != null ? `${mission.distance_m} m` : "—"}
                          </span>
                        </div>
                        <div className="flex flex-col">
                          <span className="font-label-caps text-[9px] text-on-surface-variant">
                            BEARING
                          </span>
                          <span className="font-telemetry-sm text-telemetry-sm text-white">
                            {mission.bearing_deg != null ? `${mission.bearing_deg}°` : "—"}
                          </span>
                        </div>
                        {detection?.confirmed && (
                          <div className="flex flex-col">
                            <span className="font-label-caps text-[9px] text-on-surface-variant">
                              DETECTOR
                            </span>
                            <span className="font-telemetry-sm text-telemetry-sm text-[#FF3B30]">
                              ⚠ HOLDING
                            </span>
                          </div>
                        )}
                      </div>
                    )}
                    <div className="glass-panel p-3 rounded flex gap-4">
                      <div className="flex flex-col">
                        <span className="font-label-caps text-[9px] text-on-surface-variant">LAT</span>
                        <span className="font-telemetry-sm text-telemetry-sm text-white">
                          {hasRelay ? (live ? formatLat(gps?.lat) : "—") : "34.0194° N"}
                        </span>
                      </div>
                      <div className="flex flex-col">
                        <span className="font-label-caps text-[9px] text-on-surface-variant">LONG</span>
                        <span className="font-telemetry-sm text-telemetry-sm text-white">
                          {hasRelay ? (live ? formatLon(gps?.lon) : "—") : "118.4912° W"}
                        </span>
                      </div>
                      {hasRelay && (
                        <div className="flex flex-col">
                          <span className="font-label-caps text-[9px] text-on-surface-variant">
                            ATTITUDE
                          </span>
                          <span className="font-telemetry-sm text-telemetry-sm text-white">
                            {live && telemetry
                              ? `R ${telemetry.attitude.roll.toFixed(0)}° / P ${telemetry.attitude.pitch.toFixed(0)}°`
                              : "—"}
                          </span>
                        </div>
                      )}
                    </div>
                  </div>
                  <div className="flex gap-2">
                    <button className="p-2 glass-panel rounded hover:bg-white/10 transition-colors pointer-events-auto">
                      <span className="material-symbols-outlined text-white" data-icon="videocam_off">
                        videocam_off
                      </span>
                    </button>
                    <button className="p-2 glass-panel rounded hover:bg-white/10 transition-colors pointer-events-auto">
                      <span className="material-symbols-outlined text-white" data-icon="zoom_in">
                        zoom_in
                      </span>
                    </button>
                    <button className="p-2 glass-panel rounded hover:bg-white/10 transition-colors pointer-events-auto">
                      <span className="material-symbols-outlined text-white" data-icon="screenshot">
                        screenshot
                      </span>
                    </button>
                  </div>
                </div>
              </div>
            </div>

            {/* Right Panel: CCTV Grid */}
            <div className="bg-surface-container flex flex-col overflow-hidden">
              <div className="p-4 border-b border-white/10 flex justify-between items-center bg-surface-container-high">
                <span className="font-label-caps text-label-caps text-primary">CCTV GRID // SHORELINE MONITORING</span>
                <div className="flex items-center gap-2">
                  <span className="font-telemetry-sm text-telemetry-sm text-on-surface-variant">6 NODES ACTIVE</span>
                  <span className="material-symbols-outlined text-sm text-[#00E676]" data-icon="check_circle">
                    check_circle
                  </span>
                </div>
              </div>
              <div className="flex-1 grid grid-cols-2 grid-rows-2 gap-1 p-1 overflow-y-auto">
                {gridCameras.map((camera) => (
                  <button
                    key={camera.id}
                    type="button"
                    aria-label={`Show ${camera.label} as the main camera`}
                    className="relative bg-black group overflow-hidden text-left cursor-pointer focus:outline-none focus:ring-2 focus:ring-primary"
                    onClick={() => setSelectedCamera(camera.id)}
                  >
                    {camera.src ? (
                      <img
                        alt={camera.alt}
                        className="w-full h-full object-cover opacity-60 grayscale group-hover:grayscale-0 transition-all duration-500"
                        src={camera.src}
                      />
                    ) : (
                      <div className="w-full h-full bg-[#050B14] flex flex-col items-center justify-center gap-4">
                        <span className="material-symbols-outlined text-4xl text-white/20 animate-pulse" data-icon="videocam_off">
                          videocam_off
                        </span>
                        <span className="font-label-caps text-label-caps text-on-surface-variant">SIGNAL LOST // ZONE 04</span>
                      </div>
                    )}
                    <div className="absolute top-2 left-2 bg-black/60 px-2 py-0.5 rounded text-[10px] font-telemetry-sm text-white">
                      {camera.label}
                    </div>
                    {camera.detection && (
                      <div className={`absolute ${camera.detection.className} ai-detection-box pointer-events-none`}>
                        <span className="absolute -top-5 left-0 font-label-caps text-[8px] text-tertiary bg-black/80 px-1">
                          {camera.detection.label}
                        </span>
                      </div>
                    )}
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* Bottom Log & Status Bar */}
          <footer className="h-48 border-t border-white/10 bg-surface-container-lowest flex flex-col md:flex-row overflow-hidden">
            {/* Incident Log */}
            <div className="flex-1 border-r border-white/10 flex flex-col overflow-hidden">
              <div className="p-3 bg-surface-container border-b border-white/5 flex justify-between items-center">
                <span className="font-label-caps text-label-caps text-on-surface-variant">INCIDENT LOG (LAST 60M)</span>
                <span className="material-symbols-outlined text-sm text-on-surface-variant cursor-pointer" data-icon="filter_list">
                  filter_list
                </span>
              </div>
              <div className="flex-1 overflow-y-auto p-2 space-y-1">
                <div className="flex items-center gap-4 px-3 py-2 rounded bg-surface/40 hover:bg-surface transition-colors border-l-2 border-tertiary">
                  <span className="font-telemetry-sm text-[10px] text-on-surface-variant">14:22:05</span>
                  <span className="font-body-md text-xs text-on-surface flex-1">Detection: Unidentified floating object Sector B-4</span>
                  <span className="font-label-caps text-[9px] px-1.5 py-0.5 rounded bg-tertiary-container text-on-tertiary-container">WARNING</span>
                </div>
                <div className="flex items-center gap-4 px-3 py-2 rounded bg-surface/40 hover:bg-surface transition-colors border-l-2 border-secondary">
                  <span className="font-telemetry-sm text-[10px] text-on-surface-variant">14:18:12</span>
                  <span className="font-body-md text-xs text-on-surface flex-1">Patrol Team 02 responding to Zone 01 assistance request</span>
                  <span className="font-label-caps text-[9px] px-1.5 py-0.5 rounded bg-secondary-container text-on-secondary-container">IN PROGRESS</span>
                </div>
                <div className="flex items-center gap-4 px-3 py-2 rounded bg-surface/40 hover:bg-surface transition-colors border-l-2 border-[#00E676]">
                  <span className="font-telemetry-sm text-[10px] text-on-surface-variant">14:05:30</span>
                  <span className="font-body-md text-xs text-on-surface flex-1">Drone-04 Battery Swap Complete. Relaunched.</span>
                  <span className="font-label-caps text-[9px] px-1.5 py-0.5 rounded bg-surface-variant text-on-surface-variant">RESOLVED</span>
                </div>
                <div className="flex items-center gap-4 px-3 py-2 rounded bg-surface/40 hover:bg-surface transition-colors border-l-2 border-[#00E676]">
                  <span className="font-telemetry-sm text-[10px] text-on-surface-variant">13:55:12</span>
                  <span className="font-body-md text-xs text-on-surface flex-1">Automated System Health Check: All sensors Green.</span>
                  <span className="font-label-caps text-[9px] px-1.5 py-0.5 rounded bg-surface-variant text-on-surface-variant">RESOLVED</span>
                </div>
              </div>
            </div>
            {/* Active Patrol Status */}
            <div className="w-full md:w-80 bg-surface-container flex flex-col">
              <div className="p-3 border-b border-white/5 flex justify-between items-center">
                <span className="font-label-caps text-label-caps text-on-surface-variant">PATROL STATUS</span>
                <div className="flex gap-1">
                  <div className="w-2 h-2 rounded-full bg-[#00E676]"></div>
                  <div className="w-2 h-2 rounded-full bg-[#00E676]"></div>
                  <div className="w-2 h-2 rounded-full bg-white/20"></div>
                </div>
              </div>
              <div className="p-4 flex flex-col gap-4">
                <div className="flex items-center gap-3">
                  <div className="w-10 h-10 rounded-full border border-primary/30 flex items-center justify-center bg-primary-container">
                    <span className="material-symbols-outlined text-primary" data-icon="directions_boat">
                      directions_boat
                    </span>
                  </div>
                  <div className="flex flex-col">
                    <span className="font-label-caps text-[10px] text-white">SEA-GUARDIAN 01</span>
                    <span className="font-telemetry-sm text-[10px] text-tertiary">ON-STATION // ZONE 02</span>
                  </div>
                </div>
                <div className="flex items-center gap-3">
                  <div className="w-10 h-10 rounded-full border border-primary/30 flex items-center justify-center bg-primary-container">
                    <span className="material-symbols-outlined text-primary" data-icon="helicopter">
                      helicopter
                    </span>
                  </div>
                  <div className="flex flex-col">
                    <span className="font-label-caps text-[10px] text-white">AIR-RESCUE 09</span>
                    <span className="font-telemetry-sm text-[10px] text-on-surface-variant">STANDBY // BASE ALPHA</span>
                  </div>
                </div>
              </div>
            </div>
          </footer>
        </main>
      </div>

      {/* Floating Action Button for Emergency */}
      <button className="fixed bottom-6 right-6 w-16 h-16 rounded-full bg-[#FF6B35] text-white shadow-[0_0_25px_rgba(255,107,53,0.6)] flex items-center justify-center active:scale-95 transition-all z-50 md:hidden">
        <span className="material-symbols-outlined text-3xl" data-icon="emergency">
          emergency
        </span>
      </button>
    </div>
  );
}
