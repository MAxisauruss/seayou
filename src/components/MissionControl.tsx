import { useEffect, useRef, useState } from "react";
import {
  droneLink,
  useDroneLink,
  TRIM_LIMIT,
  LEVEL_RESULT_TEXT,
} from "../services/droneTelemetry";

/**
 * Autonomous flight panel: takeoff, land, and fly-to-coordinate.
 *
 * Talks to the ground station's /mission endpoint (see
 * groundstation/mission.py). All the real safety checks live server-side -
 * height band, range, GPS fix, coordinate validity, already-airborne -
 * because a check that only exists in the browser is a check that can be
 * skipped with curl. What this adds is making the refusal legible before
 * you press anything.
 *
 * ---------------------------------------------------------------------
 * WHY TAKEOFF AND LAND ARE THE USEFUL ONES TODAY
 * ---------------------------------------------------------------------
 * They need the barometer and nothing else. FLY TO WAYPOINT needs a GPS
 * fix, and this aircraft's GPS has never produced one - so the waypoint
 * button is refused by the drone every time, while these two work.
 *
 * And the barometer is COARSE. The firmware sends pressure as
 * (uint16)(pascals / 10), so the smallest height change that can be
 * transmitted is 0.83 m. Height is displayed to two decimal places
 * because that is what arrives, but it steps in 0.83 m jumps and holding
 * a height hunts by about that much. The panel says so rather than
 * letting the decimals imply a precision that is not there.
 */

interface MissionStatus {
  state: "idle" | "running" | "holding" | "arrived" | "aborted" | "takeoff" | "landing";
  reason: string;
  target: { lat: number; lon: number } | null;
  target_alt_m: number;
  distance_m: number | null;
  bearing_deg: number | null;
  elapsed_s: number;
  /** What the integral trim has learned the hover throttle actually is. */
  hover_trim?: number;
  hover_learned?: number;
  limits: {
    max_tilt_deg: number;
    max_speed_ms: number;
    max_range_m: number;
    min_alt_m: number;
    max_alt_m: number;
    max_takeoff_alt_m?: number;
    baro_step_m?: number;
  };
  interrupted_by_detection?: boolean;
}

/** Onboard detector state, reported by the Pi in telemetry. */
interface DetectionStatus {
  enabled: boolean;
  ready: boolean;
  error: string | null;
  confirmed: boolean;
  frames: number;
  avg_ms: number | null;
  fps: number | null;
  alert: number;
}

const STATE_STYLE: Record<string, string> = {
  idle: "text-on-surface-variant",
  takeoff: "text-[#FF6B35]",
  running: "text-[#FF6B35]",
  landing: "text-[#FFC107]",
  holding: "text-[#00E676]",
  arrived: "text-[#00E676]",
  aborted: "text-[#FF3B30]",
};

/** Settings persist, because retyping a waypoint at the field is how it
 *  gets typed wrong. */
function stored(key: string, fallback: string): string {
  try {
    return window.localStorage.getItem("seayou." + key) ?? fallback;
  } catch {
    return fallback;
  }
}

function store(key: string, value: string) {
  try {
    window.localStorage.setItem("seayou." + key, value);
  } catch {
    /* private mode - the settings just will not persist */
  }
}

export default function MissionControl() {
  const link = useDroneLink();
  const telemetry = link.telemetry;
  const [status, setStatus] = useState<MissionStatus | null>(null);
  const [detect, setDetect] = useState<DetectionStatus | null>(null);
  const [showSettings, setShowSettings] = useState(false);

  const [takeoffAlt, setTakeoffAlt] = useState(() => stored("takeoffAlt", "2"));
  const [lat, setLat] = useState(() => stored("wpLat", ""));
  const [lon, setLon] = useState(() => stored("wpLon", ""));
  const [alt, setAlt] = useState(() => stored("wpAlt", "5"));

  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(false);
  // The Pi-hosted server has no /mission endpoint. Rather than showing
  // controls that cannot work, the panel stays hidden until the endpoint
  // answers - so the same build runs against both without dead UI.
  const [available, setAvailable] = useState(false);
  const timer = useRef<number | null>(null);

  useEffect(() => {
    let alive = true;
    const poll = async () => {
      try {
        const r = await fetch("/status");
        if (alive && r.ok) {
          const d = await r.json();
          if (d.mission) setStatus(d.mission);
          if (d.detection) setDetect(d.detection);
          setAvailable(!!d.mission);
        } else if (alive && r.status === 404) {
          setAvailable(false);
        }
      } catch {
        /* ground station not reachable - leave the last state showing */
      }
      if (alive) timer.current = window.setTimeout(poll, 1000);
    };
    poll();
    return () => {
      alive = false;
      if (timer.current) window.clearTimeout(timer.current);
    };
  }, []);

  const post = async (body: Record<string, unknown>) => {
    setBusy(true);
    try {
      const r = await fetch("/mission", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const d = await r.json();
      setMessage(d.message || (r.ok ? "ok" : "refused"));
      setStatus((s) => ({ ...(s as MissionStatus), ...d }));
    } catch {
      setMessage("could not reach the ground station");
    } finally {
      setBusy(false);
    }
  };

  const state = status?.state ?? "idle";
  const flying = state === "takeoff" || state === "running" ||
                 state === "holding" || state === "landing";
  const travelling = state === "running";
  const lim = status?.limits;
  const height = telemetry?.baro?.height_m;
  // What one barometer step is worth WHERE THE AIRCRAFT ACTUALLY IS.
  //
  // The transmitted resolution is a fixed 0.1 hPa, but the height that
  // corresponds to depends on the pressure: 843.7 / hPa metres. That is
  // 0.83 m at sea level and 0.97 m at the ~1280 m this drone lives at, so
  // a hardcoded sea-level figure understates the real error by 17%.
  const step = telemetry?.baro?.pressure_hpa
    ? Math.round((843.7 / telemetry.baro.pressure_hpa) * 100) / 100
    : (lim?.baro_step_m ?? 0.83);
  const maxTakeoff = lim?.max_takeoff_alt_m ?? 10;

  if (!available) return null;

  const field = (
    label: string,
    value: string,
    set: (v: string) => void,
    key: string,
    placeholder: string,
  ) => (
    <label key={label} className="flex flex-col gap-0.5">
      <span className="font-label-caps text-[8px] text-on-surface-variant">{label}</span>
      <input
        value={value}
        onChange={(e) => {
          set(e.target.value);
          store(key, e.target.value);
        }}
        placeholder={placeholder}
        inputMode="decimal"
        disabled={flying}
        className="bg-black/40 border border-white/15 rounded px-2 py-1
                   font-telemetry-sm text-[11px] text-white
                   focus:border-[#FF6B35] outline-none disabled:opacity-40"
      />
    </label>
  );

  return (
    <div className="glass-panel rounded-lg p-3 flex flex-col gap-2">
      <div className="flex items-center justify-between">
        <span className="font-label-caps text-label-caps text-primary">AUTONOMOUS</span>
        <div className="flex items-center gap-2">
          <span className={`font-telemetry-sm text-[11px] ${STATE_STYLE[state] ?? "text-white"}`}>
            {state.toUpperCase()}
          </span>
          <button
            onClick={() => setShowSettings((v) => !v)}
            title="Takeoff height and waypoint"
            className={`p-1 rounded hover:bg-white/10 transition-colors ${
              showSettings ? "text-[#FF6B35]" : "text-on-surface-variant"
            }`}
          >
            <span className="material-symbols-outlined text-[16px]" data-icon="settings">
              settings
            </span>
          </button>
        </div>
      </div>

      {/* Never flown. Saying so on the panel itself, not just in a README
          nobody opens at the field. */}
      <p className="font-body-md text-[10px] text-[#FF6B35] m-0">
        ⚠ Never flown on the real aircraft. Tether it, and keep a hand on the
        sticks — moving one aborts instantly and gives you control back.
      </p>

      {/* Onboard detector. This runs on the Pi, not here - it has to work
          when the link does not, because a dropped connection must never
          mean the drone flies past someone in the water. */}
      {detect?.enabled && (
        <div
          className={`rounded px-2 py-1.5 border ${
            detect.confirmed
              ? "bg-[#FF3B30]/15 border-[#FF3B30]"
              : "bg-black/30 border-white/10"
          }`}
        >
          <div className="flex items-center justify-between">
            <span className="font-label-caps text-[9px] text-on-surface-variant">
              ONBOARD DETECTION
            </span>
            <span
              className={`font-telemetry-sm text-[10px] ${
                detect.confirmed
                  ? "text-[#FF3B30]"
                  : detect.ready
                    ? "text-[#00E676]"
                    : "text-on-surface-variant"
              }`}
            >
              {detect.error
                ? "ERROR"
                : detect.confirmed
                  ? "⚠ DROWNING"
                  : detect.ready
                    ? "WATCHING"
                    : "STARTING…"}
            </span>
          </div>
          <div className="font-telemetry-sm text-[10px] text-on-surface-variant mt-0.5">
            {detect.error
              ? detect.error
              : `${detect.fps ?? "—"} fps on the Pi · ${detect.frames} frames`}
          </div>
        </div>
      )}

      {/* LEVEL ZERO-POINT - what the aircraft believes flat is.
          IN THE OPEN, not behind the gear: this is a pre-flight action
          done before every session, and it was previously two clicks
          and a scroll away inside a height-capped box.
          This is the REFERENCE the trim above is measured against, so
          it belongs first: trimming out a drift that is really a wrong
          zero-point just moves the problem.

          The React dashboard shipped without this entirely, although
          the Pico (Level_Capture_Run), the Pi (LEVEL_COMMANDS) and the
          ground station relay all supported it - so there was no way
          to tell the aircraft what level was.

          Hidden on firmware that predates the feature, rather than
          showing offsets that come from nowhere. */}
      {telemetry?.level && (
        <>
          <span className="font-label-caps text-[9px] text-on-surface-variant mt-1">
            LEVEL ZERO-POINT — what the drone thinks flat is
          </span>
          <div className="grid grid-cols-2 gap-2 font-telemetry-sm text-[11px]">
            <div className="flex flex-col">
              <span className="font-label-caps text-[8px] text-on-surface-variant">
                STORED OFFSET
              </span>
              <span
                className={
                  Math.max(
                    Math.abs(telemetry.level.roll_offset_deg),
                    Math.abs(telemetry.level.pitch_offset_deg),
                  ) > 10
                    ? "text-[#FF6B35]"
                    : "text-white"
                }
              >
                {telemetry.level.roll_offset_deg.toFixed(2)}° /{" "}
                {telemetry.level.pitch_offset_deg.toFixed(2)}°
              </span>
            </div>
            <div className="flex flex-col">
              <span className="font-label-caps text-[8px] text-on-surface-variant">
                STATUS
              </span>
              <span className="text-white">
                {link.levelMessage
                  ? link.levelMessage
                  : telemetry.level.state === "settling"
                    ? "Settling…"
                    : telemetry.level.state === "sampling"
                      ? "Measuring…"
                      : LEVEL_RESULT_TEXT[telemetry.level.result ?? ""] ?? "Idle"}
              </span>
            </div>
          </div>
          <div className="flex gap-2">
            <button
              onClick={() => droneLink.requestLevelCal("start")}
              disabled={telemetry.level.busy || link.armed}
              title="Stand the drone on a flat surface, throttle at zero, and do not touch it for three seconds."
              className="flex-1 px-3 py-2 rounded bg-[#00E676] text-[#04251b] font-bold
                         text-[11px] tracking-wide disabled:opacity-30 disabled:cursor-not-allowed"
            >
              SET LEVEL
            </button>
            <button
              onClick={() => droneLink.requestLevelCal("reset")}
              disabled={telemetry.level.busy || link.armed}
              title="Erase the stored zero-point and go back to the raw sensor."
              className="flex-1 px-3 py-2 rounded border border-white/20 bg-white/5
                         text-white font-bold text-[11px] tracking-wide
                         disabled:opacity-30 disabled:cursor-not-allowed"
            >
              ERASE
            </button>
          </div>
          <p className="font-body-md text-[9px] text-on-surface-variant m-0 opacity-70">
            Flat surface, throttle at zero, hands off for three seconds. It
            refuses if the drone moves, if the throttle is up, or if it is
            more than 15° out — that last one means the board is mounted
            wrong and no calibration will fix it. <b>Do this before trimming.</b>
          </p>
        </>
      )}

      {/* ---- height ---- */}
      <div className="grid grid-cols-3 gap-2 font-telemetry-sm text-[11px]">
        <div className="flex flex-col">
          <span className="font-label-caps text-[8px] text-on-surface-variant">HEIGHT</span>
          <span className="text-white">
            {height != null ? `${height.toFixed(2)} m` : "— no barometer"}
          </span>
        </div>
        <div className="flex flex-col">
          <span className="font-label-caps text-[8px] text-on-surface-variant">TARGET</span>
          <span className="text-white">
            {flying && status?.target_alt_m ? `${status.target_alt_m} m` : "—"}
          </span>
        </div>
        {/* After one steady hover this is a MEASUREMENT of the aircraft's
            hover throttle, not a guess. It is the number HOVER_THROTTLE in
            mission.py should be set to - worth writing down. */}
        <div className="flex flex-col">
          <span className="font-label-caps text-[8px] text-on-surface-variant">
            HOVER LEARNED
          </span>
          <span
            className={
              Math.abs(status?.hover_trim ?? 0) > 1
                ? "text-[#FFC107]"
                : "text-on-surface-variant"
            }
            title="The height hold trims its own hover point. Once it settles, this is what your hover throttle really is."
          >
            {status?.hover_learned != null ? `${status.hover_learned}%` : "—"}
          </span>
        </div>
      </div>

      {/* ---- the buttons ---- */}
      <div className="flex gap-2">
        <button
          onClick={() => post({ action: "takeoff", alt: parseFloat(takeoffAlt) })}
          disabled={busy || flying || !takeoffAlt || height == null}
          title={height == null ? "no barometer - takeoff needs a height reading" : ""}
          className="flex-1 px-3 py-2 rounded bg-[#00E676] text-[#04251b] font-bold
                     text-[11px] tracking-wide disabled:opacity-30 disabled:cursor-not-allowed"
        >
          TAKE OFF {takeoffAlt || "?"}m
        </button>
        <button
          onClick={() => post({ action: "land" })}
          disabled={busy || !flying}
          className="flex-1 px-3 py-2 rounded bg-[#FFC107] text-[#2a1f00] font-bold
                     text-[11px] tracking-wide disabled:opacity-30 disabled:cursor-not-allowed"
        >
          LAND
        </button>
      </div>

      <div className="flex gap-2">
        <button
          onClick={() => post({ lat: parseFloat(lat), lon: parseFloat(lon), alt: parseFloat(alt) })}
          disabled={busy || flying || !lat || !lon}
          className="flex-1 px-3 py-2 rounded border border-white/20 bg-white/5 text-white
                     font-bold text-[11px] tracking-wide
                     disabled:opacity-30 disabled:cursor-not-allowed"
        >
          FLY TO WAYPOINT
        </button>
        <button
          onClick={() => post({ action: "abort" })}
          disabled={busy || !flying}
          title="Idles the motors immediately. From height that is a drop — use LAND to come down."
          className="flex-1 px-3 py-2 rounded bg-[#FF3B30] text-white font-bold
                     text-[11px] tracking-wide disabled:opacity-30 disabled:cursor-not-allowed"
        >
          ABORT
        </button>
      </div>

      {/* ---- settings drawer ----
          Below the buttons on purpose: this panel is height-capped and
          scrolls, so a drawer above them pushed TAKE OFF and LAND out of
          sight exactly when they were wanted. */}
      {showSettings && (
        <div className="rounded border border-white/10 bg-black/30 p-2 flex flex-col gap-2">
          <span className="font-label-caps text-[9px] text-on-surface-variant">
            SETTINGS — kept on this device
          </span>
          <div className="grid grid-cols-2 gap-2">
            {field("TAKEOFF HEIGHT m", takeoffAlt, setTakeoffAlt, "takeoffAlt", "2")}
          </div>
          <span className="font-label-caps text-[9px] text-on-surface-variant mt-1">
            WAYPOINT
          </span>
          <div className="grid grid-cols-3 gap-2">
            {field("LAT", lat, setLat, "wpLat", "-34.1083")}
            {field("LON", lon, setLon, "wpLon", "18.4700")}
            {field("HEIGHT m", alt, setAlt, "wpAlt", "5")}
          </div>
          {/* DRIFT TRIM. The same control as in the flight bar and the same
              stored value - surfaced here because this is where you come
              looking for settings.

              Capped at TRIM_LIMIT (12 deg). If the aircraft genuinely needs
              more than that to sit still, the firmware's own threshold for
              "remount the board" is 15 deg - at that point it is a mounting
              or centre-of-gravity problem and trimming it out just hides a
              machine fighting itself. */}
          <span className="font-label-caps text-[9px] text-on-surface-variant mt-1">
            DRIFT TRIM — hold the correction so you do not have to
          </span>
          <div className="grid grid-cols-2 gap-2">
            {(["roll", "pitch"] as const).map((axis) => (
              <div key={axis} className="flex flex-col gap-0.5">
                <span className="font-label-caps text-[8px] text-on-surface-variant">
                  {axis.toUpperCase()} (MAX ±{TRIM_LIMIT}°)
                </span>
                <div className="flex items-center gap-1">
                  <button
                    onClick={() => droneLink.adjustTrim(axis, -1)}
                    className="px-2 py-1 rounded border border-white/20 bg-white/5
                               text-white font-bold text-[11px]"
                  >
                    −
                  </button>
                  <span className="flex-1 text-center font-telemetry-sm text-[11px] text-white">
                    {(axis === "roll" ? link.rollTrim : link.pitchTrim) > 0 ? "+" : ""}
                    {axis === "roll" ? link.rollTrim : link.pitchTrim}°
                  </span>
                  <button
                    onClick={() => droneLink.adjustTrim(axis, 1)}
                    className="px-2 py-1 rounded border border-white/20 bg-white/5
                               text-white font-bold text-[11px]"
                  >
                    +
                  </button>
                </div>
              </div>
            ))}
          </div>
          <p className="font-body-md text-[9px] text-on-surface-variant m-0 opacity-70">
            Trim only reaches the aircraft while you hold TAKE CONTROL. Takeoff
            needs only the barometer; the waypoint needs a GPS fix, which this
            aircraft has never had — the drone refuses it until it does.
          </p>
        </div>
      )}

      {travelling && (
        <div className="grid grid-cols-3 gap-2 font-telemetry-sm text-[11px]">
          <div className="flex flex-col">
            <span className="font-label-caps text-[8px] text-on-surface-variant">DISTANCE</span>
            <span className="text-white">
              {status?.distance_m != null ? `${status.distance_m} m` : "—"}
            </span>
          </div>
          <div className="flex flex-col">
            <span className="font-label-caps text-[8px] text-on-surface-variant">BEARING</span>
            <span className="text-white">
              {status?.bearing_deg != null ? `${status.bearing_deg}°` : "—"}
            </span>
          </div>
          <div className="flex flex-col">
            <span className="font-label-caps text-[8px] text-on-surface-variant">ELAPSED</span>
            <span className="text-white">{status?.elapsed_s ?? 0} s</span>
          </div>
        </div>
      )}

      {(message || status?.reason) && (
        <p className="font-body-md text-[10px] text-on-surface-variant m-0">
          {status?.reason || message}
        </p>
      )}

      {lim && (
        <p className="font-body-md text-[9px] text-on-surface-variant m-0 opacity-70">
          Takeoff {lim.min_alt_m}–{maxTakeoff} m. Waypoint {lim.max_range_m} m range ·{" "}
          {lim.min_alt_m}–{lim.max_alt_m} m · {lim.max_tilt_deg}° tilt ·{" "}
          {lim.max_speed_ms} m/s. Height is only good to ±{step} m — that is the
          barometer's resolution, not a display rounding, so a hold hunts by about
          that much.
        </p>
      )}
    </div>
  );
}
